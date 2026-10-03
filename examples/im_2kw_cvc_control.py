"""
2.2-kW IM, sensorless CVC: control system as a Simulink library
===============================================================

This script writes a MATLAB script (simulink/build_im_2kw_cvc_control.m) that builds
a Simulink library containing only the control system of `im_2kw_cvc.py`. The library
is intended for real-time targets, where the I/O blocks of the target replace the
system model. If the MATLAB Engine API for Python is
installed, the script builds the library, runs the control system with given inputs
(open loop) in Simulink and in motulator, and compares the results. The control
system is disabled by its input `enable` in the beginning and for a while in the
middle, as after a fault, so that it should start twice from its initial state.

Run from the repository root:

    python examples/im_2kw_cvc_control.py

Without the MATLAB Engine API, run the generated script in MATLAB.

"""

# %%
import sys
from math import pi
from pathlib import Path
from typing import Any, cast

import motulator.drive.control.im as control
import numpy as np
from im_2kw_cvc import SPEED_CTRL, W_M_REF, build_system
from motulator.common.utils import complex2abc
from motulator.drive.control._base import Measurements

from motulator_export.simulink import im

U_DC = 540.0  # Measured DC-bus voltage (V)
# Samples disabled and enabled, twice. The enabled periods are short, since the
# rounding errors grow in the open-loop operation of the sensorless control system.
N_OFF, N_ON = 80, 400


# %%
def simulate_motulator(inputs: np.ndarray) -> dict[str, np.ndarray]:
    """
    Run the control system of motulator from its initial state (open loop).

    The columns of the inputs are `w_M_ref`, `i_s_abc`, `u_dc`, and `w_M`.

    """
    _, ctrl = build_system()
    ctrl.set_speed_ref(lambda t: float(inputs[0, 0]))
    out = []
    for u in inputs:
        i_s_ab = (2 / 3) * (u[1] - 0.5 * (u[2] + u[3])) + 1j * (u[2] - u[3]) / 3**0.5
        fbk = cast(Any, ctrl.get_feedback(Measurements(i_s_ab, u[4], u[5], None)))
        ref = cast(Any, ctrl.compute_output(fbk))
        ctrl.update(ref, fbk)
        out.append([*ref.d_abc, fbk.w_M, ref.tau_M, abs(fbk.psi_R)])
    names = ["d_a", "d_b", "d_c", "w_M", "tau_M_ref", "psi_R"]
    return dict(zip(names, np.array(out).T, strict=True))


# %%
if __name__ == "__main__":
    _, ctrl = build_system()
    T_s = cast(control.CurrentVectorController, ctrl.vector_ctrl).cfg.T_s
    path = Path(__file__).parent / "simulink" / "im_2kw_cvc_control.slx"
    path.parent.mkdir(exist_ok=True)
    script = im.write_control_system(path, ctrl, SPEED_CTRL)
    print(f"Wrote {script}")
    try:
        import matlab.engine  # noqa: F401, PLC0415  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("MATLAB Engine API for Python not available, skipping the comparison.")
        sys.exit()

    # Inputs: constant speed reference, rotating current vector, and zero speed. The
    # control system is enabled twice.
    enable = np.tile(np.repeat([0.0, 1.0], [N_OFF, N_ON]), 2)
    t = np.arange(enable.size) * T_s
    i_s_abc = complex2abc(3 * np.exp(2j * pi * 10 * t))
    inputs = np.column_stack(
        [enable, np.full(t.size, W_M_REF.after), *i_s_abc, np.full(t.size, U_DC), 0 * t]
    )
    sl = im.simulate_control_system(path, inputs, T_s)

    # Disabled: zero voltage and zero monitored signals
    off = enable == 0
    disabled_ok = all(
        np.all(value[off] == (0.5 if name.startswith("d_") else 0.0))
        for name, value in sl.items()
    )
    print(f"Duty ratios 0.5 and signals 0 while disabled: {disabled_ok}")

    # Enabled: both periods should agree with motulator started from its initial state
    periods = [slice(N_OFF, N_OFF + N_ON), slice(2 * N_OFF + N_ON, None)]
    res = [simulate_motulator(inputs[k, 1:]) for k in periods]
    print("Maximum differences (Simulink - motulator), 1st and 2nd enabled period:")
    for name in res[0]:
        err = [
            np.max(np.abs(sl[name][k] - r[name]))
            for k, r in zip(periods, res, strict=True)
        ]
        print(f"  {name}: {err[0]:.3g}, {err[1]:.3g}")
