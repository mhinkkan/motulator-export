"""
Export induction machine drives to Simulink.

This module writes a Simulink model of a motulator induction machine drive with
current-vector control, in the same way as `motulator_export.simulink.sm` does for
synchronous machine drives. The machine is modeled with the inverse-Γ model in
stator coordinates, which is equivalent to the Γ model of `InductionMachine` in
motulator.

Currently supported: the induction machines and control systems of
`motulator_export.plecs.im`, with the converter of `motulator_export.simulink.sm`.

The control system can also be exported alone as a library (`write_control_system`),
to be used with the I/O blocks of a real-time target.

"""

from pathlib import Path

import numpy as np
from motulator.drive.control._base import VectorControlSystem
from motulator.drive.model import Drive

from motulator_export.plecs import im
from motulator_export.plecs._common import StepSignal
from motulator_export.simulink._common import simulate_control, write_control_script
from motulator_export.simulink._drive import (
    check_supported_converter,
    simulate_drive,
    write_drive_model,
)

BLOCK = im.CVC_BLOCK


def write_model(
    path: str | Path,
    mdl: Drive,
    ctrl: VectorControlSystem,
    w_M_ref: StepSignal,
    tau_L: StepSignal,
    t_stop: float,
    speed_ctrl_args: dict[str, float],
    enable: StepSignal | float = 1.0,
) -> Path:
    """Write a MATLAB script that builds the Simulink model, see `sm.write_model`."""
    im._check_supported(mdl, ctrl)
    check_supported_converter(mdl)
    values = im.export_mask_values(ctrl, speed_ctrl_args)
    variables = im._plant_variables(mdl)
    return write_drive_model(
        path, BLOCK, values, variables, "im", w_M_ref, tau_L, t_stop, enable=enable
    )


def simulate(
    path: str | Path, t_eval: np.ndarray, build: bool = True
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate the Simulink model, see `sm.simulate`."""
    return simulate_drive(path, t_eval, BLOCK, build)


def write_control_system(
    path: str | Path, ctrl: VectorControlSystem, speed_ctrl_args: dict[str, float]
) -> Path:
    """
    Write a MATLAB script that builds a Simulink library of the control system.

    The library contains only the masked subsystem of the control system, with the
    inputs `enable`, `w_M_ref`, `i_s_abc`, `u_dc`, and `w_M` and the outputs `d_abc`
    and `signals` (the monitored signals). It is intended for real-time targets, where
    the I/O blocks of the target replace the system model. Running the script in MATLAB
    compiles the S-function and saves the library in the folder of the script.

    Parameters
    ----------
    path : str | Path
        Path of the library (.slx). The script is written in the same folder, named
        `build_<library>.m`, with the standalone source of the S-function.
    ctrl : VectorControlSystem
        Discrete-time control system.
    speed_ctrl_args : dict[str, float]
        Arguments of `SpeedController` used in `ctrl`.

    Returns
    -------
    Path
        Path of the written script.

    """
    im._check_supported_control(ctrl)
    values = im.export_mask_values(ctrl, speed_ctrl_args)
    return write_control_script(Path(path), BLOCK, values)


def simulate_control_system(
    path: str | Path, inputs: np.ndarray, T_s: float, build: bool = True
) -> dict[str, np.ndarray]:
    """
    Run the control system of the library with given inputs in Simulink.

    Parameters
    ----------
    path : str | Path
        Path of the library (.slx).
    inputs : ndarray, shape (n, 7)
        Inputs `enable`, `w_M_ref`, `i_s_abc`, `u_dc`, and `w_M` at the sampling
        instants.
    T_s : float
        Sampling period (s).
    build : bool, optional
        Run the build script `build_<library>.m` first, defaults to True.

    Returns
    -------
    dict[str, ndarray]
        Duty ratios "d_a", "d_b", and "d_c" and the monitored signals at the sampling
        instants.

    """
    return simulate_control(path, BLOCK, inputs, T_s, build)
