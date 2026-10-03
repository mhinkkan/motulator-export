"""
Common parts of the Simulink export: the build scripts and the simulation.

A model is exported as a MATLAB script, which sets the fields of a struct `s` (the
parameters, the control system, and the signals) and calls a builder function of
this directory (e.g., `build_drive.m`). The builder compiles the S-function, builds
the model with the Simulink API, and saves it in the folder of the script.

"""

import os
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from motulator_export.plecs._common import ControlBlock, StepSignal, _fmt_mask
from motulator_export.plecs._schematic import _fmt
from motulator_export.simulink._sfunction import SFunction, control_sfunction

SIMULINK_SOURCES = Path(__file__).parent

# ASCII replacements in the mask prompts
ASCII = str.maketrans({"Ω": "Ohm", "²": "^2"})


def _m_str(text: str) -> str:
    """MATLAB character vector, joined from its lines if it has several."""
    if "\n" in text:
        lines = text.rstrip("\n").split("\n")
        items = ", ...\n    ".join(map(_m_str, lines))
        return "strjoin({ ...\n    " + items + "}, newline)"
    return "'" + text.replace("'", "''") + "'"


def _m(value: Any) -> str:
    """MATLAB literal of a string, a number, or a (nested) list."""
    if isinstance(value, str):
        return _m_str(value)
    if isinstance(value, Sequence):
        if value and all(isinstance(v, str) for v in value):
            return "{" + ", ".join(map(_m, value)) + "}"
        rows = [v for v in value if isinstance(v, Sequence) and not isinstance(v, str)]
        if not rows or len(rows) < len(value):
            return _fmt(list(value))  # Numeric vector
        if all(_is_numeric(r) for r in rows):
            return "[" + "; ".join(" ".join(_fmt(x) for x in r) for r in rows) + "]"
        cells = "".join("    " + ", ".join(map(_m, r)) + "\n" for r in rows)
        return "{ ...\n" + cells + "    }"
    return _fmt(value)


def _is_numeric(values: Sequence[Any]) -> bool:
    """Whether all the values are numbers."""
    return all(isinstance(v, (int, float, np.number)) for v in values)


def _m_steps(sig: StepSignal) -> list[list[float]]:
    """
    Rows [time, before, after] of the steps of a step signal (see add_step.m).

    The first row steps from `before` to the first level, and the others from zero
    to the increments of the levels, so that the sum of the steps is the signal.

    """
    times = np.atleast_1d(sig.time)
    levels = [sig.before, *np.atleast_1d(sig.after)]
    rows = [[float(times[0]), float(levels[0]), float(levels[1])]]
    rows += [
        [float(t), 0.0, float(levels[k + 2] - levels[k + 1])]
        for k, t in enumerate(times[1:])
    ]
    return rows


def _m_source(sig: StepSignal | float) -> list[list[float]] | float:
    """Steps of a step signal (see `_m_steps`), or a constant."""
    return _m_steps(sig) if isinstance(sig, StepSignal) else float(sig)


def control_fields(
    block: ControlBlock, sfun: SFunction, values: dict[str, Any]
) -> list[tuple[str, Any]]:
    """Fields of the struct of the control system (see add_control_system.m)."""
    mask = [
        [m.variable, m.prompt.translate(ASCII), m.tab, _fmt_mask(values[m.variable])]
        for m in block.mask_params
    ]
    return [
        ("name", block.name),
        ("sfunction", sfun.name),
        ("mask_type", block.mask_type),
        ("description", block.description),
        ("mask", mask),
        ("mask_init", block.mask_init),
        ("params", sfun.params),
        ("inputs", block.inputs),
        ("outputs", ["d_abc", *block.outputs]),
    ]


def scope_indices(
    scope: list[tuple[str, list[str]]], mdl_outputs: list[str], ctrl_signals: list[str]
) -> list[list[Any]]:
    """
    Scope signals as indices in [mdl; ctrl].

    The signals are given by names with the prefix "mdl." or "ctrl.".

    """
    names = [f"mdl.{n}" for n in mdl_outputs] + [f"ctrl.{n}" for n in ctrl_signals]
    return [[title, [names.index(n) + 1 for n in signals]] for title, signals in scope]


def write_script(
    path: Path,
    block: ControlBlock,
    values: dict[str, Any],
    fields: list[tuple[str, Any]],
    builder: str,
    T_s: float,
    t_stop: float,
    sfunctions: Sequence[SFunction] = (),
) -> Path:
    """
    Write the S-functions and the MATLAB script building the Simulink model.

    Parameters
    ----------
    path : Path
        Path of the model (.slx). The S-function and the script `build_<model>.m`
        are written in the same folder.
    block : ControlBlock
        Control-system block, whose C-Script code is wrapped in the S-function.
    values : dict[str, Any]
        Mask parameter values of the control system.
    fields : list[tuple[str, Any]]
        Other fields of the struct `s`, e.g., the workspace code "init".
    builder : str
        Builder function, e.g., "build_drive".
    T_s : float
        Sampling period (s).
    t_stop : float
        Simulation stop time (s).
    sfunctions : Sequence[SFunction], optional
        Other S-functions of the model (e.g., of the system model), compiled by the
        builder.

    Returns
    -------
    Path
        Path of the script.

    """
    sfun = control_sfunction(block)
    for f in (sfun, *sfunctions):
        f.write(path.parent)
    intro = (
        f"% Build the Simulink model {path.stem}.slx in the folder of this script.\n"
        "% Generated by motulator_export.simulink from motulator. The parameters of\n"
        f"% the control system are in the mask of the subsystem '{block.name}',\n"
        "% and those of the system model in the model workspace.\n"
    )
    all_fields = [
        *fields,
        *((f"control.{n}", v) for n, v in control_fields(block, sfun, values)),
        ("T_s", T_s),
        ("t_stop", t_stop),
    ]
    return _write_build_script(path, intro, sfun, all_fields, builder)


def write_control_script(
    path: Path, block: ControlBlock, values: dict[str, Any]
) -> Path:
    """
    Write the S-function and the MATLAB script building a library of a control system.

    The library contains only the masked subsystem of the control system, to be used
    in other models, e.g., with the I/O blocks of a real-time target. The source of
    the S-function is standalone, i.e., it contains the C port. The S-function is
    named after the library (`sfun_<library>`), so that it differs from that of a
    model of the whole system in the same folder.

    Parameters
    ----------
    path : Path
        Path of the library (.slx). The S-function and the script `build_<library>.m`
        are written in the same folder.
    block : ControlBlock
        Control-system block, whose C-Script code is wrapped in the S-function.
    values : dict[str, Any]
        Mask parameter values of the control system.

    Returns
    -------
    Path
        Path of the script.

    """
    sfun = control_sfunction(block, path.stem)
    sfun.write(path.parent, standalone=True)
    intro = (
        f"% Build the Simulink library {path.stem}.slx in the folder of this script.\n"
        "% Generated by motulator_export.simulink from motulator. The library\n"
        f"% contains the control system '{block.name}', whose parameters\n"
        "% are in the mask of the subsystem.\n"
    )
    fields = [(f"control.{n}", v) for n, v in control_fields(block, sfun, values)]
    return _write_build_script(path, intro, sfun, fields, "build_control")


def _write_build_script(
    path: Path, intro: str, sfun: SFunction, fields: list[tuple[str, Any]], builder: str
) -> Path:
    """Write the MATLAB script setting the struct `s` and calling the builder."""
    try:
        src = Path(os.path.relpath(SIMULINK_SOURCES, path.resolve().parent))
    except ValueError:  # Different drives (Windows)
        src = SIMULINK_SOURCES
    src_parts = ", ".join(_m(p) for p in src.parts)
    text = (
        intro + "%\n"
        f"% The S-function {sfun.name}.c is compiled with mex, which needs a C99\n"
        "% compiler with complex.h (gcc, clang, or MinGW-w64, not MSVC), see\n"
        "% mex -setup C.\n"
        "\n"
        "here = fileparts(mfilename('fullpath'));\n"
        f"addpath(fullfile(here, {src_parts}));\n"
        "\n"
        "s = struct();\n"
        "s.folder = here;\n"
        + "".join(f"s.{n} = {_m(v)};\n" for n, v in [("name", path.stem), *fields])
        + "\n"
        f"{builder}(s);\n"
    )
    script = path.with_name(f"build_{path.stem}.m")
    script.write_text(text)
    return script


def simulate_control(
    path: str | Path,
    block: ControlBlock,
    inputs: np.ndarray,
    T_s: float,
    build: bool = True,
) -> dict[str, np.ndarray]:
    """
    Run the control system of a library with given inputs in Simulink.

    The block is run alone (open loop) in a temporary model with the fixed step
    `T_s`, via the MATLAB Engine API for Python (see `run_control.m`).

    Parameters
    ----------
    path : str | Path
        Path of the library (.slx).
    block : ControlBlock
        Control-system block of the library.
    inputs : ndarray, shape (n, sum(block.input_widths))
        Inputs of the block at the sampling instants.
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
    # MATLAB Engine API for Python
    import matlab  # noqa: PLC0415  # pyright: ignore[reportMissingImports]
    import matlab.engine  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

    path = Path(path).resolve()
    eng: Any = matlab.engine.start_matlab()
    try:
        if build:
            eng.run(str(path.with_name(f"build_{path.stem}.m")), nargout=0)
        eng.addpath(str(path.parent), nargout=0)
        eng.addpath(str(SIMULINK_SOURCES), nargout=0)
        values = eng.run_control(
            path.stem,
            block.name,
            matlab.double(block.input_widths),
            matlab.double(np.asarray(inputs, float).tolist()),
            float(T_s),
        )
    finally:
        eng.quit()
    names = ["d_a", "d_b", "d_c", *block.signals]
    return dict(zip(names, np.asarray(values).T, strict=True))


def simulate(
    path: str | Path,
    t_eval: np.ndarray,
    mdl_outputs: list[str],
    ctrl_signals: list[str],
    build: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Simulate a Simulink model via the MATLAB Engine API for Python.

    Parameters
    ----------
    path : str | Path
        Path of the model (.slx).
    t_eval : ndarray
        Output times (s).
    mdl_outputs : list[str]
        Names of the signals of the output port "mdl".
    ctrl_signals : list[str]
        Names of the signals of the output port "ctrl".
    build : bool, optional
        Run the build script `build_<model>.m` first, defaults to True.

    Returns
    -------
    mdl : dict[str, ndarray]
        Signals of the system model and the time "t".
    ctrl : dict[str, ndarray]
        Monitored signals of the control system and the time "t".

    """
    # MATLAB Engine API for Python
    import matlab  # noqa: PLC0415  # pyright: ignore[reportMissingImports]
    import matlab.engine  # noqa: PLC0415  # pyright: ignore[reportMissingImports]

    path = Path(path).resolve()
    eng: Any = matlab.engine.start_matlab()
    try:
        if build:
            eng.run(str(path.with_name(f"build_{path.stem}.m")), nargout=0)
        eng.addpath(str(path.parent), nargout=0)
        eng.workspace["t_eval"] = matlab.double(np.asarray(t_eval, float).tolist())
        eng.eval(
            f"out = sim({_m(path.stem)}, 'OutputOption', "
            "'SpecifiedOutputTimes', 'OutputTimes', 't_eval');",
            nargout=0,
        )
        t = np.asarray(eng.eval("out.tout")).ravel()
        values = np.asarray(eng.eval("out.yout")).T
    finally:
        eng.quit()
    n_mdl = len(mdl_outputs)
    if values.shape[0] != n_mdl + len(ctrl_signals):
        raise RuntimeError(f"Unexpected number of output signals: {values.shape}")
    mdl = {"t": t, **dict(zip(mdl_outputs, values[:n_mdl], strict=True))}
    ctrl = {"t": t, **dict(zip(ctrl_signals, values[n_mdl:], strict=True))}
    return mdl, ctrl
