import ctypes
from typing import List, Tuple
import numpy as np

# Load the shared library
lib = ctypes.CDLL("./src/c_code/control_system.so")


def _prepare_time_and_arrays(
    stop_time: float, dt: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Prepares time vector and zero arrays for input and output."""
    samples = int(stop_time / dt)
    t = np.linspace(0, stop_time, samples, dtype=np.float64)
    u = np.zeros(samples, dtype=np.float64)
    y = np.zeros(samples, dtype=np.float64)
    return t, u, y


def _setup_ctypes_function(function, argtypes: List, restype=None):
    """Configures argument and return types for a C function."""
    function.argtypes = argtypes
    function.restype = restype


def observer_controller_with_identification_rls(
    num,
    den,
    stop_time,
    dt,
    delay,
    method,
    reference=1.0,
    pwm_cycle=100,
    digital=0,
    params=[],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulates a control system with an observer-based controller and RLS identification.
    """
    t, u, y = _prepare_time_and_arrays(stop_time, dt)
    u_obs = np.zeros(len(t), dtype=np.float64)
    y_obs = np.zeros(len(t), dtype=np.float64)
    erros = np.zeros(5, dtype=np.float64)
    values = np.zeros(8, dtype=np.float64)

    _setup_ctypes_function(
        lib.observer_controller_with_identification_rls,
        [
            ctypes.POINTER(ctypes.c_double),  # num
            ctypes.c_int,  # num_len
            ctypes.POINTER(ctypes.c_double),  # den
            ctypes.c_int,  # den_len
            ctypes.c_double,  # stop_time
            ctypes.c_double,  # delay
            ctypes.POINTER(ctypes.c_double),  # u
            ctypes.POINTER(ctypes.c_double),  # y
            ctypes.POINTER(ctypes.c_double),  # u_obs
            ctypes.POINTER(ctypes.c_double),  # y_obs
            ctypes.c_int,  # method
            ctypes.c_double,  # reference
            ctypes.c_int,  # pwm_cycle
            ctypes.c_int,  # digital
            ctypes.POINTER(ctypes.c_double),  # Params
            ctypes.POINTER(ctypes.c_double),  # erros
            ctypes.POINTER(ctypes.c_double),  # values
        ],
    )

    lib.observer_controller_with_identification_rls(
        num.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        len(num),
        den.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        len(den),
        stop_time,
        delay,
        u.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        method,
        reference,
        pwm_cycle,
        digital,
        params.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        erros.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        values.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )

    return t, u, y, u_obs, y_obs  # , erros, values


def run_all_simulations(
    num,
    den,
    stop_time,
    dt,
    delay,
    method,
    reference=1.0,
    pwm_cycle=100,
    digital=0,
    params=[],
) -> Tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Runs all simulations for the given system parameters.
    """
    t, u, y = _prepare_time_and_arrays(stop_time, dt)
    u_step = np.zeros(len(t), dtype=np.float64)
    y_step = np.zeros(len(t), dtype=np.float64)
    u_pid = np.zeros(len(t), dtype=np.float64)
    y_pid = np.zeros(len(t), dtype=np.float64)
    u_smith = np.zeros(len(t), dtype=np.float64)
    y_smith = np.zeros(len(t), dtype=np.float64)
    u_obs = np.zeros(len(t), dtype=np.float64)
    y_obs = np.zeros(len(t), dtype=np.float64)

    _setup_ctypes_function(
        lib.run_all_simulations,
        [
            ctypes.POINTER(ctypes.c_double),  # num
            ctypes.c_int,  # num_len
            ctypes.POINTER(ctypes.c_double),  # den
            ctypes.c_int,  # den_len
            ctypes.c_double,  # stop_time
            ctypes.c_double,  # delay
            ctypes.c_int,  # method
            ctypes.c_double,  # reference
            ctypes.c_int,  # pwm_cycle
            ctypes.c_int,  # digital
            ctypes.POINTER(ctypes.c_double),  # u_step
            ctypes.POINTER(ctypes.c_double),  # y_step
            ctypes.POINTER(ctypes.c_double),  # u_pid
            ctypes.POINTER(ctypes.c_double),  # y_pid
            ctypes.POINTER(ctypes.c_double),  # u_smith
            ctypes.POINTER(ctypes.c_double),  # y_smith
            ctypes.POINTER(ctypes.c_double),  # u_obs
            ctypes.POINTER(ctypes.c_double),  # y_obs
        ],
    )

    lib.run_all_simulations(
        num.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        len(num),
        den.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        len(den),
        stop_time,
        delay,
        method,
        reference,
        pwm_cycle,
        digital,
        params.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_step.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y_step.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_pid.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y_pid.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_smith.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y_smith.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        u_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        y_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )

    # return {
    #     "t": t,
    #     "u_pid": u_pid,
    #     "y_pid": y_pid,
    #     "u_smith": u_smith,
    #     "y_smith": y_smith,
    # }
    return t, u_step, y_step, u_pid, y_pid, u_smith, y_smith
