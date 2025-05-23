import numpy as np
from src.controllers.select_option import run_all_simulations_func, observer_controller_with_identification_rls_func
from tqdm import tqdm
import pandas as pd

# Helper function to configure simulation parameters
def configure_simulation(num, den, delay, set_point=50.0, stop_time=8000, dt=0.1):
    """Returns simulation parameters with dynamic numerator, denominator, and delay."""
    return {
        "num": np.array(num, dtype=np.float64),
        "den": np.array(den, dtype=np.float64),
        "delay": delay,
        "stop_time": stop_time,
        "dt": dt,
        "set_point": set_point,
        "integration_method": 1,  # 0 for Euler, 1 for Runge-Kutta
    }

# Helper function to configure additional parameters
def configure_options():
    """Returns additional simulation options."""
    return {
        "pwm_duty_cycle": 10,  # Simulated PWM duty cycle (if applicable)
        "digital": 0,  # Digital/Analog option
        "lambda_value": 2.0,  # Lambda method parameter
    }

def test_system(num, den, delay, stop_time=5000, set_point=50.0, dt=0.1):
    """Simulates the given system using the main function logic."""
    sim_params = configure_simulation(num, den, delay, set_point, stop_time, dt)
    options = configure_options()

    df = observer_controller_with_identification_rls_func(
        num=sim_params["num"],
        den=sim_params["den"],
        stop_time=sim_params["stop_time"],
        dt=sim_params["dt"],
        delay=sim_params["delay"],
        method=sim_params["integration_method"],
        digital=options["digital"],
        pwm_duty_cycle=options["pwm_duty_cycle"],
        set_point=sim_params["set_point"],
    )

    return df

def main():
    """Runs all test systems."""
    systems = [
        # First order
        # ([1], [2, 1], 1, 20),
        # ([1], [0.5, 1], 2, 20),
        ([1], [10, 1], 1, 50),
        # # Second order
        # ([1], [100, 20, 1], 0.5),
        # ([1], [4, 4, 1], 1),
        # ([1], [0.25, 1, 1], 1),
        # # Third order
        # ([1], [1, 3, 3, 1], 10),
        # # Eight order
        # ([1], [1679616, 2239488, 1306368, 435456, 90720, 12096, 1008, 48, 1], 0.6),
        # Heating Furnace
        # ([11], [50, 1], 80, 2500),
        # # Multi-Channel Biomass Hot Air Furnace
        # ([3.08], [889, 1], 50, 8000),
        # # Biomass microwave pyrolysis
        # ([1.09], [190.08, 1], 76, 3000),
        # # Benchmark Heat Exchanger
        # ([1.44], [90.77, 1], 27, 2500),
        # # Extrusors
        # ([0.92], [144, 1], 10, 2000),  # $G(s) = 0.92e^{-10s}/(144s+1)$
        # ([0.869], [21.098, 1], 0, 2000),  # $G(s) = 0.869/(21.098s+1)$
        # ([18], [130, 1], 60, 2000),  # $G(s) = 18e^{-60s}/(130s+1)$
        # ([0.896], [1.789, 1], 3, 500),  # $G(s) = 0.896e^{-3s}/(1.789s+1)$
    ]

    results = []
    for num, den, delay, stop_time in tqdm(systems, desc="Systems", leave=False):
        result = test_system(num, den, delay, stop_time)



if __name__ == "__main__":
    main()