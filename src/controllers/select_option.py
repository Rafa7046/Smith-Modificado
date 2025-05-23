from ..utils import *
import pandas as pd
from tqdm import tqdm

stddev_samples = 1
noise_scale = 3
rls_time_scale = 5.0
K_dis = 1.0
forgetting_factor = 0.999
lambda_value = 0.4
initial_set_point_factor = 0.02
filter_factor = 1.0
params = np.array([stddev_samples, noise_scale, rls_time_scale, K_dis, forgetting_factor, lambda_value, initial_set_point_factor, filter_factor])

def observer_controller_with_identification_rls_func(num, den, stop_time, dt, delay, method=0, digital=0, pwm_duty_cycle=30, set_point=1.0):
    result = observer_controller_with_identification_rls(num, den, stop_time, dt, delay, method, set_point, pwm_duty_cycle, digital, params)
    plot_response(*result, set_point=set_point, identifier=den[0])

def run_all_simulations_func(num, den, stop_time, dt, delay, method=0, digital=0, pwm_duty_cycle=30, set_point=1.0):
    result = run_all_simulations(num, den, stop_time, dt, delay, method, set_point, pwm_duty_cycle, digital, params)
    plot_all_comparative(*result, set_point=set_point, identifier=den[0])
    return result