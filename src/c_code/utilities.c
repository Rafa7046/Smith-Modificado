#include "control_system.h"

double clamp(double value, double min_val, double max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

double calculate_pwm(ControlSystemState *controller, double u_temp)
{
    ControllerState *ctrl = &controller->ctrl;
    if (ctrl->digital)
    {
        double u = ctrl->on_value;
        if (ctrl->pwm_counter <= 0 || ctrl->pwm_counter >= ctrl->pwm_cycle_length)
        {
            double duty_cycle = (u_temp >= 0) ? (u_temp / 100.0) : 0;
            ctrl->on_steps = (int)ceil(duty_cycle * ctrl->pwm_cycle_length);
        }
        if (ctrl->pwm_counter < ctrl->on_steps)
        {
            u = ctrl->on_value;
        }
        else
        {
            u = 0.0;
        }
        if (++ctrl->pwm_counter >= ctrl->pwm_cycle_length)
        {
            ctrl->pwm_counter = 0;
        }
        return u;
    }
    return u_temp;
}

void extract_final_parameters(ControlSystemState *controller, double *errors, double *values)
{
    DelayState *d = &controller->delay_info;
    RLSState *rls = &controller->rls;
    ControllerState *c = &controller->ctrl;

    values[0] = rls->theta[0];
    values[1] = rls->theta[1];
    values[2] = rls->K;
    values[3] = rls->tau;
    values[4] = d->identified_delay;
    values[5] = c->kp;
    values[6] = c->ki;
    values[7] = c->kd;
}

void standard_deviation(ControlSystemState *controller, double y)
{
    DelayState *delay_info = &controller->delay_info;
    if (!delay_info->finished_noise_estimation)
    {
        double delta = y - delay_info->mean;
        delay_info->mean += delta / (controller->counter);
        delay_info->M2 += delta * (y - delay_info->mean);
        double variance = delay_info->M2 / (controller->counter);
        delay_info->standard_deviation = sqrt(variance);
        if (controller->counter >= delay_info->stddev_samples)
        {
            delay_info->finished_noise_estimation = 1;
        }
    }
}
