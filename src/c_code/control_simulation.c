#include "control_system.h"

void controller_state(ControlSystemState *controller, double y)
{
    DelayState *delay_info = &controller->delay_info;
    RLSState *rls = &controller->rls;
    ControllerState *ctrl = &controller->ctrl;
    if (!delay_info->is_delay_identified)
    {
        delay_info->u_buffer[controller->counter] = controller->u_temp;
    }

    if (delay_info->finished_noise_estimation && !delay_info->is_delay_identified && y > delay_info->noise_scale * delay_info->standard_deviation)
    {
        delay_info->is_delay_identified = 1;
        delay_info->identified_delay = controller->counter * dt;
        delay_info->delay_samples = controller->counter;
    }

    if (!ctrl->start_control)
    {
        ctrl->start_control = (delay_info->is_delay_identified && controller->counter > rls->rls_time_scale * delay_info->delay_samples);
    }
}

void update_rls_parameters(ControlSystemState *controller, double y)
{
    DelayState *delay_info = &controller->delay_info;
    RLSState *rls = &controller->rls;

    rls->phi[0] = -rls->prev_y;
    rls->phi[1] = rls->prev_u;

    // Perform RLS update if delay is identified
    if (delay_info->is_delay_identified)
    {
        rls->y_est = RLS(rls, y);

        if (rls->theta[0] > -1 && rls->theta[1] > 0)
        {
            rls->a1 = -rls->theta[0];
            rls->b1 = rls->theta[1];

            rls->tau = (-dt / (log(rls->a1) + 1e-10)); // Avoid division by zero
            rls->K = rls->b1 / (1 - rls->a1 + 1e-10);  // Avoid division by zero
        }
    }
    rls->prev_y = y;
    rls->prev_u = delay_info->u_buffer[0];
}

void update_observer(ControlSystemState *controller)
{
    DelayState *delay_info = &controller->delay_info;
    RLSState *rls = &controller->rls;
    ControllerState *ctrl = &controller->ctrl;
    double y = 0.0;
    if (delay_info->is_delay_identified)
    {
        y = simulate_step_alpha_beta(rls, controller->u_temp, controller->y_obs_temp);
    }
    controller->y_obs_temp = y;
}

double pid_calculation(ControlSystemState *controller, double current_value, double reference)
{
    ControllerState *ctrl = &controller->ctrl;
    double u_temp = controller->u_temp;
    if (ctrl->start_control)
    {
        ctrl->error_prev = ctrl->error;
        ctrl->error = reference - current_value;
        ctrl->integral += ctrl->error * dt;
        // ctrl->integral = clamp(ctrl->integral, -100.0, 100.0);
        double derivative = (ctrl->error - ctrl->error_prev) / dt;

        u_temp = ctrl->kp * ctrl->error + ctrl->ki * ctrl->integral + ctrl->kd * derivative;
        u_temp = clamp(u_temp, 0, 100.0);
        ctrl->u_temp_continuo = u_temp;

        return u_temp;
    }
}

double pid_incremental(ControlSystemState *controller, double current_value, double reference)
{
    ControllerState *ctrl = &controller->ctrl;
    double u_temp = controller->u_temp;
    if (ctrl->start_control)
    {
        double error_prev_prev = ctrl->error_prev;
        ctrl->error_prev = ctrl->error;
        ctrl->error = reference - current_value;
        double proportional = ctrl->kp * (ctrl->error - ctrl->error_prev);
        double integral = ctrl->ki * dt * ctrl->error;
        double derivative = ctrl->kd * (ctrl->error - 2 * ctrl->error_prev + error_prev_prev) / dt;

        u_temp = ctrl->u_temp_continuo + proportional + integral + derivative;
        u_temp = clamp(u_temp, -100.0, 100.0);
        ctrl->u_temp_continuo = u_temp;
    }

    return u_temp;
}

double calculate_u(ControlSystemState *controller, double y_current, double set_point)
{
    if (controller->ctrl.start_control)
    {
        double y_ref = controller->y_obs_temp + (y_current - controller->rls.y_est) * controller->K_dis; // Smith predictor discrepancy control

        double u_temp = pid_calculation(controller, y_ref, set_point);
        // double u_temp = pid_incremental(controller, y_ref, set_point);

        controller->u_temp = u_temp;
    }

    return controller->u_temp;
}

void update_buffer(ControlSystemState *controller, double new_value)
{
    DelayState *delay_info = &controller->delay_info;
    if (delay_info->is_delay_identified)
    {
        memmove(delay_info->u_buffer, delay_info->u_buffer + 1, (delay_info->delay_samples - 1) * sizeof(double));
        delay_info->u_buffer[delay_info->delay_samples - 1] = new_value;
    }
}

double simulate_step_alpha_beta(RLSState *rls, double u_temp, double y_prev)
{
    double alpha = rls->K * dt / (rls->tau + dt);
    double beta = rls->tau / (rls->tau + dt);
    double output = alpha * u_temp + beta * y_prev;
    return clamp(output, 0.0, 100.0);
}

void lambda_tuning(ControlSystemState *controller)
{
    DelayState *delay_info = &controller->delay_info;
    RLSState *rls = &controller->rls;
    ControllerState *ctrl = &controller->ctrl;
    if (delay_info->is_delay_identified)
    {
        ctrl->kp = (rls->tau) / (rls->K * controller->lambda * (rls->tau + delay_info->identified_delay));

        ctrl->ki = ctrl->kp / rls->tau;

        ctrl->kd = 0;
        // *kd_lambda = *kp_lambda * (identified_time_constant * identified_delay) / (identified_time_constant + identified_delay);
    }
}

double RLS(RLSState *rls, double y)
{
    int n = rls->n + 1;
    double phiT[sys_n + 1]; // Transposed phi
    double phiTP[sys_n + 1];
    double thetaT[sys_n + 1];                // Transposed theta
    double Pphi[sys_n + 1];                  // P * phi
    double phiTPphi;                         // Scalar: phiT * P * phi
    double K[sys_n + 1];                     // Kalman gain vector
    double thetaUpd[sys_n + 1];              // Updated theta
    double KphiT[(sys_n + 1) * (sys_n + 1)]; // K * phiT
    double PUpd[(sys_n + 1) * (sys_n + 1)];  // Updated P matrix
    double thetaTphi;                        // Scalar: thetaT * phi
    double KP[sys_n + 1];

    // Transpose phi and theta
    matTranspose(rls->phi, phiT, 1, n);
    matTranspose(rls->theta, thetaT, 1, n);

    // P * phi
    matMul(rls->P, rls->phi, Pphi, n, n, 1);

    // phiT * P * phi (result is a scalar)
    matMul(phiT, rls->P, phiTP, 1, n, n);
    matMul(phiTP, rls->phi, &phiTPphi, 1, n, 1);

    phiTPphi += rls->forgetting_factor; // Add lambda to the denominator

    // Kalman gain: K = P * phi / (phiT * P * phi + lambda)
    for (int i = 0; i < n; i++)
    {
        K[i] = Pphi[i] / phiTPphi;
    }

    // Compute error: e = y - thetaT * phi
    matMul(thetaT, rls->phi, &thetaTphi, 1, n, 1);
    double e = y - thetaTphi;
    double y_est = thetaTphi;
    // Update theta: theta = theta + K * e
    for (int i = 0; i < n; i++)
    {
        thetaUpd[i] = rls->theta[i] + K[i] * e;
    }

    // Update P: P = (P - K * phiT * P) / lambda
    matMul(K, phiT, KphiT, n, 1, n);    // K * phiT
    matMul(KphiT, rls->P, KP, n, n, n); // (K * phiT) * P
    matSub(rls->P, KP, PUpd, n, n);     // P - KP
    for (int i = 0; i < n * n; i++)
    {
        PUpd[i] /= rls->forgetting_factor; // P = P / lambda
    }

    // Update the variables
    for (int i = 0; i < n; i++)
    {
        rls->theta[i] = thetaUpd[i];
    }
    for (int i = 0; i < n * n; i++)
    {
        rls->P[i] = PUpd[i];
    }

    return y_est; // Returning the error (optional, depending on use case)
}