#include "control_system.h"
#include "real_system.h"

// #include "control_simulation.c"
// #include "matrix_and_vector_ops.c"
// #include "error_calculations.c"
// #include "utilities.c"
// #include "real_system.c"

double control(ControlSystemState *controller, double set_point, double y_current)
{
    /* ------------ 0. Filtering ------------ */
    double y_value = controller->y_prev * (1 - controller->filter_factor) + y_current * controller->filter_factor;
    controller->y_prev = y_value;

    /* ------------ 1. Delay and Noise Identification ------------ */
    standard_deviation(controller, y_value);

    controller_state(controller, y_value);
    /* ------------ 2. RLS Parameter Estimation ------------ */
    update_rls_parameters(controller, y_value);
    /* ------------ 3. Controller Tuning ------------ */
    lambda_tuning(controller);
    /* ------------ 4. Observer Update ------------ */
    update_observer(controller);
    /* ------------ 5. Control Signal Calculation ------------ */
    double u = calculate_u(controller, y_current, set_point);
    u = calculate_pwm(controller, u);
    controller->u_temp = u;

    update_buffer(controller, u);

    return u;
}

void step_response(double *num, int num_len, double *den, int den_len, double stop_time, double delay, double *u, double *y, int method)
{
    int samples = (int)(stop_time / dt) + 1;
    RealSystem system;

    initialize_real_system(num, num_len, den, den_len, delay, &system);

    for (int i = 0; i < samples - 1; ++i)
    {   
        u[i] = 1.0; // Step input
        y[i] = update_system(&system, method, 1.0);
    }

    free_real_system(&system);
}

void pid_simulation(double *num, int num_len, double *den, int den_len, double stop_time, double delay, double *u, double *y, double kp, double ki, double kd, int method, double set_point, int pwm_cycle_length, int digital)
{
    ControlSystemState controller = {
        .rls = {
            .P = {1000, 0, 0, 1000},
            .theta = {0, 0},
            .phi = {0, 0},
            .n = sys_n,
            .prev_u = 0,
            .prev_y = 0,
        },
        .delay_info = {
            .M2 = 0,
            .standard_deviation = 0,
            .mean = 0,
            .is_delay_identified = 0,
            .finished_noise_estimation = 0,
            .u_buffer = {0},
        },
        .ctrl = {
            .kp = kp,
            .ki = ki,
            .kd = kd,
            .digital = digital,
            .pwm_cycle_length = pwm_cycle_length,
            .on_value = 100.0,
            .start_control = 1,
            .integral = 0.0,
        },
        .u_obs_temp = 0.0,
        .y_current = 0.0,
        .y_obs_temp = 0.0,
        .counter = 0,
        .y_ref = 0.0,
        .y_prev = 0.0};

    int samples = (int)(stop_time / dt) + 1;
    RealSystem system;

    initialize_real_system(num, num_len, den, den_len, delay, &system);

    double integral = 0.0, error = 0.0, u_temp = 0.0, error_prev = 0.0;
    // PWM control variables
    int pwm_counter = 0;
    int on_steps = 0;
    double on_value = 100.0;

    for (int i = 0; i < samples - 1; ++i)
    {
        y[i] = update_system(&system, method, u_temp);
        u_temp = pid_calculation(&controller, y[i], set_point);
        u[i] = u_temp;
        controller.u_temp = u_temp;
    }

    free_real_system(&system);
}

/* -------------------- Main Function -------------------- */
void observer_controller_with_identification_rls(double *num, int num_len, double *den, int den_len,
                                                 double stop_time, double delay, double *u,
                                                 double *y, double *u_obs, double *y_obs, int method,
                                                 double set_point, int pwm_cycle_length, int digital,
                                                 double *extra_params)
{
    ControlSystemState controller = {
        .K_dis = extra_params[3],
        .lambda = extra_params[5],
        .filter_factor = extra_params[7],
        .rls = {
            .P = {1000, 0, 0, 1000},
            .forgetting_factor = extra_params[4],
            .theta = {0, 0},
            .phi = {0, 0},
            .n = sys_n,
            .rls_time_scale = extra_params[2],
            .prev_u = 0,
            .prev_y = 0,
        },
        .delay_info = {
            .M2 = 0,
            .standard_deviation = 0,
            .mean = 0,
            .noise_scale = extra_params[1],
            .stddev_samples = (int)(extra_params[0] / dt),
            .is_delay_identified = 0,
            .finished_noise_estimation = 0,
            .u_buffer = {0},
        },
        .ctrl = {
            .kp = 1.0,
            .ki = 0.0,
            .kd = 0.0,
            .digital = digital,
            .pwm_cycle_length = pwm_cycle_length,
            .on_value = 100.0,
            .integral = 0.0,
        },
        .u_temp = extra_params[6] * set_point,
        .u_obs_temp = 0.0,
        .y_current = 0.0,
        .y_obs_temp = 0.0,
        .counter = 0,
        .y_ref = 0.0,
        .y_prev = 0.0};

    RealSystem system;
    initialize_real_system(num, num_len, den, den_len, delay, &system);
    int samples = (int)(stop_time / dt) + 1;
    double u_value = 0.0;
    double y_value = 0.0;

    // Main control loop
    for (int i = 1; i < samples - 1; i++)
    {
        controller.counter++;

        /* ------------ 0. System Update ------------ */
        y_value = update_system(&system, method, u_value);
        u_value = control(&controller, set_point, y_value);

        // Store values
        u[i] = u_value;
        y[i] = y_value;
        u_obs[i] = controller.u_obs_temp;
        y_obs[i] = controller.y_obs_temp;
    }

    // Final cleanup and parameter extraction
    printf("\n%f\n", controller.rls.K);
    printf("%f\n", controller.rls.tau);
    printf("%f\n", controller.delay_info.identified_delay);
    // extract_final_parameters(&controller, errors, values);
    free_real_system(&system);
}

// Função para rodar todas as simulações e coletar os valores de y
void run_all_simulations(
    double *num, int num_len, double *den, int den_len,
    double stop_time, double delay, int method, double set_point,
    int pwm_cycle_length, int digital, double *extra_params,
    double *u_step, double *y_step,
    double *u_pid, double *y_pid,
    double *u_smith, double *y_smith,
    double *u_obs, double *y_obs
) {
    // 1. Step Response
    step_response(num, num_len, den, den_len, stop_time, delay, u_step, y_step, method);

    // 2. PID Simulation (exemplo de ganhos, ajuste conforme necessário)
    // double kp = num[0], ki = den[0], kd = 0.0;
    double kp = (den[0]) / (num[0] * 0.2 * (den[0] + delay));

    double ki = kp / den[0];

    double kd = 0.0;
    pid_simulation(num, num_len, den, den_len, stop_time, delay, u_pid, y_pid, kp, ki, kd, method, set_point, pwm_cycle_length, digital);

    // 3. Observer Controller with Identification RLS
    observer_controller_with_identification_rls(num, num_len, den, den_len, stop_time, delay, u_smith, y_smith, u_obs, y_obs, method, set_point, pwm_cycle_length, digital, extra_params);

    // Result arrays are filled in-place, no allocation or freeing here.
}

// int main() {
//     double num[] = {1.0};
//     double den[] = {10.0, 1};
//     double delay = 1.0;
//     double stop_time = 7000.0;
//     int samples = (int)(stop_time / dt) + 1;
//     double *u = (double*)calloc(samples, sizeof(double));
//     double *y = (double*)calloc(samples, sizeof(double));
//     double *u_obs = (double*)calloc(samples, sizeof(double));
//     double *u_continuo = (double*)calloc(samples, sizeof(double));
//     double *y_obs = (double*)calloc(samples, sizeof(double));
//     double *t = (double*)calloc(samples, sizeof(double));
//     double *reference = (double*)calloc(samples, sizeof(double));
//     double erros[5];
//     double values[8];
//     double params[8] = {20.0, 3.0, 2.0, 5.0, 1.0, 2.0, 0.02, 0.5};
//     for (int i = 0; i < samples; i++) {
//         t[i] = i * dt;
//         reference[i] = 1.0;
//     }
//     double kp = 288.815363;
//     double ki = 0.003462;
//     double kd = 0.0;
//     observer_controller_with_identification_rls(num, 1, den, 2, stop_time, delay, u, y, u_obs, y_obs, 1, 50.0, 100, 0, params, erros, values);
//     printf("Done\n");
//     return 0;
// }