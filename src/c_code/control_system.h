#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H
#define NOISE_VALUE 0.0
#define dt 0.1
#define sys_n 1

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <float.h>

typedef struct
{
    // RLS Identification
    int n;
    double P[4];
    double theta[2];
    double phi[2];
    double y_est;
    double forgetting_factor;
    double a1;
    double b1;
    double tau;
    double K;
    double rls_time_scale;
    double prev_y;
    double prev_u;
} RLSState;

typedef struct
{
    // Delay Identification
    double M2;
    double standard_deviation;
    double mean;
    double noise_scale;
    int stddev_samples;
    int finished_noise_estimation;
    int is_delay_identified;
    double identified_delay;
    int delay_samples;
    double u_buffer[5000];
} DelayState;

typedef struct
{
    // Controller
    double error;
    double error_prev;
    double kp, ki, kd;
    double on_value;
    double u_temp_continuo;
    double integral;
    int pwm_counter;
    int on_steps;
    int pwm_cycle_length;
    int start_control;
    int digital;
} ControllerState;

typedef struct
{
    double K_dis;
    double lambda;
    double filter_factor;
    double u_temp;
    double u_obs_temp;
    double y_current;
    double y_obs_temp;
    double y_prev;
    double y_ref;
    int counter;
    DelayState delay_info;
    RLSState rls;
    ControllerState ctrl;
} ControlSystemState;

// Error calculation functions
double erro_weg(double *u, double *y, double *y_est, int len);
double integral_modulo_erro(double *y, double *y_est, int len);
double integral_erro_quadratico(double *y, double *y_est, int len);
double erro_medio_quadratico(double *y, double *y_est, int len);
double integral_time_absolute_error(double *y, double *y_est, int len);

// Matrix and vector operations
void mat_vec_mult(double *A, double *x, double *result, int rows, int cols);
void vec_add(double *x, double *y, double *result, int length);
void vec_scalar_mult(double *x, double scalar, double *result, int length);
void matMul(double *A, double *B, double *C, int m, int n, int p);
void matSub(double *A, double *B, double *C, int m, int n);
void matTranspose(double *A, double *B, int m, int n);

// Utility functions
double clamp(double value, double min_val, double max_val);
double calculate_pwm(ControlSystemState *controller, double u_temp);
void extract_final_parameters(ControlSystemState *controller, double *errors, double *values);
void standard_deviation(ControlSystemState *controller, double y);

// Control system simulation functions
void controller_state(ControlSystemState *controller, double y);
void update_rls_parameters(ControlSystemState *controller, double y);
void update_observer(ControlSystemState *controller);
void update_buffer(ControlSystemState *controller, double new_value);
void pad_arrays(double *num, int num_len, double *den, int den_len, double **num_padded, double **den_padded, int *max_len);
double pid_incremental(ControlSystemState *controller, double current_value, double reference);
double pid_calculation(ControlSystemState *controller, double current_value, double reference);
double calculate_u(ControlSystemState *controller, double y_current, double set_point);
void lambda_tuning(ControlSystemState *controller);
double simulate_step_alpha_beta(RLSState *rls, double u_temp, double y_prev);
double RLS(RLSState *rls, double y);

#endif