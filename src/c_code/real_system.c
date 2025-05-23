#include "control_system.h"
#include "real_system.h"

void pad_arrays(double *num, int num_len, double *den, int den_len, double **num_padded, double **den_padded, int *max_len)
{
    *max_len = (num_len > den_len) ? num_len : den_len;
    *num_padded = (double *)malloc(*max_len * sizeof(double));
    *den_padded = (double *)malloc(*max_len * sizeof(double));

    memset(*num_padded, 0, *max_len * sizeof(double));
    memset(*den_padded, 0, *max_len * sizeof(double));
    memcpy(*num_padded + (*max_len - num_len), num, num_len * sizeof(double));
    memcpy(*den_padded + (*max_len - den_len), den, den_len * sizeof(double));
}

void normalize(double *num, double *den, int coefs_len)
{
    double norm = den[0];
    for (int i = 0; i < coefs_len; ++i)
    {
        num[i] /= norm;
    }
    for (int i = 0; i < coefs_len; ++i)
    {
        den[i] /= norm;
    }
}

void tf2ss(double *num, double *den, int coefs_len, RealSystem *system)
{
    normalize(num, den, coefs_len);

    int n = coefs_len - 1;

    memset(system->A, 0, n * n * sizeof(double));
    memset(system->B, 0, n * sizeof(double));
    memset(system->C, 0, n * sizeof(double));
    system->D = 0.0;

    for (int i = 1; i < n; ++i)
    {
        system->A[i * n + (i - 1)] = 1.0;
    }
    for (int i = 0; i < n; ++i)
    {
        system->A[i] = -den[i + 1];
    }
    system->B[0] = 1.0;
    for (int i = 0; i < n; ++i)
    {
        system->C[i] = num[i + 1] - num[0] * den[i + 1];
    }
    system->D = num[0];
}

double randn(double mu, double sigma)
{
    double U1 = (double)(((rand() % (RAND_MAX - 10)) + 10)) / RAND_MAX;
    double U2 = (double)(((rand() % (RAND_MAX - 10)) + 10)) / RAND_MAX;
    double Z = sqrt(-2.0 * log(U1)) * cos(2.0 * 3.141592 * U2);
    return mu + sigma * Z;
}

void update_real_buffer(double *buffer, int delay_steps, double new_value)
{
    memmove(buffer, buffer + 1, (delay_steps - 1) * sizeof(double));
    buffer[delay_steps - 1] = new_value;
}

double update_system(RealSystem *system, int method, double u_temp)
{
    update_real_buffer(system->u_buffer, system->delay_steps, u_temp);
    return simulate_real_step(system, method) + randn(0.0, NOISE_VALUE);
}

void initialize_real_system(double *num, int num_len, double *den, int den_len, double delay, RealSystem *real_system)
{
    real_system->n = den_len - 1;
    real_system->delay_steps = ceil(delay / dt) + 1;

    real_system->time_constant = den[0];
    real_system->gain = num[0];
    real_system->delay = delay;

    real_system->A = (double *)malloc((real_system->n) * (real_system->n) * sizeof(double));
    real_system->B = (double *)malloc((real_system->n) * sizeof(double));
    real_system->C = (double *)malloc((real_system->n) * sizeof(double));
    real_system->D = 0.0;

    real_system->u_buffer = (double *)calloc(real_system->delay_steps, sizeof(double));

    real_system->x = (double *)calloc(real_system->n, sizeof(double));

    double *num_padded, *den_padded;
    int max_len;
    pad_arrays(num, num_len, den, den_len, &num_padded, &den_padded, &max_len);
    tf2ss(num_padded, den_padded, max_len, real_system);

    real_system->k1 = (double *)malloc((real_system->n) * sizeof(double));
    real_system->k2 = (double *)malloc((real_system->n) * sizeof(double));
    real_system->k3 = (double *)malloc((real_system->n) * sizeof(double));
    real_system->k4 = (double *)malloc((real_system->n) * sizeof(double));

    free(num_padded);
    free(den_padded);
}

double simulate_real_step(RealSystem *real_system, int method)
{

    double *temp1 = (double *)malloc(real_system->n * sizeof(double));
    double *temp2 = (double *)malloc(real_system->n * sizeof(double));
    double *temp3 = (double *)malloc(real_system->n * sizeof(double));
    double temp_output;

    if (method == 0)
    {
        mat_vec_mult(real_system->A, real_system->x, temp1, real_system->n, real_system->n);
        mat_vec_mult(real_system->B, real_system->u_buffer, temp2, real_system->n, 1);
        vec_add(temp1, temp2, temp3, real_system->n);
        vec_scalar_mult(temp3, dt, temp3, real_system->n);
        vec_add(real_system->x, temp3, real_system->x, real_system->n);
    }
    else if (method == 1)
    {
        mat_vec_mult(real_system->A, real_system->x, temp1, real_system->n, real_system->n);
        mat_vec_mult(real_system->B, real_system->u_buffer, temp2, real_system->n, 1);
        vec_add(temp1, temp2, real_system->k1, real_system->n);
        vec_scalar_mult(real_system->k1, dt / 2.0, temp3, real_system->n);
        vec_add(real_system->x, temp3, temp3, real_system->n);

        mat_vec_mult(real_system->A, temp3, temp1, real_system->n, real_system->n);
        mat_vec_mult(real_system->B, real_system->u_buffer, temp2, real_system->n, 1);
        vec_add(temp1, temp2, real_system->k2, real_system->n);
        vec_scalar_mult(real_system->k2, dt / 2.0, temp3, real_system->n);
        vec_add(real_system->x, temp3, temp3, real_system->n);

        mat_vec_mult(real_system->A, temp3, temp1, real_system->n, real_system->n);
        mat_vec_mult(real_system->B, real_system->u_buffer, temp2, real_system->n, 1);
        vec_add(temp1, temp2, real_system->k3, real_system->n);
        vec_scalar_mult(real_system->k3, dt, temp3, real_system->n);
        vec_add(real_system->x, temp3, temp3, real_system->n);

        mat_vec_mult(real_system->A, temp3, temp1, real_system->n, real_system->n);
        mat_vec_mult(real_system->B, real_system->u_buffer, temp2, real_system->n, 1);
        vec_add(temp1, temp2, real_system->k4, real_system->n);

        for (int j = 0; j < real_system->n; j++)
        {
            real_system->x[j] += (dt / 6.0) * (real_system->k1[j] + 2 * real_system->k2[j] + 2 * real_system->k3[j] + real_system->k4[j]);
        }
    }
    mat_vec_mult(real_system->C, real_system->x, temp1, 1, real_system->n);
    temp_output = temp1[0] + real_system->D * real_system->u_buffer[0];

    free(temp1);
    free(temp2);
    free(temp3);
    temp_output = clamp(temp_output, 0.0, 100.0);
    return temp_output;
}

void free_real_system(RealSystem *system)
{
    free(system->A);
    free(system->B);
    free(system->C);
    free(system->x);
    free(system->u_buffer);
    free(system->k1);
    free(system->k2);
    free(system->k3);
    free(system->k4);
}