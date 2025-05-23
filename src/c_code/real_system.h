#ifndef REAL_SYSTEM_H
#define REAL_SYSTEM_H

typedef struct
{
    int n, delay_steps;
    double time_constant, gain, delay;
    double *A, *B, *C, D;
    double *x, *u_buffer;
    double *k1, *k2, *k3, *k4;
    double temp1[1];
} RealSystem;

void pad_arrays(double *num, int num_len, double *den, int den_len, double **num_padded, double **den_padded, int *max_len);
void tf2ss(double *num, double *den, int coefs_len, RealSystem *system);
void normalize(double *num, double *den, int coefs_len);
double randn(double mu, double sigma);
double update_system(RealSystem *system, int method, double u_temp);
double simulate_real_step(RealSystem *real_system, int method);
void initialize_real_system(double *num, int num_len, double *den, int den_len, double delay, RealSystem *real_system);
void free_real_system(RealSystem *system);
void update_real_buffer(double *u_buffer, int delay_steps, double new_value);

#endif