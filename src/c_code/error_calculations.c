#include "control_system.h"

double erro_weg(double *u, double *y, double *y_est, int len)
{
    double mse_y = 0;
    double mse_u = 0;
    double mse_overshoot = 0;
    double iae = 0;
    int is_overshoot = 0;
    for (int i = 0; i < len; i++)
    {
        // mse_y += pow(y[i] - y_est[i], 2);
        mse_u += pow(u[i] - u[i - 1], 2);
        iae += fabs(y[i] - y_est[i]);
        // if (y_est[i] > y[i]*1.01) {
        //     is_overshoot = 1;
        //     // break;
        // }
    }
    // mse_y /= len;
    mse_u /= len;
    // mse_overshoot /= len;
    iae /= len;
    // if (y_est[len-1] > y[len-1]*1.01 || y_est[len-1] < y[len-1]*0.99 || is_overshoot) {
    //     mse_overshoot += 100;
    // }
    return 3 * iae + mse_u;
}

double integral_modulo_erro(double *y, double *y_est, int len)
{
    double iae = 0;
    for (int i = 0; i < len; i++)
    {
        iae += fabs(y[i] - y_est[i]);
    }
    return iae / len;
}

double integral_erro_quadratico(double *y, double *y_est, int len)
{
    double ise = 0;
    for (int i = 0; i < len; i++)
    {
        ise += pow(y[i] - y_est[i], 2);
    }
    return ise;
}

double erro_medio_quadratico(double *y, double *y_est, int len)
{
    double mse = 0;
    for (int i = 0; i < len; i++)
    {
        mse += pow(y[i] - y_est[i], 2);
    }
    return mse / len;
}

double integral_time_absolute_error(double *y, double *y_est, int len)
{
    double itae = 0;
    for (int i = 0; i < len; i++)
    {
        itae += (i * dt) * fabs(y[i] - y_est[i]);
    }
    return itae / len;
}
