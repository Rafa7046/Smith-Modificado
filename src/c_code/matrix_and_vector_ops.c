#include "control_system.h"

void mat_vec_mult(double *A, double *x, double *result, int rows, int cols)
{
    for (int i = 0; i < rows; i++)
    {
        result[i] = 0;
        for (int j = 0; j < cols; j++)
        {
            result[i] += A[i * cols + j] * x[j];
        }
    }
}

void vec_add(double *x, double *y, double *result, int length)
{
    for (int i = 0; i < length; i++)
    {
        result[i] = x[i] + y[i];
    }
}

void vec_scalar_mult(double *x, double scalar, double *result, int length)
{
    for (int i = 0; i < length; i++)
    {
        result[i] = x[i] * scalar;
    }
}

void matMul(double *A, double *B, double *C, int m, int n, int p)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < p; j++)
        {
            C[i * p + j] = 0;
            for (int k = 0; k < n; k++)
            {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

void matSub(double *A, double *B, double *C, int m, int n)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            C[i * n + j] = A[i * n + j] - B[i * n + j];
        }
    }
}

void matTranspose(double *A, double *B, int m, int n)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            B[j * m + i] = A[i * n + j];
        }
    }
}