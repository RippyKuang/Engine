#include <solver.h>
#include <cmath>

namespace Engine
{

    void LCPsolver::solve(double *A, double *b, double *x, int n)
    {
        double temp[n];
        for (int iter = 0; iter < MAX_ITER; iter++)
        {
            for (int i = 0; i < n; i++)
            {
                temp[i] = -b[i];
                for (int j = i + 1; j < n; j++)
                    temp[i] -= A[i * n + j] * x[j];
            }
            Solver::LSolve(A, temp, x, n);
            for (int i = 0; i < n; i++)
                x[i] = std::max(0.0, x[i]);
        }
    }

    void Solver::choleskySolve(double *A, double *b, double *x, int n)
    {
        double L[n][n];
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < j; ++k)
                    sum += L[i][k] * L[j][k];

                if (i == j)
                {
                    double diag = A[i * n + i] - sum;
                    L[i][j] = std::sqrt(diag);
                }
                else
                {
                    L[i][j] = (A[i * n + j] - sum) / L[j][j];
                }
            }
        }

        double y[n];
        for (int i = 0; i < n; ++i)
        {
            double sum = 0.0;
            for (int k = 0; k < i; ++k)
                sum += L[i][k] * y[k];
            y[i] = (b[i] - sum) / L[i][i];
        }

        for (int i = n - 1; i >= 0; --i)
        {
            double sum = 0.0;
            for (int k = i + 1; k < n; ++k)
                sum += L[k][i] * x[k];
            x[i] = (y[i] - sum) / L[i][i];
        }
    }

    void Solver::LSolve(double *L, double *b, double *x, int n)
    {
        for (int i = 0; i < n; ++i)
        {
            double sum = 0.0;
            for (int k = 0; k < i; ++k)
                sum += L[i * n + k] * x[k];
            x[i] = (b[i] - sum) / L[i * n + i];
        }
    }
}