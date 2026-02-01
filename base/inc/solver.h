#include <matrix.h>
#define MAX_ITER 10
namespace Engine
{
    class LCPsolver
    {
    public:
        static void solve(double *A, double *b, double *x, int n); // assume x = 0
    };

    class Solver
    {
    public:
        static void choleskySolve(double *A, double *b, double *x, int n);
        static void LSolve(double *A, double *b, double *x, int n); // assume x = 0
    };
}