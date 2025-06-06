#include "obb.h"
#include <math.h>
#include <cfloat>

#define dDOTpq(a, b, p, q) ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])
static double dDOT(const double *a, const double *b) { return dDOTpq(a, b, 1, 1); }
static double dDOT31(const double *a, const double *b) { return dDOTpq(a, b, 3, 1); }

#define dMULTIPLYOP1_331(A, op, B, C)   \
    {                                   \
        (A)[0] op dDOT31((B), (C));     \
        (A)[1] op dDOT31((B + 1), (C)); \
        (A)[2] op dDOT31((B + 2), (C)); \
    }

#define dMULTIPLYOP0_331(A, op, B, C) \
    {                                 \
        (A)[0] op dDOT((B), (C));     \
        (A)[1] op dDOT((B + 3), (C)); \
        (A)[2] op dDOT((B + 6), (C)); \
    }

#define dMULTIPLY1_331(A, B, C) dMULTIPLYOP1_331(A, =, B, C)
#define dMULTIPLY0_331(A, B, C) dMULTIPLYOP0_331(A, =, B, C)

namespace Engine
{
    int obb_Intersection(const obb_box &box1, const obb_box &box2, Vector3d &normal, double *depth)
    {
        const double fudge_factor = 1.05;
        Vector3d p, pp, normalC(0.f, 0.f, 0.f);
        const double *normalR = 0;
        double A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33,
            Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
        int i, j, invert_normal, code;

        A[0] = box1.e[0] * 0.5;
        A[1] = box1.e[1] * 0.5;
        A[2] = box1.e[2] * 0.5;

        B[0] = box2.e[0] * 0.5;
        B[1] = box2.e[1] * 0.5;
        B[2] = box2.e[2] * 0.5;
        _R rp = box1.pose ^ box2.pose;
        double *R1 = box1.pose.data;
        double *R2 = box2.pose.data;

        p = box2.center - box1.center;
        double *p_data = p.data;
        dMULTIPLY1_331(pp.data, R1, p_data); // get pp = p relative to body 1

        R11 = rp[0];
        R12 = rp[1];
        R13 = rp[2];
        R21 = rp[3];
        R22 = rp[4];
        R23 = rp[5];
        R31 = rp[6];
        R32 = rp[7];
        R33 = rp[8];

        Q11 = fabs(R11);
        Q12 = fabs(R12);
        Q13 = fabs(R13);
        Q21 = fabs(R21);
        Q22 = fabs(R22);
        Q23 = fabs(R23);
        Q31 = fabs(R31);
        Q32 = fabs(R32);
        Q33 = fabs(R33);

#define TST(expr1, expr2, norm, cc)    \
    s2 = fabs(expr1) - (expr2);        \
    if (s2 > 0)                        \
        return 0;                      \
    if (s2 > s)                        \
    {                                  \
        s = s2;                        \
        normalR = norm;                \
        invert_normal = ((expr1) < 0); \
        code = (cc);                   \
    }
        s = -DBL_MAX;
        invert_normal = 0;
        code = 0;

        TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1);
        TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2);
        TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3);

        // separating axis = v1,v2,v3
        TST(dDOT31(R2 + 0, p_data), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2 + 0, 4);
        TST(dDOT31(R2 + 1, p_data), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2 + 1, 5);
        TST(dDOT31(R2 + 2, p_data), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2 + 2, 6);

#undef TST
#define TST(expr1, expr2, n1, n2, n3, cc)               \
    s2 = fabs(expr1) - (expr2);                         \
    if (s2 > DBL_EPSILON)                               \
        return 0;                                       \
    l = sqrtf((n1) * (n1) + (n2) * (n2) + (n3) * (n3)); \
    if (l > DBL_EPSILON)                                \
    {                                                   \
        s2 /= l;                                        \
        if (s2 * fudge_factor > s)                      \
        {                                               \
            s = s2;                                     \
            normalR = 0;                                \
            normalC[0] = (n1) / l;                      \
            normalC[1] = (n2) / l;                      \
            normalC[2] = (n3) / l;                      \
            invert_normal = ((expr1) < 0);              \
            code = (cc);                                \
        }                                               \
    }
        double fudge2(1.0e-5f);

        Q11 += fudge2;
        Q12 += fudge2;
        Q13 += fudge2;

        Q21 += fudge2;
        Q22 += fudge2;
        Q23 += fudge2;

        Q31 += fudge2;
        Q32 += fudge2;
        Q33 += fudge2;

        // separating axis = u1 x (v1,v2,v3)
        TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, 7);
        TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, 8);
        TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, 9);

        // separating axis = u2 x (v1,v2,v3)
        TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, 10);
        TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, 11);
        TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, 12);

        // separating axis = u3 x (v1,v2,v3)
        TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, 13);
        TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, 14);
        TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, 15);

#undef TST
        if (!code)
            return 0;

        // if we get to this point, the boxes interpenetrate. compute the normal
        // in global coordinates.
        if (normalR)
        {
            normal[0] = normalR[0];
            normal[1] = normalR[4];
            normal[2] = normalR[8];
        }
        else
        {
            dMULTIPLY0_331(normal.data, R1, normalC.data);
        }
        if (invert_normal)
        {
            normal[0] = -normal[0];
            normal[1] = -normal[1];
            normal[2] = -normal[2];
        }
        *depth = -s;
    }
}