#include "obb.h"
#include <math.h>
#include <cfloat>

#define dDOTpq(a, b, p, q) ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])
static double dDOT(const double *a, const double *b) { return dDOTpq(a, b, 1, 1); }
static double dDOT31(const double *a, const double *b) { return dDOTpq(a, b, 3, 1); }
static double dDOT13(const double *a, const double *b) { return dDOTpq(a, b, 1, 3); }
static double dDOT33(const double *a, const double *b) { return dDOTpq(a, b, 3, 3); }

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
    static int intersectRectQuad2(double h[2], double p[8], double ret[16])
    {
        // q (and r) contain nq (and nr) coordinate points for the current (and
        // chopped) polygons
        int nq = 4, nr = 0;
        double buffer[16];
        double *q = p;
        double *r = ret;
        for (int dir = 0; dir <= 1; dir++)
        {
            // direction notation: xy[0] = x axis, xy[1] = y axis
            for (int sign = -1; sign <= 1; sign += 2)
            {
                // chop q along the line xy[dir] = sign*h[dir]
                double *pq = q;
                double *pr = r;
                nr = 0;
                for (int i = nq; i > 0; i--)
                {
                    // go through all points in q and all lines between adjacent points
                    if (sign * pq[dir] < h[dir])
                    {
                        // this point is inside the chopping line
                        pr[0] = pq[0];
                        pr[1] = pq[1];
                        pr += 2;
                        nr++;
                        if (nr & 8)
                        {
                            q = r;
                            goto done;
                        }
                    }
                    double *nextq = (i > 1) ? pq + 2 : q;
                    if ((sign * pq[dir] < h[dir]) ^ (sign * nextq[dir] < h[dir]))
                    {
                        // this line crosses the chopping line
                        pr[1 - dir] = pq[1 - dir] + (nextq[1 - dir] - pq[1 - dir]) /
                                                        (nextq[dir] - pq[dir]) * (sign * h[dir] - pq[dir]);
                        pr[dir] = sign * h[dir];
                        pr += 2;
                        nr++;
                        if (nr & 8)
                        {
                            q = r;
                            goto done;
                        }
                    }
                    pq += 2;
                }
                q = r;
                r = (q == ret) ? buffer : ret;
                nq = nr;
            }
        }
    done:
        if (q != ret)
            memcpy(ret, q, nr * 2 * sizeof(double));
        return nr;
    }

    void dLineClosestApproach(const Vector3d &pa, const Vector3d &ua,
                              const Vector3d &pb, const Vector3d &ub,
                              double *alpha, double *beta);
    void dLineClosestApproach(const Vector3d &pa, const Vector3d &ua,
                              const Vector3d &pb, const Vector3d &ub,
                              double *alpha, double *beta)
    {
        Vector3d p;
        p[0] = pb[0] - pa[0];
        p[1] = pb[1] - pa[1];
        p[2] = pb[2] - pa[2];
        double uaub = dDOT(ua.data, ub.data);
        double q1 = dDOT(ua.data, p.data);
        double q2 = -dDOT(ub.data, p.data);
        double d = 1 - uaub * uaub;
        if (d <= double(0.0001f))
        {
            // @@@ this needs to be made more robust
            *alpha = 0;
            *beta = 0;
        }
        else
        {
            d = 1.f / d;
            *alpha = (q1 + uaub * q2) * d;
            *beta = (uaub * q1 + q2) * d;
        }
    }

#define M__PI 3.14159265f

    // given n points in the plane (array p, of size 2*n), generate m points that
    // best represent the whole set. the definition of 'best' here is not
    // predetermined - the idea is to select points that give good box-box
    // collision detection behavior. the chosen point indexes are returned in the
    // array iret (of size m). 'i0' is always the first entry in the array.
    // n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
    // in the range [0..n-1].

    void cullPoints2(int n, double p[], int m, int i0, int iret[]);
    void cullPoints2(int n, double p[], int m, int i0, int iret[])
    {
        // compute the centroid of the polygon in cx,cy
        int i, j;
        double a, cx, cy, q;
        if (n == 1)
        {
            cx = p[0];
            cy = p[1];
        }
        else if (n == 2)
        {
            cx = 0.5 * (p[0] + p[2]);
            cy = 0.5 * (p[1] + p[3]);
        }
        else
        {
            a = 0;
            cx = 0;
            cy = 0;
            for (i = 0; i < (n - 1); i++)
            {
                q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
                a += q;
                cx += q * (p[i * 2] + p[i * 2 + 2]);
                cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
            }
            q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
            if (fabs(a + q) > DBL_EPSILON)
            {
                a = 1.0 / 3.0 * (a + q);
            }
            else
            {
                a = 1e30;
            }
            cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
            cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
        }

        // compute the angle of each point w.r.t. the centroid
        double A[8];
        for (i = 0; i < n; i++)
            A[i] = atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

        // search for points that have angles closest to A[i0] + i*(2*pi/m).
        int avail[8];
        for (i = 0; i < n; i++)
            avail[i] = 1;
        avail[i0] = 0;
        iret[0] = i0;
        iret++;
        for (j = 1; j < m; j++)
        {
            a = j * (2 * M__PI / m) + A[i0];
            if (a > M__PI)
                a -= 2 * M__PI;
            double maxdiff = 1e9, diff;

            *iret = i0; // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

            for (i = 0; i < n; i++)
            {
                if (avail[i])
                {
                    diff = fabs(A[i] - a);
                    if (diff > M__PI)
                        diff = 2 * M__PI - diff;
                    if (diff < maxdiff)
                    {
                        maxdiff = diff;
                        *iret = i;
                    }
                }
            }
            avail[*iret] = 0;
            iret++;
        }
    }

    int obb_Intersection(const obb_box &box1, const obb_box &box2, int maxc,std::vector<contact_results> &output, int &return_code)
    {
        const double fudge_factor = 1.05;
        Vector3d p, pp, normalC(0.f, 0.f, 0.f),normal;
        const double *normalR = 0;
        double A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33,
            Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
        int i, j, invert_normal, code;

        double depth[1];
        A[0] = box1.e[0] * 0.5;
        A[1] = box1.e[1] * 0.5;
        A[2] = box1.e[2] * 0.5;

        B[0] = box2.e[0] * 0.5;
        B[1] = box2.e[1] * 0.5;
        B[2] = box2.e[2] * 0.5;
        _R rp = box1.pose ^ box2.pose;
        double *R1 = box1.pose.data;
        double *R2 = box2.pose.data;
        Vector3d p1 = box1.center;
        Vector3d p2 = box2.center;
        p = p2 - p1;
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
            normal[1] = normalR[3];
            normal[2] = normalR[6];
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

        // compute contact point(s)

        if (code > 6)
        {
            // an edge from box 1 touches an edge from box 2.
            // find a point pa on the intersecting edge of box 1
            Vector3d pa;
            double sign;
            for (i = 0; i < 3; i++)
                pa[i] = p1[i];
            for (j = 0; j < 3; j++)
            {
                sign = (dDOT13(normal.data, R1 + j) > 0) ? double(1.0) : double(-1.0);
                for (i = 0; i < 3; i++)
                    pa[i] += sign * A[j] * R1[i * 3 + j];
            }

            // find a point pb on the intersecting edge of box 2
            Vector3d pb;
            for (i = 0; i < 3; i++)
                pb[i] = p2[i];
            for (j = 0; j < 3; j++)
            {
                sign = (dDOT13(normal.data, R2 + j) > 0) ? double(-1.0) : double(1.0);
                for (i = 0; i < 3; i++)
                    pb[i] += sign * B[j] * R2[i * 3 + j];
            }

            double alpha, beta;
            Vector3d ua, ub;
            for (i = 0; i < 3; i++)
                ua[i] = R1[((code)-7) / 3 + i * 3];
            for (i = 0; i < 3; i++)
                ub[i] = R2[((code)-7) % 3 + i * 3];

            dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
            for (i = 0; i < 3; i++)
                pa[i] += ua[i] * alpha;
            for (i = 0; i < 3; i++)
                pb[i] += ub[i] * beta;

            output.emplace_back(normal * (-1), pb,-*depth);
            return_code = code;

            return 1;
        }

        // okay, we have a face-something intersection (because the separating
        // axis is perpendicular to a face). define face 'a' to be the reference
        // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
        // the incident face (the closest face of the other box).

        const double *Ra, *Rb, *pa, *pb, *Sa, *Sb;
        if (code <= 3)
        {
            Ra = R1;
            Rb = R2;
            pa = p1.data;
            pb = p2.data;
            Sa = A;
            Sb = B;
        }
        else
        {
            Ra = R2;
            Rb = R1;
            pa = p2.data;
            pb = p1.data;
            Sa = B;
            Sb = A;
        }

        // nr = normal vector of reference face dotted with axes of incident box.
        // anr = absolute values of nr.
        Vector3d normal2, nr, anr;
        if (code <= 3)
        {
            normal2[0] = normal[0];
            normal2[1] = normal[1];
            normal2[2] = normal[2];
        }
        else
        {
            normal2[0] = -normal[0];
            normal2[1] = -normal[1];
            normal2[2] = -normal[2];
        }
        dMULTIPLY1_331(nr.data, Rb, normal2.data);
        anr[0] = fabs(nr[0]);
        anr[1] = fabs(nr[1]);
        anr[2] = fabs(nr[2]);

        // find the largest compontent of anr: this corresponds to the normal
        // for the indident face. the other axis numbers of the indicent face
        // are stored in a1,a2.
        int lanr, a1, a2;
        if (anr[1] > anr[0])
        {
            if (anr[1] > anr[2])
            {
                a1 = 0;
                lanr = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }
        else
        {
            if (anr[0] > anr[2])
            {
                lanr = 0;
                a1 = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }

        // compute center point of incident face, in reference-face coordinates
        Vector3d center;
        if (nr[lanr] < 0)
        {
            for (i = 0; i < 3; i++)
                center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 3 + lanr];
        }
        else
        {
            for (i = 0; i < 3; i++)
                center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 3 + lanr];
        }

        // find the normal and non-normal axis numbers of the reference box
        int codeN, code1, code2;
        if (code <= 3)
            codeN = code - 1;
        else
            codeN = code - 4;
        if (codeN == 0)
        {
            code1 = 1;
            code2 = 2;
        }
        else if (codeN == 1)
        {
            code1 = 0;
            code2 = 2;
        }
        else
        {
            code1 = 0;
            code2 = 1;
        }

        // find the four corners of the incident face, in reference-face coordinates
        double quad[8]; // 2D coordinate of incident face (x,y pairs)
        double c1, c2, m11, m12, m21, m22;
        c1 = dDOT13(center.data, Ra + code1);
        c2 = dDOT13(center.data, Ra + code2);
        // optimize this? - we have already computed this data above, but it is not
        // stored in an easy-to-index format. for now it's quicker just to recompute
        // the four dot products.
        m11 = dDOT33(Ra + code1, Rb + a1);
        m12 = dDOT33(Ra + code1, Rb + a2);
        m21 = dDOT33(Ra + code2, Rb + a1);
        m22 = dDOT33(Ra + code2, Rb + a2);
        {
            double k1 = m11 * Sb[a1];
            double k2 = m21 * Sb[a1];
            double k3 = m12 * Sb[a2];
            double k4 = m22 * Sb[a2];
            quad[0] = c1 - k1 - k3;
            quad[1] = c2 - k2 - k4;
            quad[2] = c1 - k1 + k3;
            quad[3] = c2 - k2 + k4;
            quad[4] = c1 + k1 + k3;
            quad[5] = c2 + k2 + k4;
            quad[6] = c1 + k1 - k3;
            quad[7] = c2 + k2 - k4;
        }

        // find the size of the reference face
        double rect[2];
        rect[0] = Sa[code1];
        rect[1] = Sa[code2];

        // intersect the incident and reference faces
        double ret[16];
        int n = intersectRectQuad2(rect, quad, ret);
        if (n < 1)
            return 0; // this should never happen

        // convert the intersection points into reference-face coordinates,
        // and compute the contact position and depth for each point. only keep
        // those points that have a positive (penetrating) depth. delete points in
        // the 'ret' array as necessary so that 'point' and 'ret' correspond.
        double point[3 * 8]; // penetrating contact points
        double dep[8];       // depths for those points
        double det1 = 1.f / (m11 * m22 - m12 * m21);
        m11 *= det1;
        m12 *= det1;
        m21 *= det1;
        m22 *= det1;
        int cnum = 0; // number of penetrating contact points found
        for (j = 0; j < n; j++)
        {
            double k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
            double k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
            for (i = 0; i < 3; i++)
                point[cnum * 3 + i] =
                    center[i] + k1 * Rb[i * 3 + a1] + k2 * Rb[i * 3 + a2];
            dep[cnum] = Sa[codeN] - dDOT(normal2.data, point + cnum * 3);
            if (dep[cnum] >= 0)
            {
                ret[cnum * 2] = ret[j * 2];
                ret[cnum * 2 + 1] = ret[j * 2 + 1];
                cnum++;
            }
        }
        if (cnum < 1)
            return 0; // this should never happen

        // we can't generate more contacts than we actually have
        if (maxc > cnum)
            maxc = cnum;
        if (maxc < 1)
            maxc = 1;

        if (cnum <= maxc)
        {
            if (code < 4)
            {
                // we have less contacts than we need, so we use them all
                for (j = 0; j < cnum; j++)
                {
                    Vector3d pointInWorld;
                    for (i = 0; i < 3; i++)
                        pointInWorld[i] = point[j * 3 + i] + pa[i];
                    output.emplace_back(normal * (-1), pointInWorld, -dep[j]);
                }
            }
            else
            {
                // we have less contacts than we need, so we use them all
                for (j = 0; j < cnum; j++)
                {
                    Vector3d pointInWorld;
                    for (i = 0; i < 3; i++)
                        pointInWorld[i] = point[j * 3 + i] + pa[i] - normal[i] * dep[j];
                    output.emplace_back(normal * (-1), pointInWorld, -dep[j]);
                }
            }
        }
        else
        {
            // we have more contacts than are wanted, some of them must be culled.
            // find the deepest point, it is always the first contact.
            int i1 = 0;
            double maxdepth = dep[0];
            for (i = 1; i < cnum; i++)
            {
                if (dep[i] > maxdepth)
                {
                    maxdepth = dep[i];
                    i1 = i;
                }
            }

            int iret[8];
            cullPoints2(cnum, ret, maxc, i1, iret);

            for (j = 0; j < maxc; j++)
            {

                Vector3d posInWorld;
                for (i = 0; i < 3; i++)
                    posInWorld[i] = point[iret[j] * 3 + i] + pa[i];
                if (code < 4)
                {
                    output.emplace_back(normal * (-1), posInWorld, -dep[iret[j]]);
                }
                else
                {
                    output.emplace_back(normal * (-1), posInWorld - normal * dep[iret[j]], -dep[iret[j]]);
                }
            }
            cnum = maxc;
        }

        return_code = code;
        return cnum;
    }

}