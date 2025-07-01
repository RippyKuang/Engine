#pragma once
#include <rmath.h>
#include <cmath>
#include <type_traits>
#include <mutex>
namespace Engine
{

    enum JType
    {
        DEFAULT,
        REVOLUTE,
        PRISMATIC
    };

    struct BaseJoint
    {
        JType type = DEFAULT;
        using space_type = void;
        virtual void jcalc(M66 &, Vector6d &) = 0;
        virtual JType get_type() = 0;
        virtual void step(double) = 0;
        virtual void force_set_speed(double) = 0;
    };

    class Revolute : public BaseJoint
    {
    private:
        Matrix<double, 6, 1> motion_subspace;
        Matrix<double, 6, 5> constraint_subspace;
        Vector3d axis;
        _R (*fE)(double);

        double q = 0;
        double q_dot = 1;
        double v_dot = 0;

    public:
        using space_type = Matrix<double, 6, 1>;
        JType get_type() override
        {
            return this->type;
        }
        void jcalc(M66 &X, Vector6d &vj) override
        {

            X = rot(this->fE(q));
            vj = this->motion_subspace * q_dot;
        }

        Matrix<double, 6, 1> get_motion_subspace()
        {
            return this->motion_subspace;
        }
        void force_set_speed(double s) override
        {
            this->q_dot = s;
        }

        void set_v_dot(double v_dot)
        {
            this->v_dot = v_dot;
        }
        double get_q_dot()
        {
            return this->q_dot;
        }

        void step(double dt) override
        {

            q_dot += v_dot * dt;
            q += q_dot * dt;
            q = fmod(q, 2 * M_PI);
        }

        Revolute(Vector3d &&axis) : axis(axis)
        {
            this->type = REVOLUTE;
            motion_subspace = Matrix<double, 6, 1>{axis[0], axis[1], axis[2], 0, 0, 0};
            if (axis[0] == 1)
            {
                this->fE = rx;
                constraint_subspace = Matrix<double, 6, 5>{0, 0, 0, 0, 0,
                                                           1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
            else if (axis[1] == 1)
            {
                this->fE = ry;
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
            else
            {
                this->fE = rz;
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
        }
    };

    class Prismatic : public BaseJoint
    {
    private:
        Matrix<double, 6, 1> motion_subspace;
        Matrix<double, 6, 5> constraint_subspace;
        Vector3d axis;

        double q = 0;
        double q_dot = 0;
        double v_dot = 0;

    public:
        using space_type = Matrix<double, 6, 1>;

        JType get_type() override
        {
            return this->type;
        }

        Matrix<double, 6, 1> get_motion_subspace()
        {
            return this->motion_subspace;
        }
        void force_set_speed(double s) override
        {
            this->q_dot = s;
        }

        void set_v_dot(double v_dot)
        {
            this->v_dot = v_dot;
        }

        double get_q_dot()
        {
            return this->q_dot;
        }

        void step(double dt) override
        {
            q_dot += v_dot * dt;
            q += q_dot * dt;
        }

        void jcalc(M66 &X, Vector6d &vj) override
        {

            X = xlt(this->axis * q);
            vj = this->motion_subspace * q_dot;
        }
        Prismatic(Vector3d &&axis) : axis(axis)
        {
            this->type = PRISMATIC;
            motion_subspace = Matrix<double, 6, 1>{0, 0, 0, axis[0], axis[1], axis[2]};
            if (axis[2] == 1)
            {
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1,
                                                           0, 0, 0, 0, 0};
            }
            else if (axis[1] == 1)
            {

                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 0, 0, 0, 1};
            }
            else
            {
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
        }
    };

}