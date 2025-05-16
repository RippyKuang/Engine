#include <robot.h>

namespace Engine
{
    void Part::add_child_link(Cube &link)
    {
        child_link = &link;
    }

    void Robot::summary() const
    {
        std::cout << "Summary of Robot" << std::endl;
        for (int i = 0; i < this->p.size(); i++)
        {
            std::cout << "Joint " << i << ":  "
                      << this->bo[this->p[i]]->get_name() << "  "
                      << this->bo[this->s[i]]->get_name()
                      << std::endl;
        }
    }

    void Robot::FK(std::vector<_T> &T)const
    {
        std::vector<M66> X;
        T.emplace_back(EYE(4));
        X.emplace_back(EYE(6));

        for (int i = 0; i < this->bo.size()-1; i++)
        {
            M66 Xj;
            jo[i]->jtype->jcalc(Xj);
            M66 Xip = Xj * jo[i]->parent2joint;
            if (this->lambda[i] != 0)
            {
                X.emplace_back(Xip * X[this->lambda[i]]);
                T.emplace_back(inv(inv_ad(Xip * X[this->lambda[i]])));
            }
            else
            {
                X.emplace_back(Xip);
                T.emplace_back(inv(inv_ad(Xip)));
            }
        }
    }
}
