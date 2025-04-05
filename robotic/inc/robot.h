#include <items.h>

namespace Engine
{
    class Joint
    {
        friend class World;

    protected:
        Cube* parent_link;
        Vector3d origin;
        int id;
        std::vector<Cube*> child_link;

    public:
        Joint(Cube &_pi, Vector3d _origin, int _id) : parent_link(&_pi), origin(_origin), id(_id)
        {
        }
        Joint(Cube &_pi, Cube &_ci, Vector3d _origin, int _id) : parent_link(&_pi), origin(_origin), id(_id)
        {
            this->add_child_link(_ci);
        }
        void add_child_link(Cube &);
    };

    class Joint_node
    {
    private:
        int id;
        std::vector<Joint_node*> childs;

    public:
        Joint_node()
        {
            id = -1;
        }
        Joint_node(int _id) : id(_id) {}
        void add_child(int);
        Joint_node * find(int);
        void insert(int,int);
    };
}
