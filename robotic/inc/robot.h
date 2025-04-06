#include <items.h>
#include <functional>
#include <map>

namespace Engine
{
    class World;
    class Joint;
    class Joint_node;

    enum JOINT_TYPE
    {
        REVOLUTE,
        CONTINUOUS,
        FIXED,
        PRISMATIC
    };

    typedef struct _INFO
    {
        JOINT_TYPE type = FIXED;
        double pos;
        _INFO(double _pos) : pos(_pos)
        {
        }
    } INFO;

    typedef struct _CONTINUOUS_INFO : INFO
    {
        Vector3d axis;
        _CONTINUOUS_INFO(Vector3d _axis, double pos = 0) : INFO(pos), axis(_axis)
        {
            type = CONTINUOUS;
        }
    } CONTINUOUS_INFO;

    typedef struct _REVOLUTE_INFO : _CONTINUOUS_INFO
    {

        double max_rad;
        double min_rad;
        _REVOLUTE_INFO(Vector3d _axis, double max, double min, double pos = 0) : CONTINUOUS_INFO(_axis, pos),
                                                                                 max_rad(max), min_rad(min)
        {
            type = REVOLUTE;
        }
    } REVOLUTE_INFO;

    class Joint_node
    {

    private:
      
        int parent_link_id;
        Vector3d origin;
        _R pose = EYE(3);
        INFO *info;
        std::vector<Joint_node *> childs;
        std::vector<int> childs_link_id;

    public:
        int joint_id;
        Joint_node()
        {
            joint_id = -1;
            parent_link_id = -1;
        }
        Joint_node(int _id, Vector3d _origin, INFO *_info) : joint_id(_id), origin(_origin), info(_info) {}
        Joint_node *add_child(const Joint *);
        void append_child_link_id(int);
        void set_parent_link_id(int);
        int get_parent_link_id();
        Joint_node *find(int);
        Joint_node *insert(int, const Joint *);
        Vector3d get_origin() const;
        _R get_pose() const;
        void transform_origin(_R, _R , Vector3d);
        INFO *get_info() const;

        void act(_R, _R, Vector3d, std::map<int, Link *> &, std::function<void(int, _T)>);
    };

    class Joint
    {
        friend class World;
        friend class Joint_node;

    protected:
        Cube *parent_link;
        Vector3d origin;
        INFO *info;
        int id;
        std::vector<Cube *> child_link;

    public:
        Joint(Cube &_pi, Vector3d _origin, int _id, INFO *_type) : parent_link(&_pi), origin(_origin),
                                                                   id(_id), info(_type)
        {
        }
        Joint(Cube &_pi, Cube &_ci, Vector3d _origin, int _id, INFO *_type) : parent_link(&_pi), origin(_origin),
                                                                              id(_id), info(_type)
        {
            this->add_child_link(_ci);
        }
        void add_child_link(Cube &);

        static double forward(std::map<int, Link *> &, Joint_node *tgt, double inc, std::function<void(int, _T)>);
    };

}
