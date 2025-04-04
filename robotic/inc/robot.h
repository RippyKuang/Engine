#include <world.h>

namespace Engine
{
    class Joint
    {
    private:
        Item parent_item;
        Item child_item;

    public:
        Joint(Item _pi, Item _ni) : parent_item(_pi), child_item(_ni)
        {
        }
    };
}
