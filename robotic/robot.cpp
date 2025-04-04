#include <robot.h>

namespace Engine
{
    void Joint::add_child_link(Cube &link)
    {
        link.transform(getTransformMat(EYE(3), origin));
        child_link.push_back(link);
    }
    int Joint::get_id()
    {
        return this->id;
    }
    void Joint_node::add_child(int id)
    {
        Joint_node* j = new Joint_node(id);
        childs.push_back(*j);
    }
    void Joint_node::insert(int parent,int id)
    {

    }
}