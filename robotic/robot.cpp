#include <robot.h>

namespace Engine
{
    void Joint::add_child_link(Cube &link)
    {
        link.transform(getTransformMat(EYE(3), origin));
        child_link.push_back(&link);
    }

    void Joint_node::add_child(int id, Vector3d _origin)
    {
        Joint_node *j = new Joint_node(id,_origin+this->origin);
        childs.push_back(j);
    }
    Joint_node *Joint_node::find(int id)
    {
        if (this->id == id)
            return this;
        else
            for (auto child : childs)
            {
                Joint_node* node = child->find(id);
                if (node!=NULL)
                    return node;
            }
            return NULL;
    }
    void Joint_node::insert(int parent, int id, Vector3d origin)
    {
        Joint_node *tgt = this->find(parent);
        tgt->add_child(id, origin);
    }
    Vector3d Joint_node::get_origin()const
    {
        return this->origin;
    }
}