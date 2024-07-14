#include <world.h>

namespace Engine
{
    void Item::transform(_T t)
    {
        for (auto it = this->corners.begin(); it != this->corners.end(); it++)
            *it = t * (*it);
    }

    Item::Item(std::vector<Vector3d> _corners)
    {
        for (auto corner : _corners)
            this->corners.push_back(Vector4d(corner, 1));
    }
    Cube::Cube(Vector3d center, double x, double y, double z)
    {
        corners.push_back(Vector4d{center[0]+x/2,center[1]-y/2,center[2]+z/2,1});
        corners.push_back(Vector4d{center[0]+x/2,center[1]-y/2,center[2]-z/2,1});
        corners.push_back(Vector4d{center[0]+x/2,center[1]+y/2,center[2]+z/2,1});
        corners.push_back(Vector4d{center[0]+x/2,center[1]+y/2,center[2]-z/2,1});

        corners.push_back(Vector4d{center[0]-x/2,center[1]-y/2,center[2]+z/2,1});
        corners.push_back(Vector4d{center[0]-x/2,center[1]-y/2,center[2]-z/2,1});
        corners.push_back(Vector4d{center[0]-x/2,center[1]+y/2,center[2]+z/2,1});
        corners.push_back(Vector4d{center[0]-x/2,center[1]+y/2,center[2]-z/2,1});
        init_pose = getTransformMat(EYE(3),center);
    }
    Camera::Camera(Vector3d center)
    {
        corners.push_back(Vector4d(center,1));
        init_pose = getTransformMat(EYE(3),center);
    }
    
    void World::emplace(Item &i, int id)
    {
        items.insert(std::pair<int, Item>(id, i));
        pose.insert(std::pair<int, _T>(id, i.init_pose));
    }
    void World::print(int id)
    {
        std::cout << id << " coord" << std::endl;
        std::cout << items.at(id) << std::endl;
        std::cout << id << " pose" << std::endl;
        std::cout << pose.at(id) << std::endl;
    }
    void World::act(int id, _T t)
    {
        pose[id] = t * pose[id];
        items.at(id).transform(t);
    }
    void World::act(int id, _R r)
    {
        auto t = catRow(catCol(r, Vector3d()), catCol(Vector3d().T(), EYE(1)));
        pose[id] = t * pose[id];
        items.at(id).transform(t);
    }
    Item World::getCoord(int id, int base)
    {
        Item it =items.at(id);
        it.transform(inv(pose[base]));
        return it;
    }
}