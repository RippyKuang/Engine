#include <world.h>

namespace Engine
{
    
    

    void World::emplace(Cube &i, int id)
    {
        Item *j = (Item *)malloc(sizeof(Cube));
        memcpy(j, &i, sizeof(Cube));
        items.insert(std::pair<int, Item *>(id, j));
        pose.insert(std::pair<int, _T>(id, i.init_pose));
    }

    void World::act(int id, _T t, int base)
    {
        pose[id] = (pose[base] * t * inv(pose[base])) * pose[id];
        (*items.at(id)).transform((pose[base] * t * inv(pose[base])));
    }
    void World::act(int id, _R r, int base)
    {
        auto t = catRow(catCol(r, Vector3d()), catCol(Vector3d().T(), EYE(1)));
        pose[id] = (pose[base] * t * inv(pose[base])) * pose[id];
        (*items.at(id)).transform((pose[base] * t * inv(pose[base])));
    }
    std::vector<Point2i> World::project()
    {
        std::map<int, Item *>::iterator iter = items.begin();
        std::vector<Point2i> projs;
        while (iter != items.end())
        {
            Item cube_in_camera = getCoord(iter->first, -2);
            std::vector<Point2i> tprojs = cam.project(cube_in_camera);
            projs.insert(projs.end(), tprojs.begin(), tprojs.end());
            iter++;
        }
        return projs;
    }
    std::vector<Vector4d> World::getCoord(int id, int base)
    {
        Item it = *items.at(id);
        it.transform(inv(pose[base]));
        return it.corners;
    }
}