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
        corners.push_back(Vector4d{center[0] + x / 2, center[1] - y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] - y / 2, center[2] - z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] + y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] + y / 2, center[2] - z / 2, 1});

        corners.push_back(Vector4d{center[0] - x / 2, center[1] - y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] - y / 2, center[2] - z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] + y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] + y / 2, center[2] - z / 2, 1});
        init_pose = getTransformMat(EYE(3), center);
    }
    Camera::Camera(Vector3d center, _R _intrinsics) : intrisics(_intrinsics)
    {
        corners.push_back(Vector4d(center, 1));
        init_pose = getTransformMat(EYE(3), center);
    }
    std::vector<Point2i> Camera::project(const Item &pw)
    {
        std::vector<Point2i> corners;
        for (auto corner : pw.corners)
        {
            Vector3d temp = this->intrisics * Vector3d{corner[1] / corner[0], corner[2] / corner[0], 1};
            corners.push_back(Point2i(temp));
        }
        return corners;
    }
    void World::emplace(Camera &i, int id)
    {
        Camera *j = (Camera *)malloc(sizeof(Camera));
        memcpy(j, &i, sizeof(Camera));
        cameras.insert(std::pair<int, Camera *>(id, j));
        items.insert(std::pair<int, Item *>(id, j));
        pose.insert(std::pair<int, _T>(id, i.init_pose));
    }
    void World::emplace(Cube &i, int id)
    {
        Item *j = (Item *)malloc(sizeof(Cube));
        memcpy(j, &i, sizeof(Cube));
        items.insert(std::pair<int, Item *>(id, j));
        pose.insert(std::pair<int, _T>(id, i.init_pose));
    }

    Item *World::get(int id)
    {
        return items.at(id);
    }
    Camera &World::getCamHandle(int id)
    {
        return *cameras.at(id);
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
    std::vector<Vector4d> World::getCoord(int id, int base)
    {
        Item it = *items.at(id);
        it.transform(inv(pose[base]));
        return it.corners;
    }
}