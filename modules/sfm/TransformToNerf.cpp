#include <Eigen/Geometry>
#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "usage: ./TransformToNerf 'folder with transform.json (result of from eacham_sfm)'\n";
        return -1;
    }

    std::string path = argv[1];

    if (path[path.size() - 1] != '/')
    {
        path += "/";
    }

    std::string pathOut = path + "transforms_nerf.json";
    path += "transform.json";

    nlohmann::json frames;
    // read a JSON file
    std::ifstream ifStream(path);

    if (!ifStream.is_open())
    {
        std::cout << "Error: no 'transform.json' in the current folder\n";
        return -1;
    }

    ifStream >> frames;
    ifStream.close();

    for (auto& frame : frames["frames"])
    {
        std::vector<std::vector<double>> posJson = frame["transform_matrix"];

        Eigen::Matrix4d pos = Eigen::Matrix4d::Identity();
        
        for (int i = 0; i < 4; ++i)
        {
            double* ptr = &posJson[i][0];
            pos.row(i) = Eigen::Map<Eigen::Vector4d>(ptr, 4);
        }

        pos = pos.inverse();

        Eigen::Matrix4d matr = Eigen::Matrix4d::Identity();
        matr(1, 1) = -1;
        matr(2, 2) = -1;
        pos = pos * matr;

        // pos.block<3, 1>(0, 3) *= 0.1f;

        frame["transform_matrix"] = {
            { pos(0, 0), pos(0, 1), pos(0, 2), pos(0, 3) },
            { pos(1, 0), pos(1, 1), pos(1, 2), pos(1, 3) },
            { pos(2, 0), pos(2, 1), pos(2, 2), pos(2, 3) },
            { pos(3, 0), pos(3, 1), pos(3, 2), pos(3, 3) }
        };
    }

    std::ofstream file(pathOut, std::ios_base::out);
    if (file.is_open())
    {
        file << std::setw(4) << frames << std::endl;

        file.close();
    }

    return 0;
}