//
// Created by 汪子琦 on 23.09.22.
//

#ifndef ROBO_CRAFT_FRAMEASSEMBLYEXTERNALFORCES_H
#define ROBO_CRAFT_FRAMEASSEMBLYEXTERNALFORCES_H
#include "FrameAssembly.h"
namespace frame{
class FrameAssemblyExternalForces : public FrameAssembly
{
public:

    Eigen::VectorXd external_force_;

public:

    FrameAssemblyExternalForces(){}

    FrameAssemblyExternalForces(const FrameAssembly& assembly);

    FrameAssemblyExternalForces(const FrameAssemblyExternalForces& assembly);

public:

    void solveElasticity(const std::vector<int> &subset_beams_index, const std::vector<int> &fix_beam_index, Eigen::VectorXd &displacement);

    void loadFromJson(std::string filename, double scale = 1.0, Eigen::Vector3d offset = Eigen::Vector3d(0, 0, 0));

    void writeToJson(nlohmann::json &json_file, std::vector<int> subset_beams = {});

    void saveToJson(std::string filename, std::vector<int> subset_beams = {});


};
}


#endif  //ROBO_CRAFT_FRAMEASSEMBLYEXTERNALFORCES_H
