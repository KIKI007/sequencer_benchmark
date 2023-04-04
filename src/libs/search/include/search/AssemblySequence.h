//
// Created by 汪子琦 on 17.03.22.
//

#ifndef ROBO_CRAFT_ASSEMBLYSEQUENCE_H
#define ROBO_CRAFT_ASSEMBLYSEQUENCE_H

#include <vector>
#include "nlohmann/json.hpp"
#include <fstream>
#include <Eigen/Dense>

namespace search
{
struct AssemblyStep {
public:
    std::vector<int> installPartIDs;
    std::vector<int> holdPartIDs;

public:
    std::vector<Eigen::Vector3d> partDrts;
    std::vector<int> deformPartIDs;
    std::vector<std::vector<Eigen::Vector3d>> deformLines;
    std::vector<std::vector<double>> deformValues;
    double compliance = 0;
};

class AssemblySequence {
public:
    std::vector<AssemblyStep> steps;

public:

    void loadFromFile(std::string filename)
    {
        std::ifstream fin(filename);
        nlohmann::json json_content;
        fin >> json_content;
        loadFromJson(json_content);
        fin.close();
    }

    void loadFromJson(nlohmann::json json_node)
    {
        steps.clear();
        for(int id = 0; id < json_node["assembly_sequence"].size(); id++)
        {
            nlohmann::json step_node = json_node["assembly_sequence"][id];
            AssemblyStep step;
            step.installPartIDs = step_node["installPartIDs"].get<std::vector<int>>();
            step.holdPartIDs = step_node["holdPartIDs"].get<std::vector<int>>();
            if(!step.installPartIDs.empty() || !step.holdPartIDs.empty()){
                steps.push_back(step);
            }
        }
    }

    void writeToJson(nlohmann::json &json_node)
    {
        nlohmann::json assembly_node = nlohmann::json::array();
        for (int id = 0; id < steps.size(); id++)
        {
            if(steps[id].installPartIDs.empty() && steps[id].holdPartIDs.empty()) continue;

            nlohmann::json step_node;
            step_node["installPartIDs"] = steps[id].installPartIDs;
            step_node["holdPartIDs"] = steps[id].holdPartIDs;
            step_node["compliance"] = steps[id].compliance;

            //disassembling direction
            nlohmann::json drts_node;
            for(int jd = 0; jd < steps[id].partDrts.size(); jd++)
            {
                double x = steps[id].partDrts[jd].x();
                double y = steps[id].partDrts[jd].y();
                double z = steps[id].partDrts[jd].z();
                nlohmann::json drt_node = {z, x, y};
                drts_node.push_back(drt_node);
            }
            step_node["partDirections"] = drts_node;

            //deformation
            nlohmann::json deformation_nodes;
            for(int jd = 0; jd < steps[id].deformPartIDs.size(); jd++)
            {
                nlohmann::json deformation_node;
                int partID = steps[id].deformPartIDs[jd];
                deformation_node["partID"] = partID;
                deformation_node["lines"] = {};
                deformation_node["distances"] = {};
                for(int kd = 0; kd < steps[id].deformLines[jd].size(); kd++)
                {
                    Eigen::Vector3d pt = steps[id].deformLines[jd][kd];
                    double dist = steps[id].deformValues[jd][kd];

                    deformation_node["lines"].push_back({pt.z(), pt.x(), pt.y()});
                    deformation_node["distances"].push_back(dist);
                }
                deformation_nodes.push_back(deformation_node);
            }
            step_node["Deformation"] = deformation_nodes;
            assembly_node.push_back(step_node);
        }
        json_node["assembly_sequence"] = assembly_node;
    }
};
}
#endif  //ROBO_CRAFT_ASSEMBLYSEQUENCE_H
