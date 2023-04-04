//
// Created by 汪子琦 on 04.04.23.
//

#ifndef GITIGNORE_TOWER_EXAMPLE_H
#define GITIGNORE_TOWER_EXAMPLE_H

#include "algorithms/optimization_holistic_dynamicsteplength.h"
#include "algorithms/search.h"
#include "algorithms/computeDeformation.h"

namespace examples
{
    class Tower_Example{
    public:
        std::vector<std::string> outputFileNames = {
                ROBOCRAFT_DATA_FOLDER "/examples/tower/holistic.json",
        };

    public:
        void launch()
        {
            runBenchMark();
        }

        void render()
        {
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                std::string filename = outputFileNames[solverID];
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(filename);
                search::AssemblySequence sequence;
                sequence.loadFromFile(filename);

                std::string deform_filename = filename;
                deform_filename.replace(deform_filename.end() - 5, deform_filename.end(), "_deform.json");

                algorithms::computeDeformation(beamAssembly, sequence);
                nlohmann::json json_content;
                beamAssembly->writeToJson(json_content);
                sequence.writeToJson(json_content);
                std::ofstream fout(deform_filename);
                fout << json_content;
                fout.close();
            }
        }

        void read(){
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(outputFileNames[solverID]);
                search::AssemblySequence sequence;
                sequence.loadFromFile(outputFileNames[solverID]);
                std::vector<double> complianceList;
                int numHand = 1;
                double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand, complianceList, false);
            }
        }

        void runBenchMark()
        {
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/tower.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                nlohmann::json json_output;
                search::AssemblySequence sequence;
                double time = 0;
                double compliance = 0;
                //
                int numHand = 5;
                if (outputFileNames[solverID].find("holistic") != std::string::npos)
                {
                    int numPart = endPartIDs.size() - startPartIDs.size();
                    int numStep = numPart / numHand - 1;
                    if (numPart % numHand != 0) numStep += 1;
                    std::cout << "holistic" << ": " << beamAssembly->beams_.size()
                              << std::endl;

                    double maxtime = 1000;
                    std::vector<double> time;
                    std::vector<double> c;
                    std::vector<double> lb;
                    algorithms::runOptimization_holistic_dynamicsteplength_return_intermediate_solutions(beamAssembly,
                                                                                                         numHand,
                                                                                                         numStep,
                                                                                                         maxtime,
                                                                                                         startPartIDs,
                                                                                                         endPartIDs,
                                                                                                         false,
                                                                                                         sequence,
                                                                                                         time,
                                                                                                         c,
                                                                                                         lb);

                    Eigen::VectorXd displacement;
                    beamAssembly->solveElasticity(endPartIDs, {}, displacement);
                    double final_compliance = beamAssembly->computeCompliance(displacement, endPartIDs);
                    for(int id = 0; id < time.size(); id++){
                        c[id] += final_compliance;
                        c[id] *= 1E3;

                        lb[id] += final_compliance;
                        lb[id] *= 1E3;
                    }

                    json_output["benchmark_time"] = time;
                    json_output["benchmark_compliance"] = c;
                    json_output["benchmark_lowerbound"] = lb;
                }

                std::vector<double> complianceList;
                double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand,
                                                                        complianceList, false);

                std::cout << benchmark_compliance << ", " << time << std::endl;

                // output
                beamAssembly->writeToJson(json_output);
                sequence.writeToJson(json_output);

                std::ofstream fout(outputFileNames[solverID]);
                fout << json_output;
                fout.close();
            }
        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/examples/model";
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}
#endif //GITIGNORE_TOWER_EXAMPLE_H
