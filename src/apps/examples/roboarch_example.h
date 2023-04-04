//
// Created by 汪子琦 on 29.03.23.
//

#ifndef GITIGNORE_ROBOARCH_EXAMPLE_H
#define GITIGNORE_ROBOARCH_EXAMPLE_H

#include "algorithms/optimization_zlandmark_sub_holistic_dynamicsteplength.h"
#include "algorithms/optimization_holistic_dynamicsteplength.h"

namespace examples
{
    class Roboarch_Example {
    public:
        std::vector<std::string> outputFileNames ={
                ROBOCRAFT_DATA_FOLDER "/examples/roboarch/z-landmark.json",
                ROBOCRAFT_DATA_FOLDER "/examples/roboarch/holistic.json",
        };


    public:
        void launch() {
            runBenchMark();
        }

        void runBenchMark() {

            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/roboarch_florian.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);
                int numHand = 4;

                nlohmann::json json_output;
                search::AssemblySequence sequence;
                double time = 0;
                double compliance = 0;
                //

                if (outputFileNames[solverID].find("holistic") != std::string::npos) {
                    std::cout << "opt-holistic" << ": " << beamAssembly->beams_.size()
                              << std::endl;
                    
                    double maxtime = 30000;
                    int numPart = endPartIDs.size() - startPartIDs.size();
                    int numStep = numPart / numHand - 1;
                    if (numPart % numHand != 0) numStep += 1;
                    auto result = algorithms::runOptimization_holistic_dynamicsteplength(beamAssembly,
                                                                                         numHand,
                                                                                         numStep,
                                                                                         maxtime,
                                                                                         startPartIDs, endPartIDs,
                                                                                         true,
                                                                                         sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                } else if (outputFileNames[solverID].find("z-landmark") != std::string::npos) {
                    std::cout << "opt-z-landmark-holistic" << ": " << beamAssembly->beams_.size() << std::endl;
                    double maxHolisticSolverTime = 300;
                    int numLandmark = 2;
                    double maxLandmarkTime = 30 * numLandmark;
                    auto result = algorithms::runOptimization_zlandmark_sub_holistic_dynamicsteplength(beamAssembly,
                                                                                                       numHand,
                                                                                                       numLandmark,
                                                                                                       maxLandmarkTime,
                                                                                                       maxHolisticSolverTime,
                                                                                                       true,
                                                                                                       sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                    json_output["num_landmark"] = numLandmark;
                }

                std::vector<double> complianceList;
                double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand,
                                                                        complianceList, true);

                std::cout << benchmark_compliance << ", " << time << std::endl;

                // output
                beamAssembly->writeToJson(json_output);
                sequence.writeToJson(json_output);
                json_output["benchmark_time"] = time;
                json_output["benchmark_compliance"] = compliance;
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
#endif //GITIGNORE_ROBOARCH_EXAMPLE_H
