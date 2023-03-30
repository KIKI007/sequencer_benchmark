//
// Created by 汪子琦 on 29.03.23.
//

#ifndef GITIGNORE_BRIDGE_BENCHMARK_H
#define GITIGNORE_BRIDGE_BENCHMARK_H

#include "algorithms/optimization_zlandmark_sub_holistic_dynamicsteplength.h"
#include "algorithms/optimization_holistic_dynamicsteplength.h"

namespace benchmark
{
    class Bridge_BenchMark {
    public:
        void launch() {
            runBenchMark();
        }

        void runBenchMark() {
            std::vector<std::string> folderNames = {
                    ROBOCRAFT_DATA_FOLDER "/bridge/hand5",
                    ROBOCRAFT_DATA_FOLDER "/bridge/hand1",
            };

            for (int solverID = 0; solverID < folderNames.size(); solverID++) {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/bridge.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                nlohmann::json json_output;
                search::AssemblySequence sequence;
                double time = 0;
                double compliance = 0;
                //
                int numHand = 0;
                if (folderNames[solverID].find("hand5") != std::string::npos)
                {
                    numHand = 5;
                    std::cout << "hand5" << ": " << beamAssembly->beams_.size()
                              << std::endl;
                    
                    double maxtime = 1000;
                    int numPart = endPartIDs.size() - startPartIDs.size();
                    int numStep = numPart / numHand - 1;
                    if (numPart % numHand != 0) numStep += 1;
                    auto result = benchmark::runOptimization_holistic_dynamicsteplength(beamAssembly,
                                                                                        numHand,
                                                                                        numStep,
                                                                                        maxtime,
                                                                                        startPartIDs, endPartIDs,
                                                                                        false,
                                                                                        sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                } else if (folderNames[solverID].find("hand1") != std::string::npos)
                {
                    numHand = 1;
                    std::cout << "hand1" << ": " << beamAssembly->beams_.size() << std::endl;
                    double maxHolisticSolverTime = 20;
                    int numLandmark = 2;
                    double maxLandmarkTime = 10 * numLandmark;
                    int maxHolisticNumPart = 10;
                    auto result = benchmark::runOptimization_zlandmark_recursive(beamAssembly,
                                                                                 numHand,
                                                                                 numLandmark,
                                                                                 maxHolisticNumPart,
                                                                                 maxLandmarkTime,
                                                                                 maxHolisticSolverTime,
                                                                                 startPartIDs,
                                                                                 endPartIDs,
                                                                                 true,
                                                                                 sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                    json_output["num_landmark"] = numLandmark;
                }

                std::vector<double> complianceList;
                double benchmark_compliance = benchmark::runEvaluation(beamAssembly, sequence, numHand,
                                                                       complianceList, true);

                std::cout << benchmark_compliance << ", " << time << std::endl;

                // output
                beamAssembly->writeToJson(json_output);
                sequence.writeToJson(json_output);
                json_output["benchmark_time"] = time;
                json_output["benchmark_compliance"] = compliance;
                std::ofstream fout(folderNames[solverID] + "/bridge.json");
                fout << json_output;
                fout.close();
            }
        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/model";
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}
#endif //GITIGNORE_BRIDGE_BENCHMARK_H
