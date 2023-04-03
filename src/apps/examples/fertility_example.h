//
// Created by 汪子琦 on 29.03.23.
//

#ifndef GITIGNORE_BRIDGE_BENCHMARK_H
#define GITIGNORE_BRIDGE_BENCHMARK_H

#include "algorithms/optimization_zlandmark_sub_beamsearch.h"
#include "algorithms/optimization_zlandmark_recursive.h"
#include "algorithms/optimization_holisticlandmark_sub_beamsearch.h"
#include "algorithms/search.h"

namespace examples
{
    class Fertility_Example{
    public:
        void launch() {
            runBenchMark();
        }

        void runBenchMark() {
            std::vector<std::string> outputFileNames = {
                    //ROBOCRAFT_DATA_FOLDER "/examples/fertility/forwardgreedy.json",
                    //ROBOCRAFT_DATA_FOLDER "/examples/fertility/zlandmark_greedy.json",
                    ROBOCRAFT_DATA_FOLDER "/examples/fertility/holisticlandmark_greedy.json",
            };

            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/fertility.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                nlohmann::json json_output;
                search::AssemblySequence sequence;
                double time = 0;
                double compliance = 0;
                //
                int numHand = 1;
                if (outputFileNames[solverID].find("forwardgreedy") != std::string::npos)
                {
                    std::cout << "greedy" << ": " << beamAssembly->beams_.size()
                              << std::endl;

                    algorithms::Search search(beamAssembly, numHand, startPartIDs, endPartIDs, true);
                    auto result = search.runSearch_ForwardGreedy(sequence);

                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                }
                else if(outputFileNames[solverID].find("holisticlandmark_greedy") != std::string::npos){
                    std::cout << "holisticlandmark_greedy" << ": " << beamAssembly->beams_.size() << std::endl;
                    int numLandmark = 1;
                    int maxTime = 300;
                    auto result = benchmark::runOptimization_holisticlandmark_sub_beamsearch(beamAssembly,
                                                                                             numHand,
                                                                                             numLandmark,
                                                                                             1,
                                                                                             maxTime,
                                                                                             false,
                                                                                             sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                    json_output["num_landmark"] = numLandmark;
                }
                else if (outputFileNames[solverID].find("zlandmark_greedy") != std::string::npos)
                {
                    std::cout << "zlandmark_greedy" << ": " << beamAssembly->beams_.size() << std::endl;
                    int numLandmark = 3;
                    double maxLandmarkTime = 300 * numLandmark;
                    auto result = benchmark::runOptimization_zlandmark_sub_beamsearch(beamAssembly,
                                                                                      numHand,
                                                                                      numLandmark,
                                                                                      1,
                                                                                      maxLandmarkTime,
                                                                                      false,
                                                                                      sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                    json_output["num_landmark"] = numLandmark;
                }

                std::vector<double> complianceList;
                double benchmark_compliance = benchmark::runEvaluation(beamAssembly, sequence, numHand,
                                                                       complianceList, false);

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
#endif //GITIGNORE_BRIDGE_BENCHMARK_H
