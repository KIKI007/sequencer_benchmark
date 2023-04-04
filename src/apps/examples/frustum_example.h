//
// Created by 汪子琦 on 29.03.23.
//

#ifndef GITIGNORE_frustum_BENCHMARK_H
#define GITIGNORE_frustum_BENCHMARK_H

#include "algorithms/optimization_zlandmark_sub_holistic_dynamicsteplength.h"
#include "algorithms/optimization_zlandmark_recursive.h"
#include "algorithms/optimization_holisticlandmark_sub_beamsearch.h"
#include "algorithms/search.h"
#include "algorithms/computeDeformation.h"

namespace examples
{
    class Frustum_Example{
    public:
        std::vector<std::string> outputFileNames = {
                ROBOCRAFT_DATA_FOLDER "/examples/frustum/forwardgreedy.json",
                ROBOCRAFT_DATA_FOLDER "/examples/frustum/zlandmark_holistic.json",
        };

    public:
        void launch()
        {
            runBenchMark();
        }

        void read(){
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(outputFileNames[solverID]);
                search::AssemblySequence sequence;
                sequence.loadFromFile(outputFileNames[solverID]);

                std::vector<double> complianceList;
                int numHand = 6;
                double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand, complianceList, false);
            }
        }

        void runBenchMark()
        {
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/frustum.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                nlohmann::json json_output;
                search::AssemblySequence sequence;
                double time = 0;
                double compliance = 0;
                //
                int numHand = 6;
                if (outputFileNames[solverID].find("forwardgreedy") != std::string::npos)
                {
                    std::cout << "greedy" << ": " << beamAssembly->beams_.size()
                              << std::endl;

                    algorithms::Search search(beamAssembly, 1, startPartIDs, endPartIDs, true);
                    auto result = search.runSearch_ForwardGreedy(sequence);

                    time = std::get<0>(result);
                    compliance = std::get<1>(result);
                }
                else if (outputFileNames[solverID].find("zlandmark_holistic") != std::string::npos)
                {
                    std::cout << "zlandmark_holistic" << ": " << beamAssembly->beams_.size() << std::endl;
                    int numLandmark = 2;
                    double maxLandmarkSolverTime = 50 * numLandmark;
                    int maxHolisticSolverTime = 300;
                    auto result = algorithms::runOptimization_zlandmark_sub_holistic_dynamicsteplength(beamAssembly,
                                                                                                       numHand,
                                                                                                       numLandmark,
                                                                                                       maxLandmarkSolverTime,
                                                                                                       maxHolisticSolverTime,
                                                                                                       false,
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
                json_output["benchmark_compliance"] = benchmark_compliance;

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
#endif //GITIGNORE_frustum_BENCHMARK_H
