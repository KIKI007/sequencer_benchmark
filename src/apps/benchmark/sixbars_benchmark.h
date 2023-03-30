//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_SIXBARS_BENCHMARK_H
#define GITIGNORE_SIXBARS_BENCHMARK_H

/*
 * search algorithms
 */
#include "algorithms/search_fowardgreedy.h"
#include "algorithms/search_backwardgreedy.h"
#include "algorithms/search_backtrackgreedy.h"
#include "algorithms/search_beam.h"

/*
 * optimization algorithms
 */
#include "algorithms/optimization_holistic_dynamicsteplength.h"
#include "algorithms/optimization_zlandmark_sub_beamsearch.h"
#include "algorithms/optimization_zlandmark_sub_holistic_dynamicsteplength.h"

#include <filesystem>

namespace benchmark {
    class SixBars_BenchMark {
    public:

    public:

        void launch() {
            getFileNames();
            runBenchMark();
        }

        void getFileNames() {
            filenames.clear();
            for (const auto &entry: std::filesystem::directory_iterator(dataFolderString)) {
                if (entry.path().stem() != ".DS_Store") {
                    filenames.push_back(entry.path().stem());
                }
            }
        };

        void runBenchMark() {

            std::vector<std::string> folderNames = {
                    ROBOCRAFT_DATA_FOLDER "/benchmark-4/merge",
                    ROBOCRAFT_DATA_FOLDER "/benchmark-4/greedy",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-6/opt-z-landmark-holistic",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-6/opt-holistic",
            };

            for (int solverID = 0; solverID < folderNames.size(); solverID++)
            {
                for (int id = 0; id < filenames.size(); id++)
                {
//                    if(filenames[id] != "70562"){
//                        continue;
//                    }

                    beamAssembly = std::make_shared<frame::FrameAssembly>();
                    beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");

                    std::vector<int> startPartIDs = {};
                    std::vector<int> endPartIDs;
                    for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);
                    int numHand = 6;

                    nlohmann::json json_output;
                    search::AssemblySequence sequence;
                    double time = 0;
                    double compliance = 0;
                    //
                    if (folderNames[solverID].find("merge") != std::string::npos) {
                        std::cout << "merge" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;

                        auto result = benchmark::runSearch_ForwardGreedy(beamAssembly, 1, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    } else if (folderNames[solverID].find("greedy") != std::string::npos) {
                        if (beamAssembly->beams_.size() >= 40)
                            continue;
                        std::cout << "Greedy" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;

                        auto result = benchmark::runSearch_Beam(beamAssembly, numHand, 1, startPartIDs, endPartIDs,
                                                                true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("opt-holistic") != std::string::npos) {
                        std::cout << "opt-holistic" << ": " << filenames[id] << ", " << beamAssembly->beams_.size()
                                  << std::endl;

                        double maxtime = 1000;
                        auto result = benchmark::runOptimization_holistic_fixedsteplength(beamAssembly,
                                                                                          numHand,
                                                                                          maxtime,
                                                                                          startPartIDs, endPartIDs,
                                                                                          true,
                                                                                          sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    } else if (folderNames[solverID].find("opt-z-landmark-holistic") != std::string::npos) {
                        std::cout << "opt-z-landmark-holistic" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;
                        double maxHolisticSolverTime = 100;
                        int numLandmark = beamAssembly->beams_.size() / 30;
                        double maxLandmarkTime = 30 * numLandmark;
                        auto result = benchmark::runOptimization_zlandmark_sub_holistic_dynamicsteplength(beamAssembly,
                                                                                                          numHand,
                                                                                                          numLandmark,
                                                                                                          maxLandmarkTime,
                                                                                                          maxHolisticSolverTime,
                                                                                                          true, sequence);
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
                    std::ofstream fout(folderNames[solverID] + "/" + filenames[id] + ".json");
                    fout << json_output;
                    fout.close();
                }
            }
        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/dataset";
        std::vector<std::string> filenames;
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}

#endif //GITIGNORE_SIXBARS_BENCHMARK_H
