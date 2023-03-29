//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_TENBARS_BENCHMARK_H
#define GITIGNORE_TENBARS_BENCHMARK_H

/*
 * search algorithms
 */
#include "algorithms/search_fowardgreedy.h"

/*
 * optimization algorithms
 */
#include "algorithms/optimization_holistic_dynamicsteplength.h"
#include "algorithms/optimization_zlandmark_sub_beamsearch.h"
#include "algorithms/optimization_zlandmark_sub_holistic_dynamicsteplength.h"

#include <filesystem>

namespace benchmark {
    class TenBars_BenchMark {
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
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-6/seach-forwardgreedy",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-6/opt-z-landmark-beam-100",
                    ROBOCRAFT_DATA_FOLDER "/benchmark-10/opt-z-landmark-holistic",
                    ROBOCRAFT_DATA_FOLDER "/benchmark-10/opt-holistic",
            };

            for (int solverID = 0; solverID < folderNames.size(); solverID++)
            {
                for (int id = 0; id < filenames.size(); id++)
                {

//                    if(filenames[id] != "84622")
//                        continue;

                    beamAssembly = std::make_shared<frame::FrameAssembly>();
                    beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");

                    std::vector<int> startPartIDs = {};
                    std::vector<int> endPartIDs;
                    for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);
                    int numHand = 10;

                    nlohmann::json json_output;
                    search::AssemblySequence sequence;
                    double time = 0;
                    double compliance = 0;
                    //
                    if (folderNames[solverID].find("seach-forwardgreedy") != std::string::npos) {
                        std::cout << "seach-forwardgreedy" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;

                        auto result = benchmark::runSearch_ForwardGreedy(beamAssembly, numHand, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("opt-holistic") != std::string::npos)
                    {
                        int numPart = endPartIDs.size() - startPartIDs.size();
                        int numStep = numPart / numHand - 1;
                        if (numPart % numHand != 0) numStep += 1;
                        std::cout << "opt-holistic" << ": " << filenames[id] << ", " << beamAssembly->beams_.size()
                                  << std::endl;

                        double maxtime = 1000;
                        auto result = benchmark::runOptimization_holistic_dynamicsteplength(beamAssembly,
                                                                                            numHand,
                                                                                            numStep,
                                                                                            maxtime,
                                                                                            startPartIDs,
                                                                                            endPartIDs,
                                                                                            true,
                                                                                            sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    } else if (folderNames[solverID].find("opt-z-landmark-holistic") != std::string::npos) {
                        std::cout << "opt-z-landmark-holistic" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;
                        double maxHolisticSolverTime = 100;
                        int numLandmark = beamAssembly->beams_.size() / 40;
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
                    } else if (folderNames[solverID].find("opt-z-landmark-beam-100") != std::string::npos) {
                        std::cout << "opt-z-landmark-beam-100" << ": " << filenames[id] << ", "
                                  << beamAssembly->beams_.size() << std::endl;
                        int numLandmark = beamAssembly->beams_.size() / 20;
                        double maxLandmarkTime = 30 * numLandmark;
                        int beamWidth = 100;
                        auto result = benchmark::runOptimization_zlandmark_sub_beamsearch(beamAssembly, numHand,
                                                                                          numLandmark, beamWidth,
                                                                                          maxLandmarkTime, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
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

#endif //GITIGNORE_TENBARS_BENCHMARK_H
