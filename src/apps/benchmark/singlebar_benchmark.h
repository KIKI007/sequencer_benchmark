//
// Created by 汪子琦 on 06.01.23.
//

#ifndef PAVILLION_JSON_BENCHMARK_H
#define PAVILLION_JSON_BENCHMARK_H

#include "frame/FrameAssembly.h"

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
#include "algorithms/optimizatiom_holistic_fixedsteplength.h"
#include "algorithms/optimization_zlandmark_sub_beamsearch.h"
#include "algorithms/optimization_zlandmark_sub_holistic_fixedsteplength.h"

#include <filesystem>

namespace simpApp {
    class BenchMark {
    public:

    public:

        void launch()
        {
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
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/seach-forwardgreedy",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/search-backwardgreedy",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/search-backtrackgreedy",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/search-beam-100X",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/search-beam-1000X",
                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/opt-2-landmark-holistic",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/opt-2-landmark-beam-100",
//                    ROBOCRAFT_DATA_FOLDER "/benchmark-1/opt-holistic",
            };

            for (int solverID = 0; solverID < folderNames.size(); solverID++)
            {
                for (int id = 0; id < filenames.size(); id++)
                {
                    beamAssembly = std::make_shared<frame::FrameAssembly>();
                    beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");

                    std::vector<int> startPartIDs = {};
                    std::vector<int> endPartIDs;
                    for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);
                    int numHand = 1;

                    nlohmann::json json_output;
                    search::AssemblySequence sequence;
                    double time = 0;
                    double compliance = 0;
                    //
                    if (folderNames[solverID].find("seach-forwardgreedy") != std::string::npos)
                    {
                        std::cout << "seach-forwardgreedy"<< ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        auto result = benchmark::runSearch_ForwardGreedy(beamAssembly, numHand, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("search-backwardgreedy") != std::string::npos)
                    {
                        std::cout << "search-backwardgreedy" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        auto result = benchmark::runSearch_BackwardGreedy(beamAssembly, numHand, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("search-backtrackgreedy") != std::string::npos)
                    {
                        std::cout << "search-backtrackgreedy"<< ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        int maxtime = 10;
                        auto result = benchmark::runSearch_BacktrackGreedy(beamAssembly, numHand, maxtime, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("search-beam-100X") != std::string::npos)
                    {
                        std::cout << "search-beam-100" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        int beamWidth = 100;
                        auto result = benchmark::runSearch_Beam(beamAssembly,
                                                                numHand, beamWidth,
                                                                startPartIDs,
                                                                endPartIDs,
                                                                true,
                                                                sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("search-beam-1000X") != std::string::npos)
                    {
                        std::cout << "search-beam-1000" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        int beamWidth = 1000;
                        auto result = benchmark::runSearch_Beam(beamAssembly, numHand, beamWidth, startPartIDs, endPartIDs,
                                                                true,
                                                                sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("opt-holistic") != std::string::npos)
                    {
                        std::cout << "opt-holistic" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

                        double maxtime = 1000;
                        auto result = benchmark::runOptimization_holistic_fixedsteplength(beamAssembly,
                                                                                          numHand,
                                                                                          maxtime,
                                                                                          startPartIDs, endPartIDs,
                                                                                          true,
                                                                                          sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }
                    else if (folderNames[solverID].find("opt-2-landmark-holistic") != std::string::npos)
                    {
                        std::cout << "opt-2-landmark-holistic" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;
                        double maxHolisticSolverTime = 100;
                        int numLandmark = beamAssembly->beams_.size() / 10;
                        double maxLandmarkTime = 10 * numLandmark;
                        auto result = benchmark::runOptimization_zlandmark_sub_holistic_fixedsteplength(beamAssembly, numHand, numLandmark, maxLandmarkTime, maxHolisticSolverTime, true, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                        json_output["num_landmark"] = numLandmark;
                    }
                    else if (folderNames[solverID].find("opt-2-landmark-beam-100") != std::string::npos)
                    {
                        std::cout << "opt-2-landmark-beam-100" << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;
                        double maxLandmarkTime = 10;
                        int numLandmark = 2;
                        int beamWidth = 100;
                        auto result = benchmark::runOptimization_zlandmark_sub_beamsearch(beamAssembly, numHand, numLandmark, beamWidth, maxLandmarkTime, sequence);
                        time = std::get<0>(result);
                        compliance = std::get<1>(result);
                    }

                    std::vector<double> complianceList;
                    double benchmark_compliance = benchmark::runEvaluation(beamAssembly, sequence, numHand, complianceList, true);

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
        int numHand = 1;
    };
}

//auto [t0, c0] = computeGreedy(beamAssembly,  false, seq0); seqs.push_back(seq0); times.push_back(t0);
//auto [t1, c1] = computeGreedyBackward(beamAssembly,  false, seq1);seqs.push_back(seq1);times.push_back(t1);
//auto [t2, c2] = computeGreedyBacktracking(beamAssembly,10, false, seq2);seqs.push_back(seq2);times.push_back(t2);
//            auto [t3, c3] = runSearch_Beam(beamAssembly, startPartIDs, endPartIDs, 100, false, seq3);seqs.push_back(seq3); times.push_back(t3);
//auto [t4, c4] = computeBeamSearch(beamAssembly, startPartIDs, endPartIDs, 1000, false, seq4);seqs.push_back(seq4); times.push_back(t4);


//            double maxtime = beamAssembly->beams_.size() > 40 ? 20 : 10;
//            maxtime = beamAssembly->beams_.size() > 60 ? 200 : maxtime;
//            maxtime = beamAssembly->beams_.size() > 100 ? 300 : maxtime;
//
//            int numLayer = beamAssembly->beams_.size() > 40 ? 4 : 3;
//            numLayer = beamAssembly->beams_.size() > 60 ? 5 : numLayer;
//            numLayer = beamAssembly->beams_.size() > 100 ? 7 : numLayer;
//            numLayer = beamAssembly->beams_.size() > 120 ? 9 : numLayer;

//            double maxtime = 20;
//            int numLayer = 3;

//            auto [t5, c5] = computeBottomUp(beamAssembly, maxtime, numLayer, seq5);seqs.push_back(seq5); times.push_back(t5);

//double time = beamAssembly->beams_.size() >= 40 ? 800 : 300;

#endif  //PAVILLION_JSON_BENCHMARK_H
