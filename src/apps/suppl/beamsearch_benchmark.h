//
// Created by 汪子琦 on 06.01.23.
//

#ifndef BEAMSEARCH_BENCHMARK_H
#define BEAMSEARCH_BENCHMARK_H

#include "frame/FrameAssembly.h"

/*
 * search algorithms
 */
#include "algorithms/search.h"

/*
 * optimization algorithms
 */
#include "algorithms/optimization_holistic_fixedsteplength.h"
#include "algorithms/optimization_zlandmark_sub_beamsearch.h"
#include "algorithms/optimization_zlandmark_sub_holistic_fixedsteplength.h"
#include "algorithms/optimization_zlandmark_recursive.h"
#include <filesystem>

namespace suppl
{
    class BeamSearchApp {
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

        void runBenchMark()
        {
            std::vector<std::vector<int>> search_data = {
                    {1, 1,     200},
                    {1, 10,    200},
                    {1, 100,   200},
                    {1, 1000,  200},
                    {1, 10000, 200},

                    {4, 1,     200},
                    {4, 10,    60},
                    {4, 100,   60},
//                    {4, 1000,  60},

//                    {6, 1,     60},
//                    {6, 10,    60},
//                    {6, 100,   40},
            };

            for (int solverID = 0; solverID < search_data.size(); solverID++)
            {

                int numHand = search_data[solverID][0];
                int beamWidth = search_data[solverID][1];
                int maxNumBars = search_data[solverID][2];

                std::string foldername = ROBOCRAFT_DATA_FOLDER "/suppl/beam";
                foldername+= "_H" + std::to_string(numHand) + "_W" + std::to_string(beamWidth);

                std::cout << foldername << std::endl;

                if(!std::filesystem::exists(foldername)){
                    std::filesystem::create_directory(foldername);
                }

                for (int id = 0; id < filenames.size(); id++)
                {
                    std::string outputfilename = foldername + "/" + filenames[id] + ".json";

                    if(std::filesystem::exists(outputfilename)){
                        continue;
                    }

                    beamAssembly = std::make_shared<frame::FrameAssembly>();
                    beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");

                    if(beamAssembly->beams_.size() >= maxNumBars){
                        continue;
                    }

                    std::vector<int> startPartIDs = {};
                    std::vector<int> endPartIDs;
                    for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                    nlohmann::json json_output;
                    search::AssemblySequence sequence;
                    double time = 0;
                    double compliance = 0;
                    //
                    std::cout << "beam search, " << "H" << numHand << ", " << "W" << beamWidth << ": " << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;
                    algorithms::Search search(beamAssembly, numHand, startPartIDs, endPartIDs, true);
                    auto result = search.runSearch_Beam(beamWidth, sequence);
                    time = std::get<0>(result);
                    compliance = std::get<1>(result);

                    std::vector<double> complianceList;
                    double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand,
                                                                            complianceList, true);

                    std::cout << benchmark_compliance << ", " << time << std::endl;

                    // output
                    beamAssembly->writeToJson(json_output);
                    sequence.writeToJson(json_output);
                    json_output["benchmark_time"] = time;
                    json_output["benchmark_compliance"] = benchmark_compliance;

                    std::ofstream fout(outputfilename);
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
#endif  //BEAMSEARCH_BENCHMARK_H
