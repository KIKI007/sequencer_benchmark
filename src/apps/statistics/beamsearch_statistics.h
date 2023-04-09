//
// Created by 汪子琦 on 30.03.23.
//

#ifndef GITIGNORE_BeamSearch_Statistics_H
#define GITIGNORE_BeamSearch_Statistics_H

#include "evaluation.h"
/*
 * search algorithms
 */
#include "frame/FrameAssembly.h"
#include <filesystem>
#include <fstream>

namespace statistics {
    class BeamSearch_Statistics {
    public:
        void getFileNames() {
            filenames.clear();
            for (const auto &entry: std::filesystem::directory_iterator(dataFolderString)) {
                if (entry.path().stem() != ".DS_Store") {
                    filenames.push_back(entry.path().stem());
                }
            }
        };

        void print_table_head() {
            std::cout << R"(\begin{tabular}{|c|c|c|ccc|ccc|}
                            \hline
                            \multirow{2}{*}{$h$}  & \multirow{2}{*}{Methods} & \multirow{2}{*}{$X$} & \multicolumn{3}{c|}{Assembly Cost}  & \multicolumn{3}{c|}{Time (s)} \\ \cline{4-9}
                            &  & & \multicolumn{1}{c|}{Small} & \multicolumn{1}{c|}{Medium}  & Large & \multicolumn{1}{c|}{Small} & \multicolumn{1}{c|}{Medium}   & Large
                            \\ \hline)" << std::endl;
        }

        void print_time(double number)
        {
            int num_digit = 0;
            if (number > 100) {
                num_digit = 0;
            } else if (number > 1) {
                num_digit = 1;
            } else {
                num_digit = 2;
            }
            std::cout << std::fixed << std::setprecision(num_digit) << number;
        }

        void print_content(std::string name,
                           int nameHeight,
                           int beamWidth,
                           Eigen::Vector3i num_c,
                           Eigen::Vector3d sum_c,
                           Eigen::Vector3d min_c,
                           Eigen::Vector3d max_c,
                           Eigen::Vector3d sum_t,
                           Eigen::Vector3d min_t,
                           Eigen::Vector3d max_t,
                           bool head = false) {
            if(nameHeight > 0){
                std::cout << R"( & \multirow{)" << nameHeight << R"(}{*}{\textsc{)" << name << R"(}} &)" << beamWidth;
            }
            else{
                std::cout << R"( & & )" << beamWidth;
            }
            for (int id = 0; id < 3; id++)
            {
                if (head) {
                    std::cout << R"( & \multicolumn{1}{c|}{1})";
                } else if (num_c[id] > 0) {
                    std::cout << R"( & \multicolumn{1}{c|}{)" << std::fixed << std::setprecision(2)
                              << sum_c[id] / num_c[id] << " (" << min_c[id] << R"($\sim$)" << max_c[id] << ")}";
                } else {
                    std::cout << R"( & \multicolumn{1}{c|}{-})";
                }
            }

            for (int id = 0; id < 3; id++)
            {
                if (num_c[id] > 0) {

                    std::cout << R"(& \multicolumn{1}{c|}{)";
                    print_time(sum_t[id] / num_c[id]);
                    std::cout << " (";
                    print_time(min_t[id]);
                    std::cout << R"($\sim$)";
                    print_time(max_t[id]);
                    std::cout << ")}";
                } else {
                    std::cout << R"(& \multicolumn{1}{c|}{-})";
                }
            }
            //std::cout << R"(\\ \cline{3-9})" << std::endl;
        }

        void print_table_end() {
            std::cout << R"(\end{tabular}
                            \label{tab: staistics-multi-bar}
                            \end{table*})" << std::endl;
        }

        void launch() {
            getFileNames();

            std::vector<int> numHands = {1, 4};
            std::vector<std::vector<int>> beamWidths = {{1, 10, 100, 1000, 10000,
                                                         1, 10, 100, 1000, 10000},
                                                        {1, 10, 100,
                                                         1, 10, 100}};
            std::vector<std::vector<int>> maxNumParts = {{200, 200, 200, 200, 200,
                                                          200, 200, 200, 200, 200},
                                                         {200, 60, 40,
                                                          200, 200, 200}};

            std::vector<std::vector<std::string>> methodNames = {{"beam", "beam", "beam", "beam", "beam",
                                                                  "landmark", "landmark", "landmark", "landmark", "landmark"},
                                                                 {"beam", "beam", "beam",
                                                                  "landmark", "landmark", "landmark"}};

            print_table_head();

            std::vector<int> numParts;
            for (int id = 0; id < filenames.size(); id++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");
                numParts.push_back(beamAssembly->beams_.size());
            }

            for (int id = 0; id < numHands.size(); id++)
            {
                int num_row = 0;
                int num_name_height = 0;
                int numHand = numHands[id];

                std::vector<std::vector<double>> compliances;
                std::vector<std::vector<double>> times;
                std::vector<int> nameHeights;

                compliances.resize(beamWidths[id].size());
                times.resize(beamWidths[id].size());

                for (int jd = 0; jd < beamWidths[id].size(); jd++)
                {
                    num_name_height++;

                    if(jd + 1 == beamWidths[id].size() || methodNames[id][jd + 1] != methodNames[id][jd])
                    {
                        nameHeights.push_back(num_name_height);
                        num_name_height = 0;
                    }

                    int beamWidth = beamWidths[id][jd];
                    std::string methodName = methodNames[id][jd];

                    std::string foldername = ROBOCRAFT_DATA_FOLDER "/suppl/";
                    foldername += methodName;
                    foldername += "_H" + std::to_string(numHand) + "_W" + std::to_string(beamWidth);

                    compliances[jd].resize(filenames.size(), -1);
                    times[jd].resize(filenames.size(), 0);

                    //if (std::filesystem::exists(foldername))
                    {
                        num_row++;
                        for (int fID = 0; fID < filenames.size(); fID++)
                        {
                            std::string seq_filaname = foldername + "/" + filenames[fID] + ".json";
                            if (std::filesystem::exists(seq_filaname))
                            {
                                nlohmann::json json_file;
                                std::ifstream fin(seq_filaname);
                                fin >> json_file;
                                fin.close();

                                beamAssembly = std::make_shared<frame::FrameAssembly>();
                                beamAssembly->loadFromJson(seq_filaname);

                                if(beamAssembly->beams_.size() >= maxNumParts[id][jd])
                                    continue;

                                search::AssemblySequence sequence;
                                sequence.loadFromJson(json_file);
                                std::vector<double> complianceList;
                                double compliance = algorithms::runEvaluation(beamAssembly,
                                                                              sequence,
                                                                              numHand,
                                                                              complianceList,
                                                                              true);
                                compliances[jd][fID] = compliance;

                                times[jd][fID] = json_file["benchmark_time"].get<double>();
                            }
                        }
                    }
                }

//                for(int jd = 0; jd < nameHeights.size(); jd++){
//                    std::cout << nameHeights[jd] << ' ';
//                }
//                std::cout << std::endl;

                std::cout << R"(\multirow{)" << num_row << "}{*}{" << numHand << "}";
                auto name_height_it = nameHeights.begin();
                for (int jd = 0; jd < beamWidths[id].size(); jd++)
                {
                    int beamWidth = beamWidths[id][jd];
                    std::string methodName = methodNames[id][jd];

                    std::string foldername = ROBOCRAFT_DATA_FOLDER "/suppl/";
                    foldername += methodName;
                    foldername += "_H" + std::to_string(numHand) + "_W" + std::to_string(beamWidth);

                    //if (std::filesystem::exists(foldername))
                    {
                        Eigen::Vector3i num_c;
                        num_c.setZero();

                        Eigen::Vector3d sum_c;
                        sum_c.setZero();
                        Eigen::Vector3d min_c(1E8, 1E8, 1E8);
                        Eigen::Vector3d max_c(0, 0, 0);

                        Eigen::Vector3d sum_t;
                        sum_t.setZero();
                        Eigen::Vector3d min_t(1E8, 1E8, 1E8);
                        Eigen::Vector3d max_t(0, 0, 0);

                        for (int fid = 0; fid < filenames.size(); fid++)
                        {
                            double base_compliance = compliances[0][fid];
                            double curr_compliance = compliances[jd][fid];
                            double curr_time = times[jd][fid];
                            if (curr_compliance < 0) {
                                continue;
                            } else {
                                int index = -1;
                                if (numParts[fid] < 40) {
                                    index = 0;
                                } else if (numParts[fid] < 60) {
                                    index = 1;
                                } else {
                                    index = 2;
                                }

                                num_c[index]++;

                                double curr_ratio = curr_compliance / base_compliance;
                                sum_c(index) += curr_ratio;
                                min_c(index) = std::min(min_c(index), curr_ratio);
                                max_c(index) = std::max(max_c(index), curr_ratio);

                                sum_t(index) += curr_time;
                                min_t(index) = std::min(min_t(index), curr_time);
                                max_t(index) = std::max(max_t(index), curr_time);
                            }
                        }
                        if(jd == 0 || methodNames[id][jd] != methodNames[id][jd - 1])
                        {
                            if(jd != 0)
                            {
                                std::cout << R"(\\ \cline{2-9})" << std::endl;
                            }
                            print_content(methodName, *name_height_it, beamWidth, num_c, sum_c, min_c, max_c, sum_t, min_t, max_t, jd == 0);

                            name_height_it++;
                        }
                        else{
                            std::cout << R"(\\ \cline{3-9})" << std::endl;
                            print_content(methodName, 0, beamWidth, num_c, sum_c, min_c, max_c, sum_t, min_t, max_t, jd == 0);
                        }
                    }
                }
                std::cout << R"(\\ \hline)" << std::endl;
            }
            print_table_end();

        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/dataset";
        std::vector<std::string> filenames;
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}

#endif //GITIGNORE_BeamSearch_Statistics_H
