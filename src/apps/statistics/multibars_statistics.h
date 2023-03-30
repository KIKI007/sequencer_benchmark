//
// Created by 汪子琦 on 30.03.23.
//

#ifndef GITIGNORE_MultiBars_Statistics_H
#define GITIGNORE_MultiBars_Statistics_H

#include "evaluation.h"
/*
 * search algorithms
 */
#include "frame/FrameAssembly.h"
#include <filesystem>
#include <fstream>

namespace statistics {
    class MultiBars_Statistics {
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
            std::cout << R"(\begin{tabular}{|c|c|ccc|ccc|}
\hline
\multirow{2}{*}{h}  & \multirow{2}{*}{Methods} & \multicolumn{3}{c|}{Assembly Cost}                                                                               & \multicolumn{3}{c|}{Time (s)}                                                                                         \\ \cline{3-8}
                    &                          & \multicolumn{1}{c|}{Small}                  & \multicolumn{1}{c|}{Medium}                & Large                 & \multicolumn{1}{c|}{Small}                   & \multicolumn{1}{c|}{Medium}                  & Large                   \\ \hline)"
                      << std::endl;
        }

        void print_time(double number) {
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
                           Eigen::Vector3i num_c,
                           Eigen::Vector3d sum_c,
                           Eigen::Vector3d min_c,
                           Eigen::Vector3d max_c,
                           Eigen::Vector3d sum_t,
                           Eigen::Vector3d min_t,
                           Eigen::Vector3d max_t,
                           bool head = false,
                           bool fixed_time = false) {
            std::cout << "& " << name;
            for (int id = 0; id < 3; id++) {
                if (head) {
                    std::cout << R"( & \multicolumn{1}{c|}{1})";
                } else if (num_c[id] > 0) {
                    std::cout << R"( & \multicolumn{1}{c|}{)" << std::fixed << std::setprecision(2)
                              << sum_c[id] / num_c[id] << " (" << min_c[id] << R"($\sim$)" << max_c[id] << ")}";
                } else {
                    std::cout << R"( & \multicolumn{1}{c|}{-})";
                }
            }

            for (int id = 0; id < 3; id++) {
                if (fixed_time) {
                    std::cout << R"( & \multicolumn{1}{c|}{10})";
                } else if (num_c[id] > 0) {

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
            std::cout << R"(\\ \cline{2-8})" << std::endl;
        }

        void print_table_end() {
            std::cout << R"(\end{tabular}
                            \label{tab: staistics-multi-bar}
                            \end{table*})" << std::endl;
        }

        void launch() {
            getFileNames();

            std::vector<int> benchmark_hands = {4, 6, 10};

            std::vector<std::string> table_captions = {
                    "Merge",
                    "Greedy",
                    "z-Landmark (Ours)",
                    "Holistic (Ours)"
            };

            std::vector<std::string> folderNames = {
                    "merge",
                    "greedy",
                    "opt-z-landmark-holistic",
                    "opt-holistic",
            };
            print_table_head();

            std::vector<int> numParts;
            for (int id = 0; id < filenames.size(); id++) {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");
                numParts.push_back(beamAssembly->beams_.size());
            }
            for (int benchmark: benchmark_hands) {
                std::vector<std::vector<double>> compliances;
                std::vector<std::vector<double>> times;

                int num_row = 0;
                for (int fid = 0; fid < folderNames.size(); fid++) {
                    std::string foldername =
                            ROBOCRAFT_DATA_FOLDER "/benchmark-" + std::to_string(benchmark) + "/" + folderNames[fid];
                    if (std::filesystem::exists(foldername)) {
                        num_row++;
                        compliances.resize(folderNames.size());
                        times.resize(folderNames.size());
                        compliances[fid].resize(filenames.size(), -1);
                        times[fid].resize(filenames.size(), 0);
                        for (int id = 0; id < filenames.size(); id++) {
                            std::string result_filename = foldername + "/" + filenames[id] + ".json";
                            if (std::filesystem::exists(result_filename)) {
                                nlohmann::json json_file;
                                std::ifstream fin(result_filename);
                                fin >> json_file;
                                fin.close();

                                if (json_file.contains("benchmark_compliance")) {
                                    compliances[fid][id] = json_file["benchmark_compliance"].get<double>();
                                } else {
                                    beamAssembly = std::make_shared<frame::FrameAssembly>();
                                    beamAssembly->loadFromJson(result_filename);
                                    search::AssemblySequence sequence;
                                    sequence.loadFromJson(json_file);
                                    std::vector<double> complianceList;
                                    double compliance = benchmark::runEvaluation(beamAssembly, sequence, benchmark,
                                                                                 complianceList, true);
                                    compliances[fid][id] = compliance;
                                }
                                times[fid][id] = json_file["benchmark_time"].get<double>();
                            }
                        }
                    }
                }

                std::cout << R"(\multirow{)" << num_row << "}{*}{" << benchmark << "}";
                for (int fid = 0; fid < folderNames.size(); fid++) {
                    std::string foldername =
                            ROBOCRAFT_DATA_FOLDER "/benchmark-" + std::to_string(benchmark) + "/" + folderNames[fid];
                    if (std::filesystem::exists(foldername)) {
                        Eigen::Vector3i num_c;
                        num_c.setZero();

                        Eigen::Vector3d sum_c;
                        sum_c.setZero();
                        Eigen::Vector3d min_c(1E8, 1E8, 1E8);
                        Eigen::Vector3d max_c;
                        max_c.setZero();

                        Eigen::Vector3d sum_t;
                        sum_t.setZero();
                        Eigen::Vector3d min_t(1E8, 1E8, 1E8);
                        Eigen::Vector3d max_t;
                        max_t.setZero();

                        for (int id = 0; id < filenames.size(); id++) {
                            double base_compliance = compliances[0][id];
                            double curr_compliance = compliances[fid][id];
                            double curr_time = times[fid][id];
                            if (curr_compliance < 0) {
                                continue;
                            } else {
                                int index = -1;

                                if (numParts[id] < 40) {
                                    index = 0;
                                } else if (numParts[id] < 60) {
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
                        print_content(table_captions[fid], num_c, sum_c, min_c, max_c, sum_t, min_t, max_t, fid == 0,
                                      table_captions[fid] == "Backtrack-greedy");
                    }

                }
                std::cout << R"(\hline)";

            }
            print_table_end();

        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/dataset";
        std::vector<std::string> filenames;
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}

#endif //GITIGNORE_MultiBars_Statistics_H
