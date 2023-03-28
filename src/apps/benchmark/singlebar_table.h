//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_SINGLEBAR_TABLE_H
#define GITIGNORE_SINGLEBAR_TABLE_H

void computeResult()
//    {
//        std::vector<std::string> folderNames = {
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-forward",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backward",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backtrack",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-100",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-1000",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-landmark",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-forward",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-beam-100",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-greedy-merge",
//           //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark-new",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-forward",
//            ROBOCRAFT_DATA_FOLDER "/benchmark/1-bnb"
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-bnb"
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-greedy-merge",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/10-bnb",
//            //ROBOCRAFT_DATA_FOLDER "/benchmark/10-greedy-merge",
//        };
//
//
//        for(auto folder: folderNames)
//        {
//            dataFolderString = folder;
//            getFileNames();
//            std::ofstream fout(dataFolderString + ".txt");
//            for(int id = 0; id < filenames.size(); id++)
//            {
//                beamAssembly = std::make_shared<frame::FrameAssembly>();
//                std::string json_file = dataFolderString + "/" + filenames[id] + ".json";
//                beamAssembly->loadFromJson(json_file);
//                search::AssemblySequence sequence;
//                sequence.loadFromFile(json_file);
//                std::vector<double> complianceList;
//                double compliance = computeComplianceList(beamAssembly, sequence, complianceList, true);
//                nlohmann::json json_input;
//                std::ifstream fin(json_file);
//                fin >> json_input;
//                fout << filenames[id] << ", " << beamAssembly->beams_.size() << ", " << compliance << ", " << json_input["benchmark_time"].get<double>() << ", " << json_input["obj_lnb"].get<double>() << std::endl;
//                std::cout << filenames[id] << ", " << beamAssembly->beams_.size() << ", " << compliance << ", " << json_input["benchmark_time"].get<double>() << std::endl;
//
//            }
//            fout.close();
//        }
//    }
//
//

#endif //GITIGNORE_SINGLEBAR_TABLE_H
