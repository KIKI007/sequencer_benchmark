//
// Created by 汪子琦 on 31.03.23.
//

#ifndef GITIGNORE_MIP_HEURISTICS_BENCHMARK_H
#define GITIGNORE_MIP_HEURISTICS_BENCHMARK_H
#include "algorithms/util/evaluation.h"
namespace benchmark{
class MipHeuristicApp{
public:

    void launch(){
        std::vector<std::string> filenames = {
                ROBOCRAFT_DATA_FOLDER "/mip_heuristics/roboarch_none.json",
                ROBOCRAFT_DATA_FOLDER "/mip_heuristics/roboarch_basic.json",
                ROBOCRAFT_DATA_FOLDER "/mip_heuristics/roboarch_advanced.json",
                ROBOCRAFT_DATA_FOLDER "/mip_heuristics/roboarch_extensive.json",
        };
        for (int solverID = 0; solverID < filenames.size(); solverID++)
        {
            beamAssembly = std::make_shared<frame::FrameAssembly>();
            beamAssembly->loadFromJson(dataFolderString + "/roboarch_florian.json");
            std::vector<int> startPartIDs = {};
            std::vector<int> endPartIDs;
            for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);
            int numHand = 4;
            int numPart = endPartIDs.size() - startPartIDs.size();
            int numStep = numPart / numHand - 1;
            if (numPart % numHand != 0) numStep += 1;

            std::shared_ptr<dto::FrameTORangeSum> frameTO = std::make_shared<dto::FrameTORangeSum>(*beamAssembly);
            frameTO->setStartnEnd(startPartIDs, endPartIDs);
            frameTO->sections_.clear();
            frameTO->sections_ = {{0, endPartIDs.size()}};
            frameTO->num_steps_ = {numStep};
            frameTO->num_arm_ = numHand;
            frameTO->setParameters(1E-8, 0);

            dto::FrameTOKnitroSolver solver(frameTO);
            solver.mip_tol = 1E-8;
            solver.nlp_tol = 1E-10;
            solver.maxtime = 1000;
            solver.hessian_provided = true;
            solver.MINLP = true;
            solver.silence = false;

            if (filenames[solverID].find("roboarch_none") != std::string::npos) {
                solver.heuristic = KN_MIP_HEUR_STRATEGY_NONE;
            }
            else if(filenames[solverID].find("roboarch_basic") != std::string::npos){
                solver.heuristic = KN_MIP_HEUR_STRATEGY_BASIC;
            }
            else if(filenames[solverID].find("roboarch_advanced") != std::string::npos){
                solver.heuristic = KN_MIP_HEUR_STRATEGY_ADVANCED;
            }
            else if(filenames[solverID].find("roboarch_extensive") != std::string::npos){
                solver.heuristic = KN_MIP_HEUR_STRATEGY_EXTENSIVE;
            }

            Eigen::VectorXd x;
            solver.solve(x);
            std::vector<Eigen::VectorXd> rhos;
            frameTO->computeRhos(x.data(), rhos);
            search::AssemblySequence sequence;
            double compliance = computeOptimizationSequence(beamAssembly, startPartIDs, endPartIDs, rhos, sequence);
            double time = computeShortestTimeFindingIncumbentSolution(solver);

            //output
            nlohmann::json json_output;
            beamAssembly->writeToJson(json_output);
            sequence.writeToJson(json_output);
            json_output["benchmark_time"] = time;
            json_output["benchmark_compliance"] = compliance;
            json_output["time_curve"] = solver.data.time;
            json_output["compliance_curve"] = solver.data.compliance;
            json_output["lnb_curve"] = solver.data.low_bnd;

            std::ofstream fout(filenames[solverID]);
            fout << json_output;
            fout.close();
        }
    }

public:

    std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/dataset";
    std::shared_ptr<frame::FrameAssembly> beamAssembly;
};
}
#endif //GITIGNORE_MIP_HEURISTICS_BENCHMARK_H
