//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_EVALUATION_H
#define GITIGNORE_EVALUATION_H
#include "frame/FrameAssembly.h"
#include "search/PartGraphFrame.h"
#include "search/StateGraphMultipleArms.h"
namespace benchmark
{
    static double computeOptimizationSequence(std::shared_ptr<frame::FrameAssembly> assembly,
                                              std::vector<int> startPartIDs,
                                              std::vector<int> endPartIDs,
                                              const std::vector<Eigen::VectorXd> &rhos,
                                              search::AssemblySequence &sequence){

        std::vector<std::vector<int>> seq_partIDs;
        seq_partIDs.push_back(startPartIDs);
        for(int id = 0; id < rhos.size(); id++)
        {
            std::vector<int> partIDs;
            for(int jd = 0; jd < rhos[id].size(); jd++)
            {
                if(rhos[id][jd] > 0.5) partIDs.push_back(jd);
            }
            seq_partIDs.push_back(partIDs);
        }
        seq_partIDs.push_back(endPartIDs);

        double sum_compliance = 0;
        std::vector<int> prev_partIDs = startPartIDs;
        for(int id = 0; id < seq_partIDs.size(); id++)
        {
            Eigen::VectorXd displacement;
            assembly->solveElasticity(seq_partIDs[id], {}, displacement);
            sum_compliance += assembly->computeCompliance(displacement, seq_partIDs[id]);

            search::AssemblyStep step;
            for(int jd = 0; jd < seq_partIDs[id].size(); jd++)
            {
                int partID = seq_partIDs[id][jd];
                if(find(prev_partIDs.begin(), prev_partIDs.end(), partID) == prev_partIDs.end()){
                    step.installPartIDs.push_back(partID);
                }
            }
            prev_partIDs = seq_partIDs[id];
            sequence.steps.push_back(step);
        }
        return sum_compliance;
    }


    static double computeShortestTimeFindingIncumbentSolution(const dto::FrameTOKnitroSolver &solver)
    {
        double value = solver.data.objective * 0.5;
        double time = 0;
        for(int id = (int) solver.data.time.size() - 1; id >= 0; id --)
        {
            if(solver.data.compliance[id] > 1.05 * value)
            {
                break ;
            }
//            std::cout << solver.data.time[id] << ", " << solver.data.compliance[id] << ", " << value << std::endl;
            time = solver.data.time[id];
        }
//        std::cout << time << std::endl;
        return time;
    }

    static double runEvaluation(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                search::AssemblySequence &sequence,
                                int numHand,
                                std::vector<double> &complianceList,
                                bool silence){
        std::vector<int> subset_beam_index = {};
        complianceList.clear();
        complianceList.push_back(0);

        double result = 0;
        int prevNumBars = 0;

        for (int jd = 0; jd < sequence.steps.size(); jd++)
        {
            int numNewBars = sequence.steps[jd].installPartIDs.size();
            if(subset_beam_index.size() + numNewBars - prevNumBars > numHand)
            {
                Eigen::VectorXd displacement;
                beamAssembly->solveElasticity(subset_beam_index, {}, displacement);
                double compliance =  beamAssembly->computeCompliance(displacement, subset_beam_index);
                complianceList.push_back(compliance);
                if(!silence) std::cout << subset_beam_index.size() << ", " << compliance << std::endl;
                result += compliance;
                prevNumBars = subset_beam_index.size();
                jd  --;
            }
            else{
                subset_beam_index.insert(subset_beam_index.end(),
                                         sequence.steps[jd].installPartIDs.begin(),
                                         sequence.steps[jd].installPartIDs.end());
            }
        }

        //final
        {
            Eigen::VectorXd displacement;
            beamAssembly->solveElasticity(subset_beam_index, {}, displacement);
            double compliance =  beamAssembly->computeCompliance(displacement, subset_beam_index);
            complianceList.push_back(compliance);
            if(!silence) std::cout << subset_beam_index.size() << ", " << compliance << std::endl;
            result += compliance;
        }

        return result;
    }
}


#endif //GITIGNORE_EVALUATION_H
