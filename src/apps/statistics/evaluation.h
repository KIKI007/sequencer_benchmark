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
