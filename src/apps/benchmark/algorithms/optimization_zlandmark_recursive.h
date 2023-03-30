//
// Created by 汪子琦 on 30.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_ZLANDMARK_RECURSIVE_H
#define GITIGNORE_OPTIMIZATION_ZLANDMARK_RECURSIVE_H

#include "optimization_zlandmark_sub_holistic_fixedsteplength.h"

namespace benchmark {
    static std::tuple<double, double>
    runOptimization_zlandmark_recursive(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                        int numHand,
                                        int numLandmarks,
                                        int maxHolisticNumPart,
                                        double maxLandmarkSolverTime,
                                        double maxHolisticSolverTime,
                                        const std::vector<int> &startPartIDs,
                                        const std::vector<int> &endPartIDs,
                                        bool silence,
                                        search::AssemblySequence &sequence) {

        std::vector<std::vector<int>> landmarks;
        tbb::tick_count timer = tbb::tick_count::now();
        compute_zlandmarks(beamAssembly, maxLandmarkSolverTime / numLandmarks, numLandmarks, startPartIDs, endPartIDs,
                           landmarks, silence);
        double time = (tbb::tick_count::now() - timer).seconds();

        double sub_time = 0;
        sequence.steps.clear();
        for (int id = 0; id + 1 < landmarks.size(); id++)
        {
            std::vector<int> start = landmarks[id];
            std::vector<int> end = landmarks[id + 1];
            std::cout << "[" << start.size() << ", " << end.size() << "]" << std::endl;

            search::AssemblySequence tmp_sequence;
            if (end.size() - start.size() < maxHolisticNumPart) {
                auto [tmp_time, tmp_value] = runOptimization_holistic_fixedsteplength(beamAssembly,
                                                                                      numHand,
                                                                                      maxHolisticSolverTime,
                                                                                      start,
                                                                                      end,
                                                                                      true,
                                                                                      tmp_sequence);
                sub_time += tmp_time;
            } else {
                auto [tmp_time, tmp_value] = runOptimization_zlandmark_recursive(beamAssembly,
                                                                                 numHand,
                                                                                 numLandmarks,
                                                                                 maxHolisticNumPart,
                                                                                 maxLandmarkSolverTime,
                                                                                 maxHolisticSolverTime,
                                                                                 start,
                                                                                 end,
                                                                                 silence,
                                                                                 tmp_sequence);
                sub_time += tmp_time;
            }
            sequence.steps.insert(sequence.steps.end(), tmp_sequence.steps.begin(), tmp_sequence.steps.end());
        }

        time += sub_time;
        if((endPartIDs.size() - startPartIDs.size()) == beamAssembly->beams_.size()){
            std::vector<double> complianceList;
            double value = runEvaluation(beamAssembly, sequence, numHand, complianceList, true);
            return {time, value};
        }
        else{
            return {time, -1};
        }
    }
}
#endif //GITIGNORE_OPTIMIZATION_ZLANDMARK_RECURSIVE_H
