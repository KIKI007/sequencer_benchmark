//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_ZLANDMARK_SUB_HOLISTIC_FIXEDSTEPLENGTH_H
#define GITIGNORE_OPTIMIZATION_ZLANDMARK_SUB_HOLISTIC_FIXEDSTEPLENGTH_H

#include "optimization_holistic_fixedsteplength.h"

namespace benchmark {

    static std::tuple<double, double>
    runOptimization_zlandmark_sub_holistic_fixedsteplength(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                           int numHand,
                                                           int numLandmarks,
                                                           double maxLandmarkSolverTime,
                                                           double maxHolisticSolverTime,
                                                           bool silence,
                                                           search::AssemblySequence &sequence) {

        std::vector<std::vector<int>> landmarks;
        std::vector<int> startPartIDs = {};
        std::vector<int> endPartIDs = {};
        for(int id = 0; id < beamAssembly->beams_.size(); id++){
            endPartIDs.push_back(id);
        }

        tbb::tick_count timer = tbb::tick_count::now();
        compute_zlandmarks(beamAssembly, maxLandmarkSolverTime / numLandmarks, numLandmarks, startPartIDs, endPartIDs, landmarks);
        double time = (tbb::tick_count::now() - timer).seconds();

        double sub_time = 0;
        sequence.steps.clear();
        for (int id = 0; id + 1 < landmarks.size(); id++) {
            std::vector<int> startPartIDs = landmarks[id];
            std::vector<int> endPartIDs = landmarks[id + 1];
            search::AssemblySequence tmp_sequence;
            auto [tmp_time, tmp_value] = runOptimization_holistic_fixedsteplength(beamAssembly, numHand,
                                                                                  maxHolisticSolverTime, startPartIDs,
                                                                                  endPartIDs, silence, tmp_sequence);
            sub_time += tmp_time;
            sequence.steps.insert(sequence.steps.end(), tmp_sequence.steps.begin(), tmp_sequence.steps.end());
        }

        time += sub_time;
        std::vector<double> complianceList;
        double value = runEvaluation(beamAssembly, sequence, numHand, complianceList, true);

        return {time, value};
    }
}

#endif //GITIGNORE_OPTIMIZATION_ZLANDMARK_SUB_HOLISTIC_FIXEDSTEPLENGTH_H