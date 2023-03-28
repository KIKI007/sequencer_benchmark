//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H
#define GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H

#include "search_beam.h"
#include "util/optimization_zlandmark.h"
#include "util/evaluation.h"
namespace benchmark
{
    static std::tuple<double, double> runOptimization_zlandmark_sub_beamsearch(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                                               int numHand,
                                                                               int numLandmarks,
                                                                               int beamWidth,
                                                                               double maxTime,
                                                                               search::AssemblySequence &sequence)
    {
        std::vector<std::vector<int>> landmarks;

        tbb::tick_count timer = tbb::tick_count::now();
        compute_zlandmarks(beamAssembly, maxTime / numLandmarks, numLandmarks, landmarks);
        double time = (tbb::tick_count::now() - timer).seconds();
        double sub_time = 0;
        for(int id = 0; id + 1 < landmarks.size(); id++)
        {
            timer = tbb::tick_count::now();
            std::vector<int> startPartIDs = landmarks[id];
            std::vector<int> endPartIDs = landmarks[id + 1];
            search::AssemblySequence tmp_sequence;
            runSearch_Beam(beamAssembly, numHand, beamWidth, startPartIDs, endPartIDs, false, tmp_sequence);
            sequence.steps.insert(sequence.steps.end(), tmp_sequence.steps.begin(), tmp_sequence.steps.end());
            sub_time = std::max(sub_time, (tbb::tick_count::now() - timer).seconds());
        }
        time += sub_time;
        std::vector<double> compliance_list;
        double compliance = runEvaluation(beamAssembly, sequence, numHand, compliance_list, true);
        //std::cout << numLandmarks << "-landmark-beam-" << beamWidth << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }
}

#endif //GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H
