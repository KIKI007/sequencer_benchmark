//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H
#define GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H

#include "search.h"
#include "optimization_zlandmark.h"
#include "evaluation.h"
namespace benchmark
{
    static std::tuple<double, double> runOptimization_zlandmark_sub_beamsearch(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                                               int numHand,
                                                                               int numLandmarks,
                                                                               int beamWidth,
                                                                               double maxTime,
                                                                               bool silence,
                                                                               search::AssemblySequence &sequence)
    {
        std::vector<std::vector<int>> landmarks;
        tbb::tick_count timer = tbb::tick_count::now();

        std::vector<int> startPartIDs = {};
        std::vector<int> endPartIDs = {};
        for(int id = 0; id < beamAssembly->beams_.size(); id++){
            endPartIDs.push_back(id);
        }

        compute_zlandmarks(beamAssembly, maxTime / numLandmarks, numLandmarks, startPartIDs, endPartIDs, landmarks, silence);

        double time = (tbb::tick_count::now() - timer).seconds();
        for(int id = 0; id + 1 < landmarks.size(); id++)
        {
            timer = tbb::tick_count::now();
            std::vector<int> startPartIDs = landmarks[id];
            std::vector<int> endPartIDs = landmarks[id + 1];
            search::AssemblySequence tmp_sequence;
            algorithms::Search search(beamAssembly, numHand, startPartIDs, endPartIDs, silence);
            auto [tmp_time, tmp_compliance] = search.runSearch_Beam(beamWidth, tmp_sequence);
            sequence.steps.insert(sequence.steps.end(), tmp_sequence.steps.begin(), tmp_sequence.steps.end());
            time += tmp_time;
        }

        std::vector<double> compliance_list;
        double compliance = runEvaluation(beamAssembly, sequence, numHand, compliance_list, silence);
        //std::cout << numLandmarks << "-landmark-beam-" << beamWidth << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }
}

#endif //GITIGNORE_OPTIMIZATION_ZLANDMARK_BEAMSEARCH_H
