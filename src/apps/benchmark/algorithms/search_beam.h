//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_SEARCH_BEAM_H
#define GITIGNORE_SEARCH_BEAM_H

#include "tbb/tick_count.h"
#include "search/SearchAlgorithmBeamSearch.h"
#include "search/StateGraphMultipleArms.h"
#include "search/PartGraphFrame.h"

namespace benchmark{
    static std::tuple<double, double> runSearch_Beam(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                     int numHand,
                                                     int beamWidth,
                                                     std::vector<int> startPartIDs,
                                                     std::vector<int> endPartIDs,
                                                     bool silence,
                                                     search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);
        stateGraph->startPartIDs_ = startPartIDs;
        stateGraph->endPartIDs_ = endPartIDs;

        std::shared_ptr<search::SearchAlgorithmBeamSearch> searcher = std::make_shared<search::SearchAlgorithmBeamSearch>(stateGraph, beamWidth, 0.1);
        searcher->use_max_operator_ = false;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);

        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "search-beam-" << beamWidth << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }
}
#endif //GITIGNORE_SEARCH_BEAM_H
