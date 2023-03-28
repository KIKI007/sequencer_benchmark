//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_SEARCH_BACKTRACKGREEDY_H
#define GITIGNORE_SEARCH_BACKTRACKGREEDY_H

#include "tbb/tick_count.h"
#include "search/SearchAlgorithmGreedy.h"
#include "search/StateGraphMultipleArms.h"
#include "search/PartGraphFrame.h"

namespace benchmark{
    std::tuple<double, double> runSearch_BacktrackGreedy(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                         int numHand,
                                                         double maxtime,
                                                         bool silence,
                                                         search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);

        std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
        searcher->use_max_operator_ = false;
        searcher->maxtime = maxtime;
        searcher->method = search::SearchAlgorithmGreedy::backtrackMethod;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "backtrack-greedy" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }
}

#endif //GITIGNORE_SEARCH_BACKTRACKGREEDY_H
