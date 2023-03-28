//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_SEARCH_FOWARDGREEDY_H
#define GITIGNORE_SEARCH_FOWARDGREEDY_H

#include "tbb/tick_count.h"
#include "search/SearchAlgorithmGreedy.h"
#include "search/StateGraphMultipleArms.h"
#include "search/PartGraphFrame.h"

namespace benchmark
{
    static std::tuple<double, double> runSearch_ForwardGreedy(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                              int numHand,
                                                              bool silence,
                                                              search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);

        std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
        searcher->use_max_operator_ = false;
        searcher->method = search::SearchAlgorithmGreedy::forwardMethod;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "forward-greedy" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }
}


#endif //GITIGNORE_SEARCH_FOWARDGREEDY_H
