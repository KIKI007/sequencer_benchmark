//
// Created by 汪子琦 on 03.04.23.
//

#include "algorithms/search.h"
#include "search/StateGraphMultipleArms.h"
#include "search/SearchAlgorithmGreedy.h"
#include "search/PartGraphFrame.h"
#include "search/SearchAlgorithmBeamSearch.h"

std::tuple<double, double> algorithms::Search::runSearch_ForwardGreedy(search::AssemblySequence &sequence)
{
    std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly_);
    partGraph->use_displacement_ = false;

    std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand_);

    std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
    searcher->use_max_operator_ = false;
    searcher->method = search::SearchAlgorithmGreedy::forwardMethod;

    tbb::tick_count timer = tbb::tick_count::now();
    double compliance = searcher->search(sequence);
    double time = (tbb::tick_count::now() - timer).seconds();
    if(!silence_) std::cout << "forward-greedy" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
    return {time, compliance};
}

std::tuple<double, double> algorithms::Search::runSearch_BackwardGreedy(search::AssemblySequence &sequence) {
    std::shared_ptr <search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly_);
    partGraph->use_displacement_ = false;

    std::shared_ptr <search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(
            partGraph, numHand_);

    std::shared_ptr <search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(
            stateGraph);
    searcher->use_max_operator_ = false;
    searcher->method = search::SearchAlgorithmGreedy::backwardMethod;

    tbb::tick_count timer = tbb::tick_count::now();
    double compliance = searcher->search(sequence);
    double time = (tbb::tick_count::now() - timer).seconds();
    if (!silence_)
    {
        std::cout << "backward-greedy" << ", " << time << ", " << compliance << ", " << compliance * time << std::endl;
    }
    return {time, compliance};
}

std::tuple<double, double>
algorithms::Search::runSearch_BacktrackGreedy(double maxtime, search::AssemblySequence &sequence)
{
    std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly_);
    partGraph->use_displacement_ = false;

    std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand_);

    std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
    searcher->use_max_operator_ = false;
    searcher->maxtime = maxtime;
    searcher->method = search::SearchAlgorithmGreedy::backtrackMethod;

    tbb::tick_count timer = tbb::tick_count::now();
    double compliance = searcher->search(sequence);
    double time = (tbb::tick_count::now() - timer).seconds();
    if(!silence_) std::cout << "backtrack-greedy" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
    return {time, compliance};
}

std::tuple<double, double> algorithms::Search::runSearch_Beam(int beamWidth, search::AssemblySequence &sequence)
{
    std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly_);
    partGraph->use_displacement_ = false;

    std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand_);
    stateGraph->startPartIDs_ = startPartIDs_;
    stateGraph->endPartIDs_ = endPartIDs_;

    std::shared_ptr<search::SearchAlgorithmBeamSearch> searcher = std::make_shared<search::SearchAlgorithmBeamSearch>(stateGraph, beamWidth, 0.1);
    searcher->use_max_operator_ = false;

    tbb::tick_count timer = tbb::tick_count::now();
    double compliance = searcher->search(sequence);

    double time = (tbb::tick_count::now() - timer).seconds();
    if(!silence_) std::cout << "search-beam-" << beamWidth << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
    return {time, compliance};
}
