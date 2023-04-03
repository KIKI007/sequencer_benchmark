//
// Created by 汪子琦 on 03.04.23.
//

#ifndef GITIGNORE_SEARCH_H
#define GITIGNORE_SEARCH_H
#include <vector>
#include "frame/FrameAssembly.h"
#include "search/AssemblySequence.h"
namespace algorithms
{
    class Search{
    public:
        Search(std::shared_ptr<frame::FrameAssembly> beamAssembly,
               int numHand,
               const std::vector<int> &startPartIDs,
               const std::vector<int> &endPartIDs,
               bool silence){
            beamAssembly_ = beamAssembly;
            numHand_ = numHand;
            startPartIDs_ = startPartIDs;
            endPartIDs_= endPartIDs;
            silence_ = silence;
        }

    public:

        std::tuple<double, double> runSearch_BackwardGreedy(search::AssemblySequence &sequence);

        std::tuple<double, double> runSearch_ForwardGreedy(search::AssemblySequence &sequence);

        std::tuple<double, double> runSearch_BacktrackGreedy(double maxtime, search::AssemblySequence &sequence);

        std::tuple<double, double> runSearch_Beam(int beamWidth, search::AssemblySequence &sequence);

    private:

        std::shared_ptr<frame::FrameAssembly> beamAssembly_;

        int numHand_;

        std::vector<int> startPartIDs_, endPartIDs_;

        bool silence_;


    };
}



#endif //GITIGNORE_SEARCH_H
