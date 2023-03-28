//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATIOM_HOLISTIC_FIXEDSTEPLENGTH_H
#define GITIGNORE_OPTIMIZATIOM_HOLISTIC_FIXEDSTEPLENGTH_H

#include "dto/FrameTOSequenceSum.h"
#include "dto/FrameTOKnitroSolver.h"
#include "util/evaluation.h"

namespace benchmark
{
    static std::tuple<double, double> runOptimization_holistic_fixedsteplength(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                                        int numHand,
                                                                        double maxTime,
                                                                        std::vector<int> startPartIDs,
                                                                        std::vector<int> endPartIDs,
                                                                        bool silence,
                                                                        search::AssemblySequence &sequence)
    {

        std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);

        frameTO->setStartnEnd(startPartIDs, endPartIDs);

        frameTO->sections_.clear();
        for(int id = startPartIDs.size() + numHand; id < endPartIDs.size(); id += numHand)
        {
            frameTO->sections_.push_back({id, id});
        }

        frameTO->setParameters(1E-8, 0);
        dto::FrameTOKnitroSolver solver(frameTO);

        solver.mip_tol = 1E-8;
        solver.nlp_tol = 1E-10;
        solver.maxtime = maxTime;
        solver.hessian_provided = true;
        solver.MINLP = true;
        solver.silence = silence;
        Eigen::VectorXd x;
        solver.solve(x);
        std::vector<Eigen::VectorXd> rhos;
        frameTO->computeRhos(x.data(), rhos);
        double value = computeOptimizationSequence(beamAssembly, startPartIDs, endPartIDs, rhos, sequence);
        double time =  computeShortestTimeFindingIncumbentSolution(solver);
        return {time, value};
    }
}

#endif //GITIGNORE_OPTIMIZATIOM_HOLISTIC_FIXEDSTEPLENGTH_H
