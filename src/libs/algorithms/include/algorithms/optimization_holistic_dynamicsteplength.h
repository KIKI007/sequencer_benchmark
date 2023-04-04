//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_HOLISTIC_DYNAMICSTEPLENGTH_H
#define GITIGNORE_OPTIMIZATION_HOLISTIC_DYNAMICSTEPLENGTH_H

#include "dto/FrameTORangeSum.h"
#include "dto/FrameTOKnitroSolver.h"
#include "evaluation.h"

namespace algorithms {
    void runOptimization_holistic_dynamicsteplength_return_intermediate_solutions(
            std::shared_ptr<frame::FrameAssembly> beamAssembly,
            int numHand,
            int numStep,
            double maxTime,
            std::vector<int> startPartIDs,
            std::vector<int> endPartIDs,
            bool silence,
            search::AssemblySequence &sequence,
            std::vector<double> &time,
            std::vector<double> &c,
            std::vector<double> &lb) {
        std::shared_ptr<dto::FrameTORangeSum> frameTO = std::make_shared<dto::FrameTORangeSum>(*beamAssembly);

        frameTO->setStartnEnd(startPartIDs, endPartIDs);

        frameTO->sections_ = {{startPartIDs.size(), endPartIDs.size()}};
        frameTO->num_steps_ = {numStep};
        frameTO->num_arm_ = numHand;

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

        c = solver.data.compliance;
        lb = solver.data.low_bnd;
        time = solver.data.time;

        return;
    }

    static std::tuple<double, double>
    runOptimization_holistic_dynamicsteplength(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                               int numHand,
                                               int numStep,
                                               double maxTime,
                                               std::vector<int> startPartIDs,
                                               std::vector<int> endPartIDs,
                                               bool silence,
                                               search::AssemblySequence &sequence) {

        std::shared_ptr<dto::FrameTORangeSum> frameTO = std::make_shared<dto::FrameTORangeSum>(*beamAssembly);

        frameTO->setStartnEnd(startPartIDs, endPartIDs);

        frameTO->sections_ = {{startPartIDs.size(), endPartIDs.size()}};
        frameTO->num_steps_ = {numStep};
        frameTO->num_arm_ = numHand;

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
        double time = computeShortestTimeFindingIncumbentSolution(solver);
        return {time, value};
    }
}

#endif //GITIGNORE_OPTIMIZATION_HOLISTIC_DYNAMICSTEPLENGTH_H
