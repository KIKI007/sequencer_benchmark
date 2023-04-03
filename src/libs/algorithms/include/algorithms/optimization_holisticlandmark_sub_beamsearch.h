//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_HOLISTICLANDMARK_BEAMSEARCH_H
#define GITIGNORE_OPTIMIZATION_HOLISTICLANDMARK_BEAMSEARCH_H

#include "search.h"
#include "optimization_zlandmark.h"
#include "evaluation.h"

namespace benchmark
{
    static std::tuple<double, double> runOptimization_holisticlandmark_sub_beamsearch(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                                               int numHand,
                                                                               int numLandmarks,
                                                                               int beamWidth,
                                                                               double maxTime,
                                                                               bool silence,
                                                                               search::AssemblySequence &sequence)
    {
        tbb::tick_count timer = tbb::tick_count::now();

        std::vector<int> startPartIDs = {};
        std::vector<int> endPartIDs = {};
        for(int id = 0; id < beamAssembly->beams_.size(); id++){
            endPartIDs.push_back(id);
        }

        int numPart = beamAssembly->beams_.size();
        double numPartPerStep = numPart / (1 +  numLandmarks);

        std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);

        frameTO->setStartnEnd(startPartIDs, endPartIDs);

        frameTO->sections_.clear();
        for(int id = 1; id <= numLandmarks; id++)
        {
            frameTO->sections_.push_back({id * numPartPerStep, id * numPartPerStep});
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

        std::vector<std::vector<int>> landmarks;
        landmarks.push_back(startPartIDs);
        for(int id = 0; id < rhos.size(); id++)
        {
            landmarks.push_back({});
            for(int jd = 0; jd < rhos[id].size(); jd++)
            {
                if(rhos[id][jd] > 0.5){
                    landmarks.back().push_back(jd);
                }
            }
        }
        landmarks.push_back(endPartIDs);
        double time =  computeShortestTimeFindingIncumbentSolution(solver);

        for(int id = 0; id + 1 < landmarks.size(); id++)
        {
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

#endif //GITIGNORE_OPTIMIZATION_HOLISTICLANDMARK_BEAMSEARCH_H
