//
// Created by 汪子琦 on 28.03.23.
//

#ifndef GITIGNORE_OPTIMIZATION_ZLANDMARK_H
#define GITIGNORE_OPTIMIZATION_ZLANDMARK_H

#include "dto/FrameTOSequenceSum.h"
#include "dto/FrameTOKnitroSolver.h"

namespace benchmark
{
    void compute_zlandmarks(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                            double maxtime,
                            int numLandmarks,
                            std::vector<std::vector<int>> &landmarks_barIDs)
    {
        std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);

        std::vector<int> start = {};
        std::vector<int> end = frameTO->end_partIDs_;
        int num_bar_per_step = beamAssembly->beams_.size() / (numLandmarks + 1);

        landmarks_barIDs.push_back({});
        for(int step = 1; step < numLandmarks + 1; step++)
        {
            frameTO->setStartnEnd(start, end);
            frameTO->sections_ = {{num_bar_per_step * ((double)step), num_bar_per_step * ((double)step )}};

            frameTO->setParameters(1E-8, 0);
            dto::FrameTOKnitroSolver solver(frameTO);

            solver.mip_tol = 1E-10;
            solver.nlp_tol = 1E-12;
            solver.maxtime = maxtime;
            solver.hessian_provided = true;
            solver.MINLP = true;
            solver.silence = false;

            Eigen::VectorXd x;
            double value = solver.solve(x);
            std::vector<Eigen::VectorXd> rhos;
            frameTO->computeRhos(x.data(), rhos);
            for(int id = 0; id < rhos.size(); id++)
            {
                std::vector<int> lm;
                for(int jd = 0; jd < rhos[id].size(); jd++)
                {
                    if(rhos[id][jd] > 0.5) lm.push_back(jd);
                }
                landmarks_barIDs.push_back(lm);
            }
            start = landmarks_barIDs.back();
        }
        landmarks_barIDs.push_back(end);
    }
}

#endif //GITIGNORE_OPTIMIZATION_ZLANDMARK_H
