//
// Created by 汪子琦 on 03.04.23.
//

#ifndef GITIGNORE_COMPUTEDEFORMATION_H
#define GITIGNORE_COMPUTEDEFORMATION_H

#include "frame/FrameAssembly.h"
#include "search/AssemblySequence.h"
#include "search/InsertionDirection.h"

namespace algorithms
{
    void computeDeformation(std::shared_ptr<frame::FrameAssembly> beamAssembly, search::AssemblySequence &sequence)
    {
        double maxCompliance = 0;
        double maxDisplacement = 0;
        double maxStress = 0;

        std::vector<int> subset_beam_index = {};

        for (int id = 0; id < sequence.steps.size(); id++)
        {

            subset_beam_index.insert(subset_beam_index.end(),
                                     sequence.steps[id].installPartIDs.begin(),
                                     sequence.steps[id].installPartIDs.end());

            std::vector<int> fixed_beams_index = sequence.steps[id].holdPartIDs;

            std::shared_ptr<frame::FrameAssembly> deformAssembly
                    = std::make_shared<frame::FrameAssembly>(*beamAssembly);


            Eigen::VectorXd displacement;
            deformAssembly->displacement_scaling_factor = 1;
            deformAssembly->solveElasticity(subset_beam_index, fixed_beams_index, displacement);
            double compliance = deformAssembly->computeCompliance(displacement, subset_beam_index);
            std::vector<std::vector<Eigen::Vector3d>> segments;
            std::vector<std::vector<double>> deviation;
            deformAssembly->num_deformed_segment_ = 20;
            deformAssembly->visualizeDisplacement(subset_beam_index, displacement, segments, deviation);
            sequence.steps[id].deformPartIDs = subset_beam_index;
            sequence.steps[id].deformLines = segments;
            sequence.steps[id].deformValues = deviation;
            sequence.steps[id].compliance = compliance;
        }
    }
}

#endif //GITIGNORE_COMPUTEDEFORMATION_H
