//
// Created by 汪子琦 on 14.12.22.
//

#ifndef PAVILLION_JSON_INSERTIONDIRECTION_H
#define PAVILLION_JSON_INSERTIONDIRECTION_H
#include "AssemblySequence.h"
#include "frame/FrameAssembly.h"
namespace search
{
class InsertionDirection
{
public:

    std::shared_ptr<frame::FrameAssembly> assembly_;
    AssemblySequence sequence_;

    std::vector<std::vector<Eigen::Vector3d>> drts_;

    double length_ = 1.0;

    int num_segment_ = 10;

    int num_sphere_sample = 10;

public:

    InsertionDirection(){

    }

public:

    double distance(int currBeamIndex,
                  Eigen::Vector3d curr_drt,
                  std::vector<int> fixedBeamIndices,
                  std::vector<int> installingBeamIndices,
                  const std::vector<Eigen::Vector3d> &installing_drts);

    double distance(int beamA, Eigen::Vector3d drtA,
                    int beamB, Eigen::Vector3d drtB);

    double distance(Eigen::Vector3d a0, Eigen::Vector3d a1, Eigen::Vector3d b0, Eigen::Vector3d b1);

    void samplingSphere(std::vector<Eigen::Vector3d> &sample_drts);

    void compute();
};
}


#endif  //PAVILLION_JSON_INSERTIONDIRECTION_H
