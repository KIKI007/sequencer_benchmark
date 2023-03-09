//
// Created by 汪子琦 on 14.12.22.
//

#include "search/InsertionDirection.h"
double search::InsertionDirection::distance(int currBeamIndex,
                                            Eigen::Vector3d curr_drt,
                                            std::vector<int> fixedBeamIndices,
                                            std::vector<int> installingBeamIndices,
                                            const std::vector<Eigen::Vector3d>& installing_drts)
{
    std::vector<int> baseIndex;
    std::vector<Eigen::Vector3d> baseDrts;
    for(int id = 0; id < fixedBeamIndices.size(); id++){
        baseIndex.push_back(fixedBeamIndices[id]);
        baseDrts.push_back(Eigen::Vector3d(0, 0 ,0));
    }

    for(int id = 0; id < installingBeamIndices.size(); id++){
        baseIndex.push_back(installingBeamIndices[id]);
        baseDrts.push_back(installing_drts[id]);
    }

    double result = 0.0;
    for(int jd = 0; jd <= num_segment_; jd++)
    {
        double min_dist = std::numeric_limits<double>::max();
        double scale = (double)jd / num_segment_ * length_;
        for(int id = 0; id < baseIndex.size(); id++)
        {
            double dist = distance(currBeamIndex, curr_drt * scale,
                                   baseIndex[id], baseDrts[id] * scale);
            min_dist = std::min(dist, min_dist);
        }
        result += min_dist;
    }
    return result;
}
double search::InsertionDirection::distance(int beamA, Eigen::Vector3d drtA, int beamB, Eigen::Vector3d drtB) {
    Eigen::Vector3d ptA0 = assembly_->beams_[beamA]->frame_.origin + drtA;
    Eigen::Vector3d ptA1 = assembly_->beams_[beamA]->frame_.origin
                           + assembly_->beams_[beamA]->frame_.zaxis * assembly_->beams_[beamA]->beam_length_ + drtA;

    if(ptA0.y() < -1E-4 || ptA1.y() < -1E-4)
        return std::numeric_limits<double>::lowest();

    Eigen::Vector3d ptB0 = assembly_->beams_[beamB]->frame_.origin + drtB;
    Eigen::Vector3d ptB1 = assembly_->beams_[beamB]->frame_.origin
                           + assembly_->beams_[beamB]->frame_.zaxis * assembly_->beams_[beamB]->beam_length_ + drtB;

    return distance(ptA0, ptA1, ptB0, ptB1);
}

void search::InsertionDirection::compute()
{
    std::vector<int> fixedBeamIndex = {};

    std::vector<Eigen::Vector3d> sample_drts;
    samplingSphere(sample_drts);
    bool first_element = true;

    for(int istep = 0; istep < sequence_.steps.size(); istep++)
    {
        std::vector<int> installingBeamIndex;
        std::vector<Eigen::Vector3d> installingBeamDrts;
        for(int id = 0; id < sequence_.steps[istep].installPartIDs.size(); id++)
        {
            int partID = sequence_.steps[istep].installPartIDs[id];
            double max_dist = 0.0;
            int max_index = -1;

            if(first_element){
                installingBeamDrts.push_back(Eigen::Vector3d(0, 0, 1));
                first_element = false;
            }
            else
            {
                for(int jd = 0; jd < sample_drts.size(); jd++)
                {
                    double dist = distance(partID, sample_drts[jd], fixedBeamIndex, installingBeamIndex, installingBeamDrts);
                    if(max_dist < dist){
                        max_index = jd;
                        max_dist = dist;
                    }
                }
                if(max_index == -1){
                    installingBeamDrts.push_back(Eigen::Vector3d(0, 0, 1));
                }
                else{
                    installingBeamDrts.push_back(sample_drts[max_index]);
                }
            }
            installingBeamIndex.push_back(partID);
        }

        fixedBeamIndex.insert(fixedBeamIndex.end(), installingBeamIndex.begin(), installingBeamIndex.end());
        drts_.push_back(installingBeamDrts);
    }
}
double search::InsertionDirection::distance(Eigen::Vector3d a0, Eigen::Vector3d a1, Eigen::Vector3d b0, Eigen::Vector3d b1)
{
    Eigen::Vector3d r = b0 - a0;
    Eigen::Vector3d u = a1 - a0;
    Eigen::Vector3d v = b1 - b0;
    double ru = r.dot(u);
    double rv = r.dot(v);
    double uu = u.dot(u);
    double uv = u.dot(v);
    double vv = v.dot(v);
    double det =  uu*vv - uv*uv;
    double s = 0, t = 0;
    const double eta = 1E-6;
    if(det < eta*uu*vv)
    {
        s = std::clamp(ru/uu, 0.0, 1.0);
        t = 0;
    }
    else{
        s = std::clamp((ru*vv - rv*uv)/det, 0.0, 1.0);
        t = std::clamp((ru*uv - rv*uu)/det, 0.0, 1.0);
    }

    double S = std::clamp((t*uv + ru)/uu, 0.0, 1.0);
    double T = std::clamp((s*uv - rv)/vv, 0.0, 1.0);

    Eigen::Vector3d A = a0 + S*u;
    Eigen::Vector3d B = b0 + T*v;
    return (A - B).norm();
}

void search::InsertionDirection::samplingSphere(std::vector<Eigen::Vector3d>& sample_drts)
{
    sample_drts.clear();
    for(int jd = 2; jd + 1 < num_sphere_sample; jd++)
    {
        for(int id = 0; id <= num_sphere_sample; id++) {
            double theta = 2.0 * M_PI / num_sphere_sample * id;
            double phi = 1.0 * M_PI / num_sphere_sample * jd;
            sample_drts.push_back({std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi)});
        }
    }
}
