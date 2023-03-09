//
// Created by ziqwang on 04.08.21.
//

#ifndef TIMBER_JOINT_BEAMASSEMBLY_H
#define TIMBER_JOINT_BEAMASSEMBLY_H

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>

//#include "PartGraph.h"
#include "BeamLinearElastic.h"

namespace frame
{

class FrameAssembly
{

public:
    typedef Eigen::SparseMatrix<double> SparseMatrixD;

public:
    int num_deformed_segment_ = 10;

    double displacement_scaling_factor = 1;

    std::vector<Eigen::Vector3d> vertices_;

    std::vector<Eigen::Vector2i> edges_;

    std::vector<Eigen::Vector3d> crosssection_xaxis_;

    std::vector<int> support_vertex_indices_;

    std::vector<BeamMaterial> materials_;

public:
    //beams
    std::vector<std::shared_ptr<Beam>> beams_;

    //elastic beams
    //be aware that the beam elastics are different from the beams_;
    //each joint will divide the beams_ into two beam_elastics_;
    std::vector<std::shared_ptr<BeamLinearElastic>> beams_elasticity_;

    std::vector<std::vector<int>> vertexDofIndices;

    std::vector<std::vector<int>> edgeDofIndices;

public:
    double insignificant_beam_length_;

public:
    FrameAssembly() {
        insignificant_beam_length_ = 0;
    }

    FrameAssembly(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector2i> &edges, const std::vector<int> &support_vertices_indices);

    FrameAssembly(const FrameAssembly &assembly);

    FrameAssembly(const FrameAssembly &assembly, Eigen::Vector3d offset);

public:
    void loadFromOBJ(std::string filename, double scale = 1.0, Eigen::Vector3d offset = Eigen::Vector3d(0, 0, 0));

    virtual void loadFromJson(std::string filename, double scale = 1.0, Eigen::Vector3d offset = Eigen::Vector3d(0, 0, 0));

    void writeToJson(nlohmann::json &json_file);

    void writeToJsonFullMaterials(nlohmann::json &json_file);

    void saveToJson(std::string filename);

    void setMaterial(const BeamMaterial &material) {
        materials_.clear();
        materials_.resize(edges_.size(), material);
    }

public:
    Frame computeFrame(int eID);

    Eigen::Vector3d computeCrossSectionXaxis(Eigen::Vector3d p0, Eigen::Vector3d p1);

    void updateCrossSectionXaxis();

public:
    void update() {
        computeBeams();
        computeBeamElasticity();
    }

    //beam
    void computeBeams();

    //elasticity

    void computeBeamElasticity();

    int computeDoFsMapping(const std::unordered_map<int, bool> &dynamic_vertices,
                           std::unordered_map<int, int> &map_dof_entire2subset,
                           std::unordered_map<int, int> &map_dof_subset2entire);

    void computeDynamicVertices(const std::vector<int> &subset_beams_index,
                                const std::vector<int> &fixture_beam_index,
                                std::unordered_map<int, bool> &dynamic_vertices);

    int computeDoFsMapping(const std::vector<int> &subset_beams_index,
                           const std::vector<int> &fixture_beam_index,
                           std::unordered_map<int, int> &map_dof_entire2subset,
                           std::unordered_map<int, int> &map_dof_subset2entire);

    bool solveElasticity(const std::vector<int> &subset_beams_index,
                         const std::vector<int> &fixture_beam_index,
                         Eigen::VectorXd &displacement);

    void computeStiffMatrix(const std::vector<int> &subset_beams_index,
                            int total_dofs,
                            const std::unordered_map<int, int> &map_dof_entire2subset,
                            const std::unordered_map<int, int> &map_dof_subset2entire,
                            SparseMatrixD &K);

    void computeLoads(const std::vector<int> &subset_beams_index,
                      int total_dofs,
                      const std::unordered_map<int, int> &map_dof_entire2subset,
                      const std::unordered_map<int, int> &map_dof_subset2entire,
                      Eigen::VectorXd &F);


    double computeCompliance(const Eigen::VectorXd &displacement,
                             const std::vector<int> &subset_beams);

    double computeMaxDisplacement(const Eigen::VectorXd &displacement);

    double computeMaxStress(const std::vector<int> &subset_beams_index, const Eigen::VectorXd &displacement, std::vector<Eigen::VectorXd> &internal_forces,
                            std::vector<Eigen::Vector2d> &joint_stresses);

public:

    void assembleStiffMatrix(std::vector<Eigen::Triplet<double>> &K_tri, const Eigen::MatrixXd &k_G, const std::unordered_map<int, int> &map_dof_entire2subset,
                             int edgeID);

    void assemblyForce(Eigen::VectorXd &F, const Eigen::VectorXd &g, const std::unordered_map<int, int> &map_dof_entire2subset, int edgeID);

    std::vector<Eigen::VectorXd> interpolatePolynomial(Eigen::VectorXd D_local, double L);

    void visualizeDisplacement(const std::vector<int> &subset_beams_index,
                               Eigen::VectorXd &displacement,
                               std::vector<std::vector<Eigen::Vector3d>> &segments,
                               std::vector<std::vector<double>> &deviation);

public:

    std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXi>> getMesh(std::vector<int> subset_beams_index = {});

    Eigen::VectorXd getDisplacement(std::vector<int> subset_beams_index,
                                    std::vector<int> fix_beam_index);

    Eigen::VectorXd getGravityForce(std::vector<int> subset_beams_index,
                                    std::vector<int> fix_beam_index);

    Eigen::VectorXd getDisplacementByForce(std::vector<int> subset_beams_index,
                                          std::vector<int> fix_beam_index,
                                          Eigen::VectorXd &F);

    void chooseFrameFEM(bool frame){
        for(auto beam : beams_elasticity_){
            beam->frame_node_ = frame;
        }
    }

    std::vector<Eigen::Vector3d> getLines(std::vector<int> subset_beams_index = {});

    std::vector<std::vector<Eigen::Vector3d>> getDeformedLines(std::vector<int> subset_beams_index = {}, std::vector<int> fix_beam_index = {});

    std::vector<std::vector<Eigen::Vector3d>> getVisualizationLines(std::vector<int> subset_beams_index, Eigen::VectorXd& displacement);

    std::vector<Eigen::Vector3d> getVisualizationJoints(std::vector<int> subset_beams_index, Eigen::VectorXd& displacement);

    std::vector<Eigen::Vector3d> getJoints(std::vector<int> subset_beams_index);

    std::vector<Eigen::Vector3d> getDeformedJoints(std::vector<int> subset_beams_index, std::vector<int> fix_beam_index);

    std::vector<Eigen::Vector3d> getGroundFixtures(std::vector<int> subset_beams_index);

    std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXi>> getDeformedMesh(
        std::vector<int> subset_beams_index = {},
        std::vector<int> fix_beam_index = {});

    Eigen::Vector3d direction(int edgeID) {
        Eigen::Vector3d p1 = vertices_[edges_[edgeID][1]];
        Eigen::Vector3d p0 = vertices_[edges_[edgeID][0]];
        return (p1 - p0).normalized();
    }

    std::tuple<Eigen::Matrix2Xi, std::vector<int>, Eigen::Matrix2Xi, Eigen::Matrix3Xd> computeOffsetBeamConstraintsPairs(double sinEps = 1E-3);

public:

    Eigen::Vector3d computeBeamCentroid(int beamID);

};

}  // namespace frame

#endif  //TIMBER_JOINT_BEAMASSEMBLY_H
