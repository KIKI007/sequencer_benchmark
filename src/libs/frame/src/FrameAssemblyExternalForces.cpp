//
// Created by 汪子琦 on 23.09.22.
//

#include "frame/FrameAssemblyExternalForces.h"

namespace frame
{

FrameAssemblyExternalForces::FrameAssemblyExternalForces(const frame::FrameAssembly& assembly) : FrameAssembly(assembly)
{

}

FrameAssemblyExternalForces::FrameAssemblyExternalForces(const frame::FrameAssemblyExternalForces& assembly) : FrameAssembly(assembly)
{
    external_force_ = assembly.external_force_;
}


void FrameAssemblyExternalForces::solveElasticity(const std::vector<int>& subset_beams_index,
                                                  const std::vector<int> &fix_beam_index,
                                                  Eigen::VectorXd& displacement) {
    std::unordered_map<int, int> map_dof_entire2subset;
    std::unordered_map<int, int> map_dof_subset2entire;
    int total_dofs = computeDoFsMapping(subset_beams_index, fix_beam_index, map_dof_entire2subset, map_dof_subset2entire);

    SparseMatrixD K_rho;
    computeStiffMatrix(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, K_rho);

    Eigen::VectorXd F;
    computeLoads(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, F);

    for(int id = 0; id < external_force_.size(); id++){
        int old_dof = id;
        auto find_it = map_dof_entire2subset.find(old_dof);
        if(find_it != map_dof_entire2subset.end()){
            int new_dof = find_it->second;
            F(new_dof) += external_force_[id];
        }
    }

    Eigen::VectorXd D;

    SparseMatrixD K = K_rho;

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if (llt.info() == Eigen::Success) {
        D = llt.solve(F);
        displacement = Eigen::VectorXd::Zero(vertices_.size() * 6);
        for (int id = 0; id < D.rows(); id++) {
            int old_dof = map_dof_subset2entire[id];
            displacement[old_dof] = D[id];
        }
    }
}

void FrameAssemblyExternalForces::loadFromJson(std::string filename, double scale, Eigen::Vector3d offset) {
    FrameAssembly::loadFromJson(filename, scale, offset);

    std::ifstream fin(filename);
    nlohmann::json json_file;
    fin >> json_file;

    external_force_ = Eigen::VectorXd(vertices_.size() * 6);
    external_force_.setZero();
    for (int id = 0; id < json_file["nodes"].size(); id++)
    {
        auto node = json_file["nodes"][id];
        if(node.contains("force")){
            auto force = node["force"];
            std::vector<double> force_data = force.get<std::vector<double>>();
            external_force_.segment(id * 6, 3) = Eigen::Vector3d(force_data[1], force_data[2], force_data[0]);
        }
    }

    fin.close();

}
void FrameAssemblyExternalForces::writeToJson(nlohmann::json& json_file, std::vector<int> subset_beams)
{
    std::unordered_map<int, int> map_old_new;
    int num_new_vertiecs = 0;
    for(int id = 0; id < vertices_.size(); id++)
    {
        bool included = false;
        for(int jd = 0; jd < subset_beams.size(); jd++)
        {
            int edgeID = subset_beams[jd];
            int v0 = edges_[edgeID][0];
            int v1 = edges_[edgeID][1];
            if(id == v0 || id == v1){
                included = true;
                break ;
            }
        }
        if(included){
            map_old_new[id] = num_new_vertiecs;
            num_new_vertiecs++;
        }
        else{
            map_old_new[id] = -1;
        }
    }

    std::vector<int> vertex_grounded;
    vertex_grounded.resize(vertices_.size(), false);
    for (int id = 0; id < support_vertex_indices_.size(); id++) {
        vertex_grounded[support_vertex_indices_[id]] = true;
    }

    //nodes
    nlohmann::json json_nodes = nlohmann::json::array();
    for (int id = 0; id < vertices_.size(); id++)
    {
        if(map_old_new[id] != -1)
        {
            nlohmann::json json_node;
            json_node["point"] = {vertices_[id][2], vertices_[id][0], vertices_[id][1]};
            json_node["node_ind"] = map_old_new[id];
            json_node["is_grounded"] = vertex_grounded[id];
            Eigen::Vector3d force = external_force_.segment(id * 6, 3);
            if(force.norm() > 1E-6) json_node["forces"] = {force[0], force[1], force[2]};
            json_nodes.push_back(json_node);
        }
    }
    json_file["nodes"] = json_nodes;

    //edges
    nlohmann::json json_elements = nlohmann::json::array();
    for (int id = 0; id < subset_beams.size(); id++)
    {
        int edgeID = subset_beams[id];
        int v0 = map_old_new[edges_[edgeID][0]];
        int v1 = map_old_new[edges_[edgeID][1]];
        nlohmann::json json_element;
        json_element["end_node_inds"] = {v0, v1};
        json_elements.push_back(json_element);
    }
    json_file["elements"] = json_elements;

    if (!materials_.empty())
    {
        BeamMaterial material = materials_.front();

        //cross section
        nlohmann::json json_crosssections = nlohmann::json::array();
        {
            nlohmann::json json_crossection;
            json_crossection["A"] = material.Ax;
            json_crosssections.push_back(json_crossection);
        }
        json_file["cross_secs"] = json_crosssections;

        //material
        nlohmann::json json_materials = nlohmann::json::array();
        {
            nlohmann::json json_material;

            double mu = material.mu;
            double E = material.E;

            json_material["E"] = E;
            json_material["G12"] = E / (2 * (1 + mu));
            json_material["density"] = material.rho;

            json_materials.push_back(json_material);
        }
        json_file["materials"] = json_materials;
    }
}

void FrameAssemblyExternalForces::saveToJson(std::string filename, std::vector<int> subset_beams) {
    nlohmann::json json_output;
    writeToJson(json_output, subset_beams);
    std::ofstream fout(filename.c_str());
    fout << json_output;
    fout.close();
}

}

