//
// Created by ziqwang on 04.08.21.
//

#include "frame/FrameAssembly.h"

namespace frame {

FrameAssembly::FrameAssembly(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector2i> &edges,
                           const std::vector<int> &support_vertices_indices) {
    vertices_ = vertices;
    edges_ = edges;
    support_vertex_indices_ = support_vertices_indices;
    insignificant_beam_length_ = 0;
    for (int id = 0; id < edges_.size(); id++) {
        int node0 = edges_[id][0];
        int node1 = edges_[id][1];
        crosssection_xaxis_.push_back(computeCrossSectionXaxis(vertices_[node1], vertices_[node0]));
    }
}

FrameAssembly::FrameAssembly(const FrameAssembly &assembly) {
    vertices_ = assembly.vertices_;
    edges_ = assembly.edges_;
    crosssection_xaxis_ = assembly.crosssection_xaxis_;
    support_vertex_indices_ = assembly.support_vertex_indices_;
    materials_ = assembly.materials_;
    insignificant_beam_length_ = assembly.insignificant_beam_length_;
    computeBeams();
    computeBeamElasticity();
}

FrameAssembly::FrameAssembly(const FrameAssembly &assembly, Eigen::Vector3d offset) {
    vertices_ = assembly.vertices_;
    for (int id = 0; id < vertices_.size(); id++) {
        vertices_[id] += offset;
    }
    edges_ = assembly.edges_;
    crosssection_xaxis_ = assembly.crosssection_xaxis_;
    support_vertex_indices_ = assembly.support_vertex_indices_;
    insignificant_beam_length_ = assembly.insignificant_beam_length_;
    materials_ = assembly.materials_;
    computeBeams();
    computeBeamElasticity();
}

void FrameAssembly::updateCrossSectionXaxis() {
    crosssection_xaxis_.clear();
    for (int id = 0; id < edges_.size(); id++) {
        int node0 = edges_[id][0];
        int node1 = edges_[id][1];
        crosssection_xaxis_.push_back(computeCrossSectionXaxis(vertices_[node1], vertices_[node0]));
    }
}

void FrameAssembly::loadFromOBJ(std::string filename, double scale, Eigen::Vector3d offset) {
    support_vertex_indices_.clear();

    std::ifstream fin(filename);

    char dummy;
    int num_part = 0;
    BeamMaterial material;

    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line))
        {
            switch (line[0]) {
                case 'w': {
                    sscanf(line.c_str(), "%c %lf", &dummy, &material.width);
                    material.width *= scale;
                    break;
                }
                case 'c': {
                    sscanf(line.c_str(), "%c %d", &dummy, &material.type);
                    break;
                }
                case 'p': {
                    sscanf(line.c_str(), "%c %lf %lf %lf", &dummy, &material.E, &material.mu, &material.rho);
                    break;
                }
                case 'v': {
                    double pt_x, pt_y, pt_z;
                    sscanf(line.c_str(), "%c %lf %lf %lf", &dummy, &pt_x, &pt_y, &pt_z);
                    Eigen::Vector3d pt(pt_x, pt_y, pt_z);
                    pt *= scale;
                    pt += offset;
                    vertices_.push_back(pt);
                    break;
                }
                case 'e': {
                    int end0, end1;
                    double cx, cy, cz;
                    sscanf(line.c_str(), "%c %d %d %lf %lf %lf", &dummy, &end0, &end1, &cx, &cy, &cz);
                    edges_.push_back(Eigen::Vector2i(end0, end1));
                    crosssection_xaxis_.push_back(Eigen::Vector3d(cx, cy, cz));
                    break;
                }
                case 'g': {
                    int ground_v;
                    sscanf(line.c_str(), "%c %d", &dummy, &ground_v);
                    support_vertex_indices_.push_back(ground_v);
                    break;
                }
            }
        }
        fin.close();
    }

    material.computeProfile();
    setMaterial(material);

    computeBeams();
    computeBeamElasticity();
}

void FrameAssembly::saveToJson(std::string filename) {
    nlohmann::json json_output;
    writeToJson(json_output);
    std::ofstream fout(filename.c_str());
    fout << json_output;
    fout.close();
}

void FrameAssembly::loadFromJson(std::string filename, double scale, Eigen::Vector3d offset)
{

    vertices_.clear();
    edges_.clear();
    support_vertex_indices_.clear();
    crosssection_xaxis_.clear();

    std::ifstream fin(filename);
    nlohmann::json json_file;
    fin >> json_file;

    vertices_.resize(json_file["nodes"].size());

    for (int id = 0; id < json_file["nodes"].size(); id++)
    {
        auto node = json_file["nodes"][id];
        auto point = node["point"];

        double x, y, z;
        if (point.contains("X")) {
            x = point["X"].get<double>() * scale;
            y = point["Y"].get<double>() * scale;
            z = point["Z"].get<double>() * scale;
        } else {
            x = point[0].get<double>() * scale;
            y = point[1].get<double>() * scale;
            z = point[2].get<double>() * scale;
        }

        Eigen::Vector3d pt(y, z, x);

        //std::cout << node << std::endl;
        int node_ind = id;
        if (node.contains("node_ind")) {
            node_ind = node["node_ind"].get<int>();
        }

        if (node.contains("node_id")) {
            node_ind = node["node_id"].get<int>();
        }

        vertices_[node_ind] = pt + offset;

        bool grounded = node["is_grounded"].get<int>();
        if (grounded) {
            support_vertex_indices_.push_back(id);
        }
    }

    for (int id = 0; id < json_file["elements"].size(); id++) {
        nlohmann::basic_json end_node_inds;
        if (json_file["elements"][id].contains("end_node_ids")) {
            end_node_inds = json_file["elements"][id]["end_node_ids"];
        }
        if (json_file["elements"][id].contains("end_node_inds")) {
            end_node_inds = json_file["elements"][id]["end_node_inds"];
        }

        int node0 = end_node_inds[0].get<int>();
        int node1 = end_node_inds[1].get<int>();
        edges_.push_back(Eigen::Vector2i(node0, node1));

        crosssection_xaxis_.push_back(computeCrossSectionXaxis(vertices_[node1], vertices_[node0]));
    }

    //cross section
    materials_.clear();

    for(int id = 0; id < json_file["cross_secs"].size() && id < json_file["materials"].size(); id++)
    {
        BeamMaterial material;
        material.type = ROUND;
        auto crosssection = json_file["cross_secs"][id];
        if(crosssection.contains("type"))
        {
            material.type = CrossSectionType(crosssection["type"].get<int>());
        }

        double A = crosssection["A"].get<double>();
        if(material.type == ROUND){
            material.width = std::sqrt(A / M_PI) * scale;
            material.Ax = A;
        }
        else{
            material.Ax = A;
        }

        if(crosssection.contains("radius")){
            material.width = crosssection["radius"].get<double>();
        }

        bool recompute_profile = true;
        if(crosssection.contains("Jx"))
        {
            material.Jxx = crosssection["Jx"].get<double>();
            recompute_profile = false;
        }
        if(crosssection.contains("Iy"))
        {
            material.Iyy = crosssection["Iy"].get<double>();
            recompute_profile = false;
        }
        if(crosssection.contains("Iz")){
            material.Izz = crosssection["Iz"].get<double>();
            recompute_profile = false;
        }

        auto json_materials = json_file["materials"][id];
        material.E = json_materials["E"].get<double>();
        double G = json_materials["G12"].get<double>();
        material.mu = material.E / (2 * G) - 1;
        material.rho = json_materials["density"].get<double>() ;

        if(recompute_profile){
            material.computeProfile();
        }

        materials_.push_back(material);
    }

    fin.close();

    BeamMaterial material = materials_.back();
    for(int id = materials_.size(); id < edges_.size(); id++){
        materials_.push_back(material);
    }

    computeBeams();
    computeBeamElasticity();
}

Eigen::Vector3d FrameAssembly::computeCrossSectionXaxis(Eigen::Vector3d p0, Eigen::Vector3d p1) {
    Eigen::Vector3d zaxis = p0 - p1;
    Eigen::Vector3d xaxis = Eigen::Vector3d(1, 0, 0).cross(zaxis);
    if (xaxis.norm() < FLOAT_ERROR_LARGE) {
        xaxis = Eigen::Vector3d(0, 1, 0).cross(zaxis);
    }
    xaxis.normalize();
    return xaxis;
}

void FrameAssembly::writeToJsonFullMaterials(nlohmann::json &json_file) {
    std::vector<int> vertex_grounded;
    vertex_grounded.resize(vertices_.size(), false);
    for (int id = 0; id < support_vertex_indices_.size(); id++) {
        vertex_grounded[support_vertex_indices_[id]] = true;
    }

    //nodes
    nlohmann::json json_nodes = nlohmann::json::array();
    for (int id = 0; id < vertices_.size(); id++) {
        nlohmann::json json_node;
        json_node["point"] = {vertices_[id][2], vertices_[id][0], vertices_[id][1]};
        json_node["node_ind"] = id;
        json_node["is_grounded"] = vertex_grounded[id];
        json_nodes.push_back(json_node);
    }
    json_file["nodes"] = json_nodes;

    //edges
    nlohmann::json json_elements = nlohmann::json::array();
    for (int id = 0; id < edges_.size(); id++) {
        nlohmann::json json_element;
        json_element["end_node_inds"] = {edges_[id][0], edges_[id][1]};
        json_elements.push_back(json_element);
    }
    json_file["elements"] = json_elements;

    nlohmann::json json_crosssections = nlohmann::json::array();
    nlohmann::json json_materials = nlohmann::json::array();

    for(int id = 0; id < beams_elasticity_.size(); id++)
    {
        BeamMaterial material = beams_elasticity_[id]->material_;

        //cross section
        {
            nlohmann::json json_crossection;
            json_crossection["A"] = material.Ax;
            json_crossection["radius"] = material.width;
            json_crossection["Jx"] = material.Jxx;
            json_crossection["Iy"] = material.Iyy;
            json_crossection["Iz"] = material.Izz;
            json_crosssections.push_back(json_crossection);
        }

        //material
        {
            nlohmann::json json_material;

            double mu = material.mu;
            double E = material.E;

            json_material["E"] = E;
            json_material["G12"] = E / (2 * (1 + mu));
            json_material["density"] = material.rho;

            json_materials.push_back(json_material);
        }
    }

    json_file["cross_secs"] = json_crosssections;
    json_file["materials"] = json_materials;

}

void FrameAssembly::writeToJson(nlohmann::json &json_file) {
    std::vector<int> vertex_grounded;
    vertex_grounded.resize(vertices_.size(), false);
    for (int id = 0; id < support_vertex_indices_.size(); id++) {
        vertex_grounded[support_vertex_indices_[id]] = true;
    }

    //nodes
    nlohmann::json json_nodes = nlohmann::json::array();
    for (int id = 0; id < vertices_.size(); id++) {
        nlohmann::json json_node;
        json_node["point"] = {vertices_[id][2], vertices_[id][0], vertices_[id][1]};
        json_node["node_ind"] = id;
        json_node["is_grounded"] = vertex_grounded[id];
        json_nodes.push_back(json_node);
    }
    json_file["nodes"] = json_nodes;

    //edges
    nlohmann::json json_elements = nlohmann::json::array();
    for (int id = 0; id < edges_.size(); id++) {
        nlohmann::json json_element;
        json_element["end_node_inds"] = {edges_[id][0], edges_[id][1]};
        json_elements.push_back(json_element);
    }
    json_file["elements"] = json_elements;

    if (!materials_.empty()) {
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

Frame FrameAssembly::computeFrame(int eID) {
    Frame frame;
    if (eID >= 0 && eID < edges_.size()) {
        Eigen::Vector3d end0 = vertices_[edges_[eID][0]];
        Eigen::Vector3d end1 = vertices_[edges_[eID][1]];

        Eigen::Vector3d zaxis = end1 - end0;
        Eigen::Vector3d xaxis = crosssection_xaxis_[eID];

        zaxis.normalize();
        xaxis.normalize();

        Eigen::Vector3d yaxis = zaxis.cross(xaxis);
        yaxis.normalize();

        frame = Frame(end0, xaxis, yaxis, zaxis);
    }
    return frame;
}

void FrameAssembly::computeBeams() {
    beams_.clear();
    for (int id = 0; id < edges_.size(); id++) {
        Frame frame = computeFrame(id);
        double L = (vertices_[edges_[id][1]] - vertices_[edges_[id][0]]).norm();
        std::shared_ptr<Beam> beam;
        switch (materials_[id].type) {
            case ROUND:
                beam = std::make_shared<BeamRound>(frame, L, materials_[id].width);
                break;
            case SQUARE:
                beam = std::make_shared<BeamSquare>(frame, L, materials_[id].width);
                break;
            default:
                beam = std::make_shared<BeamRound>(frame, L, materials_[id].width);
                break;
        }
        beam->partID_ = id;
        beams_.push_back(beam);
    }
}

void FrameAssembly::computeBeamElasticity() {
    beams_elasticity_.clear();
    vertexDofIndices.clear();
    edgeDofIndices.clear();

    //vertex degree of freedom
    for (int id = 0; id < vertices_.size(); id++) {
        vertexDofIndices.push_back(std::vector<int>());
        for (int kd = 0; kd < 6; kd++) {
            vertexDofIndices.back().push_back(id * 6 + kd);
        }
    }

    for (int id = 0; id < beams_.size(); id++) {
        BeamMaterial material = materials_[id];

        std::shared_ptr<BeamLinearElastic> beamElastic = std::make_shared<BeamLinearElastic>(*beams_[id], material);
        beams_elasticity_.push_back(beamElastic);

        edgeDofIndices.push_back(std::vector<int>());
        std::vector<int> &edgeDof = edgeDofIndices.back();
        for (int kd = 0; kd < 2; kd++) {
            int vID = edges_[id][kd];
            edgeDof.insert(edgeDof.end(), vertexDofIndices[vID].begin(), vertexDofIndices[vID].end());
        }
    }
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXi>> FrameAssembly::getMesh(std::vector<int> subset_beams_index) {
    std::vector<Eigen::MatrixXd> Vs;
    std::vector<Eigen::MatrixXi> Fs;
    for (int id = 0; id < subset_beams_index.size(); id++) {
        int beamIndex = subset_beams_index[id];
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        beams_[beamIndex]->getMesh(V, F);
        Vs.push_back(V);
        Fs.push_back(F);
    }
    return std::make_tuple(Vs, Fs);
}



std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXi>> FrameAssembly::getDeformedMesh(std::vector<int> subset_beams_index,
                                                                                                      std::vector<int> fix_beams_index)
{
    int num_circle_sample = 16;

    Eigen::VectorXd displacement;
    solveElasticity(subset_beams_index, fix_beams_index, displacement);

    std::vector<std::vector<Eigen::Vector3d>> segments;
    std::vector<std::vector<double>> deviation;
    visualizeDisplacement(subset_beams_index, displacement, segments, deviation);

    std::vector<Eigen::MatrixXd> Vs;
    std::vector<Eigen::MatrixXi> Fs;

    for (int id = 0; id < segments.size(); id++) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<Eigen::Vector3i> tris;
        for (int jd = 0; jd < (int)segments[id].size() - 1; jd++) {
            Eigen::Vector3d endu = segments[id][jd];
            Eigen::Vector3d endv = segments[id][jd + 1];

            Eigen::Vector3d zaxis = endv - endu;
            zaxis.normalize();

            Eigen::Vector3d xaxis = Eigen::Vector3d(1, 0, 0).cross(zaxis);
            if (xaxis.norm() < FLOAT_ERROR_LARGE) {
                xaxis = Eigen::Vector3d(0, 1, 0).cross(zaxis);
            }
            xaxis.normalize();
            Eigen::Vector3d yaxis = zaxis.cross(xaxis);

            Eigen::MatrixXd V(2 * num_circle_sample, 3);
            Eigen::MatrixXi F(2 * num_circle_sample, 3);

            for (int kd = 0; kd < num_circle_sample; kd++) {
                double angle = (double)kd / num_circle_sample * M_PI * 2;
                Eigen::Vector3d pt = endu + (cos(angle) * xaxis + sin(angle) * yaxis) * beams_elasticity_[subset_beams_index[id]]->beam_width_ / 2;
                V.row(kd) = pt;
                V.row(kd + num_circle_sample) = pt + (endv - endu);
            }

            for (int kd = 0; kd < num_circle_sample; kd++) {
                int next_kd = (kd + 1) % num_circle_sample;
                F.row(2 * kd) = Eigen::Vector3i(kd, next_kd, next_kd + num_circle_sample);
                F.row(2 * kd + 1) = Eigen::Vector3i(kd, next_kd + num_circle_sample, kd + num_circle_sample);
            }

            int nV = pts.size();
            for (int id = 0; id < V.rows(); id++) {
                pts.push_back(V.row(id));
            }
            for (int id = 0; id < F.rows(); id++) {
                tris.push_back(Eigen::Vector3i(F(id, 0) + nV, F(id, 1) + nV, F(id, 2) + nV));
            }
        }
        Eigen::MatrixXd V(pts.size(), 3);
        Eigen::MatrixXi F(tris.size(), 3);
        for (int id = 0; id < pts.size(); id++) {
            V.row(id) = pts[id];
        }
        for (int id = 0; id < tris.size(); id++) {
            F.row(id) = tris[id];
        }
        Vs.push_back(V);
        Fs.push_back(F);
    }

    return std::make_tuple(Vs, Fs);
}

int FrameAssembly::computeDoFsMapping(const std::unordered_map<int, bool> &dynamic_vertices,
                                      std::unordered_map<int, int> &map_dof_entire2subset,
                                      std::unordered_map<int, int> &map_dof_subset2entire){
    //dof: 6 * |V|
    int new_dof = 0;
    for (int dof = 0; dof < 6 * vertices_.size(); dof++)
    {
        int nodeID = (int)dof / 6;
        auto find_it = dynamic_vertices.find(nodeID);
        if (find_it != dynamic_vertices.end() && find_it->second == true)
        {
            map_dof_entire2subset[dof] = new_dof;
            map_dof_subset2entire[new_dof] = dof;
            new_dof++;
        }
    }
    return new_dof;
}

void FrameAssembly::computeDynamicVertices(const std::vector<int> &subset_beams_index,
                                           const std::vector<int> &fix_beams_index,
                                           std::unordered_map<int, bool> &dynamic_vertices)
{

    for (int id = 0; id < subset_beams_index.size(); id++)
    {
        for (int jd = 0; jd < 2; jd++)
        {
            int end = edges_[subset_beams_index[id]][jd];
            dynamic_vertices[end] = true;
        }
    }

    for (int id = 0; id < support_vertex_indices_.size(); id++)
    {
        dynamic_vertices[support_vertex_indices_[id]] = false;
    }

    for (int id = 0; id < fix_beams_index.size(); id++)
    {
        for (int jd = 0; jd < 2; jd++)
        {
            int end = edges_[fix_beams_index[id]][jd];
            dynamic_vertices[end] = false;
        }
    }
    return ;
}

int FrameAssembly::computeDoFsMapping(const std::vector<int> &subset_beams_index,
                                      const std::vector<int> &fix_beams_index,
                                     std::unordered_map<int, int> &map_dof_entire2subset,
                                      std::unordered_map<int, int> &map_dof_subset2entire)
{
    std::unordered_map<int, bool> dynamic_vertices;
    computeDynamicVertices(subset_beams_index, fix_beams_index, dynamic_vertices);
    return computeDoFsMapping(dynamic_vertices, map_dof_entire2subset, map_dof_subset2entire);
}

void FrameAssembly::assembleStiffMatrix(std::vector<Eigen::Triplet<double>> &K_tri,
                                        const Eigen::MatrixXd &k_G,
                                        const std::unordered_map<int, int> &map_dof_entire2subset, int edgeID) {
    std::vector<int> new_dofs;
    for (int jd = 0; jd < edgeDofIndices[edgeID].size(); jd++) {
        int old_dof = edgeDofIndices[edgeID][jd];
        if (map_dof_entire2subset.find(old_dof) != map_dof_entire2subset.end()) {
            new_dofs.push_back(map_dof_entire2subset.at(old_dof));
        } else {
            new_dofs.push_back(-1);
        }
    }

    for (int jd = 0; jd < 12; jd++) {
        for (int kd = 0; kd < 12; kd++) {
            if (abs(k_G(jd, kd)) < 1E-12)
                continue;
            int new_dofJ = new_dofs[jd];
            int new_dofK = new_dofs[kd];
            if (new_dofJ != -1 && new_dofK != -1) {
                K_tri.push_back(Eigen::Triplet<double>(new_dofJ, new_dofK, k_G(jd, kd)));
            }
        }
    }
}

void FrameAssembly::computeStiffMatrix(const std::vector<int> &subset_beams_index, int total_dofs,
                                      const std::unordered_map<int, int> &map_dof_entire2subset,
                                      const std::unordered_map<int, int> &map_dof_subset2entire,
                                      SparseMatrixD &K) {
    std::vector<Eigen::Triplet<double>> tripletList;

    for (int id = 0; id < subset_beams_index.size(); id++)
    {
        int edgeID = subset_beams_index[id];
        Eigen::MatrixXd k_G = beams_elasticity_[edgeID]->create_global_stiffness_matrix();
        assembleStiffMatrix(tripletList, k_G, map_dof_entire2subset, edgeID);
    }

    K = SparseMatrixD(total_dofs, total_dofs);
    K.setFromTriplets(tripletList.begin(), tripletList.end());
}

void FrameAssembly::assemblyForce(Eigen::VectorXd &F, const Eigen::VectorXd &g, const std::unordered_map<int, int> &map_dof_entire2subset, int edgeID) {
    std::vector<int> new_dofs;
    for (int jd = 0; jd < edgeDofIndices[edgeID].size(); jd++) {
        int old_dof = edgeDofIndices[edgeID][jd];
        if (map_dof_entire2subset.find(old_dof) != map_dof_entire2subset.end()) {
            new_dofs.push_back(map_dof_entire2subset.at(old_dof));
        } else {
            new_dofs.push_back(-1);
        }
    }

    for (int jd = 0; jd < new_dofs.size(); jd++) {
        int new_dofJ = new_dofs[jd];
        if (new_dofJ != -1) {
            F(new_dofs[jd]) += g[jd];
        }
    }
}

void FrameAssembly::computeLoads(const std::vector<int> &subset_beams_index,
                                 int tot_dofs,
                                 const std::unordered_map<int, int> &map_dof_entire2subset,
                                 const std::unordered_map<int, int> &map_dof_subset2entire,
                                 Eigen::VectorXd &F) {
    F = Eigen::VectorXd::Zero(tot_dofs);

    for (int id = 0; id < subset_beams_index.size(); id++) {
        int edgeID = subset_beams_index[id];
        Eigen::VectorXd load = beams_elasticity_[edgeID]->create_global_self_weight();
        assemblyForce(F, load, map_dof_entire2subset, edgeID);
    }
}


bool FrameAssembly::solveElasticity(const std::vector<int> &subset_beams_index,
                                    const std::vector<int> &fix_beams_index,
                                    Eigen::VectorXd &displacement)
{
    std::unordered_map<int, bool> dynamic_vertices;
    computeDynamicVertices(subset_beams_index, fix_beams_index, dynamic_vertices);

    std::unordered_map<int, int> map_dof_entire2subset;
    std::unordered_map<int, int> map_dof_subset2entire;
    int total_dofs = computeDoFsMapping(dynamic_vertices, map_dof_entire2subset, map_dof_subset2entire);


    SparseMatrixD K;
    computeStiffMatrix(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, K);

    Eigen::VectorXd F;
    computeLoads(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, F);

    Eigen::VectorXd D;

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if (llt.info() == Eigen::Success)
    {
        D = llt.solve(F);
        displacement = Eigen::VectorXd::Zero(vertices_.size() * 6);
        for (int id = 0; id < D.rows(); id++) {
            int old_dof = map_dof_subset2entire[id];
            displacement[old_dof] = D[id];
        }
        displacement *= displacement_scaling_factor;
        return true;
    }
    else{
        return false;
    }
}

std::vector<Eigen::VectorXd> FrameAssembly::interpolatePolynomial(Eigen::VectorXd D_local, double L) {
    //    std::cout << Ac << std::endl;

    Eigen::Vector4d d_y(D_local[1], D_local[7], D_local[5], D_local[11]);
    Eigen::Vector4d d_z(D_local[2], D_local[8], -D_local[4], -D_local[10]);

    Eigen::MatrixXd A(4, 4);

    double u0 = D_local[0];
    double u6 = D_local[6];

    u6 += L;
    A(0, 0) = 1.0;
    A(0, 1) = u0;
    A(0, 2) = u0 * u0;
    A(0, 3) = u0 * u0 * u0;

    A(1, 0) = 1.0;
    A(1, 1) = u6;
    A(1, 2) = u6 * u6;
    A(1, 3) = u6 * u6 * u6;

    A(2, 0) = 0.0;
    A(2, 1) = 1.;
    A(2, 2) = 2. * u0;
    A(2, 3) = 3. * u0 * u0;

    A(3, 0) = 0.0;
    A(3, 1) = 1.;
    A(3, 2) = 2. * u6;
    A(3, 3) = 3. * u6 * u6;
    u6 -= L;

    std::vector<Eigen::VectorXd> u_poly;

    u_poly.push_back(A.inverse() * d_y);
    u_poly.push_back(A.inverse() * d_z);

    return u_poly;
}

void FrameAssembly::visualizeDisplacement(const std::vector<int> &subset_beams_index,
                                         Eigen::VectorXd &displacement,
                                         std::vector<std::vector<Eigen::Vector3d>> &segments,
                                         std::vector<std::vector<double>> &deviation)
{
    auto polyval = [](double x, Eigen::VectorXd c) -> double {
        double base = 1;
        double result = 0;
        for (int id = 0; id < c.rows(); id++) {
            result += base * c[id];
            base *= x;
        }
        return result;
    };

    for (int eID : subset_beams_index)
    {
        int nodeu = edges_[eID][0];
        int nodev = edges_[eID][1];

        Eigen::Vector3d end_u = vertices_[nodeu];
        Eigen::Vector3d end_v = vertices_[nodev];

        Eigen::MatrixXd R3 = beams_elasticity_[eID]->create_global_transformation_matrix();
        Eigen::MatrixXd R = beams_elasticity_[eID]->turn_diagblock(R3);

        Eigen::VectorXd D_global(12);
        D_global.segment(0, 6) = displacement.segment(nodeu * 6, 6);
        D_global.segment(6, 6) = displacement.segment(nodev * 6, 6);

        Eigen::VectorXd D_local(12);
        D_local = R * D_global;

        double dx_u = D_local[0];
        double dx_v = D_local[6];

        double L = beams_elasticity_[eID]->beam_length_;

        std::vector<Eigen::VectorXd> u_poly;
        u_poly = interpolatePolynomial(D_local, L);

        std::vector<Eigen::Vector3d> polylines;
        std::vector<double> distance;
        for (int id = 0; id <= num_deformed_segment_; id++)
        {
            double t = (double)id / num_deformed_segment_;
            if(beams_elasticity_[eID]->frame_node_)
            {
                double s = dx_u + t * (dx_v + L - dx_u);
                double v = u_poly[0][0] + u_poly[0][1] * s + u_poly[0][2] * s * s + u_poly[0][3] * s * s * s;
                double w = u_poly[1][0] + u_poly[1][1] * s + u_poly[1][2] * s * s + u_poly[1][3] * s * s * s;
                Eigen::Vector3d d(s, v, w);
                Eigen::Vector3d inter_pt = end_u + R3.transpose() * d;
                Eigen::Vector3d ori_pt = (end_v - end_u) * t + end_u;
                polylines.push_back(inter_pt);
                distance.push_back((inter_pt - ori_pt).norm());
            }
            else{
                Eigen::Vector3d ori_pt = (end_v - end_u) * t + end_u;
                Eigen::Vector3d dis_pt = (end_v + D_global.segment(6, 6) - end_u - D_global.segment(0, 6)) * t + end_u + D_global.segment(0, 6);
                distance.push_back((dis_pt - ori_pt).norm());
                polylines.push_back(dis_pt);
            }
        }
        deviation.push_back(distance);
        segments.push_back(polylines);
    }
}
Eigen::Vector3d FrameAssembly::computeBeamCentroid(int beamID) {
    int end0 = edges_[beamID][0];
    int end1 = edges_[beamID][1];
    return (vertices_[end0] + vertices_[end1]) / 2.0;
}

// currently we only handle valence 3 and valence 2 part graph
std::tuple<Eigen::Matrix2Xi, std::vector<int>, Eigen::Matrix2Xi, Eigen::Matrix3Xd> FrameAssembly::computeOffsetBeamConstraintsPairs(double sinEps) {
    std::vector<std::vector<int>> vertex_beam_ids;
    vertex_beam_ids.resize(vertices_.size());

    //collect intersection beams
    for (int id = 0; id < edges_.size(); id++) {
        int end0 = edges_[id][0];
        int end1 = edges_[id][1];
        vertex_beam_ids[end0].push_back(id);
        vertex_beam_ids[end1].push_back(id);
    }

    //check valence
    for (int id = 0; id < vertex_beam_ids.size(); id++) {
        if (vertex_beam_ids[id].size() > 3) {
            return {};
        }
    }

    //
    std::vector<Eigen::Vector2i> pairs;
    std::vector<int> pairs_sign;
    std::vector<Eigen::Vector2i> parallel_pairs_2;
    std::vector<Eigen::Vector3d> parallel_pairs_2_drt;
    std::vector<Eigen::Vector3i> parallel_pairs_3;
    for (int id = 0; id < vertex_beam_ids.size(); id++) {
        if (vertex_beam_ids[id].size() == 2) {
            int e0 = vertex_beam_ids[id][0];
            int e1 = vertex_beam_ids[id][1];
            Eigen::Vector3d d0 = direction(e0);
            Eigen::Vector3d d1 = direction(e1);

            //if two lines are in parallel
            if (d0.cross(d1).norm() < sinEps) {
                parallel_pairs_2.push_back(Eigen::Vector2i(e0, e1));
                Eigen::Vector3d drt(0, 0, 1);

                //find a direction that is not parallel to d0
                if (drt.cross(d0).norm() < sinEps) {
                    drt = Eigen::Vector3d(1, 0, 0);
                }
                parallel_pairs_2_drt.push_back(drt);
            } else {
                pairs.push_back(Eigen::Vector2i(e0, e1));
                pairs_sign.push_back(1);
            }
        }
        if (vertex_beam_ids[id].size() == 3) {
            Eigen::Vector3d nr(0, 0, 0);
            int parallel_index = -1;
            for (int kd = 0; kd < vertex_beam_ids[id].size(); kd++) {
                int e0 = vertex_beam_ids[id][kd];
                int e1 = vertex_beam_ids[id][(kd + 1) % 3];
                Eigen::Vector3d d0 = direction(e0);
                Eigen::Vector3d d1 = direction(e1);
                if (d0.cross(d1).norm() < sinEps) {
                    parallel_index = kd;
                    break;
                }
            }

            if (parallel_index == -1) {
                Eigen::Vector3d nr(0, 0, 0);
                for (int kd = 0; kd < 3; kd++) {
                    int e0 = vertex_beam_ids[id][kd];
                    int e1 = vertex_beam_ids[id][(kd + 1) % 3];
                    int e2 = vertex_beam_ids[id][(kd + 2) % 3];
                    Eigen::Vector3d d0 = direction(e0);
                    Eigen::Vector3d d1 = direction(e1);
                    Eigen::Vector3d d2 = direction(e2);
                    Eigen::Vector3d n = d0.cross(d1);
                    if (n.dot(nr) >= 0) {
                        pairs_sign.push_back(1);
                    } else {
                        pairs_sign.push_back(-1);
                    }
                    pairs.push_back(Eigen::Vector2i(e0, e1));
                    nr = n;
                }
            } else {
                int e0 = vertex_beam_ids[id][parallel_index];
                int e1 = vertex_beam_ids[id][(parallel_index + 1) % 3];
                int e2 = vertex_beam_ids[id][(parallel_index + 2) % 3];
                Eigen::Vector3d d0 = direction(e0);
                Eigen::Vector3d d1 = direction(e1);
                Eigen::Vector3d d2 = direction(e2);
                Eigen::Vector3d n = d0.cross(d1);

                Eigen::Vector3d n12 = d1.cross(d2);
                Eigen::Vector3d n20 = d2.cross(d0);
                pairs.push_back(Eigen::Vector2i(e1, e2));
                pairs.push_back(Eigen::Vector2i(e2, e0));
                pairs_sign.push_back(1);
                if (n12.dot(n20) >= 0) {
                    pairs_sign.push_back(1);
                } else {
                    pairs_sign.push_back(-1);
                }
            }
        }
    }

    Eigen::Matrix2Xi pairsMat(2, pairs.size());
    for (int id = 0; id < pairs.size(); id++) {
        pairsMat.col(id) = pairs[id];
    }

    Eigen::Matrix2Xi parallel_pairs_2_Mat(2, parallel_pairs_2.size());
    for (int id = 0; id < parallel_pairs_2.size(); id++) {
        parallel_pairs_2_Mat.col(id) = parallel_pairs_2[id];
    }

    Eigen::Matrix3Xi parallel_pairs_3_Mat(3, parallel_pairs_3.size());
    for (int id = 0; id < parallel_pairs_3.size(); id++) {
        parallel_pairs_3_Mat.col(id) = parallel_pairs_3[id];
    }

    Eigen::Matrix3Xd parallel_pairs_2_drt_Mat(3, parallel_pairs_2_drt.size());
    for (int id = 0; id < parallel_pairs_2_drt.size(); id++) {
        parallel_pairs_2_drt_Mat.col(id) = parallel_pairs_2_drt[id];
    }
    return std::make_tuple(pairsMat, pairs_sign, parallel_pairs_2_Mat, parallel_pairs_2_drt_Mat);
}

double FrameAssembly::computeCompliance(const Eigen::VectorXd &displacement, const std::vector<int> &subset_beams)
{
    double compliance = 0;
    for(int id = 0; id < subset_beams.size(); id++)
    {
        int egdeID = subset_beams[id];
        Eigen::VectorXd u = Eigen::VectorXd::Zero(12);
        for (int kd = 0; kd < 2; kd++)
        {
            int nodeID = edges_[egdeID][kd];
            u.segment(kd * 6, 6) = displacement.segment(nodeID * 6, 6);
        }
        Eigen::VectorXd f = beams_elasticity_[egdeID]->create_global_self_weight();
        compliance += 0.5 * f.dot(u);
    }
    return compliance;
}
double FrameAssembly::computeMaxDisplacement(const Eigen::VectorXd &displacement) {
    //max displacement

    double max_displacement = 0.0;
    for(int id = 0; id < displacement.size() / 6; id++){
        Eigen::Vector3d u = displacement.segment(id * 6, 3);
//        std::cout << u.norm() << std::endl;
        max_displacement = std::max(u.norm(), max_displacement);
    }
    return max_displacement;
    //std::cout << displacement.cwiseAbs().maxCoeff() << std::endl;
    //return displacement.cwiseAbs().maxCoeff();
}
double FrameAssembly::computeMaxStress(const std::vector<int> &subset_beams_index,
                                      const Eigen::VectorXd &displacement,
                                      std::vector<Eigen::VectorXd> &internal_forces,
                                      std::vector<Eigen::Vector2d> &joint_stresses) {
    //max stress (von mises stress)
    internal_forces.clear();
    joint_stresses.clear();
    for (int id = 0; id < beams_.size(); id++) {
        Eigen::VectorXd force = Eigen::VectorXd::Zero(12);
        internal_forces.push_back(force);
        joint_stresses.push_back(Eigen::Vector2d::Zero());
    }

    double maxStress = 0;
    for (int id = 0; id <subset_beams_index.size(); id++)
    {
        int edgeID = subset_beams_index[id];
        Eigen::VectorXd u = Eigen::VectorXd::Zero(12);
        for (int kd = 0; kd < 2; kd++) {
            int nodeID = edges_[edgeID][kd];
            u.segment(kd * 6, 6) = displacement.segment(nodeID * 6, 6);
        }
        internal_forces[edgeID] = beams_elasticity_[edgeID]->compute_internal_force(u);
        joint_stresses[edgeID] = beams_elasticity_[edgeID]->compute_joint_stress(u);
        maxStress = std::max(maxStress, joint_stresses[edgeID].maxCoeff());
    }
    return maxStress;
}

std::vector<Eigen::Vector3d> FrameAssembly::getLines(std::vector<int> subset_beams_index)
{
    std::vector<Eigen::Vector3d> points;
    for (int id = 0; id < subset_beams_index.size(); id++)
    {
        int beamIndex = subset_beams_index[id];
        Eigen::Vector3d p0 = vertices_[edges_[beamIndex][0]];
        Eigen::Vector3d p1 = vertices_[edges_[beamIndex][1]];
        points.push_back(p0);
        points.push_back(p1);
    }
    return points;
}
std::vector<std::vector<Eigen::Vector3d>> FrameAssembly::getDeformedLines(std::vector<int> subset_beams_index, std::vector<int> fix_beam_index)
{
    Eigen::VectorXd displacement;
    solveElasticity(subset_beams_index, fix_beam_index, displacement);
    std::vector<std::vector<Eigen::Vector3d>> segments;
    std::vector<std::vector<double>> deviation;
    visualizeDisplacement(subset_beams_index, displacement, segments, deviation);
    return segments;
}

std::vector<Eigen::Vector3d> FrameAssembly::getJoints(std::vector<int> subset_beams_index) {
    std::vector<int> dynamic_vertices_id;
    std::vector<Eigen::Vector3d> dynamic_vertices;
    for(int id = 0; id < subset_beams_index.size(); id++)
    {
        int edgeID = subset_beams_index[id];
        for(int jd = 0; jd < edges_[edgeID].size(); jd++){
            int vID = edges_[edgeID][jd];
            if(std::find(dynamic_vertices_id.begin(), dynamic_vertices_id.end(), vID) == dynamic_vertices_id.end()
                && std::find(support_vertex_indices_.begin(), support_vertex_indices_.end(), vID) == support_vertex_indices_.end()){
                dynamic_vertices.push_back(vertices_[vID]);
                dynamic_vertices_id.push_back(vID);
            }
        }
    }
    return dynamic_vertices;
}

std::vector<Eigen::Vector3d> FrameAssembly::getDeformedJoints(std::vector<int> subset_beams_index, std::vector<int> fix_beam_index)
{
        Eigen::VectorXd displacement;
        solveElasticity(subset_beams_index, fix_beam_index, displacement);
        std::vector<int> dynamic_vertices_id;
        std::vector<Eigen::Vector3d> dynamic_vertices;
        for(int id = 0; id < subset_beams_index.size(); id++)
        {
            int edgeID = subset_beams_index[id];
            for(int jd = 0; jd < edges_[edgeID].size(); jd++){
                int vID = edges_[edgeID][jd];
                if(std::find(dynamic_vertices_id.begin(), dynamic_vertices_id.end(), vID) == dynamic_vertices_id.end()
                    && std::find(support_vertex_indices_.begin(), support_vertex_indices_.end(), vID) == support_vertex_indices_.end()){
                    dynamic_vertices.push_back(vertices_[vID] + displacement.segment(vID * 6, 6));
                    dynamic_vertices_id.push_back(vID);
                }
            }
        }
        return dynamic_vertices;
}

std::vector<Eigen::Vector3d> FrameAssembly::getGroundFixtures(std::vector<int> subset_beams_index) {
    std::vector<int> ground_vertices_id;
    std::vector<Eigen::Vector3d> ground_vertices;
    for(int id = 0; id < subset_beams_index.size(); id++)
    {
        int edgeID = subset_beams_index[id];
        for(int jd = 0; jd < edges_[edgeID].size(); jd++){
            int vID = edges_[edgeID][jd];
            if(std::find(ground_vertices_id.begin(), ground_vertices_id.end(), vID) == ground_vertices_id.end()
                && std::find(support_vertex_indices_.begin(), support_vertex_indices_.end(), vID) != support_vertex_indices_.end()){
                ground_vertices.push_back(vertices_[vID]);
                ground_vertices_id.push_back(vID);
            }
        }
    }
    return ground_vertices;
}
Eigen::VectorXd FrameAssembly::getDisplacement(std::vector<int> subset_beams_index, std::vector<int> fix_beam_index)
{
    Eigen::VectorXd displacement;
    solveElasticity(subset_beams_index, fix_beam_index, displacement);
    return displacement;
}
std::vector<std::vector<Eigen::Vector3d>> FrameAssembly::getVisualizationLines(std::vector<int> subset_beams_index, Eigen::VectorXd &displacement) {
    std::vector<std::vector<Eigen::Vector3d>> segments;
    std::vector<std::vector<double>> deviation;
    visualizeDisplacement(subset_beams_index, displacement, segments, deviation);
    return segments;
}

std::vector<Eigen::Vector3d> FrameAssembly::getVisualizationJoints(std::vector<int> subset_beams_index, Eigen::VectorXd &displacement) {
    std::vector<int> dynamic_vertices_id;
    std::vector<Eigen::Vector3d> dynamic_vertices;
    for(int id = 0; id < subset_beams_index.size(); id++)
    {
        int edgeID = subset_beams_index[id];
        for(int jd = 0; jd < edges_[edgeID].size(); jd++){
            int vID = edges_[edgeID][jd];
            if(std::find(dynamic_vertices_id.begin(), dynamic_vertices_id.end(), vID) == dynamic_vertices_id.end()
                && std::find(support_vertex_indices_.begin(), support_vertex_indices_.end(), vID) == support_vertex_indices_.end()){
                dynamic_vertices.push_back(vertices_[vID] + displacement.segment(vID * 6, 6));
                dynamic_vertices_id.push_back(vID);
            }
        }
    }
    return dynamic_vertices;
}

Eigen::VectorXd FrameAssembly::getGravityForce(std::vector<int> subset_beams_index,
                                               std::vector<int> fix_beam_index) {
    std::unordered_map<int, int> map_dof_entire2subset;
    std::unordered_map<int, int> map_dof_subset2entire;
    int total_dofs = computeDoFsMapping(subset_beams_index, fix_beam_index, map_dof_entire2subset, map_dof_subset2entire);
    Eigen::VectorXd f;
    computeLoads(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, f);

    Eigen::VectorXd F(vertices_.size() * 6);
    F.setZero();
    for (int id = 0; id < f.rows(); id++) {
        int old_dof = map_dof_subset2entire[id];
        F[old_dof] = f[id];
    }
    return F;
}


Eigen::VectorXd FrameAssembly::getDisplacementByForce(std::vector<int> subset_beams_index,
                                                      std::vector<int> fix_beam_index,
                                                      Eigen::VectorXd &F) {
    std::unordered_map<int, int> map_dof_entire2subset;
    std::unordered_map<int, int> map_dof_subset2entire;
    int total_dofs = computeDoFsMapping(subset_beams_index, fix_beam_index, map_dof_entire2subset, map_dof_subset2entire);

    SparseMatrixD K;
    computeStiffMatrix(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, K);

    Eigen::VectorXd D;

    Eigen::MatrixXd Kdense = K.toDense();
    //if (ldlt.info() == Eigen::Success)
    {
        Eigen::VectorXd f(total_dofs);
        f.setZero();
        for(int id = 0; id < F.rows(); id++){
            if(map_dof_entire2subset.find(id) != map_dof_entire2subset.end()){
                int new_id = map_dof_entire2subset[id];
                f(new_id) = F[id];
            }
        }

        Eigen::VectorXd displacement;
        D = Kdense.colPivHouseholderQr().solve(f);
        //D = ldlt.solve(f);
        displacement = Eigen::VectorXd::Zero(vertices_.size() * 6);
        for (int id = 0; id < D.rows(); id++) {
            int old_dof = map_dof_subset2entire[id];
            displacement[old_dof] = D[id];
        }
        return displacement;
    }
    return Eigen::VectorXd();
}

}