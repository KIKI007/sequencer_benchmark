//
// Created by 汪子琦 on 04.09.22.
//

#include "search/PartGraphFrame.h"
namespace search
{
PartGraphFrame::PartGraphFrame(const search::PartGraphFrame& graph) {
    assembly_ = graph.assembly_;
    for (int id = 0; id < graph.edges_.size(); id++) {
        std::shared_ptr<PartGraphFrameEdge> edge = std::static_pointer_cast<PartGraphFrameEdge>(graph.edges_[id]);
        edges_.push_back(std::make_shared<PartGraphFrameEdge>(*edge));
    }

    for (int id = 0; id < graph.nodes_.size(); id++) {
        std::shared_ptr<PartGraphFrameNode> node = std::static_pointer_cast<PartGraphFrameNode>(graph.nodes_[id]);
        nodes_.push_back(std::make_shared<PartGraphFrameNode>(*node));
    }
}

PartGraphFrame::PartGraphFrame(std::shared_ptr<frame::FrameAssembly> assembly) {
    assembly_ = assembly;
    assembly_->computeBeamElasticity();
    computeGraph();
}

void PartGraphFrame::computeGraph()
{
    //edges
    for (int id = 0; id < assembly_->edges_.size(); id++)
    {
        std::shared_ptr<PartGraphFrameEdge> edge = std::make_shared<PartGraphFrameEdge>();
        edges_.push_back(edge);
    }

    //dual graph
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        for (int jd = 0; jd < id; jd++) {
            //if both share a vertex
            std::set<int> sets = {assembly_->edges_[id][0], assembly_->edges_[id][1], assembly_->edges_[jd][0], assembly_->edges_[jd][1]};
            if (sets.size() < 4) {
                edges_[id]->partIDs.push_back(jd);
                edges_[jd]->partIDs.push_back(id);
            }
        }
    }

    //support beams
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int end0 = assembly_->edges_[id][0];
        int end1 = assembly_->edges_[id][1];
        bool grounded = false;
        for (int jd = 0; jd < assembly_->support_vertex_indices_.size(); jd++) {
            int support_vertexID = assembly_->support_vertex_indices_[jd];
            if (support_vertexID == end0 || support_vertexID == end1) {
                grounded = true;
                break;
            }
        }

        std::shared_ptr<PartGraphFrameNode> node = std::make_shared<PartGraphFrameNode>();
        node->partID = id;
        node->grounded = grounded;
        node->visited = false;

        node->pu = assembly_->vertices_[end0];
        node->pv = assembly_->vertices_[end1];

        nodes_.push_back(node);
    }
}


double PartGraphFrame::evaluateStability(const std::vector<int>& subPartIDs, const std::vector<int>& dynamic_scaffold_partIDs)
{

//    if(subPartIDs.size() != 0 && subPartIDs.size() != assembly_->beams_.size()
//        && subPartIDs.size() % 1 != 0){
//        return std::numeric_limits<double>::max();
//    }

    Eigen::VectorXd displacement;

    std::vector<int> fixed_indices;
    fixed_indices.insert(fixed_indices.end(), dynamic_scaffold_partIDs.begin(), dynamic_scaffold_partIDs.end());
    fixed_indices.insert(fixed_indices.end(), static_scaffold_partIDs_.begin(), static_scaffold_partIDs_.end());

//    double cost = 0;
//    for(int id = 0; id < subPartIDs.size(); id++){
//        double y = assembly_->beams_[subPartIDs[id]]->computeCenterY();
//        cost += y;
//    }
//    return cost;

    if(assembly_->solveElasticity(subPartIDs, fixed_indices, displacement)){
        if (use_displacement_)
        {
            return assembly_->computeMaxDisplacement(displacement);
        } else {
            return assembly_->computeCompliance(displacement, subPartIDs);
        }
    }
    else{
        return std::numeric_limits<double>::max();
    }

}

}