//
// Created by 汪子琦 on 14.03.22.
//

#include "search/StateGraph.h"
#include <tbb/parallel_for.h>

namespace search {

/*******************************************************************************************************************************
 *                                                      StateGraph
 ********************************************************************************************************************************/

int StateGraph::numDigitPerMode() {
    return std::ceil(std::log2(nMode_));
}

int StateGraph::numPartPerChunk() {
    int num_digit_per_mode = numDigitPerMode();
    int num_part_per_chunk = chunkBit_ / num_digit_per_mode;
    return num_part_per_chunk;
}

int StateGraph::numChunkRequired() {
    int num_part_per_chunk = numPartPerChunk();
    int num_chunk = std::ceil((double)nPart_ / num_part_per_chunk);
    return num_chunk;
}

unsigned StateGraph::modeMask(int N) {
    unsigned mask = 0;
    for (int id = 0; id < N; id++) {
        mask += 1 << (id);
    }
    return mask;
}

std::tuple<unsigned, unsigned> StateGraph::computeChunkOffset(unsigned partID) {
    int num_digit_per_mode = numDigitPerMode();
    int num_part_per_chunk = chunkBit_ / num_digit_per_mode;
    int offset0 = partID / num_part_per_chunk;
    unsigned offset1 = (partID % num_part_per_chunk) * num_digit_per_mode;
    return std::make_tuple(offset0, offset1);
}

int StateGraph::getChunk(const StateGraphNode &node, int offset) {
    return assemblyStates_[node.chunkOffset + offset];
}

unsigned StateGraph::computePartState(unsigned partID, const StateGraphNode &node) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    return (getChunk(node, offset0) & (modeMask(numDigitPerMode()) << offset1)) >> offset1;
}

unsigned StateGraph::computePartState(unsigned int partID, const std::vector<unsigned> &assemblyState) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    return (assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1)) >> offset1;
}

std::vector<unsigned> StateGraph::setPartMode(unsigned partID, int partState, const StateGraphNode &node) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    std::vector<unsigned> assemblyState = getAssemblyState(node);
    unsigned prevPartState = assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1);
    assemblyState[offset0] -= prevPartState;
    assemblyState[offset0] += partState << offset1;
    return assemblyState;
}

void StateGraph::setPartMode(unsigned int partID, int partState, std::vector<unsigned int> &assemblyState) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    unsigned prevPartState = assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1);
    assemblyState[offset0] -= prevPartState;
    assemblyState[offset0] += partState << offset1;
}

std::vector<unsigned> StateGraph::getAssemblyState(const StateGraphNode &node) {
    std::vector<unsigned> assemblyState;
    for (int id = 0; id < nChunk_; id++) {
        assemblyState.push_back(getChunk(node, id));
    }
    return assemblyState;
}

std::vector<int> StateGraph::getInstalledParts(PtrN node) {
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned state = computePartState(id, *node);
        if (state > 0) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::vector<int> StateGraph::getInstalledParts(const std::vector<unsigned int> &assemblyState) {
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned state = computePartState(id, assemblyState);
        if (state > 0) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::string StateGraph::node_label(PtrN node) {
    std::vector<int> parts = getInstalledParts(node);
    std::string node_label = "\"";
    for (int id = 0; id < parts.size(); id++) {
        node_label += std::to_string(parts[id]);
        if (id + 1 != parts.size()) {
            node_label += " ,";
        }
    }
    node_label += "\"";
    return node_label;
}

std::string StateGraph::DotGraphString() {
    std::stringstream buffer;
    buffer << "digraph {\n";
    for (int id = 0; id < nodes_.size(); id++) {
        if (visibility(id)) {
            buffer << id << " [label = " << node_label(nodes_[id]) << "]\n";
        }
    }

    for (int id = 0; id < nodes_.size(); id++)
    {
        for (int jd = nodeEdgeOffsetStart_[id]; jd < nodeEdgeOffsetEnd_[id]; jd++)
        {
            int adj_node = edges_[jd];
            if(!edge_costs_.empty())
            {
                if (visibility(id) && visibility(adj_node))
                {
                    buffer << id << "-> " << adj_node;
                    buffer << " [label = " << std::to_string(edge_costs_[jd]) << "]" << std::endl;
                }
            }
            else
            {
                if (visibility(id) && visibility(adj_node))
                {
                    buffer << id << "-> " << adj_node << std::endl;
                }
            }
        }
    }

    buffer << "}\n";
    return buffer.str();
}

void StateGraph::writeDotGraph(std::string filename) {
    std::ofstream fout(filename);
    std::string buffer = DotGraphString();
    fout << buffer;
    fout.close();
}

std::tuple<int, bool> StateGraph::addNewNode(std::vector<unsigned int> &newAssemblyState)
{
    auto find_it = map_astate_nodeID_.find(newAssemblyState);
    int newNodeID = -1;
    bool createNewNode = false;
    if (find_it == map_astate_nodeID_.end()) {
        assemblyStates_.insert(assemblyStates_.end(), newAssemblyState.begin(), newAssemblyState.end());
        PtrN newNode = std::make_shared<StateGraphNode>();

        int index = assemblyStates_.size() - nChunk_;
        newNode->chunkOffset = index;

        newNodeID = nodes_.size();
        newNode->nodeID = newNodeID;
        newNode->cost = 0.0;

        nodes_.push_back(newNode);
        map_astate_nodeID_[newAssemblyState] = newNodeID;
        createNewNode = true;
    } else {
        newNodeID = find_it->second;
    }
    return std::make_tuple(newNodeID, createNewNode);
}

std::vector<int> StateGraph::computeStablestSubAssemblyWithKParts(int K) {
    createRootNode();

    int preStepOffset = 0;
    int currStepOffset = nodes_.size();
    for (int id = 0; id < K; id++) {
        enumerateNextAssemblyStates(preStepOffset, currStepOffset);
        preStepOffset = currStepOffset;
        currStepOffset = nodes_.size();
    }

    tbb::parallel_for( tbb::blocked_range<int>(preStepOffset, currStepOffset),
                      [&](tbb::blocked_range<int> r)
                      {
                          for (int id =r.begin(); id < r.end(); ++id)
                          {
                              evaluateNode(nodes_[id]);
                          }
                      });

    double min_cost = std::numeric_limits<double>::max();
    PtrN min_node;
    for(int id = preStepOffset; id < currStepOffset; id++)
    {
        if(min_cost > nodes_[id]->cost){
            min_cost = nodes_[id]->cost;
            min_node = nodes_[id];
        }
    }
    std::vector<int> partIDs;
    if(min_node){
        partIDs = getInstalledParts(min_node);
    }
    return partIDs;
}


void StateGraph::enumerateAssemblyStates() {
    createRootNode();

    int preStepOffset = 0;
    int currStepOffset = nodes_.size();
    for (int id = 0; id <= nPart_; id++)
    {
        enumerateNextAssemblyStates(preStepOffset, currStepOffset);
        preStepOffset = currStepOffset;
        currStepOffset = nodes_.size();
        if(checkEndNode(nodes_.back())){
            return ;
        }
    }
}

void StateGraph::enumerateNextAssemblyStates(int preStepOffset, int currStepOffset) {
    std::map<std::vector<unsigned>, int, StateCompare> map_astate_nodeID;

    for (int iS = preStepOffset; iS < currStepOffset; iS++) {
        //expand the current node
        PtrN prevNode = nodes_[iS];
        std::vector<PtrN> childNodes;
        std::vector<PtrN> prechild, newchild;
        expandNode(prevNode, prechild, newchild);
        childNodes.insert(childNodes.end(), prechild.begin(), prechild.end());
        childNodes.insert(childNodes.end(), newchild.begin(), newchild.end());

        for (int id = 0; id < newchild.size(); id++) {
            nodeEdgeOffsetStart_.push_back(0);
            nodeEdgeOffsetEnd_.push_back(0);
        }

        nodeEdgeOffsetStart_[prevNode->nodeID] = edges_.size();
        nodeEdgeOffsetEnd_[prevNode->nodeID] = edges_.size();

        for (int id = 0; id < childNodes.size(); id++)
        {
            edges_.push_back(childNodes[id]->nodeID);
            nodeEdgeOffsetEnd_[prevNode->nodeID]++;
        }
    }
}

void StateGraph::expandNode(StateGraph::PtrN node,
                            int remainArms,
                            std::map<std::vector<unsigned int>, bool, StateCompare> &nodeIncluded,
                            std::vector<PtrN> &preChildNodes,
                            std::vector<PtrN> &newChildNodes) {
    if (remainArms == 0)
        return;

    std::vector<int> visited_nodeIDs = getInstalledParts(node);

    std::vector<int> neighbourIDs;
    partGraph_->computeNodesNeighbour(visited_nodeIDs, neighbourIDs, 1);

    for (int id = 0; id < neighbourIDs.size(); id++)
    {
        int partID = neighbourIDs[id];
        std::vector<unsigned> newAssemblyState = setPartMode(partID, 1, *node);

        if (nodeIncluded.find(newAssemblyState) != nodeIncluded.end()) continue;
        nodeIncluded[newAssemblyState] = true;
        auto [newNode, createNewNode] = addNewNode(newAssemblyState);

        if (createNewNode) {
            newChildNodes.push_back(nodes_[newNode]);
        } else {
            preChildNodes.push_back(nodes_[newNode]);
        }

        expandNode(nodes_[newNode], remainArms - 1, nodeIncluded, preChildNodes, newChildNodes);

    }
}

void StateGraph::createRootNode() {
    std::vector<unsigned> assemblyState;
    assemblyState.resize(nChunk_, 0);
    addNewNode(assemblyState);
    nodeEdgeOffsetStart_.push_back(0);
    nodeEdgeOffsetEnd_.push_back(0);
}

void StateGraph::evaluateAssembly() {
    if (nodes_.empty())
        return;

    nodes_.front()->cost = 0.0;

    tbb::parallel_for( tbb::blocked_range<int>(1, nodes_.size()),
                      [&](tbb::blocked_range<int> r)
                      {
                          for (int id =r.begin(); id < r.end(); ++id)
                          {
                              evaluateNode(nodes_[id]);
                          }
                      });
}
std::tuple<std::vector<std::vector<int>>, std::vector<double>> StateGraph::getNodeData() {
    std::vector<std::vector<int>> node_xs;
    std::vector<double> label;

    for (unsigned iState = 0; iState < (1 << nPart_); iState++) {
        std::vector<int> x;
        x.resize(nPart_, 0);
        unsigned binary = iState;
        for (int id = 0; id < nPart_; id++) {
            if (binary & 1)
                x[id] = 1;
            else
                x[id] = 0;
            binary >>= 1;
        }
        node_xs.push_back(x);

        std::vector<unsigned> state = {iState};
        auto find_it = map_astate_nodeID_.find(state);

        if (find_it != map_astate_nodeID_.end()) {
            int nodeID = find_it->second;
            label.push_back(nodes_[nodeID]->cost);
        } else {
            label.push_back(std::numeric_limits<double>::max());
        }
    }

    return std::make_tuple(node_xs, label);
}

bool StateGraph::checkEndNode(StateGraph::PtrN node)
{
    return getInstalledParts(node).size() == nPart_;
}


}
