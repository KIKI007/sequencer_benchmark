//
// Created by 汪子琦 on 10.03.22.
//

#ifndef ROBO_CRAFT_STATEGRAPH_H
#define ROBO_CRAFT_STATEGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

#include "frame/FrameAssembly.h"
#include "search/AssemblySequence.h"
#include "search/PartGraph.h"

namespace search
{
class StateGraphNode
{
public:
    int chunkOffset;
    int nodeID;
    double cost;
};

struct StateCompare {
    bool operator()(const std::vector<unsigned> &nodeA, const std::vector<unsigned> &nodeB) const {
        for (int id = 0; id < nodeA.size(); id++) {
            if (nodeA[id] < nodeB[id]) {
                return true;
            }
            if (nodeA[id] > nodeB[id]) {
                return false;
            }
        }
        return false;
    }
};

/*
 * part_mode: Every part has its own mode (uninstalled/installed/fixed ..)
 * assembly_state: Combine all parts' mode as an integer list.
*/

class StateGraph {
public:
    typedef std::shared_ptr<StateGraphNode> PtrN;

public:
    int nPart_;  //number of parts

    int nMode_;  //number of state for each part

    int nChunk_;  //number of chunk required for describing the states of the assembly

    const int chunkBit_ = 32;  //the number of bit each chuck can have

public:
    std::vector<unsigned> assemblyStates_;

    std::shared_ptr<PartGraph> partGraph_;

    std::vector<unsigned long> edges_;

    std::vector<double> edge_costs_;

    std::vector<PtrN> nodes_;

    std::vector<unsigned> nodeEdgeOffsetStart_;

    std::vector<unsigned> nodeEdgeOffsetEnd_;

    std::map<std::vector<unsigned>, unsigned, StateCompare> map_astate_nodeID_;

public:
    StateGraph() {
        nMode_ = nPart_ = 0;
        nChunk_ = 0;
    }

    StateGraph(int nPart, int nMode) : nPart_(nPart), nMode_(nMode) {
        nChunk_ = numChunkRequired();
    }

    StateGraph(std::shared_ptr<PartGraph> graph, int nMode) : partGraph_(graph), nMode_(nMode) {
        nPart_ = graph->nNode();
        nChunk_ = numChunkRequired();
    }

public:
    virtual void clear() {
        assemblyStates_.clear();
        nodeEdgeOffsetEnd_.clear();
        nodeEdgeOffsetStart_.clear();
        nodes_.clear();
        edges_.clear();
        edge_costs_.clear();
        map_astate_nodeID_.clear();
    }

    int numDigitPerMode();

    int numPartPerChunk();

    int numChunkRequired();

    unsigned modeMask(int N);

    std::tuple<unsigned, unsigned> computeChunkOffset(unsigned partID);

    int getChunk(const StateGraphNode &node, int offset);

    unsigned computePartState(unsigned partID, const StateGraphNode &node);

    unsigned computePartState(unsigned partID, const std::vector<unsigned> &assemblyState);

    void setPartMode(unsigned partID, int partState, std::vector<unsigned> &assemblyState);

    std::vector<unsigned> setPartMode(unsigned partID, int partState, const StateGraphNode &node);

    std::vector<unsigned> getAssemblyState(const StateGraphNode &node);

    virtual std::tuple<int, bool> addNewNode(std::vector<unsigned> &newAssemblyState);

    std::vector<int> getInstalledParts(PtrN node);

    std::vector<int> getInstalledParts(const std::vector<unsigned> &assemblyState);

    virtual std::string node_label(PtrN node);

    void writeDotGraph(std::string filename);

    std::string DotGraphString();

    bool virtual visibility(int nodeID) {
        return true;
    }

public:
    virtual void createRootNode();

    virtual bool checkEndNode(PtrN node);

    virtual std::vector<unsigned> finishedAssemblyState() {
        std::vector<unsigned> fullAssemblyState;
        fullAssemblyState.resize(nChunk_, 0);
        for (int id = 0; id < nPart_; id++) {
            setPartMode(id, 1, fullAssemblyState);
        }
        return fullAssemblyState;
    }

    virtual void expandNode(PtrN node, std::vector<PtrN> &preChildNodes, std::vector<PtrN> &newChildNodes){

    };

    void expandNode(PtrN node, int remainArms, std::map<std::vector<unsigned>, bool, StateCompare> &nodeIncluded, std::vector<PtrN> &preChildNodes,
                    std::vector<PtrN> &newChildNodes);

    void enumerateAssemblyStates();

    std::vector<int> computeStablestSubAssemblyWithKParts(int K);

    virtual void enumerateNextAssemblyStates(int preStepOffset, int currStepOffset);

    void evaluateAssembly();

    virtual void evaluateNode(PtrN node) {
        node->cost = 0.0;
    };

    virtual double evaluateNode2Node(PtrN nodeA, PtrN nodeB){
        return 0.0;
    }

    //the nodes must follow the assembling order
    virtual void getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence) {}

    //only support less than 32 parts' assembly
    std::tuple<std::vector<std::vector<int>>, std::vector<double>> getNodeData();
};
}
#endif  //ROBO_CRAFT_STATEGRAPH_H
