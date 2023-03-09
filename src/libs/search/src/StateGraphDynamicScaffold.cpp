//
// Created by 汪子琦 on 04.09.22.
//

#include "search/StateGraphDynamicScaffold.h"

namespace search
{

void StateGraphDynamicScaffold::evaluateNode(PtrN node)
{
    std::vector<int> installPartIDs = getInstalledParts(node);
    std::vector<int> fixedPartIDs = getFixedParts(node);
    node->cost = partGraph_->evaluateStability(installPartIDs, fixedPartIDs);
}

void StateGraphDynamicScaffold::expandNode(PtrN node,
                                           std::vector<PtrN> &preChildNodes,
                                           std::vector<PtrN> &newChildNodes)
{
    std::vector<int> nodePartIDs = getInstalledParts(node);
    std::vector<unsigned> nodeState = getAssemblyState(*node);
    std::vector<std::vector<unsigned>> newAssemblyStatesWithoutScaffold;
    std::map<std::vector<unsigned>, bool, StateCompare> nodeIncluded;
    expandNodeWithoutScaffold(nodeState, numArms_, nodeIncluded, newAssemblyStatesWithoutScaffold);

    std::vector<std::vector<unsigned>> newAssemblyStates;
    for(int id = 0; id < newAssemblyStatesWithoutScaffold.size(); id++)
    {
        std::vector<int> installPartList = getInstalledParts(newAssemblyStatesWithoutScaffold[id]);
        std::vector<int> fixPartList = installPartList;
        std::vector<int> newAddPartList; difference(installPartList, nodePartIDs, newAddPartList);

        for(auto it = fixPartList.begin(); it != fixPartList.end();){
            if(simplifyScaffoldSpace
                && checkFixedPartinStartnEndFixed(*it) == false
                && find(newAddPartList.begin(), newAddPartList.end(), *it) == newAddPartList.end()){
                it = fixPartList.erase(it);
            }
            else{
                it ++;
            }
        }

        std::vector<std::vector<unsigned>> tmp;
        enumerateScaffold(installPartList, fixPartList, tmp);
        newAssemblyStates.insert(newAssemblyStates.end(), tmp.begin(), tmp.end());
    }

    //std::cout << newAssemblyStates.size() << std::endl;
    addNodes(newAssemblyStates, preChildNodes, newChildNodes);

}

void StateGraphDynamicScaffold::addNodes(std::vector<std::vector<unsigned int>> &assemblyStates,
                                         std::vector<PtrN> &preChildNodes,
                                         std::vector<PtrN> &newChildNodes) {

    for (auto &newAssemblyState : assemblyStates)
    {

        std::vector<int> fixedPartIDs = getFixedParts(newAssemblyState);
        std::vector<int> installPartIDs = getInstalledParts(newAssemblyState);

        if(installPartIDs.size() == endPartIDs_.size()){
            if(!checkEndNode(newAssemblyState))
                continue ;
        }

        auto [newNode, createNewNode] = addNewNode(newAssemblyState);

        if (createNewNode)
        {
            newChildNodes.push_back(nodes_[newNode]);
        } else
        {
            preChildNodes.push_back(nodes_[newNode]);
        }
    }
}

void StateGraphDynamicScaffold::enumerateScaffold(const std::vector<int> &installPartList,
                                                  const std::vector<int> &fixedPartsList,
                                                  std::vector<std::vector<unsigned>> &states)
{
    for(int id = 0; id < fixedPartsList.size(); id++)
    {
        std::vector<int> fixedParts;
        fixedParts.push_back(id);
        enumerateScaffold(installPartList, fixedPartsList, fixedParts, states);
    }
}

void StateGraphDynamicScaffold::enumerateScaffold(const std::vector<int> &installPartList,
                                                  const std::vector<int> &fixedPartsList,
                                                  std::vector<int> fixedParts,
                                                  std::vector<std::vector<unsigned int>> &states) {
    if(fixedParts.size() == fixedPartsList.size()
        || fixedParts.size() == numFixtures_)
    {
        std::vector<unsigned int> newState;
        newState.resize(nChunk_, 0);
        for(int id = 0; id < installPartList.size(); id++)
        {
            int partID = installPartList[id];
            setPartMode(partID, Install, newState);
        }
        for (int id = 0; id < fixedParts.size(); id++)
        {
            int partID = fixedPartsList[fixedParts[id]];
            setPartMode(partID, Fix, newState);
        }
        states.push_back(newState);
        return ;
    }

    for(int id = fixedParts.back() + 1; id < fixedPartsList.size(); id++)
    {
        std::vector<int> newFixedParts = fixedParts;
        newFixedParts.push_back(id);
        enumerateScaffold(installPartList, fixedPartsList, newFixedParts, states);
    }
}

std::vector<int> StateGraphDynamicScaffold::getFixedParts(const std::vector<unsigned > &assemblyState){
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned state = computePartState(id, assemblyState);
        if (state == Fix) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::vector<int> StateGraphDynamicScaffold::getFixedParts(StateGraph::PtrN node) {
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned state = computePartState(id, *node);
        if (state == Fix) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::string StateGraphDynamicScaffold::node_label(PtrN node) {
    std::vector<int> parts = getInstalledParts(node);
    std::vector<int> fixedParts = getFixedParts(node);
    std::map<int, bool> mapFixedParts;

    for(int id = 0; id < fixedParts.size(); id++){
        mapFixedParts[fixedParts[id]] = true;
    }

    std::string node_label = "\"";
    for (int id = 0; id < parts.size(); id++)
    {
        node_label += std::to_string(parts[id]);
        if(mapFixedParts[parts[id]]){
            node_label += "*";
        }

        if (id + 1 != parts.size())
        {
            node_label += " ,";
        }
    }
    node_label += "\t(";
    node_label += std::to_string(node->cost);
    node_label += ")";
    node_label += "\"";
    return node_label;
}

void StateGraphDynamicScaffold::getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence)
{
    sequence.steps.clear();
    std::vector<int> prevFixedPartIDs = {};

    for (int id = 1; id < nodes.size(); id++)
    {
        PtrN prevNode = nodes[id - 1];
        PtrN currNode = nodes[id];

        std::vector<int> prevNodePartIDs = getInstalledParts(prevNode);
        std::vector<int> currNodePartIDs = getInstalledParts(currNode);
        std::vector<int> currFixedPartIDs = getFixedParts(currNode);
        std::vector<int> newInstalledParts;

        std::sort(currNodePartIDs.begin(), currNodePartIDs.end());
        std::sort(prevNodePartIDs.begin(), prevNodePartIDs.end());

        std::set_difference(currNodePartIDs.begin(),
                            currNodePartIDs.end(),
                            prevNodePartIDs.begin(),
                            prevNodePartIDs.end(),
                            std::back_inserter(newInstalledParts));

        std::vector<std::vector<int>> fixedPartIDs;
        if(id != 1 && edgeCost)
        {
            evaluateNode2Node(prevNodePartIDs, currNodePartIDs, prevFixedPartIDs, currFixedPartIDs, fixedPartIDs);
        }
        else{
            fixedPartIDs.push_back(currFixedPartIDs);
        }

        for(int jd = 0; jd < fixedPartIDs.size(); jd++)
        {
            AssemblyStep step;
            if(jd == 0) step.installPartIDs = newInstalledParts;
            step.holdPartIDs = fixedPartIDs[jd];
            sequence.steps.push_back(step);
        }
        prevFixedPartIDs = fixedPartIDs.back();
    }
}

double StateGraphDynamicScaffold::evaluateNode2Node(const std::vector<int> &partA,
                                                    const std::vector<int> &partB,
                                                    const std::vector<int> &scaffoldA,
                                                    const std::vector<int> &scaffoldB,
                                                    AssemblySequence &sequence)
{
    std::vector<int> newInstalledParts;
    difference(partB, partA,  newInstalledParts);

    std::vector<int> unionScaffold;
    unionSet(scaffoldA, scaffoldB, unionScaffold);

    AssemblyStep step;
    step.holdPartIDs = unionScaffold;
    step.installPartIDs = {};
    sequence.steps.push_back(step);

    step.holdPartIDs = unionScaffold;
    step.installPartIDs = newInstalledParts;
    sequence.steps.push_back(step);

    step.holdPartIDs = scaffoldB;
    step.installPartIDs = {};
    sequence.steps.push_back(step);
    return 0;
}

double StateGraphDynamicScaffold::evaluateNode2Node(const std::vector<int> &partA,
                                                    const std::vector<int> &partB,
                                                    const std::vector<int> &scaffoldA,
                                                    const std::vector<int> &scaffoldB,
                                                    std::vector<std::vector<int>> &fixedPartIDs) {
    std::vector<int> newScaffold = scaffoldA;
    fixedPartIDs.push_back(newScaffold);

    double edge_cost = partGraph_->evaluateStability(partB, newScaffold);

    while(true)
    {
        std::vector<int> releaseList, installList;
        difference(newScaffold, scaffoldB,  releaseList);
        difference(scaffoldB, newScaffold,  installList);

        if(installList.empty()) break;

        int resultID;
        if(newScaffold.size() == numFixtures_)
        {
            edge_cost = std::max(edge_cost, getReleaseNode(partB, newScaffold, releaseList, resultID));
            if(resultID == -1) return std::numeric_limits<double>::max();
            newScaffold.erase(std::remove(newScaffold.begin(), newScaffold.end(), resultID));
            fixedPartIDs.push_back(newScaffold);
        }

        // add one bar in nodeB
        edge_cost = std::max(edge_cost, getInstallNode(partB, newScaffold, installList, resultID));
        if(resultID == -1) return std::numeric_limits<double>::max();
        newScaffold.push_back(resultID);
        fixedPartIDs.push_back(newScaffold);

    }
    return edge_cost;
}


double StateGraphDynamicScaffold::evaluateNode2Node(StateGraph::PtrN nA,
                                                    StateGraph::PtrN nB,
                                                    std::vector<std::vector<int>> &fixedPartIDs)
{

    std::vector<int> partA = getInstalledParts(nA);
    std::vector<int> partB = getInstalledParts(nB);

    std::vector<int> scaffoldA = getFixedParts(nA);
    std::vector<int> scaffoldB = getFixedParts(nB);

    if(!edgeCost)
    {
        return 0.0;
    }

    return evaluateNode2Node(partA, partB, scaffoldA, scaffoldB, fixedPartIDs);
}


double StateGraphDynamicScaffold::evaluateNode2Node(StateGraph::PtrN nA,
                                                      StateGraph::PtrN nB)
{
    std::vector<std::vector<int>> scaffolds;
    return evaluateNode2Node(nA, nB, scaffolds);
}

void StateGraphDynamicScaffold::difference(const std::vector<int> &listA,
                                           const std::vector<int> &listB,
                                           std::vector<int> &listC)
{
    listC.clear();
    for(int id = 0; id < listA.size(); id++)
    {
        int eA = listA[id];
        if(std::find(listB.begin(), listB.end(), eA) == listB.end()){
            listC.push_back(eA);
        }
    }
}

void StateGraphDynamicScaffold::enumerateNextAssemblyStates(int preStepOffset, int currStepOffset)
{
    std::map<std::vector<unsigned>, int, StateCompare> map_astate_nodeID;

    for (int iS = preStepOffset; iS < currStepOffset; iS++)
    {
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
            edge_costs_.push_back(evaluateNode2Node(prevNode, childNodes[id]));
            nodeEdgeOffsetEnd_[prevNode->nodeID]++;
        }
    }
}

void StateGraphDynamicScaffold::createRootNode()
{
    std::vector<unsigned> assemblyState;
    assemblyState.resize(nChunk_, 0);

    for(int id = 0; id < startPartIDs_.size(); id++)
    {
        int partID = startPartIDs_[id];
        if(std::find(startPartIDsFixed_.begin(), startPartIDsFixed_.end(), partID) != startPartIDsFixed_.end()){
            setPartMode(startPartIDs_[id], Fix, assemblyState);
        }
        else{
            setPartMode(startPartIDs_[id], Install, assemblyState);
        }
    }

    addNewNode(assemblyState);

    evaluateNode(nodes_[0]);

    std::vector<int> installPartIDs = getInstalledParts(nodes_[0]);

    nodeEdgeOffsetStart_.push_back(0);

    nodeEdgeOffsetEnd_.push_back(0);
}

void StateGraphDynamicScaffold::createEndNode(std::vector<unsigned int> &state) {
    state.clear();
    state.resize(nChunk_, 0);

    for(int id = 0; id < endPartIDs_.size(); id++)
    {
        int partID = endPartIDs_[id];
        if(std::find(endPartIDsFixed_.begin(), endPartIDsFixed_.end(), partID) != endPartIDsFixed_.end()){
            setPartMode(partID, Fix, state);
        }
        else{
            setPartMode(partID, Install, state);
        }
    }
}

bool StateGraphDynamicScaffold::checkEndNode(std::vector<unsigned int> &node_state) {
    std::vector<unsigned> state;
    createEndNode(state);

    for(int id = 0; id < nChunk_; id++){
        if(node_state[id] != state[id]){
            return false;
        }
    }
    return true;
}

bool StateGraphDynamicScaffold::checkEndNode(StateGraph::PtrN node)
{
    std::vector<int> partIDs = getInstalledParts(node);
    if(partIDs.size() != endPartIDs_.size()) return false;

    std::vector<unsigned> state;
    createEndNode(state);

    std::vector<unsigned> node_state = getAssemblyState(*node);
    for(int id = 0; id < nChunk_; id++){
        if(node_state[id] != state[id]){
            return false;
        }
    }
    return true;
}
double StateGraphDynamicScaffold::getReleaseNode(const std::vector<int> &partIDs,
                                                 const std::vector<int> &fixedPartIDs,
                                                 const std::vector<int> &releasePartIDs,
                                                 int &resultID) {
    // remove one bar in nodeA
    double min_cost = std::numeric_limits<double>::max();
    resultID = -1;
    for(int id = 0; id < releasePartIDs.size(); id++)
    {
        int partID = releasePartIDs[id];
        std::vector<int> newFixedPartIDs = fixedPartIDs;
        newFixedPartIDs.erase(std::remove(newFixedPartIDs.begin(), newFixedPartIDs.end(), partID));
        double cost = partGraph_->evaluateStability(partIDs, newFixedPartIDs);
        if(min_cost > cost)
        {
            min_cost = cost;
            resultID = partID;
        }
    }
    return min_cost;
}
double StateGraphDynamicScaffold::getInstallNode(const std::vector<int> &partIDs,
                                                 const std::vector<int> &fixedPartIDs,
                                                 const std::vector<int> &installPartIDs,
                                                 int &resultID)
{
    double min_cost = std::numeric_limits<double>::max();
    resultID = -1;
    for(int id = 0; id < installPartIDs.size(); id++)
    {
        int partID = installPartIDs[id];
        std::vector<int> newFixedPartIDs = fixedPartIDs;
        newFixedPartIDs.push_back(partID);
        double cost = partGraph_->evaluateStability(partIDs, newFixedPartIDs);
        if(min_cost > cost)
        {
            min_cost = cost;
            resultID = partID;
        }
    }
    return min_cost;
}

void StateGraphDynamicScaffold::expandNodeWithoutScaffold(std::vector<unsigned> nodeState,
                                                          int remainArms,
                                                          std::map<std::vector<unsigned>, bool, StateCompare> &nodeIncluded,
                                                          std::vector<std::vector<unsigned >> &assemblyStates)
{
    if (remainArms == 0) return;

    std::vector<int> installPartIDs = getInstalledParts(nodeState);

    std::vector<int> neighbourIDs;
    partGraph_->computeNodesNeighbour(installPartIDs, endPartIDs_,  neighbourIDs);

    for (int id = 0; id < neighbourIDs.size(); id++)
    {
        int partID = neighbourIDs[id];
        std::vector<unsigned> newAssemblyState = nodeState;
        setPartMode(partID, Install, newAssemblyState);

        if (nodeIncluded.find(newAssemblyState) != nodeIncluded.end()) continue;
        nodeIncluded[newAssemblyState] = true;
        assemblyStates.push_back(newAssemblyState);

        StateGraphDynamicScaffold::expandNodeWithoutScaffold(newAssemblyState, remainArms - 1, nodeIncluded, assemblyStates);
    }
}

bool StateGraphDynamicScaffold::checkFixedPartinStartnEndFixed(int partID)
{
    if(find(startPartIDsFixed_.begin(), startPartIDsFixed_.end(), partID) != startPartIDsFixed_.end()){
        return true;
    }

    if(find(endPartIDsFixed_.begin(), endPartIDsFixed_.end(), partID) != endPartIDsFixed_.end()){
        return true;
    }
    return false;
}

void StateGraphDynamicScaffold::unionSet(const std::vector<int> &listA, const std::vector<int> &listB, std::vector<int> &listC)
{
    listC.clear();
    listC.insert(listC.end(), listA.begin(), listA.end());
    listC.insert(listC.end(), listB.begin(), listB.end());
    std::sort(listC.begin(), listC.end());
    listC.erase( unique( listC.begin(), listC.end() ), listC.end() );
    return ;
}

}

