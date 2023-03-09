//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_STATEGRAPHVIZ_H
#define ROBO_CRAFT_STATEGRAPHVIZ_H
#include "StateGraphMultipleArms.h"
namespace search{
class StateGraphViz: public StateGraphMultipleArms
{
public:

    StateGraphViz(std::shared_ptr<PartGraph> graph):
          StateGraphMultipleArms(graph, 1){

    }

public:

    typedef boost::adjacency_list < boost::listS,
                                  boost::vecS,
                                  boost::directedS,
                                  boost::no_property,
                                  boost::property < boost::edge_weight_t, double > > graph_t;

    void createBoostGraph(graph_t &graph);

    std::vector<double> computeShortestDistanceSummation();

    std::vector<double> computeShortestDistanceMaximum();
};
}


#endif  //ROBO_CRAFT_STATEGRAPHVIZ_H
