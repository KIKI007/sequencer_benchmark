//
// Created by 汪子琦 on 01.12.22.
//
#include "dto/FrameTOAssemblability.h"

void dto::FrameTOAssemblability::setPartialOrders(const std::vector<std::pair<int, int>> &partial_orders)
{
    for(int id = 0; id < partial_orders.size(); id++){
        int partA = partial_orders[id].first;
        int partB = partial_orders[id].second;
        int nPartA = get_dynamic_id(partA);
        int nPartB = get_dynamic_id(partB);
        if(nPartA != -1 && nPartB != -1){
            dynamic_beam_partial_orders.push_back({nPartA, nPartB});
        }
    }
}


void dto::FrameTOAssemblability::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    // constraints
    c[0] = -std::min((int)(numTargetBeam_ - start_partIDs_.size()), nX());
    for (int id = 0; id < nX(); id++) {
        c[0] += x[id];
    }

    for(int id = 0; id < dynamic_beam_partial_orders.size(); id++){
        int xA = dynamic_beam_partial_orders[id].first;
        int xB = dynamic_beam_partial_orders[id].second;
        c[id + 1] = x[xA] - x[xB];
    }

    // grad
    int iJ = 0;
    for (int id = 0; id < nX(); id++) {
        jac[iJ] = 1;
        iJ++;
    }

    for(int id = 0; id < dynamic_beam_partial_orders.size(); id++){
        int xA = dynamic_beam_partial_orders[id].first;
        int xB = dynamic_beam_partial_orders[id].second;
        jac[iJ] = 1; iJ++;
        jac[iJ] = -1; iJ++;
    }}

void dto::FrameTOAssemblability::computeJacIndex(std::vector<Eigen::Vector2i> &index)
{
    for (int id = 0; id < nX(); id++) {
        index.push_back(Eigen::Vector2i(0, id));
    }

    for(int id = 0; id < dynamic_beam_partial_orders.size(); id++) {
        int xA = dynamic_beam_partial_orders[id].first;
        int xB = dynamic_beam_partial_orders[id].second;
        index.push_back(Eigen::Vector2i(id + 1, xA));
        index.push_back(Eigen::Vector2i(id + 1, xB));
    }

}

void dto::FrameTOAssemblability::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax)
{
    cmin.push_back(-0.01);
    cmax.push_back(0.01);

    for(int id = 0; id < dynamic_beam_partial_orders.size(); id++) {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }
}


