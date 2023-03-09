//
// Created by 汪子琦 on 01.12.22.
//

#ifndef PAVILLION_JSON_FRAMETOASSEMBLABILITY_H
#define PAVILLION_JSON_FRAMETOASSEMBLABILITY_H

#include <vector>
#include <map>
#include <Eigen/Dense>
#include "FrameTO.h"
namespace dto
{

class FrameTOAssemblability: public FrameTO
{
public:

    FrameTOAssemblability(const frame::FrameAssembly &assembly) : FrameTO(assembly){

    }

private:
    //first has to be assemble before second
    //x[first] <= x[second]
    std::vector<std::pair<int, int>> dynamic_beam_partial_orders;

public:

    void init();

    int nC() override{
        return 1 + dynamic_beam_partial_orders.size();
    }


    void setPartialOrders(const std::vector<std::pair<int, int>> &partial_orders);

    void computeConstraintsGradient(const double *x, double *c, double *jac) override;

    void computeJacIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeConstraintsBound(std::vector<double> &xmin, std::vector<double> &xmax) override;

public:

};
}

#endif  //PAVILLION_JSON_FRAMETOASSEMBLABILITY_H
