//
// Created by 汪子琦 on 11.05.22.
//

#ifndef ROBO_CRAFT_FRAMETOKNITROSOLVER_H
#define ROBO_CRAFT_FRAMETOKNITROSOLVER_H

#include "FrameTO.h"
#include "FrameTOSequence.h"
#include "knitro.h"
#include "tbb/tick_count.h"


namespace dto
{

struct FrameToData
{
    FrameTO *assembly;
    double objective;
    double lbnd_obj;
    std::vector<double> time;
    std::vector<double> compliance;
    std::vector<double> low_bnd;
    tbb::tick_count start_timer;
    bool silence ;
};

/*------------------------------------------------------------------*/
/*     FUNCTION callbackEvalFCGA                                    */
/*------------------------------------------------------------------*/

static int callbackEvalFCGA(KN_context_ptr kc,
                          CB_context_ptr cb,
                          KN_eval_request_ptr const evalRequest,
                          KN_eval_result_ptr const evalResult,
                          void *const userParams){
    const double *x;
    double *obj;
    double *c;
    double *objGrad;
    double *jac;

    if (evalRequest->type != KN_RC_EVALFCGA) {
        printf("*** callbackEvalFC incorrectly called with eval type %d\n", evalRequest->type);
        return (-1);
    }
    FrameTO * assembly = (FrameTO *)userParams;

    x = evalRequest->x;

    obj = evalResult->obj;
    c = evalResult->c;
    objGrad = evalResult->objGrad;
    jac = evalResult->jac;

    assembly->computeObjectiveGradient(x, obj, objGrad);
    assembly->computeConstraintsGradient(x, c, jac);
    return( 0 );
}

/*------------------------------------------------------------------*/
/*     FUNCTION callbackEvalH                                    */
/*------------------------------------------------------------------*/

static int frameCallbackEvalH(KN_context_ptr             kc,
                         CB_context_ptr             cb,
                         KN_eval_request_ptr const  evalRequest,
                         KN_eval_result_ptr  const  evalResult,
                         void              * const  userParams){
    const double *x;
    const double *lambda;
    double sigma;
    double *hess;
    if (   evalRequest->type != KN_RC_EVALH
        && evalRequest->type != KN_RC_EVALH_NO_F )
    {
        printf ("*** callbackEvalHess incorrectly called with eval type %d\n",
               evalRequest->type);
        return( -1 );
    }

    x = evalRequest->x;
    lambda = evalRequest->lambda;
    /** Scale objective component of the Hessian by sigma */
    sigma = *(evalRequest->sigma);
    hess = evalResult->hess;

    FrameTO * assembly = (FrameTO *)userParams;
    assembly->computeHessian(x, sigma, lambda, hess);
    return( 0 );
}

static int frameMIPNodeCallBack (KN_context_ptr  kcSub,
                         const double * const  x,
                         const double * const  lambda,
                         void   * const  userParams){

    KN_context_ptr kc = (KN_context_ptr) kcSub;
    FrameToData* data = (FrameToData *)userParams;

    std::vector<double> xdata;

    int numNode = 0;
    double objective;
    double bnd = 0;
    int nV = 0;
    double time = (tbb::tick_count::now() - data->start_timer).seconds();
    KN_get_number_vars(kc, &nV);
    KN_get_mip_relaxation_bnd(kc, &bnd);
    KN_get_mip_incumbent_obj(kc, &objective);

    xdata.resize(nV, 0);
    if(KN_get_mip_incumbent_x(kc, xdata.data()) == 0)
    {
        if(objective < data->objective){
        //if(objective < data->objective || bnd > data->lbnd_obj ){
            data->time.push_back(time);
            data->compliance.push_back(objective / 2);
            data->low_bnd.push_back(bnd / 2);
        }

        if(objective < data->objective)
        {
            data->objective = objective;
            if(!data->silence)
            {
                data->assembly->printSolution(xdata.data());
            }
        }

        if(bnd > data->lbnd_obj){
            data->lbnd_obj = bnd;
        }
    }
    return 0;
}

class FrameTOKnitroSolver
{
public:

    std::shared_ptr<FrameTO> assembly_opt;

    int num_thread = 10;
    int hessian_provided = false;
    bool MINLP = false;
    bool silence = true;
    double maxtime = 1E8;

    int heuristic = KN_MIP_HEUR_STRATEGY_ADVANCED;
    double mip_tol = 1E-3;
    double nlp_tol = 1E-6;

    Eigen::VectorXd init_x_;
    std::vector<double> lb_x_;
    std::vector<double> ub_x_;
    std::vector<int> priorities_;

    FrameToData data;

public:

    double result_incumbent_obj;
    double result_relaxation_bnd;
    int result_number_nodes;
    int result_number_solves;
    double result_real_time;

public:

    FrameTOKnitroSolver(std::shared_ptr<FrameTO> assembly){
        assembly_opt = assembly;
        assembly_opt->initializeX(init_x_);
        assembly_opt->computeVariableBounds(lb_x_, ub_x_);
        assembly_opt->computePriorities(priorities_);
    }

    FrameTOKnitroSolver(const FrameTO & assembly){
        assembly_opt = std::make_shared<FrameTO>(assembly);
        assembly_opt->initializeX(init_x_);
        assembly_opt->computeVariableBounds(lb_x_, ub_x_);
    }

    FrameTOKnitroSolver(const FrameTOSequence& assembly){
        assembly_opt = std::make_shared<FrameTOSequence>(assembly);
        assembly_opt->initializeX(init_x_);
        assembly_opt->computeVariableBounds(lb_x_, ub_x_);
    }

public:

    void setVarBound(int index, double lb, double ub){
        lb_x_[index] = lb;
        ub_x_[index] = ub;
    }

    std::tuple<double, Eigen::VectorXd> computeSolution(){
        Eigen::VectorXd xvar;
        double value = solve(xvar);
        return {value, xvar};
    }

    double solve(Eigen::VectorXd &xvar)
    {
        assembly_opt->setup();
        int  i, nStatus, error;
        KN_context   *kc;

        /** Create a new Knitro solver instance. */
        error = KN_new(&kc);
        if (error) exit(-1);
        if (kc == NULL)
        {
            printf ("Failed to find a valid license.\n");
            return( -1 );
        }

        /** Initialize Knitro with the problem definition. */

        /** Add the variables and specify initial values for them.
        *  Note: any unset lower bounds are assumed to be
        *  unbounded below and any unset upper bounds are
        *  assumed to be unbounded above. */
        int n = assembly_opt->nX();
        error = KN_add_vars(kc, n, NULL);
        if (error) exit(-1);

        for (i=0; i<n; i++)
        {
            error = KN_set_var_primal_init_value(kc, i, init_x_[i]);
            if (error) exit(-1);
        }

        std::vector<int> xTypes;
        assembly_opt->getXTypes(xTypes, MINLP);
        error = KN_set_var_types_all(kc, xTypes.data()); if (error) exit(-1);

        error = KN_set_var_lobnds_all(kc, lb_x_.data());  if (error) exit(-1);
        error = KN_set_var_upbnds_all(kc, ub_x_.data());  if (error) exit(-1);
        if(!priorities_.empty()) error = KN_set_mip_branching_priorities_all(kc, priorities_.data()); if(error) exit(-1);

        /** Add the constraints and set the rhs and coefficients */
        int m = assembly_opt->nC();
        error = KN_add_cons(kc, m, NULL);
        if (error) exit(-1);

        std::vector<double> cmin, cmax;
        assembly_opt->computeConstraintsBound(cmin, cmax);
        error = KN_set_con_lobnds_all(kc, cmin.data());
        error = KN_set_con_upbnds_all(kc, cmax.data());
        if (error) exit(-1);

        /** Add callback to evaluate nonlinear (non-quadratic) terms in the model */
        CB_context   *cb;
        std::vector<int> cst_index;
        for(int id = 0; id < assembly_opt->nC(); id++){
            cst_index.push_back(id);
        }

        error = KN_add_eval_callback (kc, KNTRUE, cst_index.size(), cst_index.data(), callbackEvalFCGA, &cb);

        std::vector<Eigen::Vector2i> jacIndex;
        assembly_opt->computeJacIndex(jacIndex);

        std::vector<int> jacRow;
        std::vector<int> jacCol;
        for(int id = 0; id < jacIndex.size(); id++)
        {
            jacRow.push_back(jacIndex[id][0]);
            jacCol.push_back(jacIndex[id][1]);
        }

        error = KN_set_cb_grad(kc, cb, KN_DENSE, NULL, jacIndex.size(), jacRow.data(), jacCol.data(), callbackEvalFCGA);

        if(hessian_provided == 1){
            std::vector<Eigen::Vector2i> hessianIndex;
            assembly_opt->computeHessIndex(hessianIndex);

            std::vector<int> hessRow;
            std::vector<int> hessCol;
            for(int id = 0; id < hessianIndex.size(); id++)
            {
                hessRow.push_back(hessianIndex[id][0]);
                hessCol.push_back(hessianIndex[id][1]);
            }

            error = KN_set_cb_hess(kc,
                                   cb,
                                   hessianIndex.size(),
                                   hessRow.data(),
                                   hessCol.data(),
                                   frameCallbackEvalH);
            if (error) exit(-1);
        }


        /** Set minimize or maximize (if not set, assumed minimize) */
        error = KN_set_obj_goal(kc, KN_OBJGOAL_MINIMIZE);
        if (error) exit(-1);

        /** Set option to print output after every iteration. */
        if(silence)
        {
            error = KN_set_int_param (kc, KN_PARAM_OUTLEV, KN_OUTLEV_NONE);
        }
        else{
            error = KN_set_int_param (kc, KN_PARAM_OUTLEV, 4);
        }

        if(hessian_provided == 0) {
            error = KN_set_int_param (kc, KN_PARAM_HESSOPT, KN_HESSOPT_LBFGS);
        }
        else if(hessian_provided == 1){
            error = KN_set_int_param (kc, KN_PARAM_HESSOPT, KN_HESSOPT_EXACT);
        }

        error = KN_set_int_param(kc, KN_PARAM_CONVEX, KN_CONVEX_YES);
        //KN_set_int_param(kc, KN_PARAM_MIP_OUTSUB, 2);

        KN_set_double_param(kc, KN_PARAM_DERIVCHECK_TOL, 1E-4);
        KN_set_double_param(kc, KN_PARAM_MIP_OPTGAPABS, mip_tol);
        KN_set_double_param(kc, KN_PARAM_MIP_OPTGAPREL, mip_tol);
        KN_set_double_param(kc, KN_PARAM_OPTTOL, nlp_tol);
        KN_set_double_param(kc, KN_PARAM_MIP_MAXTIMEREAL, maxtime);

        KN_set_int_param(kc, KN_PARAM_EVAL_FCGA, KN_EVAL_FCGA_YES);

        KN_set_int_param(kc, KN_PARAM_MIP_HEUR_STRATEGY, heuristic);

        //KN_set_int_param(kc, KN_PARAM_DERIVCHECK, KN_DERIVCHECK_ALL);

#ifdef __APPLE__
        KN_set_int_param_by_name(kc, "mip_numthreads", num_thread);
#else
        KN_set_int_param_by_name(kc, "mip_numthreads", 128 );
#endif
        KN_set_int_param_by_name(kc, "convex", 1);

        if (error) exit(-1);

        /** Set User Parameters. */
        error = KN_set_cb_user_params(kc, cb, assembly_opt.get());
        if (error) exit(-1);

        data = FrameToData();
        data.assembly = assembly_opt.get();
        data.objective = 100;
        data.lbnd_obj = -100;
        data.silence = silence;
        data.start_timer = tbb::tick_count::now();
        data.low_bnd.clear();
        data.time.clear();
        data.compliance.clear();
        error = KN_set_mip_node_callback(kc, &frameMIPNodeCallBack, &data);
        if(error) exit(-1);

        /** Solve the problem.
     *
     *  Return status codes are defined in "knitro.h" and described
     *  in the Knitro manual. */
        nStatus = KN_solve (kc);
        std::vector<double> x(assembly_opt->nX());
        double objSol;
        double feasError, optError;
        /** An example of obtaining solution information. */
        error = KN_get_solution(kc, &nStatus, &objSol, x.data(), NULL);
        error = KN_get_abs_feas_error (kc, &feasError);
        error = KN_get_abs_opt_error (kc, &optError);

        xvar = Eigen::VectorXd(x.size());
        xvar.setZero();
        for(int id = 0; id < x.size(); id++){
            xvar[id] = x[id];
        }

        KN_get_solve_time_real(kc, &result_real_time);
        KN_get_mip_incumbent_obj(kc, &result_incumbent_obj);
        KN_get_mip_number_nodes(kc, &result_number_nodes);
        KN_get_mip_number_solves(kc, &result_number_solves);
        KN_get_mip_relaxation_bnd(kc, &result_relaxation_bnd);

        /** Delete the Knitro solver instance. */
        KN_free (&kc);
        data.objective = objSol;

        return objSol;
    }

};

}


#endif  //ROBO_CRAFT_FRAMETOKNITROSOLVER_H
