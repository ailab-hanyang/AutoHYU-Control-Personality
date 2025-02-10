#pragma once

#include "acados_solver_integrate_dynamic_bicycle_nmpc.h"
#include <vector>

class IntegrateDynamicBicycleNMPCAcadosWrapper {
    public:
        IntegrateDynamicBicycleNMPCAcadosWrapper(const double& time_step);
        ~IntegrateDynamicBicycleNMPCAcadosWrapper();

        int solve();
        
        void setInitialState(const std::vector<double>& x0);
        void setInitialGuess(int stage, const std::vector<double>& x_guess);
        void setInitialGuess(int stage, const std::vector<double>& x_guess, const std::vector<double>& u_guess);
        void setReference(int stage, const std::vector<double>& yref);
        void setParameters(int stage, const std::vector<double>& p);        
        void setInputConstraints(int stage, const std::vector<double>& lbu, const std::vector<double>& ubu);
        void setStateConstraints(int stage, const std::vector<double>& lbx, const std::vector<double>& ubx);
        void setWeights(int stage, const std::vector<double>& Q_diag,
                                   const std::vector<double>& R_diag);
                                   
        void setTerminalWeights(const std::vector<double>& Qe_diag);
        
        std::vector<double> getState(int stage);
        std::vector<double> getControl(int stage);
        double getSolveTime();
        int getSQPIterations();
        void printSolverInfo();
        double getTimeStep();

        void initializeSolver(const double& time_step);
        
    private:
        integrate_dynamic_bicycle_nmpc_solver_capsule* acados_ocp_capsule;
        ocp_nlp_config* nlp_config;
        ocp_nlp_dims* nlp_dims;
        ocp_nlp_in* nlp_in;
        ocp_nlp_out* nlp_out;
        ocp_nlp_solver* nlp_solver;
        void* nlp_opts;

        double time_step_;
};