// @junheelee 
// reference : https://github.dev/SokhengDin/NMPC-ACADOS-CPP
// please modify your wrapper based on c_generated_code/main_*.c files!!!!!

#include "integrate_control/integrate_control_core/integrate_kinematic_bicycle_nmpc/integrate_kinematic_bicycle_nmpc_acados_wrapper.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>

IntegrateKinematicBicycleNMPCAcadosWrapper::IntegrateKinematicBicycleNMPCAcadosWrapper(const double& time_step) 
{

    initializeSolver(time_step);

}

IntegrateKinematicBicycleNMPCAcadosWrapper::~IntegrateKinematicBicycleNMPCAcadosWrapper() {
    if (acados_ocp_capsule) {
        integrate_kinematic_bicycle_nmpc_acados_free(acados_ocp_capsule);
        integrate_kinematic_bicycle_nmpc_acados_free_capsule(acados_ocp_capsule);
    }
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::initializeSolver(const double& time_step){
    acados_ocp_capsule = integrate_kinematic_bicycle_nmpc_acados_create_capsule();
    if (acados_ocp_capsule == nullptr) {
        throw std::runtime_error("Failed to create acados capsule");
    }

    
    time_step_ = time_step;
    std::vector<double> new_time_steps(INTEGRATE_KINEMATIC_BICYCLE_NMPC_N, time_step); 
    int status = integrate_kinematic_bicycle_nmpc_acados_create_with_discretization(acados_ocp_capsule, INTEGRATE_KINEMATIC_BICYCLE_NMPC_N, const_cast<double*>(new_time_steps.data()));
    if (status != 0) {
        throw std::runtime_error("Failed to create acados solver");
    }

    nlp_config = integrate_kinematic_bicycle_nmpc_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = integrate_kinematic_bicycle_nmpc_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = integrate_kinematic_bicycle_nmpc_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = integrate_kinematic_bicycle_nmpc_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = integrate_kinematic_bicycle_nmpc_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = integrate_kinematic_bicycle_nmpc_acados_get_nlp_opts(acados_ocp_capsule);
    
}

int IntegrateKinematicBicycleNMPCAcadosWrapper::solve() {
    return integrate_kinematic_bicycle_nmpc_acados_solve(acados_ocp_capsule);
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setInitialState(const std::vector<double>& x0) {
    if (x0.size() != INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX) {
        throw std::invalid_argument("Invalid initial state size");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", const_cast<double*>(x0.data()));
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setInitialGuess(int stage, const std::vector<double>& x_guess){
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "x",  const_cast<double*>(x_guess.data()));
    // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "u",  const_cast<double*>(u_guess.data()));
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setInitialGuess(int stage, const std::vector<double>& x_guess, const std::vector<double>& u_guess){
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "x",  const_cast<double*>(x_guess.data()));
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "u",  const_cast<double*>(u_guess.data()));
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setReference(int stage, const std::vector<double>& yref) {
    if (stage > INTEGRATE_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", const_cast<double*>(yref.data()));
}


void IntegrateKinematicBicycleNMPCAcadosWrapper::setParameters(int stage, const std::vector<double>& p) {
    if (stage > INTEGRATE_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, stage, "parameter_values", const_cast<double*>(p.data()));
}


void IntegrateKinematicBicycleNMPCAcadosWrapper::setStateConstraints(int stage, const std::vector<double>& lbx, const std::vector<double>& ubx) {
    if (stage > INTEGRATE_KINEMATIC_BICYCLE_NMPC_N || stage == 0) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lbx", const_cast<double*>(lbx.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ubx", const_cast<double*>(ubx.data()));
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setInputConstraints(int stage, const std::vector<double>& lbu, const std::vector<double>& ubu) {
    if (stage > INTEGRATE_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lbu", const_cast<double*>(lbu.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ubu", const_cast<double*>(ubu.data()));
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setWeights(int stage, const std::vector<double>& Q_diag,
                                   const std::vector<double>& R_diag) {
    if (stage > INTEGRATE_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    std::vector<double> W(INTEGRATE_KINEMATIC_BICYCLE_NMPC_NY * INTEGRATE_KINEMATIC_BICYCLE_NMPC_NY, 0.0);   

    for (int i = 0; i < INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX; ++i) {
        W[i * INTEGRATE_KINEMATIC_BICYCLE_NMPC_NY + i] = Q_diag[i];
    }
    for (int i = 0; i < INTEGRATE_KINEMATIC_BICYCLE_NMPC_NU; ++i) {
        W[(INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX + i) * INTEGRATE_KINEMATIC_BICYCLE_NMPC_NY + INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX + i] = R_diag[i];
    }

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W.data());

}

void IntegrateKinematicBicycleNMPCAcadosWrapper::setTerminalWeights(const std::vector<double>& Qe_diag){
    std::vector<double> W_e(INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX * INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX, 0.0);
    for (int i = 0; i < INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX; ++i) {
        W_e[i * INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX + i] = Qe_diag[i];
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, INTEGRATE_KINEMATIC_BICYCLE_NMPC_N, "W", W_e.data());
}


std::vector<double> IntegrateKinematicBicycleNMPCAcadosWrapper::getState(int stage) {
    std::vector<double> state(INTEGRATE_KINEMATIC_BICYCLE_NMPC_NX);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", state.data());
    return state;
}

std::vector<double> IntegrateKinematicBicycleNMPCAcadosWrapper::getControl(int stage) {
    if (stage >= INTEGRATE_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage for control");
    }
    std::vector<double> control(INTEGRATE_KINEMATIC_BICYCLE_NMPC_NU);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", control.data());
    return control;
}

double IntegrateKinematicBicycleNMPCAcadosWrapper::getSolveTime() {
    double solve_time;
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &solve_time);
    return solve_time;
}

int IntegrateKinematicBicycleNMPCAcadosWrapper::getSQPIterations() {
    int sqp_iter;
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    return sqp_iter;
}

double IntegrateKinematicBicycleNMPCAcadosWrapper::getTimeStep(){
    return time_step_;
}

void IntegrateKinematicBicycleNMPCAcadosWrapper::printSolverInfo() {
    std::cout << "\nSolver info:\n";
    std::cout << " SQP iterations " << getSQPIterations() << std::endl;
    std::cout << " Solve time " << getSolveTime() * 1000 << " [ms]\n";
}
