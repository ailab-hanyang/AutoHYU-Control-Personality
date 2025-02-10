// @junheelee 
// reference : https://github.dev/SokhengDin/NMPC-ACADOS-CPP
// please modify your wrapper based on c_generated_code/main_*.c files!!!!!

#include "lateral_control/lateral_control_core/lateral_kinematic_bicycle_nmpc/lateral_kinematic_bicycle_nmpc_acados_wrapper.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>

LateralKinematicBicycleNMPCAcadosWrapper::LateralKinematicBicycleNMPCAcadosWrapper(const double& time_step) 
{

    initializeSolver(time_step);

}

LateralKinematicBicycleNMPCAcadosWrapper::~LateralKinematicBicycleNMPCAcadosWrapper() {
    if (acados_ocp_capsule) {
        lateral_kinematic_bicycle_nmpc_acados_free(acados_ocp_capsule);
        lateral_kinematic_bicycle_nmpc_acados_free_capsule(acados_ocp_capsule);
    }
}

void LateralKinematicBicycleNMPCAcadosWrapper::initializeSolver(const double& time_step){
    acados_ocp_capsule = lateral_kinematic_bicycle_nmpc_acados_create_capsule();
    if (acados_ocp_capsule == nullptr) {
        throw std::runtime_error("Failed to create acados capsule");
    }

    
    time_step_ = time_step;
    std::vector<double> new_time_steps(LATERAL_KINEMATIC_BICYCLE_NMPC_N, time_step); 
    int status = lateral_kinematic_bicycle_nmpc_acados_create_with_discretization(acados_ocp_capsule, LATERAL_KINEMATIC_BICYCLE_NMPC_N, const_cast<double*>(new_time_steps.data()));
    if (status != 0) {
        throw std::runtime_error("Failed to create acados solver");
    }

    nlp_config = lateral_kinematic_bicycle_nmpc_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = lateral_kinematic_bicycle_nmpc_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = lateral_kinematic_bicycle_nmpc_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = lateral_kinematic_bicycle_nmpc_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = lateral_kinematic_bicycle_nmpc_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = lateral_kinematic_bicycle_nmpc_acados_get_nlp_opts(acados_ocp_capsule);
    
}

int LateralKinematicBicycleNMPCAcadosWrapper::solve() {
    return lateral_kinematic_bicycle_nmpc_acados_solve(acados_ocp_capsule);
}

void LateralKinematicBicycleNMPCAcadosWrapper::setInitialState(const std::vector<double>& x0) {
    if (x0.size() != LATERAL_KINEMATIC_BICYCLE_NMPC_NX) {
        throw std::invalid_argument("Invalid initial state size");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", const_cast<double*>(x0.data()));
}

void LateralKinematicBicycleNMPCAcadosWrapper::setInitialGuess(int stage, const std::vector<double>& x_guess){
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "x",  const_cast<double*>(x_guess.data()));
    // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, stage, "u",  const_cast<double*>(u_guess.data()));
}

void LateralKinematicBicycleNMPCAcadosWrapper::setReference(int stage, const std::vector<double>& yref) {
    if (stage > LATERAL_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", const_cast<double*>(yref.data()));
}


void LateralKinematicBicycleNMPCAcadosWrapper::setParameters(int stage, const std::vector<double>& p) {
    if (stage > LATERAL_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, stage, "parameter_values", const_cast<double*>(p.data()));
}


void LateralKinematicBicycleNMPCAcadosWrapper::setStateConstraints(int stage, const std::vector<double>& lbx, const std::vector<double>& ubx) {
    if (stage > LATERAL_KINEMATIC_BICYCLE_NMPC_N || stage == 0) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lbx", const_cast<double*>(lbx.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ubx", const_cast<double*>(ubx.data()));
}

void LateralKinematicBicycleNMPCAcadosWrapper::setInputConstraints(int stage, const std::vector<double>& lbu, const std::vector<double>& ubu) {
    if (stage > LATERAL_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lbu", const_cast<double*>(lbu.data()));
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ubu", const_cast<double*>(ubu.data()));
}

void LateralKinematicBicycleNMPCAcadosWrapper::setWeights(int stage, const std::vector<double>& Q_diag,
                                   const std::vector<double>& R_diag) {
    if (stage > LATERAL_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage");
    }
    std::vector<double> W(LATERAL_KINEMATIC_BICYCLE_NMPC_NY * LATERAL_KINEMATIC_BICYCLE_NMPC_NY, 0.0);   

    for (int i = 0; i < LATERAL_KINEMATIC_BICYCLE_NMPC_NX; ++i) {
        W[i * LATERAL_KINEMATIC_BICYCLE_NMPC_NY + i] = Q_diag[i];
    }
    for (int i = 0; i < LATERAL_KINEMATIC_BICYCLE_NMPC_NU; ++i) {
        W[(LATERAL_KINEMATIC_BICYCLE_NMPC_NX + i) * LATERAL_KINEMATIC_BICYCLE_NMPC_NY + LATERAL_KINEMATIC_BICYCLE_NMPC_NX + i] = R_diag[i];
    }

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W.data());

}

void LateralKinematicBicycleNMPCAcadosWrapper::setTerminalWeights(const std::vector<double>& Qe_diag){
    std::vector<double> W_e(LATERAL_KINEMATIC_BICYCLE_NMPC_NX * LATERAL_KINEMATIC_BICYCLE_NMPC_NX, 0.0);
    for (int i = 0; i < LATERAL_KINEMATIC_BICYCLE_NMPC_NX; ++i) {
        W_e[i * LATERAL_KINEMATIC_BICYCLE_NMPC_NX + i] = Qe_diag[i];
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, LATERAL_KINEMATIC_BICYCLE_NMPC_N, "W", W_e.data());
}


std::vector<double> LateralKinematicBicycleNMPCAcadosWrapper::getState(int stage) {
    std::vector<double> state(LATERAL_KINEMATIC_BICYCLE_NMPC_NX);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", state.data());
    return state;
}

std::vector<double> LateralKinematicBicycleNMPCAcadosWrapper::getControl(int stage) {
    if (stage >= LATERAL_KINEMATIC_BICYCLE_NMPC_N) {
        throw std::invalid_argument("Invalid stage for control");
    }
    std::vector<double> control(LATERAL_KINEMATIC_BICYCLE_NMPC_NU);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", control.data());
    return control;
}

double LateralKinematicBicycleNMPCAcadosWrapper::getSolveTime() {
    double solve_time;
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &solve_time);
    return solve_time;
}

int LateralKinematicBicycleNMPCAcadosWrapper::getSQPIterations() {
    int sqp_iter;
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    return sqp_iter;
}

double LateralKinematicBicycleNMPCAcadosWrapper::getTimeStep(){
    return time_step_;
}

void LateralKinematicBicycleNMPCAcadosWrapper::printSolverInfo() {
    std::cout << "\nSolver info:\n";
    std::cout << " SQP iterations " << getSQPIterations() << std::endl;
    std::cout << " Solve time " << getSolveTime() * 1000 << " [ms]\n";
}
