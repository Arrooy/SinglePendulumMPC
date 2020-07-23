    //
// Created by adria on 17/07/20.
//

#ifndef SINGLEPENDULUM_CONTROLLER_H
#define SINGLEPENDULUM_CONTROLLER_H


#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solver-base.hpp"

#include "ActuationModelSinglePendulum.h"
#include "CostModelSinglePendulum.h"


#include "src/robot.h"
#include <iostream>
#include <chrono>

#include <thread>
#include "Graph_Logger.h"

#include <chrono>
#include <future>
#include <algorithm>    // std::rotate#include <algorithm>    // std::rotate

#include "yaml-cpp/yaml.h"


#define USE_GRAPHS true

class Controller
{
private:
    // Model related vars
    pinocchio::Model model;
    boost::shared_ptr<crocoddyl::StateMultibody> state;
    boost::shared_ptr<ActuationModelSinglePendulum> actuation_model;

    // Cost related vars
    boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model_sum;
    boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model_sum;
    
    boost::shared_ptr<crocoddyl::CostModelState> x_reg_cost;
    boost::shared_ptr<crocoddyl::CostModelControl> u_reg_cost;
    boost::shared_ptr<CostModelSinglePendulum> x_goal_cost;

    std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>> differential_models_running;
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> integrated_models_running;

    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> differential_terminal_model;
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> integrated_terminal_model;

    Eigen::VectorXd activation_model_weights;

    boost::shared_ptr<crocoddyl::ShootingProblem> problem;
    boost::shared_ptr<crocoddyl::SolverBoxFDDP> solver;

    // Cost weights
    double x_reg_weight;
    double u_reg_weight;
    double running_model_goal_weight;
    double terminal_model_goal_weight;
    
    double dt;
    
    Eigen::VectorXd torque_limit_ub;
    Eigen::VectorXd torque_limit_lb;

    int trajectory_solver_iterations;
    int mpc_solver_iterations;
    
    bool goto_base_position;
    bool zero_the_initial_position;

    Eigen::VectorXd initial_state;

    // ODrive
    Robot *r;
    ODrive *odrive;

    // Graphs
    Graph_Logger *graph_logger;
    std::promise<void> exitSignal;
    std::future<void> futureObj;
    std::thread graphs_thread;
    
    std::vector<Eigen::VectorXd> state_ref;
    std::vector<Eigen::VectorXd> control_ref;

    bool use_callback_verbose;
    int control_loop_iterations;

    // Trajectory vectors
    std::vector<Eigen::VectorXd> trajectory_xs;
    std::vector<Eigen::VectorXd> trajectory_us;

    std::vector<Eigen::VectorXd> mpc_warmStart_xs;// = { xs.begin(), xs.begin() + c.T_MPC };
    std::vector<Eigen::VectorXd> mpc_warmStart_us;// = { us.begin(), us.begin() + c.T_MPC };
    
    double T_ROUTE;
    double T_MPC;

public:
    static bool signalFlag;
        
    Controller();
    Controller(std::string model_path,std::string config_path);
    
    static void signalHandler(int s);
    
    void loadModel(std::string path);
    void loadConfig(std::string configPath);
    void createDOCP(bool trajectory);
    void addCallbackVerbose();
    void connectODrive();
    void defineInitialState();
    void startGraphsThread();
    void initGraphs();
    void createTrajectory();
    void showGraphs();
    void stopGraphs();
    void stopMotors();
    
    void controlLoop();

    void setReferences(const std::vector<Eigen::VectorXd>& state_trajectory,
                                       const std::vector<Eigen::VectorXd>& control_trajectory);

    void rollReferences(const Eigen::Ref<Eigen::VectorXd>& new_state,
                                          const Eigen::Ref<Eigen::VectorXd>& new_control);

    double iterationsToSeconds(int iterations);
    int secondsToIterations(int seconds);


    
    ~Controller();
};

#endif