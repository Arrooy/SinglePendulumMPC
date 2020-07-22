
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

#define USE_GRAPHS false

void wait_for_key ();

int main(int argc, char ** argv) {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(std::string("/home/adria/TFG/SinglePendulumFDDP/SinglePendulum_cpp/single_pendulum_description/urdf/single_pendulum.urdf"), model);

    boost::shared_ptr<crocoddyl::StateMultibody> state =
            boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model));

    YAML::Node config = YAML::LoadFile("/home/adria/TFG/SinglePendulumMPC/config.yaml");
    std::cout << "Configuration loaded!"<< std::endl;

    boost::shared_ptr<ActuationModelSinglePendulum> actuation_model =
            boost::make_shared<ActuationModelSinglePendulum>(state, 1, model.nv);


    boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
            boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());


    boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
            boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());


	Eigen::VectorXd weights(3);
	weights <<  config["SinWeight"].as<double>(), config["CosWeight"].as<double>(), config["VelocityWeight"].as<double>();
	
	boost::shared_ptr<crocoddyl::CostModelState> xRegCost = boost::make_shared<crocoddyl::CostModelState>(
            state, boost::make_shared<crocoddyl::ActivationModelQuad>(2), state->zero(),
            actuation_model->get_nu());

	boost::shared_ptr<crocoddyl::CostModelControl> uRegCost =
            boost::make_shared<crocoddyl::CostModelControl>(state,boost::make_shared<crocoddyl::ActivationModelQuad>(1) ,
			actuation_model->get_nu());  

	boost::shared_ptr<CostModelSinglePendulum> xPendCost =  boost::make_shared<CostModelSinglePendulum>(state,
	 		boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(weights),actuation_model->get_nu());
    
    double dt = config["dt"].as<double>();

    running_cost_model->addCost("uReg", uRegCost, config["uReg"].as<double>());
	running_cost_model->addCost("xReg", xRegCost, config["xReg"].as<double>());
    running_cost_model->addCost("xGoal", xPendCost, config["running_model_goal_cost"].as<double>());
    
    terminal_cost_model->addCost("uReg", uRegCost, config["uReg"].as<double>());
    terminal_cost_model->addCost("xReg", xRegCost, config["xReg"].as<double>());
    
    terminal_cost_model->addCost("xGoal", xPendCost, config["terminal_model_goal_cost"].as<double>());
    
	boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
            boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
                    boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, running_cost_model),
                    dt);
                  
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model =
            boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
                    boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, terminal_cost_model),
                    dt);

    Eigen::VectorXd tau_ub = Eigen::VectorXd(actuation_model->get_nu());
    Eigen::VectorXd tau_lb = Eigen::VectorXd(actuation_model->get_nu());

    tau_ub.fill(config["tau_ub"].as<double>());
	tau_lb.fill(config["tau_lb"].as<double>());

    running_model->set_u_ub(tau_ub);
	running_model->set_u_lb(tau_lb);

    terminal_model->set_u_ub(tau_ub);
    terminal_model->set_u_lb(tau_lb);

	double T_OCP = config["T_OCP"].as<double>();
    double T_MPC = config["T_MPC"].as<double>();

    //ODRIVE STUFF
    Robot r = Robot();
    r.set_dt(dt);

    auto *m0 = new Motor(M0);
    
    //Check if there is any odrive connected.
    if(r.odrives.empty()) return -1;
    auto odrive = r.odrives[0];

    std::cout << "Invput voltage is " << odrive->inputVoltage() << std::endl;
    
    //Configure both motors
    odrive->configureMotor(m0);
    
    odrive->m0->setRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    
    if(config["goto_base_position"].as<bool>())
        odrive->m0->moveStartingPosition(150);
    
    if(config["zero_the_initial_position"].as<bool>())
        odrive->m0->zeroPosition();

    odrive->m0->setControlMode(CTRL_MODE_CURRENT_CONTROL);

    //END ODRIVE INIT

    Eigen::VectorXd x_0 = Eigen::VectorXd(state->get_nx());
	x_0 << odrive->m0->castCPRToRad(odrive->m0->getPosEstimate()), odrive->m0->castCPRToRads(odrive->m0->getVelEstimate());

    std::vector<boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>> running_models_OCP(T_OCP, running_model);
    //std::vector<boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>> running_models_MPC(T_MPC, running_model);
    
    boost::shared_ptr<crocoddyl::ShootingProblem> problem_OCP =
            boost::make_shared<crocoddyl::ShootingProblem>(x_0, running_models_OCP, terminal_model);
    
    //boost::shared_ptr<crocoddyl::ShootingProblem> problem_MPC =
    //        boost::make_shared<crocoddyl::ShootingProblem>(x_0, running_models_MPC, terminal_model);
    
    crocoddyl::SolverBoxFDDP fddp_OCP(problem_OCP);

    //crocoddyl::SolverBoxFDDP fddp_MPC(problem_MPC);
    
    //Callback Verbose per informar
	std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;
	cbs.emplace_back(boost::make_shared<crocoddyl::CallbackVerbose>());
    fddp_OCP.setCallbacks(cbs);
    //fddp_MPC.setCallbacks(cbs);

    fddp_OCP.solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR, config["initial_solver_iterations"].as<int>());
    
    #if USE_GRAPHS
    Graph_Logger *graph_logger = new Graph_Logger(dt);
    //Extraiem les posicions i les comandes de torque inicials.
    std::vector<Eigen::VectorXd> xs_eigen = fddp.get_xs();
    std::vector<Eigen::VectorXd> us_eigen = fddp.get_us();

    std::vector<double> xs;
    std::vector<double> us;

    for(auto const& x: xs_eigen)
    {
        xs.push_back(x[0]);
        graph_logger->appendToBuffer("Crocoddyl initial calculated position", x[0]);
        graph_logger->appendToBuffer("Crocoddyl initial calculated velocity", x[1]);
    }

    for(auto const& u: us_eigen)
    {
        us.push_back(u[0]);
        graph_logger->appendToBuffer("Crocoddyl initial calculated current",odrive->m0->castTorqueToCurrent(u[0]));
    }
    
    //Senyals per al thread del graphlogger.    
    std::promise<void> exitSignal;
    std::future<void> futureObj = exitSignal.get_future();
    #endif

    int solver_iterations = config["solver_iterations"].as<int>();
    int data_index = solver_iterations - 1;
    int time_skips = 0;
    //int model_index_mod = config["model_index_mod"].as<double>();

    #if USE_GRAPHS
        std::thread th(&Graph_Logger::adquisitionThread, graph_logger, odrive, std::move(futureObj));
    #endif


    auto start = std::chrono::high_resolution_clock::now();
    auto a = boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(fddp_OCP.get_problem()->get_runningModels()[0]);
    auto b = boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamics>(a->get_differential());
    boost::static_pointer_cast<crocoddyl::CostModelState>(b->get_costs()->get_costs().find("xReg")->second->cost)->set_xref(x_0);
    
    float elapsedTime = ((float) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count());

    std::cout << "Elapsed time in big cast is " << elapsedTime << std::endl;

    start = std::chrono::high_resolution_clock::now();
    xRegCost->set_xref(x_0);
    elapsedTime = ((float) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count());

    std::cout << "Elapsed time in small cast is " << elapsedTime << std::endl;


    /*
    int i = 0;
    while(1){

        auto start = std::chrono::high_resolution_clock::now();
        double torke = fddp_MPC.get_us()[data_index][0];
        odrive->m0->setTorque(torke);
        
        #if USE_GRAPHS
        graph_logger->appendToBuffer("computed currents", odrive->m0->castTorqueToCurrent(torke));
        graph_logger->appendToBuffer("computed positions", fddp.get_xs()[data_index][0]);
        graph_logger->appendToBuffer("computed velocities", fddp.get_xs()[data_index][1]);
        graph_logger->appendToBuffer("computed cost", fddp.get_cost());
        #endif
        
        auto probl = fddp.get_problem();
        
        x_0[0] = odrive->m0->getPosEstimateInRad();
        x_0[1] = odrive->m0->getVelEstimateInRads();
        
        probl->set_x0(x_0);

        //int model_index = T - 1 - round(i / model_index_mod);

        //probl->updateModel(T - 1 - i, terminal_model);
        
        //std::cout << "First terminal model is " << model_index << std::endl;    

        //if(++i >= T){
        //    break;
        //}
        
        fddp.solve(fddp.get_xs(), fddp.get_us(), solver_iterations);

        float elapsedTime = (dt * 1000000.0f) - ((float) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count());
        if(elapsedTime > 0){
            usleep(elapsedTime);
        }else{
            time_skips++;
            if(time_skips >= 10){
                std::cout << "Skipped 2 frames." << std::endl;
                break;
            }
        }
    }

    #if USE_GRAPHS
    std::cout << "Asking Thread to Stop" << std::endl;
    //Set the value in promise
    exitSignal.set_value();
    //Wait for thread to join
    th.join();
    #endif
    
    odrive->m0->disable();
    
    #if USE_GRAPHS
    std::vector<std::string> datasets = {"Crocoddyl initial calculated position","ODrive real position","computed positions"};
    graph_logger->plot("Positions", datasets,"ms","rad", false);
    
    datasets = {"Crocoddyl initial calculated velocity","ODrive real velocity","computed velocities"};
    graph_logger->plot("Velocities", datasets,"ms","rad/s", false);
    
    datasets = {"Crocoddyl initial calculated current","ODrive real current","computed currents"};
    graph_logger->plot("Currents", datasets,"ms","Amps", false);
    
    datasets = {"computed cost","ODrive real position","computed positions"};
    graph_logger->plot("Cost vs position", datasets,"ms","rad", false);
    
    datasets = {"computed cost","ODrive real velocity","computed velocities"};
    graph_logger->plot("Cost vs velocity", datasets,"ms","rad/s", false);
    #endif

    int aux = fddp.get_xs().size() - 2;

    std::cout << "Size is " << aux << std::endl;
    
    std::cout << "Running datas zero cost is " << fddp.get_problem()->get_runningDatas()[0]->cost << std::endl;
    
    std::cout << "Finish values: x0: " << fddp.get_xs()[aux][0] << " x1: " << fddp.get_xs()[aux][1] << " u: " <<  fddp.get_us()[aux][0] << std::endl << "Cost: " << fddp.get_cost() << std::endl;

    wait_for_key();

    #if USE_GRAPHS
    delete graph_logger;
    #endif
    */
    return 0;
}

void wait_for_key ()
{
    std::cout << std::endl << "Press ENTER to continue..." << std::endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
    return;
}
