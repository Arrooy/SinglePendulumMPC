#include "Controller.h"

Controller::Controller()
{
    config_loaded = false;
}

Controller::~Controller()
{
    #if USE_GRAPHS
    if(graph_logger != nullptr)
        delete graph_logger;
    #endif
    if(r != nullptr)
        delete r;
}

void Controller::loadModel(std::string path)
{
    // Create the arm model based on the URDF.
    pinocchio::urdf::buildModel(path, model);
    
    // Create the state vector. Simple pendulum has q and dot_q.
    state = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model));
    actuation_model = boost::make_shared<ActuationModelSinglePendulum>(state, 1, model.nv);

    initial_state = Eigen::VectorXd(state->get_nx());
}

void Controller::allocateReferenceVectors()
{
    // Alloc memory for the reference vectors
    state_ref.resize(T_MPC, state->zero());
    control_ref.resize(T_MPC, Eigen::VectorXd::Zero(1));
}


void Controller::loadConfig(std::string configPath)
{
    YAML::Node config = YAML::LoadFile(configPath);
    // Init all the predefined variables.
    dt = config["dt"].as<double>();
    
    // Costs
    activation_model_weights = Eigen::VectorXd(3);
    activation_model_weights <<  config["SinWeight"].as<double>(), config["CosWeight"].as<double>(), config["VelocityWeight"].as<double>();

    x_reg_weight = config["xReg"].as<double>();
    u_reg_weight = config["uReg"].as<double>();

    running_model_goal_weight = config["running_model_goal_weight"].as<double>();
    terminal_model_goal_weight = config["terminal_model_goal_weight"].as<double>();

    torque_limit_ub = Eigen::VectorXd(actuation_model->get_nu());
    torque_limit_lb = Eigen::VectorXd(actuation_model->get_nu());

    torque_limit_ub.fill(config["tau_ub"].as<double>());
	torque_limit_lb.fill(config["tau_lb"].as<double>());

    T_OCP = config["T_OCP"].as<double>();
    T_MPC = config["T_MPC"].as<double>();

    trajectory_solver_iterations = config["initial_solver_iterations"].as<int>();
    mpc_solver_iterations = config["solver_iterations"].as<int>();

    goto_base_position = config["goto_base_position"].as<bool>();
    zero_the_initial_position = config["zero_the_initial_position"].as<bool>();

    add_callback_verbose = config["add_callback_verbose"].as<bool>();

    control_loop_iterations = config["control_loop_iterations"].as<int>();
    config_loaded = true;
}   

void Controller::create(bool trajectory)
{
    if(!config_loaded)
    {
        std::cout << "Must loadConfig() first!" << std::endl;
        return;
    }

    differential_models_running.clear();
    integrated_models_running.clear();
    
    running_cost_model_sum  = boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());
    terminal_cost_model_sum = boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());
    
    x_reg_cost = boost::make_shared<crocoddyl::CostModelState>(state, boost::make_shared<crocoddyl::ActivationModelQuad>(2), state->zero(),
            actuation_model->get_nu());   
    
    u_reg_cost = boost::make_shared<crocoddyl::CostModelControl>(state,boost::make_shared<crocoddyl::ActivationModelQuad>(1) ,
			actuation_model->get_nu());  

    x_goal_cost = boost::make_shared<CostModelSinglePendulum>(state,
	 		boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(activation_model_weights),actuation_model->get_nu());

    // Add the var regularization
    running_cost_model_sum->addCost("u_reg", u_reg_cost, u_reg_weight);
    running_cost_model_sum->addCost("x_reg", x_reg_cost, x_reg_weight);

    terminal_cost_model_sum->addCost("u_reg", u_reg_cost, u_reg_weight);
    terminal_cost_model_sum->addCost("x_reg", x_reg_cost, x_reg_weight);

    running_cost_model_sum-> addCost("x_goal", x_goal_cost, running_model_goal_weight);
    terminal_cost_model_sum->addCost("x_goal", x_goal_cost, terminal_model_goal_weight);

    std::cout << " Running costs size " << running_cost_model_sum->get_costs().size() << std::endl;

    int nodes = trajectory ? T_OCP : T_MPC;

    for (int i = 0; i < nodes - 1; ++i)
    {
        boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> diff_model = 
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, running_cost_model_sum);

        diff_model->set_u_ub(torque_limit_ub);
        diff_model->set_u_lb(torque_limit_lb);

        boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> int_model =
        boost::make_shared<crocoddyl::IntegratedActionModelEuler>(diff_model, dt);

        differential_models_running.push_back(diff_model);
        integrated_models_running.push_back(int_model);
    }

    differential_terminal_model = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, terminal_cost_model_sum);
    
    differential_terminal_model->set_u_ub(torque_limit_ub);
    differential_terminal_model->set_u_lb(torque_limit_lb);
    
    integrated_terminal_model   = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(differential_terminal_model, dt);

    problem = boost::make_shared<crocoddyl::ShootingProblem>(initial_state, integrated_models_running, integrated_terminal_model);

    solver = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem);
}

void Controller::addCallbackVerbose()
{
    std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;
	cbs.emplace_back(boost::make_shared<crocoddyl::CallbackVerbose>());
    solver->setCallbacks(cbs);
}

//Very Specific code for the simple pendulum.
void Controller::connectODrive()
{
    r = new Robot();
    r->set_dt(dt);

    auto *m0 = new Motor(M0);
    
    //Check if there is any odrive connected.
    if(r->odrives.empty()) return;
    odrive = r->odrives[0];

    std::cout << "Input voltage is " << odrive->inputVoltage() << std::endl;
    
    //Configure both motors
    odrive->configureMotor(m0);
    
    odrive->m0->setRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    
    if(goto_base_position)
        odrive->m0->moveStartingPosition(250);
    
    if(zero_the_initial_position)
        odrive->m0->zeroPosition();

    odrive->m0->setControlMode(CTRL_MODE_CURRENT_CONTROL);
}

void Controller::stopMotors()
{
    odrive->m0->disable();
}

void Controller::defineInitialState()
{
    initial_state << odrive->m0->castCPRToRad(odrive->m0->getPosEstimate()), odrive->m0->castCPRToRads(odrive->m0->getVelEstimate());
}

void Controller::createTrajectory()
{
    initial_state << odrive->m0->castCPRToRad(odrive->m0->getPosEstimate()), odrive->m0->castCPRToRads(odrive->m0->getVelEstimate());
    problem->set_x0(initial_state);
    solver->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR, trajectory_solver_iterations);
    std::cout << "Trajectory generated!" << std::endl;
}


const std::vector<Eigen::VectorXd> Controller::getXs()
{
    return solver->get_xs();
}   

const std::vector<Eigen::VectorXd> Controller::getUs()
{
    return solver->get_us();
}

void Controller::controlLoop(int iterations, std::vector<Eigen::VectorXd>& trajectory_xs, std::vector<Eigen::VectorXd>& trajectory_us)
{
    int data_index = mpc_solver_iterations - 1;
    int time_skips = 0;
    int end_iterations = iterations;
    bool oneTime = true;
    int i = 0, j = 0, index = T_MPC - 2;

    float elapsedTime = 0, torke = 0;

    while(1){
        
        auto start = std::chrono::high_resolution_clock::now();
        
        initial_state[0] = odrive->m0->getPosEstimateInRad();
        initial_state[1] = odrive->m0->getVelEstimateInRads();
        
        problem->set_x0(initial_state);
        
        solver->solve(solver->get_xs(), solver->get_us(), mpc_solver_iterations);
        
        torke = solver->get_us()[data_index][0];
        odrive->m0->setTorque(torke);
    
        #if USE_GRAPHS
        graph_logger->appendToBuffer("computed currents", odrive->m0->castTorqueToCurrent(torke));
        graph_logger->appendToBuffer("computed positions", solver->get_xs()[data_index][0]);
        graph_logger->appendToBuffer("computed velocities", solver->get_xs()[data_index][1]);
        graph_logger->appendToBuffer("computed cost", solver->get_cost());
        #endif
        
        /*
        if(i < T_MPC - 2){
            //problem->updateModel(T_MPC - 2 - i, integrated_terminal_model);
            
            // boost::shared_ptr<CostModelAbstract> x_goal_running_cost = boost::static_pointer_cast<CostModelAbstract>(
            //     differential_models_running[T_MPC - 1 - i]->get_costs()->get_costs().find("x_goal")->second->cost
            // );
        
            // x_goal_running_cost = differential_terminal_model->get_costs()->get_costs().find("x_goal")->second->cost;
            i++;
        }else{
            if(oneTime){
                std::cout << "All nodes are terminal." << std::endl;
                oneTime = false;
            }
            if(end_iterations > 0 && ++j >= end_iterations)
            break;
        }
        */
        if(end_iterations > 0 && ++j >= end_iterations)
            break;

        elapsedTime = (dt * 1000000.0f) - ((float) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count());
        
        if(elapsedTime > 0){
            usleep(elapsedTime);
        }else{
            time_skips++;
            if(time_skips >= 10){
                std::cout << "Skipped 10 frames. Elapsed time was " << elapsedTime << std::endl;
               // break;
            }
        }
    }
    /*
    while(1){

        auto start = std::chrono::high_resolution_clock::now();
        double torke = solver->get_us()[data_index][0];
        odrive->m0->setTorque(torke);
        
        #if USE_GRAPHS
        graph_logger->appendToBuffer("computed currents", odrive->m0->castTorqueToCurrent(torke));
        graph_logger->appendToBuffer("computed positions", solver->get_xs()[data_index][0]);
        graph_logger->appendToBuffer("computed velocities", solver->get_xs()[data_index][1]);
        graph_logger->appendToBuffer("computed cost", solver->get_cost());
        #endif

        initial_state[0] = odrive->m0->getPosEstimateInRad();
        initial_state[1] = odrive->m0->getVelEstimateInRads();
        
        problem->set_x0(initial_state);

        
        // if(++i >= T_OCP - T_MPC - 1){
        //     if(index >= 0){
        //         problem->updateModel(index, integrated_terminal_model);
        //         index --;
        //     }

        //     if(end_iterations > 0 && ++j >= end_iterations)
        //         break;
        // }
        

        if(i < T_MPC - 2){
            //problem->updateModel(T_MPC - 2 - i, integrated_terminal_model);
            
            // boost::shared_ptr<CostModelAbstract> x_goal_running_cost = boost::static_pointer_cast<CostModelAbstract>(
            //     differential_models_running[T_MPC - 1 - i]->get_costs()->get_costs().find("x_goal")->second->cost
            // );
        
            // x_goal_running_cost = differential_terminal_model->get_costs()->get_costs().find("x_goal")->second->cost;
            i++;
        }else{
            if(oneTime){
                std::cout << "All nodes are terminal." << std::endl;
                oneTime = false;
            }
            if(end_iterations > 0 && ++j >= end_iterations)
            break;
        }

        // else{
        //     rollReferences(trajectory_xs[T_MPC + i], trajectory_us[T_MPC + i]);
        // }
        
        solver->solve(solver->get_xs(), solver->get_us(), mpc_solver_iterations);

        float elapsedTime = (dt * 1000000.0f) - ((float) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count());
        
        if(elapsedTime > 0){
            usleep(elapsedTime);
        }else{
            time_skips++;
            if(time_skips >= 10){
                std::cout << "Skipped 10 frames. Elapsed time was " << elapsedTime << std::endl;
               // break;
            }
        }
    }*/
}

double Controller::iterationsToSeconds(int iterations)
{
    return iterations * dt;
}

int Controller::secondsToIterations(int seconds)
{
    return std::ceil((double)seconds / dt);
}

void Controller::setReferences(const std::vector<Eigen::VectorXd>& state_trajectory,
                                       const std::vector<Eigen::VectorXd>& control_trajectory)
{
    
    assert(state_trajectory.size() == state_ref.size());

    std::copy(state_trajectory.begin(), state_trajectory.end(), state_ref.begin());
    std::copy(control_trajectory.begin(), control_trajectory.end(), control_ref.begin());

    for (std::size_t t = 0; t < T_MPC - 1; ++t) {
        
        boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
            differential_models_running[t]->get_costs()->get_costs().find("x_reg")->second->cost);
        cost_state->set_xref(state_ref[t]);
        
        boost::shared_ptr<crocoddyl::CostModelControl> cost_control = boost::static_pointer_cast<crocoddyl::CostModelControl>(
            differential_models_running[t]->get_costs()->get_costs().find("u_reg")->second->cost);
        cost_control->set_uref(control_ref[t]);

        //boost::shared_ptr<CostModelSinglePendulum> x_goal_cost = boost::static_pointer_cast<CostModelSinglePendulum>();   
    }

    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        differential_terminal_model->get_costs()->get_costs().find("x_reg")->second->cost);
    cost_state->set_xref(state_ref[T_MPC - 1]);
}

void Controller::rollReferences(const Eigen::Ref<Eigen::VectorXd>& new_state,
                                          const Eigen::Ref<Eigen::VectorXd>& new_control)
{   
    assert(new_state.size() == state->get_nx());


    for (std::size_t t = 0; t < T_MPC - 1; ++t) {
        state_ref[t] = state_ref[t + 1];

        boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
            differential_models_running[t]->get_costs()->get_costs().find("x_reg")->second->cost);
        cost_state->set_xref(state_ref[t]);

        boost::shared_ptr<crocoddyl::CostModelControl> cost_control = boost::static_pointer_cast<crocoddyl::CostModelControl>(
            differential_models_running[t]->get_costs()->get_costs().find("u_reg")->second->cost);

        if (t < T_MPC - 2) {
            control_ref[t] = control_ref[t + 1];
            cost_control->set_uref(control_ref[t]);
        } else {
            control_ref[t] = new_control;
            cost_control->set_uref(control_ref[t]);
        }
    }

    state_ref[T_MPC - 1] = new_state;
    boost::shared_ptr<crocoddyl::CostModelState> cost_state = boost::static_pointer_cast<crocoddyl::CostModelState>(
        differential_terminal_model->get_costs()->get_costs().find("x_reg")->second->cost);
    cost_state->set_xref(state_ref[T_MPC - 1]);
}



/*******************GRAPHS*****************/

//Use this after the OCP route creation
void Controller::initGraphs()
{
    #if USE_GRAPHS
    graph_logger = new Graph_Logger(dt);

    //Extraiem les posicions i les comandes de torque inicials.
    std::vector<Eigen::VectorXd> xs_eigen = solver->get_xs();
    std::vector<Eigen::VectorXd> us_eigen = solver->get_us();

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
        graph_logger->appendToBuffer("Crocoddyl initial calculated current", odrive->m0->castTorqueToCurrent(u[0]));
    }
    
    //Senyals per al thread del graphlogger.    
    futureObj = exitSignal.get_future();
    #endif
}

void Controller::startGraphsThread()
{
    #if USE_GRAPHS
    graphs_thread = std::thread(&Graph_Logger::adquisitionThread, graph_logger, odrive, std::move(futureObj));
    #endif
}

void Controller::stopGraphs()
{
    #if USE_GRAPHS
    std::cout << "Asking Thread to Stop" << std::endl;
    //Set the value in promise
    exitSignal.set_value();
    
    //Wait for thread to join
    graphs_thread.join();
    #endif
}

void Controller::showGraphs()
{
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
    
    int aux = solver->get_xs().size() - 2;

    std::cout << "Size is " << aux << std::endl;
    
    std::cout << "Running datas zero cost is " << problem->get_runningDatas()[0]->cost << std::endl;
    
    std::cout << "Finish values: x0: " << solver->get_xs()[aux][0] << " x1: " << solver->get_xs()[aux][1] << " u: " <<  solver->get_us()[aux][0] << std::endl << "Cost: " << solver->get_cost() << std::endl;
    #endif
}