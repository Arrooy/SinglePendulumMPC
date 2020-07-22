#include "Controller.h"

void wait_for_key ();


int main(int argc, char ** argv) {

    Controller c;
    c.loadModel(std::string("/home/adria/TFG/SinglePendulumMPC/SinglePendulum_cpp/single_pendulum_description/urdf/single_pendulum.urdf"));
    c.loadConfig(std::string("/home/adria/TFG/SinglePendulumMPC/config_weight_terminal.yaml"));
    //c.loadConfig(std::string("/home/adria/TFG/SinglePendulumMPC/config_swing_up.yaml"));
    
    c.allocateReferenceVectors();
    
    // Create a trajectory with 300 nodes.
    c.create(true);
    
    if(c.add_callback_verbose)
        c.addCallbackVerbose();

    c.connectODrive();
    
    c.createTrajectory();
    c.initGraphs();
  
    // Extract the trajectory information.
    std::vector<Eigen::VectorXd> xs(c.getXs());
    std::vector<Eigen::VectorXd> us(c.getUs());

    const std::vector<Eigen::VectorXd> mpc_xs = { xs.begin(), xs.begin() + c.T_MPC };
    const std::vector<Eigen::VectorXd> mpc_us = { us.begin(), us.begin() + c.T_MPC };
    
    // Refactor the class to have only 100 nodes for the MPC.
    c.create(false);

    if(c.add_callback_verbose)
        c.addCallbackVerbose();
    
    // c.setReferences(mpc_xs, mpc_us);
  
    c.startGraphsThread();
    c.controlLoop(c.control_loop_iterations, xs, us);
    
    c.stopGraphs();
    c.stopMotors();
    c.showGraphs();

    wait_for_key();

    return 0;
}

/*
int main(int argc, char ** argv) {

    Controller OCP;
    OCP.loadModel(std::string("/home/adria/TFG/SinglePendulumMPC/SinglePendulum_cpp/single_pendulum_description/urdf/single_pendulum.urdf"));
    OCP.loadConfig(std::string("/home/adria/TFG/SinglePendulumMPC/config_weight_terminal.yaml"));
    
    OCP.create(true);
    
    if(OCP.add_callback_verbose)
        OCP.addCallbackVerbose();
    
    OCP.connectODrive();
    OCP.createTrajectory();
    
    //c.initGraphs();
    //c.startGraphsThread();

    c.controlLoop(c.control_loop_iterations);

    //c.stopGraphs();
    c.stopMotors();
    //c.showGraphs();

    wait_for_key();

    return 0;
}
*/

void wait_for_key ()
{
    std::cout << std::endl << "Press ENTER to continue..." << std::endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
    return;
}
