
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

#include "yaml_parser/parser_yaml.h"
#include "ActuationModelSinglePendulum.h"
#include "CostModelSinglePendulum.h"

int main(int argc, char ** argv) {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(std::string("/home/adria/TFG/Crocoddyl_adria/SinglePendulum/single_pendulum_description/urdf/single_pendulum.urdf"), model);

    boost::shared_ptr<crocoddyl::StateMultibody> state =
            boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(model));

    boost::shared_ptr<ActuationModelSinglePendulum> actuation_model =
            boost::make_shared<ActuationModelSinglePendulum>(state,1,model.nv);


    boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model =
            boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());


    boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model =
            boost::make_shared<crocoddyl::CostModelSum>(state, actuation_model->get_nu());

	Eigen::VectorXd weights(3);
	weights << 1,1,0.1;
	
	boost::shared_ptr<crocoddyl::CostModelAbstract> xRegCost = boost::make_shared<crocoddyl::CostModelState>(
            state, boost::make_shared<crocoddyl::ActivationModelQuad>(2), state->zero(),
            actuation_model->get_nu());
	boost::shared_ptr<crocoddyl::CostModelAbstract> uRegCost =
            boost::make_shared<crocoddyl::CostModelControl>(state,boost::make_shared<crocoddyl::ActivationModelQuad>(1) ,
			actuation_model->get_nu());

	boost::shared_ptr<CostModelSinglePendulum> xPendCost =  boost::make_shared<CostModelSinglePendulum>(state,
	 		boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(weights),actuation_model->get_nu());

	double dt = 5e-2;

    running_cost_model->addCost("uReg", uRegCost, 1e-6);
	//running_cost_model->addCost("xReg", xRegCost, 1e-6);
    running_cost_model->addCost("xGoal", xPendCost, 1e-7);
    terminal_cost_model->addCost("xGoal", xPendCost, 1e4);

	boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model =
            boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
                    boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, running_cost_model),
                    dt);
    boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model =
            boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
                    boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation_model, terminal_cost_model),
                    dt);

    Eigen::VectorXd tau_lb = Eigen::VectorXd(actuation_model->get_nu());
    Eigen::VectorXd tau_ub = Eigen::VectorXd(actuation_model->get_nu());

	tau_lb.fill(-0.01);
	tau_ub.fill(0.01);
	running_model->set_u_lb(tau_lb);
    running_model->set_u_ub(tau_ub);

	double T = 100;
	Eigen::VectorXd x_0 = Eigen::VectorXd(state->get_nx());
	x_0 << 3.14, 0;

    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> running_models(T, running_model);

    boost::shared_ptr<crocoddyl::ShootingProblem> problem =
            boost::make_shared<crocoddyl::ShootingProblem>(x_0, running_models, terminal_model);

    crocoddyl::SolverFDDP fddp(problem);
    
	std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;
	cbs.emplace_back(boost::make_shared<crocoddyl::CallbackVerbose>());

    fddp.setCallbacks(cbs);
    
    fddp.solve();


	//std::vector<Eigen::VectorXd> xs = fddp.get_xs();
	//std::vector<Eigen::VectorXd> us = fddp.get_us();
	
    return 0;
}
