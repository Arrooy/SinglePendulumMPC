//
// Created by adria on 27/6/20.
//

#ifndef SINGLEPENDULUM_COSTMODELSINGLEPENDULUM_H
#define SINGLEPENDULUM_COSTMODELSINGLEPENDULUM_H
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

class CostModelSinglePendulum: public crocoddyl::CostModelAbstract 
{
private:
    double reference_theta;
public:

    CostModelSinglePendulum(const boost::shared_ptr<StateMultibody> &state,
                            const boost::shared_ptr<ActivationModelAbstract> &activation, const size_t &nu);

    void calc(const boost::shared_ptr <CostDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
              const Eigen::Ref<const VectorXs> &u) override;

    void calcDiff(const boost::shared_ptr<CostDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                  const Eigen::Ref<const VectorXs> &u) override;

    void setReference(double new_theta);
};


#endif