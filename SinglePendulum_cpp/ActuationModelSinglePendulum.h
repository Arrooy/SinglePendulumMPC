//
// Created by adria on 26/6/20.
//

#ifndef SINGLEPENDULUM_ACTUATIONMODELSINGLEPENDULUM_H
#define SINGLEPENDULUM_ACTUATIONMODELSINGLEPENDULUM_H


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


class ActuationModelSinglePendulum: public crocoddyl::ActuationModelAbstract {
public:
    ActuationModelSinglePendulum(const boost::shared_ptr<StateAbstract> &state, const size_t &nu, size_t nv);

private:

    void calc(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
              const Eigen::Ref<const VectorXs> &u) override;

    void calcDiff(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                  const Eigen::Ref<const VectorXs> &u) override;

    size_t nv;
};


#endif //SINGLEPENDULUM_ACTUATIONMODELSINGLEPENDULUM_H
