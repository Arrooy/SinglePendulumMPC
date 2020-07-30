//
// Created by Adria on 26/6/20.
//

#include "ActuationModelSinglePendulum.h"

ActuationModelSinglePendulum::ActuationModelSinglePendulum(const boost::shared_ptr<crocoddyl::StateAbstract> &state,
                                                           const size_t &nu, size_t nv) : ActuationModelAbstractTpl(state, nu), nv(nv) {
    this->nv = state->get_nv();
}

void ActuationModelSinglePendulum::calc(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
              const Eigen::Ref<const VectorXs> &u) {
    MathBase::MatrixXs S(this->nv,this->nu_);
    S(0,0) = 1;
    data->tau = S * u;
}

void ActuationModelSinglePendulum::calcDiff(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                  const Eigen::Ref<const VectorXs> &u){

    MathBase::MatrixXs S(this->nv,this->nu_);    
    S(0,0) = 1;
    data->dtau_du = S;
}
