//
// Created by Adria on 27/6/20.
//

#include "CostModelSinglePendulum.h"


CostModelSinglePendulum::CostModelSinglePendulum(const boost::shared_ptr<crocoddyl::StateMultibody> &state,
                                                 const boost::shared_ptr<crocoddyl::ActivationModelAbstract> &activation,
                                                 const size_t &nu) : CostModelAbstractTpl(state, activation, nu) 
{
    this->reference_theta = 0;   
    this->reference_dot_theta = 0;
}

void CostModelSinglePendulum::setReference(double new_theta,double new_dot_theta)
{
    this->reference_theta = new_theta;
    this->reference_dot_theta = new_dot_theta;
}

void CostModelSinglePendulum::calc(const boost::shared_ptr<crocoddyl::CostDataAbstract> &data,
                                   const Eigen::Ref<const VectorXs> &x,
                                   const Eigen::Ref<const VectorXs> &u) {
    double c1 = cos(x[0] - reference_theta);
    double s1 = sin(x[0] - reference_theta);

    data->r << s1, 1 - c1, x[1] - reference_dot_theta;
    
    activation_->calc(data->activation,data->r);
    
    data->cost = data->activation->a_value;
}

void CostModelSinglePendulum::calcDiff(const boost::shared_ptr<crocoddyl::CostDataAbstract> &data,
                                       const Eigen::Ref<const VectorXs> &x,
                                       const Eigen::Ref<const VectorXs> &u) {
    double c1 = cos(x[0] - reference_theta);
    double s1 = sin(x[0] - reference_theta);
    
    activation_->calcDiff(data->activation,data->r);

    //Jacobià
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,2);
    J(0,0) = c1;
    J(1,0) = s1;
    J(2,1) = 1;
    J.transposeInPlace();
    data->Lx = J * data->activation->Ar;
    
    //Matriu Hessiana
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,2);
    H(0, 0) = pow(c1,2) - pow(s1,2);
    H(1, 0) = pow(s1,2) + (1 - c1) * c1;
    H(2, 1) = 1;
    H.transposeInPlace();

    MatrixXs A = H * data->activation->Arr.diagonal();
    data->Lxx = A.asDiagonal();
}
