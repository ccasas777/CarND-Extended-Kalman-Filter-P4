#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  int l = estimations.size();
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd temp(4);
  temp << 0,0,0,0;
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector sizeㄛ  
  // TODO: accumulate squared residuals

  for (int i=0; i < l; ++i) {
    // ... your code here

    temp = (estimations[i]- ground_truth[i]);
    temp = temp.array()*temp.array();
    temp = (temp.array()/l);
    rmse += temp;
  }
    rmse = rmse.array().sqrt();
  // TODO: calculate the mean

  // TODO: calculate the squared root

  // return the result
  return rmse;
}

