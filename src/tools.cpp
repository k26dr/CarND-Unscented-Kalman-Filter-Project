#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Error: estimations size error" << endl;
  }

  int rmse_length = estimations[0].size();
  VectorXd rmse = VectorXd(rmse_length);
  
  //accumulate squared residuals
  VectorXd residual_sum(rmse_length);
  residual_sum.fill(0);
  for(int i=0; i < estimations.size(); ++i) {
	  VectorXd diff = ground_truth[i] - estimations[i];
      cout << diff << endl;
	  residual_sum += diff.cwiseProduct(diff);
  }

  //calculate the mean
  VectorXd mean(rmse_length);
  mean = 1.0 / rmse_length * residual_sum;

  //calculate the squared root
  rmse = mean.cwiseSqrt();

  return rmse;
}
