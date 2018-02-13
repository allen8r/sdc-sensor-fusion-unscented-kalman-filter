#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0) {
    cout << "Error: estimation vector is empty!" << endl;
    return rmse;
  }

  if (estimations.size() != ground_truth.size()) {
    cout << "Error: estimation vector size does not equal ground truth vector size!" << endl;
    return rmse;
  }

  // Add up the squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // <vector>.array() allows element-wise operations
    VectorXd residual_squared = residual.array() * residual.array();
    rmse += residual_squared;
  }

  // Find the mean of the squared residuals
  VectorXd mean_squared_error = rmse / estimations.size();

  // Take the square root of the mean squared error
  rmse = mean_squared_error.array().sqrt();

  return rmse;
}

void Tools::Msg(const string s) {
  cout << s << endl;
}

void Tools::Msg(const double val) {
  cout << val << endl;
}

void Tools::Msg(const Eigen::MatrixXd mat) {
  cout << mat << endl;
}
