#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"

int main(){
	Eigen::VectorXd x_test(4);
	x_test << 1, 2, 0.2, 0.4;

	Tools test_tool;

	Eigen::MatrixXd Hj = test_tool.CalculateJacobian(x_test);

	std::cout << "The value of the test matrix is : " << Hj << endl;

}