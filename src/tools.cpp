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
  	VectorXd rmse(4);
  	rmse << 0,0,0,0;
	if(estimations.size() != ground_truth.size()
		|| estimations.size() == 0){
	cout << "Invalid estimation or ground_truth data" << endl;
	return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	
	MatrixXd Hj(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px2py2 = px * px + py * py;

	// Check division by zero
	if (px2py2 <= 0.0001){
		std::cout << "CalculateJacobian() - error, dividing by zero, return the empty Hj" << std::endl;
		return Hj;
	} else {
		//Calculate each elements of the matrix
		float Hj11 = px/sqrt(px2py2);
        float Hj12 = py/sqrt(px2py2);
        float Hj21 = -py/px2py2;
        float Hj22 = px/px2py2;
        float Hj31 = (py*(vx*py-vy*px))/pow(px2py2,3/2);
        float Hj32 = (px*(vy*px-vx*py))/pow(px2py2,3/2);
        float Hj33 = px/sqrt(px2py2);
        float Hj34 = py/sqrt(px2py2);

        Hj << Hj11, Hj12, 0, 0,
	          Hj21, Hj22, 0, 0,
	          Hj31, Hj32, Hj33, Hj34;
	    return Hj;
	}
	

	
}
