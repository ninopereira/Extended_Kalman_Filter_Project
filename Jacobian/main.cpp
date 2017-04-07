#include <iostream>
#include "../Eigen/Dense"
#include <vector>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

    /*
     * Compute the Jacobian Matrix
     */

    //predicted state  example
    //px = 1, py = 2, vx = 0.2, vy = 0.4
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    std::cout << "Hj:" << std::endl << Hj << std::endl;



        return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE
    double c1 = px*px + py*py;
    double c2 = sqrt(c1);
    //check division by zero
    if (c1<0.0001)
    {
        std::cout << "Division by zero ERROR!" << std::endl;
        return Hj;
    }
    //compute the Jacobian matrix
    double dp_dpx = px/c2;
    double dp_dpy = py/c2;
    double dp_dvx = 0;
    double dp_dvy = 0;

    double dphi_dpx = -py/c1;
    double dphi_dpy = px/c1;
    double dphi_dvx = 0;
    double dphi_dvy = 0;

    double dpdot_dpx = py*(vx*py - vy*px) / pow(c2,3);
    double dpdot_dpy = px*(vy*px - vx*py) / pow(c2,3);
    double dpdot_dvx = dp_dpx;
    double dpdot_dvy = dp_dpy;

    Hj <<   dp_dpx, dp_dpy, dp_dvx, dp_dvy,
            dphi_dpx, dphi_dpy, dphi_dvx, dphi_dvy,
            dpdot_dpx, dpdot_dpy, dpdot_dvx, dpdot_dvy;

    return Hj;
}
