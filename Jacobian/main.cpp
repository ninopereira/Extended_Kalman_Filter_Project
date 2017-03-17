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
        double px_2 = px*px;
        double py_2 = py*py;
        //check division by zero
        if ( (px_2+py_2)==0 )
        {
            std::cout << "Division by zero ERROR!" << std::endl;
            return Hj;
        }
        //compute the Jacobian matrix
        double dp_dpx = px/(sqrt(px_2+py_2));
        double dp_dpy = py/(sqrt(px_2+py_2));
        double dp_dvx = 0;
        double dp_dvy = 0;

        double dphi_dpx = -py/(px_2+py_2);
        double dphi_dpy = px/(px_2+py_2);
        double dphi_dvx = 0;
        double dphi_dvy = 0;

        double dpdot_dpx = py*(vx*py - vy*px) / pow(px_2+py_2,3/2);
        double dpdot_dpy = px*(vy*px - vx*py) / pow(px_2+py_2,3/2);
        double dpdot_dvx = dp_dpx;
        double dpdot_dvy = dp_dpy;

        Hj <<   dp_dpx, dp_dpy, dp_dvx, dp_dvy,
                dphi_dpx, dphi_dpy, dphi_dvx, dphi_dvy,
                dpdot_dpx, dpdot_dpy, dpdot_dvx, dpdot_dvy;

        return Hj;
    }
