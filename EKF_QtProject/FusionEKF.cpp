#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  // Note: The H matrix for the radar will change every iteration and has to be updated
  // according to the linear approximation (using the Jacobian matrix)
  //  Hj_ = MatrixXd(3, 4);

  // generally the parameters for the random noise measurement matrix
  // are provided by the sensor manufacturer. Here we assume the values:
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises (done later at initialisation stage)
  */
  noise_ax = 9;
  noise_ay = 9;

//  process covariance matrix

//  measurement covariance matrix

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
//    ekf_.x_ = VectorXd(4);
//    ekf_.x_ << 1, 1, 1, 1;
    float px = 0;
    float py = 0;
    float vx = 0;
    float vy = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      px = ro * cos(phi);
      py = ro * sin(phi);
      vx = 0; // vx can be initialised as zero; KF will make the correct prediction over time
      vy = 0; // vx can be initialised as zero; KF will make the correct prediction over time

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
      vx = 0; // vx can be initialised as zero; KF will make the correct prediction over time
      vy = 0; // vx can be initialised as zero; KF will make the correct prediction over time
    }
//    ekf_.x_ << px, py, vx, vy;

    VectorXd x_in(4);
    x_in << px, py, vx, vy;
    ekf_.x_ = x_in;

    // initialise the P matrix
    MatrixXd P_in(4,4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    ekf_.P_ = P_in;

    // initialise the F_ matrix
    MatrixXd F_in(4,4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    ekf_.F_ = F_in;

//    MatrixXd Q_in(4,4);
//    Q_in << 0, 0, 0, 0,
//            0, 0, 0, 0,
//            0, 0, 0, 0,
//            0, 0, 0, 0;

//    ekf_.Q_ = Q_in;
    previous_timestamp_ = measurement_pack.timestamp_ ;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

//  * Update the state transition matrix F according to the new elapsed time.
//   - Time is measured in seconds.

  long dt = measurement_pack.timestamp_ - previous_timestamp_ ;
  previous_timestamp_ = measurement_pack.timestamp_ ;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

//  * Update the process noise covariance matrix Q.
  MatrixXd Q_in(4,4);
  Q_in << pow(dt,4)/4*noise_ax, 0, pow(dt,3)/2*noise_ax, 0,
            0, pow(dt,4)/4*noise_ay, 0, pow(dt,3)/2*noise_ay,
            pow(dt,3)/2*noise_ax, 0, pow(dt,2)*noise_ax, 0,
            0, pow(dt,3)/2*noise_ay, 0, pow(dt,2)*noise_ay;

  ekf_.Q_ = Q_in;

  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // For radar updates
    // In the radar update step, the Jacobian matrix Hj is used to calculate S, K and P.
    // To calculate y, we use the equations that map the predicted location x' from
    // Cartesian coordinates to polar coordinates h(x')
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_; // we need to revert the H_ matrix to the original for the laser
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
