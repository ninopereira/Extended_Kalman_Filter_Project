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
  VectorXd x_in(4);

  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  // generally the parameters for the random noise measurement matrix
  // are provided by the sensor manufacturer. Here we assume the values:
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

  //state covariance matrix P
  // initialise the P matrix
  MatrixXd P_in(4,4);
  P_in << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

  // initialise the F_ matrix
  MatrixXd F_in(4,4);
  F_in << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  // Note: The H matrix for the radar will change every iteration and has to be updated
  // according to the linear approximation (using the Jacobian matrix)
  //  Hj_ = MatrixXd(3, 4);

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises (done later at initialisation stage)
  */
  noise_ax = 9;
  noise_ay = 9;

  //  process covariance matrix
  MatrixXd Q_in(4,4);

  ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
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
    double px = 0;
    double py = 0;
    double vx = 0;
    double vy = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double ro = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
//      double ro_dot = measurement_pack.raw_measurements_(2);

      px = ro * cos(phi);
      py = ro * sin(phi);

      vx = 0.0; // vx can be initialised as zero; KF will make the correct prediction over time
      vy = 0.0; // vx can be initialised as zero; KF will make the correct prediction over time
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
      vx = 0.0; // vx can be initialised as zero; KF will make the correct prediction over time
      vy = 0.0; // vx can be initialised as zero; KF will make the correct prediction over time
    }

    ekf_.x_ << px, py, vx, vy;

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

  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0 ;
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
}
