/*
 *  ft_calib.h
 *  Modified By Chuan QIN, July 2019
 * 	Logs: 
 *    * Remove the require for IMU sensor, the TF of manipulator is used instead.
 *    * The base frame's inclination to ground is not required, only the local gravity acceleration is required (usually 9.81)
 *    * In order to accelerate the calibration SVD process, the force and torque are calibrated separately
 * 
 * 	Original file infomation:
 *  Least squares calibration of:
 *  - Bias of F/T sensor
 *  - Mass of attached gripper
 *  - Location of the center of mass of the gripper
 *
 *  Requires calibrated accelerometer readings
 *  (calibrated with respect to the robot).
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



#ifndef FTCALIB_H_
#define FTCALIB_H_
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>


// Least Squares calibration of bias of FT sensor and the mass and location of the COM of the gripper
namespace Calibration{
  class FTCalib
  {
  public:

    FTCalib(double local_gravity_acceleration);
    virtual ~FTCalib();


    // adds a F/T measurement and the corresponding TF from fixed frame (usually the base of manipulator) to the sensor frame.
    virtual void addMeasurement(const geometry_msgs::TransformStamped &tf_base_to_sensor,
        const geometry_msgs::WrenchStamped &ft_raw);


    // Least squares to estimate the F/T sensor parameters
    // The estimated parameters are :
    //                  [mass, 
    //									 g_x_in_base, g_y_in_base, g_z_in_base,
    //									 x_center_in_sensor_frame, y_center_in_sensor_frame, z_center_in_sensor_frame, 
    //									 f_bias_x, f_bias_y, f_bias_z, 
    //									 t_bias_x, t_bias_y, t_bias_z]
    
    virtual Eigen::VectorXd getCalib();



  protected:

    Eigen::MatrixXd stacked_A_force; // stacked measurement matrices for force
    Eigen::MatrixXd stacked_A_biased_torque; // stacked measurement matrices for torque

    Eigen::VectorXd stacked_measured_force; // stacked force measurements by sensor
    Eigen::VectorXd stacked_measured_torque; // stacked torque measurements by sensor


    unsigned int m_num_meas; // number of stacked measurements;
    double m_local_gravitational_acceleration;
    Eigen::Matrix<double, 3,6> getForceMeasurementMatrix(const geometry_msgs::TransformStamped &tf_base_to_sensor);
    Eigen::Matrix<double, 3,6> getBiasedTorqueMeasurementMatrix(const geometry_msgs::WrenchStamped &ft_raw);

  };
}


#endif /* INERTIALPARAMESTIMATOR_H_ */
