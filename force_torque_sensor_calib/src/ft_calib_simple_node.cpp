/*
 *  ft_calib_node.cpp
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

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <force_torque_sensor_calib/ft_calib.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf/model.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>

using namespace Calibration;

class FTCalibNode
{
 public:
  ros::NodeHandle n_;
  ros::AsyncSpinner *spinner;
  ros::Subscriber topicSub_ft_raw_;
  ros::Subscriber topicSub_joint_states_;
  ros::Subscriber topicSub_Accelerometer_;
  std::mutex joint_states_mutex_;

  FTCalibNode()
  {
    n_ = ros::NodeHandle("~");
    spinner = new ros::AsyncSpinner(1);
    spinner->start();

    getROSParameters();
    topicSub_ft_raw_ = n_.subscribe("ft_raw", 1, &FTCalibNode::topicCallback_ft_raw, this);
    topicSub_joint_states_ =
        n_.subscribe("joint_states", 1, &FTCalibNode::topicCallback_joint_states, this);

    m_pose_counter = 0;
    m_ft_counter = 0;

    m_received_ft = false;
    m_received_imu = false;

    m_finished = false;
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

    m_ft_calib = new FTCalib(m_local_gravitational_acceleration);
  }

  ~FTCalibNode()
  {
    delete spinner;
    delete m_ft_calib;
  }

  bool getROSParameters()
  {
    // Get the name of output calibration file
    if (n_.hasParam("calib_file_name"))
    {
      n_.getParam("calib_file_name", m_calib_file_name);
    }

    else
    {
      ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
      m_calib_file_name = std::string("ft_calib_data.yaml");
    }

    // Get the name of calibration file directory
    if (n_.hasParam("calib_file_dir"))
    {
      n_.getParam("calib_file_dir", m_calib_file_dir);
    }
    else
    {
      ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
      m_calib_file_dir = std::string("~/.ros/ft_calib");
    }

    // Get the name of file to store the gravity and F/T measurements
    if (n_.hasParam("meas_file_name"))
    {
      n_.getParam("meas_file_name", m_meas_file_name);
    }

    else
    {
      ROS_WARN("No meas_file_name parameter, setting to default 'ft_calib_meas.txt'");
      m_meas_file_name = std::string("ft_calib_meas.txt");
    }

    // Get the name of directory to save gravity and force-torque measurements
    if (n_.hasParam("meas_file_dir"))
    {
      n_.getParam("meas_file_dir", m_meas_file_dir);
    }

    else
    {
      ROS_WARN("No meas_file_dir parameter, setting to default '~/.ros/ft_calib' ");
      m_meas_file_dir = std::string("~/.ros/ft_calib");
    }

    if (n_.hasParam("fixed_frame_id"))
    {
      n_.getParam("fixed_frame_id", m_fixed_frame_id);
    }
    else
    {
      ROS_ERROR("NO fixed_frame_id, shutting down node ...");
      n_.shutdown();
      return false;
    }

    if (n_.hasParam("local_gravitational_acceleration"))
    {
      n_.getParam("local_gravitational_acceleration", m_local_gravitational_acceleration);
    }
    else
    {
      ROS_INFO("No local_gravity_acceleration, setting to 9.80665");
      m_local_gravitational_acceleration = 9.80665;
    }

    // Read poses
    if (n_.hasParam("poses"))
    {
      XmlRpc::XmlRpcValue calibration_poses_param;
      n_.getParam("poses", calibration_poses_param);
      ROS_ASSERT(calibration_poses_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < calibration_poses_param.size(); ++i)
      {
        auto one_pose = calibration_poses_param[i];
        ROS_ASSERT(one_pose.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        JointAngleMap joint_angle_map;
        for (auto joint_name_and_value : one_pose)
        {
          const std::string joint_name = static_cast<std::string>(joint_name_and_value.first);
          const double joint_angle = static_cast<double>(joint_name_and_value.second);
          joint_angle_map[joint_name] = joint_angle;
        }
        m_calibration_poses.emplace_back(joint_angle_map);
      }
    }
    else
    {
      ROS_ERROR("Cannnot find calibration poses by ~poses parameter");
      return false;
    }

    if (n_.hasParam("controller_name"))
    {
      n_.getParam("controller_name", m_controller_name);
    }
    else
    {
      ROS_ERROR("Cannnot find ~controller_name parameter");
      return false;
    }

    if (n_.hasParam("pose_send_duration_rate"))
    {
      n_.param("pose_send_duration_rate", m_pose_send_duration_rate, 0.1);
    }

    if (!m_urdf_model.initParam("/robot_description"))
    {
      ROS_ERROR("Failed to initialize urdf model from robot_description");
      return false;
    }

    // initialize the file with gravity and F/T measurements

    // expand the path
    if (!m_meas_file_dir.empty() && m_meas_file_dir[0] == '~')
    {
      assert(m_meas_file_dir.size() == 1 or m_meas_file_dir[1] == '/');  // or other error handling
      char const *home = getenv("HOME");
      if (home or (home = getenv("USERPROFILE")))
      {
        m_meas_file_dir.replace(0, 1, home);
      }
      else
      {
        char const *hdrive = getenv("HOMEDRIVE"), *hm_meas_file_dir = getenv("HOMEPATH");
        assert(hdrive);  // or other error handling
        assert(hm_meas_file_dir);
        m_meas_file_dir.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
      }
    }

    std::ofstream meas_file;
    meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out);

    std::stringstream meas_file_header;

    meas_file_header << "\% rotation from sensor frame to fixed frame , f/t measurements all "
                        "expressed in F/T sensor frame\n";
    meas_file_header << "\% fixed frame id: " << m_fixed_frame_id << "\n";
    meas_file_header << "\% [rot_qx, rot_qy, rot_qz, rot_qw, fx, fy, fz, tx, ty, tz]\n";
    meas_file << meas_file_header.str();

    meas_file.close();

    return true;
  }
  // connects to the move arm servers
  void init()
  {
    // TODO: initialize actionlib
    m_action_client =
        std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
            m_controller_name, true);
    m_action_client->waitForServer();
  }

  // Calibrates the FT sensor by putting the arm in several different positions
  bool moveNextPose()
  {
    ROS_INFO("move next!");
    if (m_pose_counter >= m_calibration_poses.size())
    {
      m_finished = true;
      return true;
    }

    const auto joint_angle = m_calibration_poses[m_pose_counter];
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint p;
    for (const auto &joint_name_and_value : joint_angle)
    {
      goal.trajectory.joint_names.push_back(joint_name_and_value.first);
      p.positions.push_back(joint_name_and_value.second);
    }
    p.time_from_start = computeDuration(joint_angle);
    goal.trajectory.points.push_back(p);

    m_action_client->sendGoal(goal);
    m_action_client->waitForResult();
    m_pose_counter++;
    return true;
  }

  // gets the next pose from the parameter server
  // pose in [x y z r p y] format ([m], [rad])
  bool getPose(const std::string &pose_param_name, Eigen::Matrix<double, 6, 1> &pose)
  {
    XmlRpc::XmlRpcValue PoseXmlRpc;
    if (n_.hasParam(pose_param_name))
    {
      n_.getParam(pose_param_name, PoseXmlRpc);
    }

    else
    {
      ROS_WARN("Pose parameter %s not found", pose_param_name.c_str());
      return false;
    }

    if (PoseXmlRpc.size() != 6)
    {
      ROS_ERROR("Pose parameter %s wrong size (must be 6)", pose_param_name.c_str());
      return false;
    }

    for (unsigned int i = 0; i < 6; i++) pose(i) = (double)PoseXmlRpc[i];

    return true;
  }

  // prints out the pose (3-D positions) of the calibration frame at each of the positions
  // of the left arm
  void saveCalibData(double &mass, Eigen::Vector3d &g_in_base,
                     Eigen::Vector3d &center_mass_in_sensor_frame, Eigen::Vector3d &f_bias,
                     Eigen::Vector3d &t_bias)
  {
    XmlRpc::XmlRpcValue bias;
    bias.setSize(6);
    for (unsigned int i = 0; i < 3; i++) bias[i] = (double)f_bias(i);

    for (unsigned int i = 0; i < 3; i++) bias[i + 3] = (double)t_bias(i);

    XmlRpc::XmlRpcValue COM_pose;
    COM_pose.setSize(6);
    for (unsigned int i = 0; i < 3; i++) COM_pose[i] = (double)center_mass_in_sensor_frame(i);

    for (unsigned int i = 0; i < 3; i++) COM_pose[i + 3] = 0.0;

    XmlRpc::XmlRpcValue fixed_frame_gravity;
    fixed_frame_gravity.setSize(3);

    for (unsigned int i = 0; i < 3; i++) fixed_frame_gravity[i] = g_in_base(i);

    // set the parameters in the parameter server
    n_.setParam("/ft_calib/bias", bias);
    n_.setParam("/ft_calib/gripper_mass", mass);
    n_.setParam("/ft_calib/gripper_com_frame_id", m_ft_raw.header.frame_id.c_str());
    n_.setParam("/ft_calib/gripper_com_pose", COM_pose);
    n_.setParam("/ft_calib/fixed_frame_id", m_fixed_frame_id.c_str());
    n_.setParam("/ft_calib/fixed_frame_gravity", fixed_frame_gravity);
    // dump the parameters to YAML file
    std::string file = m_calib_file_dir + std::string("/") + m_calib_file_name;

    // first create the directory
    std::string command = std::string("mkdir -p ") + m_calib_file_dir;
    std::system(command.c_str());

    // now dump the yaml file
    command.clear();
    command = std::string("rosparam dump ") + file + std::string(" /ft_calib");
    std::system(command.c_str());
  }

  // saves the gravity and force-torque measurements to a file for postprocessing
  void saveMeasurements(geometry_msgs::TransformStamped transform_sensor_to_base,
                        geometry_msgs::WrenchStamped ft_meas)
  {
    std::ofstream meas_file;
    meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(),
                   std::ios::out | std::ios::app);

    std::stringstream meas_file_text;

    meas_file_text << transform_sensor_to_base.transform.rotation.x << " "
                   << transform_sensor_to_base.transform.rotation.y << " "
                   << transform_sensor_to_base.transform.rotation.z << " "
                   << transform_sensor_to_base.transform.rotation.w << " ";
    meas_file_text << ft_meas.wrench.force.x << " " << ft_meas.wrench.force.y << " "
                   << ft_meas.wrench.force.z << " ";
    meas_file_text << ft_meas.wrench.torque.x << " " << ft_meas.wrench.torque.y << " "
                   << ft_meas.wrench.torque.z << "\n";

    meas_file << meas_file_text.str();

    meas_file.close();
  }

  // finished moving the arm through the poses set in the config file
  bool finished() { return (m_finished); }

  void topicCallback_joint_states(const sensor_msgs::JointState::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    m_latest_joint_states.clear();
    for (int i = 0; i < msg->name.size(); ++i)
    {
      m_latest_joint_states[msg->name[i]] = msg->position[i];
    }
    m_received_joint_states = true;
  }

  void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    ROS_DEBUG("In ft sensorcallback");
    m_ft_raw = *msg;
    m_received_ft = true;
  }

  void addMeasurement()
  {
    m_ft_avg.wrench.force.x = -m_ft_avg.wrench.force.x / (double)m_ft_counter;
    m_ft_avg.wrench.force.y = -m_ft_avg.wrench.force.y / (double)m_ft_counter;
    m_ft_avg.wrench.force.z = -m_ft_avg.wrench.force.z / (double)m_ft_counter;

    m_ft_avg.wrench.torque.x = -m_ft_avg.wrench.torque.x / (double)m_ft_counter;
    m_ft_avg.wrench.torque.y = -m_ft_avg.wrench.torque.y / (double)m_ft_counter;
    m_ft_avg.wrench.torque.z = -m_ft_avg.wrench.torque.z / (double)m_ft_counter;

    m_ft_counter = 0;

    if (!m_received_ft)
    {
      ROS_ERROR("Haven't received F/T sensor measurements");
      return;
    }

    if (!m_received_joint_states)
    {
      ROS_ERROR("Haven't received joint states");
      return;
    }

    if (!m_received_imu)
    {
      ROS_ERROR("Haven't received accelerometer readings, but ignore");
      // return;
    }

    geometry_msgs::TransformStamped transform_sensor_to_base;
    try
    {
      transform_sensor_to_base = tfBuffer.lookupTransform(
          m_ft_avg.header.frame_id, m_fixed_frame_id, ros::Time(0), ros::Duration(4.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    m_ft_calib->addMeasurement(transform_sensor_to_base, m_ft_avg);
    saveMeasurements(transform_sensor_to_base, m_ft_avg);
  }

  void getCalib(double &mass, Eigen::Vector3d &g_in_base,
                Eigen::Vector3d &center_mass_in_sensor_frame, Eigen::Vector3d &f_bias,
                Eigen::Vector3d &t_bias)
  {
    Eigen::VectorXd ft_calib = m_ft_calib->getCalib();

    mass = ft_calib(0);
    if (mass <= 0.0)
    {
      ROS_ERROR("Error in estimated mass (<= 0)");
      //		return;
    }
    g_in_base = ft_calib.segment<3>(1);
    center_mass_in_sensor_frame = ft_calib.segment<3>(4);

    f_bias = -ft_calib.segment<3>(7);
    t_bias = -ft_calib.segment<3>(10);
    /*
    f_bias(0) = -ft_calib(4);
    f_bias(1) = -ft_calib(5);
    f_bias(2) = -ft_calib(6);
    t_bias(0) = -ft_calib(7);
    t_bias(1) = -ft_calib(8);
    t_bias(2) = -ft_calib(9);
    */
  }

  void averageFTMeas()
  {
    if (m_ft_counter == 0)
    {
      m_ft_avg = m_ft_raw;
    }

    else
    {
      m_ft_avg.wrench.force.x = m_ft_avg.wrench.force.x + m_ft_raw.wrench.force.x;
      m_ft_avg.wrench.force.y = m_ft_avg.wrench.force.y + m_ft_raw.wrench.force.y;
      m_ft_avg.wrench.force.z = m_ft_avg.wrench.force.z + m_ft_raw.wrench.force.z;

      m_ft_avg.wrench.torque.x = m_ft_avg.wrench.torque.x + m_ft_raw.wrench.torque.x;
      m_ft_avg.wrench.torque.y = m_ft_avg.wrench.torque.y + m_ft_raw.wrench.torque.y;
      m_ft_avg.wrench.torque.z = m_ft_avg.wrench.torque.z + m_ft_raw.wrench.torque.z;
    }
    m_ft_counter++;
  }
  typedef std::map<std::string, double> JointAngleMap;

  ros::Duration computeDuration(const JointAngleMap &next_pose)
  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    ros::Duration max_duration = ros::Duration(0);
    for (const auto &next_joint_name_and_angle : next_pose)
    {
      const std::string joint_name = next_joint_name_and_angle.first;
      const double next_joint_angle = next_joint_name_and_angle.second;
      const double current_joint_angle = m_latest_joint_states[joint_name];
      const double diff_joint = std::abs(next_joint_angle - current_joint_angle);
      const urdf::JointConstSharedPtr urdf_joint = m_urdf_model.getJoint(joint_name);
      if (urdf_joint != nullptr)
      {
        const auto velocity_limit = urdf_joint->limits->velocity;
        ros::Duration duration(diff_joint / velocity_limit / m_pose_send_duration_rate);
        if (max_duration < duration)
        {
          max_duration = duration;
        }
      }
      else
      {
        ROS_ERROR("failed to find joint %s on urdf", joint_name.c_str());
      }
    }
    return max_duration;
  }

 private:
  unsigned int m_pose_counter;
  unsigned int m_ft_counter;

  bool m_finished;

  bool m_received_ft;
  bool m_received_joint_states;
  bool m_received_imu;

  // ft calib stuff
  FTCalib *m_ft_calib;

  // expressed in FT sensor frame
  geometry_msgs::WrenchStamped m_ft_raw;
  geometry_msgs::WrenchStamped m_ft_avg;  // average over 100 measurements

  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  tf2_ros::Buffer tfBuffer;
  //	***** ROS parameters ***** //

  // name of output calibration file
  std::string m_calib_file_name;

  // name of output directory
  // default: ~/.ros/ft_calib
  std::string m_calib_file_dir;

  // name of file with recorded gravity and F/T measurements
  std::string m_meas_file_name;

  // name of directory for saving gravity and F/T measurements
  // default: ~/.ros/ft_calib
  std::string m_meas_file_dir;

  // frame id of a fixed frame, e.g. "base"
  std::string m_fixed_frame_id;

  double m_local_gravitational_acceleration;

  std::string m_controller_name;

  std::vector<JointAngleMap> m_calibration_poses;

  double m_pose_send_duration_rate;

  JointAngleMap m_latest_joint_states;

  std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>
      m_action_client;

  urdf::Model m_urdf_model;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ft_calib_node");
  ros::NodeHandle nh;

  FTCalibNode ft_calib_node;

  ft_calib_node.init();

  /// main loop
  double loop_rate_;
  ft_calib_node.n_.param("loop_rate", loop_rate_, 650.0);
  ros::Rate loop_rate(loop_rate_);  // Hz

  // waiting time after end of each pose to take F/T measurements
  double wait_time;
  ft_calib_node.n_.param("wait_time", wait_time, 4.0);

  bool ret = false;
  unsigned int n_measurements = 0;

  ros::Time t_end_move_arm = ros::Time::now();

  while (ft_calib_node.n_.ok() && !ft_calib_node.finished())
  {
    //		Move the arm, then calibrate sensor
    if (!ret)
    {
      ret = ft_calib_node.moveNextPose();
      t_end_move_arm = ros::Time::now();
    }

    // average 100 measurements to calibrate the sensor in each position
    else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time)
    {
      n_measurements++;
      ft_calib_node.averageFTMeas();  // average over 100 measurements;

      if (n_measurements == 100)
      {
        ret = false;
        n_measurements = 0;

        ft_calib_node.addMeasurement();  // stacks up measurement matrices and FT measurement
        double mass;
        Eigen::Vector3d center_mass_in_sensor_frame;
        Eigen::Vector3d g_in_base_frame;
        Eigen::Vector3d f_bias;
        Eigen::Vector3d t_bias;

        ft_calib_node.getCalib(mass, g_in_base_frame, center_mass_in_sensor_frame, f_bias, t_bias);
        std::cout << "-------------------------------------------------------------" << std::endl;
        std::cout << "Current calibration estimate:" << std::endl;
        std::cout << std::endl << std::endl;

        std::cout << "Mass: " << mass << std::endl << std::endl;

        std::cout << "gravity in fixed frame:" << std::endl;
        std::cout << "[" << g_in_base_frame(0) << ", " << g_in_base_frame(1) << ", "
                  << g_in_base_frame(2) << "]";
        std::cout << std::endl << std::endl;

        std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
        std::cout << "[" << center_mass_in_sensor_frame(0) << ", " << center_mass_in_sensor_frame(1)
                  << ", " << center_mass_in_sensor_frame(2) << "]";
        std::cout << std::endl << std::endl;

        std::cout << "FT bias: " << std::endl;
        std::cout << "[" << f_bias(0) << ", " << f_bias(1) << ", " << f_bias(2) << ", ";
        std::cout << t_bias(0) << ", " << t_bias(1) << ", " << t_bias(2) << "]";
        std::cout << std::endl << std::endl;

        std::cout << "-------------------------------------------------------------" << std::endl
                  << std::endl
                  << std::endl;
        ft_calib_node.saveCalibData(mass, g_in_base_frame, center_mass_in_sensor_frame, f_bias,
                                    t_bias);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  double mass;
  Eigen::Vector3d center_mass_in_sensor_frame;
  Eigen::Vector3d g_in_base_frame;
  Eigen::Vector3d f_bias;
  Eigen::Vector3d t_bias;
  ft_calib_node.getCalib(mass, g_in_base_frame, center_mass_in_sensor_frame, f_bias, t_bias);
  ft_calib_node.saveCalibData(mass, g_in_base_frame, center_mass_in_sensor_frame, f_bias, t_bias);
  ros::shutdown();
  return 0;
}
