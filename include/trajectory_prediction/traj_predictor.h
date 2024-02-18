/*
MIT License

Copyright (c) 2021 SystemTrio Robotics
Copyright (c) 2021 Mohamed Abdelkader, mohamedashraf123@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * Some references:
 *  Circle model fitting in 3D : https://meshlogic.github.io/posts/jupyter/curve-fitting/fitting-a-circle-to-cluster-of-3d-points/
*/

#ifndef TRAJ_PREDICTOR_H
#define TRAJ_PREDICTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>

#include "multi_target_kf/KFTracks.h"
#include "custom_trajectory_msgs/StateTrajectory.h"

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include "prediction_models.h"

#include <mutex>


/************************************************************************/
/*                         TrajectoryPredictor                          */
/************************************************************************/
class TrajectoryPredictor
{
private:
  ros::NodeHandle       _nh;                    /** ROS node handle */
  ros::NodeHandle       _nh_private;            /** ROS private node handle */

  bool                  _debug;                 /** Print debug messages */

  ros::Subscriber       _state_sub;             /** Subscriber to the the current state/odom */
  ros::Subscriber       _kftrack_sub;           /** Subscriber to the Kalman filter tracks estimator */
  ros::Subscriber       _poseArray_sub;           /** Subscriber to target pose array. Just for testing. Should use KF callback!! */

  ros::Publisher        _traj_pub;              /** Pulisher of the predicted trajectory */
  ros::Publisher        _predictedPos_pub;      /** ROS Publisher for precited positions for visulaization in RViz */
  ros::Publisher        _cons_vel_predictedPos_pub;      /** ROS Publisher for precited positions for visulaization in RViz */
  ros::Publisher        _bezier_predictedPos_pub;      /** ROS Publisher for precited positions for visulaization in RViz */
  ros::Publisher        _sampled_poses_pub;      /** ROS Publisher for re-sampled positions measurements, for visulaization in RViz */
  ros::Publisher        _stateBuff_pub;         /** ROS publisher for collected positions for visulaization in RViz */
  ros::Publisher        _const_vel_model_rviz_pub;   /** Published the fitted positions of constant velocity model */
  ros::Publisher        _bezier_model_rviz_pub;   /** Published the fitted positions of Bezier model */
  ros::Publisher        _const_vel_traj_pub;      /** Publisher for the constant velocity trajectory & RMSE */
  ros::Publisher        _bezier_traj_pub;         /** Publisher for the Bezier trajectory & RMSE */

  ros::Timer            _pred_loop_timer;        /** Timer for the prediciton loop with constant loop rate */

  ConstantVelocityModel *_const_vel_model = new ConstantVelocityModel(); /** Constant velocity prediction model */
  DubinsPathModel       *_dubins_model = new DubinsPathModel();
  BezierModel           *_bezier_model = new BezierModel();

  custom_trajectory_msgs::State _current_state_msg;    /** ROS msg of the Current target's estimated state (measurement) */
  custom_trajectory_msgs::State _last_state_msg;    /** Last ROS msg of the target's estimated state (measurement) */
  custom_trajectory_msgs::StateTrajectory _predicted_traj_msg; /** ROS message of the predicited state trajectory */
  custom_trajectory_msgs::StateTrajectory _evaluated_const_vel_traj_msg; /** Holds the predicted trajectory that was used for evaluation only */
  custom_trajectory_msgs::StateTrajectory _evaluated_bezier_traj_msg; /** Holds the predicted trajectory that was used for evaluation only */
  State _latest_resampled_state;                                /** Last resampeld state in _resampeld_measurement_buff */

  boost::circular_buffer<State> _resampeld_measurement_buff;    /** Stores measurements at prediction sampling rate _dt. This is used for predication trajectory evaluation. */
  

  double                _dt;                    /** Prediction Sampling time in seconds */
  double                _data_dt;               /** Observation sampling time. Calculated from observation time stamps. */
  double                _resampling_dt;         /** Resampling time in seconds of the incoming measurements */
  int                   _N;                     /** Number of prediciton steps */
  int                   _MIN_NPOS_BUFF;             /** Minimum Number of measuerment to colelct */
  int                   _MAX_NPOS_BUFF;          /** Maximum number of collected position measurements */
  const int             NUM_OF_STATES=6;        /** Number of states [p_x, v_x, p_y, v_y, p_z, v_z]*/

  int                   _bezier_deg;            /** Bezier curve degree . Used for the Bezier model */
  bool                  _estimate_velocity_bezier;     /** True: Estimate velocity as a bezier curve. False: Estimate only position's B?ezier curve */

  bool                  _calc_speed_form_state;     /** Calculate _current_speed from current state */
  bool                  _publish_state_buff;    /** Publish the collected measurements for visulaization in RViz */

  ros::Time             _last_meas_t;        /** last measurement time. to make sure we consume new measurements only. */
  double                _last_sampled_sate_time;

  std::vector<geometry_msgs::PoseStamped> _predicted_poses; /** ROS msg for predicted positions for visualization */
  std::vector<geometry_msgs::PoseStamped> _predicted_poses_const_vel; /** ROS msg for predicted positions (using const velocity model) for visualization */
  std::vector<geometry_msgs::PoseStamped> _predicted_poses_bezier; /** ROS msg for predicted positions (using bezier model) for visualization */
  std::vector<geometry_msgs::PoseStamped> _predicted_poses_const_vel_to_evaluate;
  std::vector<geometry_msgs::PoseStamped> _predicted_poses_bezier_to_evaluate;

  static constexpr int    CONSTANT_VEL_MODEL=0;
  static constexpr int    DUBINS_MODEL=1;
  static constexpr int    BEZIER_MODEL=2;

  int                   _model_type;      /** User-defined model type (ROS param). This is used to select which prediction model to use */

  double                _rmse_const_vel;                 /** RMSE of constant velocity trajectory */
  double                _rmse_bezier;                 /** RMSE of Bezier trajectory */

  std::mutex _buff_mutex;

  
  /**
  * @brief Odometery callback
  */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
  * @brief Kalman filter tracks callback
  * @param msg multi_target_kf::KFTracks::ConstPtr
  */
  void kfTracksCallback(const multi_target_kf::KFTracks::ConstPtr& msg);

  /**
  * @brief Target detections callback
  * @param msg geometry_msgs::PoseArray::ConstPtr
  */
  void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  /**
  * @brief Prediction loop callback
  */
  void predloopCallback(const ros::TimerEvent &event);

  /**
  * @brief Trajectory evaluation callback. Executed periodically by a timer
  */
  void trajEvaluationCallback(const ros::TimerEvent &event);

  /**
  * @brief initialize the trajectory preditcor object
  * @return bool. False on failure.
  */
  bool init(void);
  
  /**
  * @brief Constructs the predicted trajectory using _current_state and _model
  */
  bool buildTrajectory();

  bool buildTrajectory(ros::Time ros_time, std::string frame_id);

  /**
  * @brief Publishes ROS msg of predicted positions that are stored in _posehistory_vector
  */
  void pubPredictedPos(void);

  /**
  * @brief Publishes ROS msg of measurement history that i used for prediction.
  */
  void pubPosHistory(void);

  /**
   * @brief Publishes paths of the fitted models
  */
  void pubFittedPaths(void);

  /**
  * @brief Computes RMSE between predicted trajectory and the corresponding observerd trajectory
  * @param predicted_traj std::vector<geometry_msgs::PoseStamped>
  * @param observed_traj boost::circular_buffer<State>
  * @return double RMSE
  */
  double evaluateTrajectory(const std::vector<geometry_msgs::PoseStamped>& predicted_traj, const boost::circular_buffer<State>& observed_traj);

  bool evaluateTrajectory(void);

  /**
   * @brief Resamples incoming measurements at _dt and stores them in _resampeld_measurement_buff
   * @param x of Type State. Input state measurement
  */
  void resampleMeasurement(State x);

public:
  TrajectoryPredictor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~TrajectoryPredictor();
};

#endif // TRAJ_PREDICTOR_H

