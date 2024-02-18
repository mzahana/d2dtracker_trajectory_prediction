/*
MIT License

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

#include "trajectory_prediction/traj_predictor.h"

/* **************************************** */
/*            TrajectoryPredictor           */
/* **************************************** */

TrajectoryPredictor::TrajectoryPredictor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
_nh(nh),
_nh_private(nh_private),
_dt(0.05),
_N(40),
_MIN_NPOS_BUFF(100),
_MAX_NPOS_BUFF(1000),
_calc_speed_form_state(false),
_publish_state_buff(false),
_last_meas_t(ros::Time::now()),
_rmse_bezier(-1.0),
_rmse_const_vel(-1.0),
_resampling_dt(0.05)
{
  _nh_private.param("debug", _debug, true);
  _nh_private.param("dt", _dt, 0.05);
  _nh_private.param("resampling_dt", _resampling_dt, 0.05);
  _nh_private.param<int>("steps", _N, 40);
  _nh_private.param<int>("min_measurement_buffer_size", _MIN_NPOS_BUFF, 100);
  _nh_private.param<int>("max_measurement_buffer_size", _MAX_NPOS_BUFF, 1000);
  _nh_private.param("calc_speed_form_state", _calc_speed_form_state, true);
  _nh_private.param("publish_state_buff", _publish_state_buff, true);
  _nh_private.param<int>("bezier_deg", _bezier_deg, 3);
  _nh_private.param("estimate_velocity", _estimate_velocity_bezier, false);
  _nh_private.param<int>("model_type", _model_type, 0);
  
  

  init();

  // Subscribers
  // _state_sub = _nh.subscribe("target/odom", 1, &TrajectoryPredictor::odomCallback, this);
  _kftrack_sub = _nh.subscribe("kf/tracks", 10, &TrajectoryPredictor::kfTracksCallback, this);
  // _poseArray_sub = _nh.subscribe("target/pose_array", 1, &TrajectoryPredictor::poseArrayCallback, this);

  // Publishers
  _traj_pub = _nh.advertise<custom_trajectory_msgs::StateTrajectory>("traj_predictor/ref_traj", 10);
  _const_vel_traj_pub = _nh.advertise<custom_trajectory_msgs::StateTrajectory>("traj_predictor/const_vel_traj", 10);
  _bezier_traj_pub = _nh.advertise<custom_trajectory_msgs::StateTrajectory>("traj_predictor/bezier_traj", 10);
  _predictedPos_pub =  _nh.advertise<nav_msgs::Path>("traj_predictor/path", 10);
  _cons_vel_predictedPos_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/const_vel_path", 10);
  _bezier_predictedPos_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/bezier_path", 10);
  _sampled_poses_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/sampled_poses", 10);
  _stateBuff_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/state_buff", 10);
  _const_vel_model_rviz_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/const_vel_model", 10);
  _bezier_model_rviz_pub = _nh.advertise<nav_msgs::Path>("traj_predictor/bezier_model", 10);

  _pred_loop_timer = _nh.createTimer(ros::Duration(0.01), &TrajectoryPredictor::predloopCallback, this);

}

// Destructor
TrajectoryPredictor::~TrajectoryPredictor()
{
  delete _const_vel_model;
  delete _dubins_model;
  delete _bezier_model;
}

bool TrajectoryPredictor::init()
{

  if(!_const_vel_model->init(_dt, _N))
  {
    ROS_ERROR("[TrajectoryPredictor::init] Could not initialize  constant velocity predicion model");
    return false;
  }
  else{
    _const_vel_model->setMinPosBuffLength(_MIN_NPOS_BUFF);
    _const_vel_model->setMaxPosBuffLength(_MAX_NPOS_BUFF);
    _const_vel_model->setDebug(_debug);
  }

  if(!_dubins_model->init(_dt, _N))
  {
    ROS_ERROR("[TrajectoryPredictor::init] Could not initialize Dubins predicion model");
    return false;
  }
  else{
    _dubins_model->setMinPosBuffLength(_MIN_NPOS_BUFF);
    _dubins_model->setMaxPosBuffLength(_MAX_NPOS_BUFF);
    _dubins_model->_calc_speed_form_x = _calc_speed_form_state;
    _dubins_model->setDebug(_debug);
  }

  if(!_bezier_model->init(_dt, _N))
  {
    ROS_ERROR("[TrajectoryPredictor::init] Could not initialize Bezier predicion model");
    return false;
  }
  else{
    _bezier_model->setMinPosBuffLength(_MIN_NPOS_BUFF);
    _bezier_model->setMaxPosBuffLength(_MAX_NPOS_BUFF);
    _bezier_model->setDebug(_debug);
    _bezier_model->setDeg((unsigned int)_bezier_deg);
    _bezier_model->estimateVelocity(_estimate_velocity_bezier);
    if(!_bezier_model->initCntPts()) return false;
  }

  _predicted_traj_msg.states.resize(_N+1); // +1 for the initial state, _current_state_msg
  _predicted_poses.resize(_N+1);
  _predicted_poses_const_vel.resize(_N+1);
  _predicted_poses_bezier.resize(_N+1);

  _resampeld_measurement_buff.rset_capacity(2*int(_N*_dt/_resampling_dt));

  return true;
}

bool TrajectoryPredictor::buildTrajectory(ros::Time ros_time, std::string frame_id)
{
  _predicted_traj_msg.header.frame_id = frame_id;
  _predicted_traj_msg.header.stamp = ros_time;
  return buildTrajectory();
}

bool TrajectoryPredictor::buildTrajectory()
{
  // state x=[x, vx, y, vy, z, vz]

  State state_stamped;
  state_stamped.state.resize(NUM_OF_STATES);
  state_stamped.time = _current_state_msg.header.stamp.toSec();
  state_stamped.state(0) = _current_state_msg.position.x; state_stamped.state(1) = _current_state_msg.velocity.x;
  state_stamped.state(2) = _current_state_msg.position.y; state_stamped.state(3) = _current_state_msg.velocity.y;
  state_stamped.state(4) = _current_state_msg.position.z; state_stamped.state(5) = _current_state_msg.velocity.z;

  // Collect measurements, and fit a model (if needed)
  if(!_const_vel_model->insertX(state_stamped)) return false;
  if(!_const_vel_model->gotEnoughMeasurements()) return false;
  // if(! _const_vel_model->fitModel()) return false;

  // ROS_WARN("[TrajectoryPredictor::buildTrajectory] Fitting const vel is done");

  if(!_bezier_model->insertX(state_stamped)) return false;
  if(!_bezier_model->gotEnoughMeasurements()) return false;
  if(!_bezier_model->fitModel()) return false;

  // ROS_WARN("[TrajectoryPredictor::buildTrajectory] Fitting bezier model is done");

  // Initialize the predicted tarjectories with the latest resampled measeurement
  Eigen::VectorXd state; state.resize(NUM_OF_STATES);
  State const_vel_state_stamped, bezier_state_stamped;
  const_vel_state_stamped.state.resize(NUM_OF_STATES);
  bezier_state_stamped.state.resize(NUM_OF_STATES);

  const_vel_state_stamped.time = state_stamped.time; //_latest_resampled_state.time;
  const_vel_state_stamped.state = state_stamped.state; //_latest_resampled_state.state;
  
  bezier_state_stamped.time = state_stamped.time;//_latest_resampled_state.time;
  bezier_state_stamped.state = state_stamped.state; //_latest_resampled_state.state;


  // compute predicted trajectories
  custom_trajectory_msgs::State next_state_msg;
  geometry_msgs::PoseStamped pose_msg;
  auto start_t = _current_state_msg.header.stamp;
  for (int i=0; i < _N+1; ++i)
  {
    // Add state to the trajectory ROS msgs, and RViz msgs

    next_state_msg.time_from_start = i*_dt;
    next_state_msg.header.stamp = _current_state_msg.header.stamp + ros::Duration(i*_dt);
    //RViz msg for visualization
    pose_msg.header.frame_id = _predicted_traj_msg.header.frame_id;
    pose_msg.header.stamp = start_t + ros::Duration(i*_dt);

    //------ next state of const_vel model
    next_state_msg.position.x = const_vel_state_stamped.state(0); next_state_msg.velocity.x = const_vel_state_stamped.state(1);
    next_state_msg.position.y = const_vel_state_stamped.state(2); next_state_msg.velocity.y = const_vel_state_stamped.state(3);
    next_state_msg.position.z = const_vel_state_stamped.state(4); next_state_msg.velocity.z = const_vel_state_stamped.state(5);

    //_predicted_traj_msg.states[i] = next_state_msg;

    // RViz msg
    pose_msg.pose.position = next_state_msg.position;
    pose_msg.pose.orientation.w=1.0; // Keep 0 rotation
    _predicted_poses_const_vel[i] = pose_msg;

    // ------next state of  bezier model
    next_state_msg.position.x = bezier_state_stamped.state(0); next_state_msg.velocity.x = bezier_state_stamped.state(1);
    next_state_msg.position.y = bezier_state_stamped.state(2); next_state_msg.velocity.y = bezier_state_stamped.state(3);
    next_state_msg.position.z = bezier_state_stamped.state(4); next_state_msg.velocity.z = bezier_state_stamped.state(5);

    // for now we chose the best trajectory to be Bezier
    _predicted_traj_msg.states[i] = next_state_msg;

    
    // RViz msg
    pose_msg.pose.position = next_state_msg.position;
    pose_msg.pose.orientation.w=1.0; // Keep 0 rotation
    _predicted_poses_bezier[i] = pose_msg;

    
    // Predict next state, for each model
    const_vel_state_stamped.state = _const_vel_model->forward();
    bezier_state_stamped.state = _bezier_model->forward((1+i*_dt));
  }

  if(evaluateTrajectory())
  {
    if( _rmse_const_vel>=0 and _rmse_bezier>=0)
    {
      if(_debug)
        ROS_INFO(" ---- Const Vel RMSE = %f, Bezier RMSE = %f ---- ", _rmse_const_vel, _rmse_bezier);
      if(_rmse_const_vel <= _rmse_bezier)
      {
        ROS_WARN("Const Vel is better...");
        _predicted_poses = _predicted_poses_const_vel_to_evaluate;
      }
        
      else
      {
        ROS_WARN("Bezier is better...");
        _predicted_poses = _predicted_poses_bezier_to_evaluate;
      }
      _predicted_poses_const_vel_to_evaluate.clear();
      _predicted_poses_bezier_to_evaluate.clear();
    }

  }
  else{
    if(_debug)
      ROS_WARN("[TrajectoryPredictor::buildTrajectory] Could not evaluate trajectories");
  }
  


  return true;
}

void
TrajectoryPredictor::pubFittedPaths(void)
{
  nav_msgs::Path cv_path_msg, bz_path_msg;
  geometry_msgs::PoseStamped pose;

  auto const_vel_fitted_pos = _const_vel_model->getFittedPositions();
  auto bezier_fitted_pos = _bezier_model->getFittedPositions();

  int b_s = (int) const_vel_fitted_pos.size();
  int c_s = (int) bezier_fitted_pos.size();
  if(b_s==0 or c_s==0)
  {
    if(_debug)
      ROS_WARN("[TrajectoryPredictor::pubFittedPaths] FittedPoisitions is empty");

    return;
  }

  auto start_t = ros::Time::now();

  

  cv_path_msg.header.stamp = bz_path_msg.header.stamp = _predicted_poses_bezier[0].header.stamp;
  cv_path_msg.header.frame_id = bz_path_msg.header.frame_id =  _predicted_traj_msg.header.frame_id;
  for(int i=0; i< b_s; i++)
  {
    // constant vel
    pose.header.frame_id =  _predicted_traj_msg.header.frame_id;
    pose.header.stamp = start_t + ros::Duration(i*_dt);
    pose.pose.position.x = const_vel_fitted_pos(i,0);
    pose.pose.position.y = const_vel_fitted_pos(i,1);
    pose.pose.position.z = const_vel_fitted_pos(i,2);
    cv_path_msg.poses.push_back(pose);

    // Bezier
    pose.pose.position.x = bezier_fitted_pos(i,0);
    pose.pose.position.y = bezier_fitted_pos(i,1);
    pose.pose.position.z = bezier_fitted_pos(i,2);
    bz_path_msg.poses.push_back(pose);
  }

  _const_vel_model_rviz_pub.publish(cv_path_msg);
  _bezier_model_rviz_pub.publish(bz_path_msg);

}

void
TrajectoryPredictor::pubPosHistory(void)
{
  // Publish the state history for visualization in Rviz
  if(_publish_state_buff)
  {
    boost::circular_buffer<State> buff;
    buff = _bezier_model->getStateBuffer();
    geometry_msgs::PoseStamped pose,resampled_pose;
    nav_msgs::Path history_path, resampeld_history_path;

    history_path.header.stamp = _predicted_poses_bezier[0].header.stamp;
    history_path.header.frame_id = _predicted_traj_msg.header.frame_id;

    resampeld_history_path.header.stamp = _predicted_poses_bezier[0].header.stamp;
    resampeld_history_path.header.frame_id = _predicted_traj_msg.header.frame_id;

    for (int i=0; i<buff.size() ; i++)
    {
      pose.header.frame_id =  _predicted_traj_msg.header.frame_id;
      pose.header.stamp = ros::Time( buff[i].time);
      pose.pose.position.x = buff[i].state(0);
      pose.pose.position.y = buff[i].state(2);
      pose.pose.position.z = buff[i].state(4);
      history_path.poses.push_back(pose);

      //re-sampled positions
      /** @warning We assume that the resampled buffer legth is less than the measurement size used in the prediciton models */
      if(i < _resampeld_measurement_buff.size())
      {
        resampled_pose.header.frame_id =  _predicted_traj_msg.header.frame_id;
        resampled_pose.header.stamp = ros::Time( _resampeld_measurement_buff[i].time);
        resampled_pose.pose.position.x = _resampeld_measurement_buff[i].state(0);
        resampled_pose.pose.position.y = _resampeld_measurement_buff[i].state(2);
        resampled_pose.pose.position.z = _resampeld_measurement_buff[i].state(4);
        resampeld_history_path.poses.push_back(resampled_pose);
      }
      
    }
    _stateBuff_pub.publish(history_path);
    _sampled_poses_pub.publish(resampeld_history_path);
  }

  
}

void TrajectoryPredictor::pubPredictedPos(void)
{  
  nav_msgs::Path predicted_path;

  //cosnt_vel
  predicted_path.header.stamp = _predicted_poses_const_vel[0].header.stamp;
  predicted_path.header.frame_id = _predicted_traj_msg.header.frame_id;
  predicted_path.poses = _predicted_poses_const_vel;

  _cons_vel_predictedPos_pub.publish(predicted_path);

  //bezier
  predicted_path.header.stamp = _predicted_poses_bezier[0].header.stamp;
  predicted_path.header.frame_id = _predicted_traj_msg.header.frame_id;
  predicted_path.poses = _predicted_poses_bezier;

  _bezier_predictedPos_pub.publish(predicted_path);

  // best one
  predicted_path.header.stamp = _predicted_poses[0].header.stamp;
  predicted_path.header.frame_id = _predicted_traj_msg.header.frame_id;
  predicted_path.poses = _predicted_poses;
  _predictedPos_pub.publish(predicted_path);

  // Publish the predicted trajectories that are used for evaluation
  _bezier_traj_pub.publish(_evaluated_bezier_traj_msg);
  _const_vel_traj_pub.publish(_evaluated_const_vel_traj_msg);
}

void TrajectoryPredictor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Make sure we have new measurements
  if ( !(msg->header.stamp.toSec() > _last_meas_t.toSec() ) )
  {
    if (_debug)
      ROS_WARN("[odomCallback] No new measuremetns");
    return;
  }
  _last_meas_t = msg->header.stamp;

  _current_state_msg.time_from_start = 0.0;
  _current_state_msg.header.stamp = msg->header.stamp;
  _current_state_msg.header.frame_id = msg->header.frame_id;
  _current_state_msg.position = msg->pose.pose.position;
  _current_state_msg.velocity = msg->twist.twist.linear;

  _predicted_traj_msg.header.frame_id = msg->header.frame_id;
  _predicted_traj_msg.header.stamp = ros::Time::now(); // msg->header;

  if (!buildTrajectory()) 
  {
    ROS_WARN_THROTTLE(1,"[TrajectoryPredictor::odomCallback] Could not build trajectory");
    return;
  }

  _traj_pub.publish(_predicted_traj_msg);
  pubPredictedPos();
  
}

void TrajectoryPredictor::kfTracksCallback(const multi_target_kf::KFTracks::ConstPtr& msg)
{
  /** @warning WARNING multi-track prediciton is not supported yet!
  * Currenlty, the first track is only used!!
  */
 
  // Sanity check
  if (msg->tracks.empty())
  {
    ROS_WARN("[TrajectoryPredictor::kfTracksCallback] Tracks are empty! Skipping prediction");
    return;
  }

  // Make sure we have new measurements
  if ( !(msg->tracks[0].header.stamp.toSec() > _last_meas_t.toSec() ) )
  {
    if (_debug)
      ROS_WARN("[TrajectoryPredictor::kfTracksCallback] No new measuremetns");
    return;
  }

  // Calculate data sampling time
  _data_dt = msg->tracks[0].header.stamp.toSec() - _last_meas_t.toSec();
  //DEBUG ROS_INFO("_data_dt = %f", _data_dt);

  // Update _resampeld_measurement_buff at prediction sampling rate _dt
  State x;
  x.state.resize(NUM_OF_STATES);
  x.time = msg->tracks[0].header.stamp.toSec();
  x.state(0) = msg->tracks[0].pose.pose.position.x;
  x.state(1) = msg->tracks[0].twist.twist.linear.x;
  x.state(2) = msg->tracks[0].pose.pose.position.y;
  x.state(3) = msg->tracks[0].twist.twist.linear.y;
  x.state(4) = msg->tracks[0].pose.pose.position.z;
  x.state(5) = msg->tracks[0].twist.twist.linear.z;

  _buff_mutex.lock();
  resampleMeasurement(x);
  _buff_mutex.unlock();

  // Update last time stamp
  _last_meas_t = msg->tracks[0].header.stamp;
  
  _current_state_msg.time_from_start = 0.0;
  _current_state_msg.header.stamp = msg->tracks[0].header.stamp;
  _current_state_msg.header.frame_id=msg->tracks[0].header.frame_id;
  _current_state_msg.position = msg->tracks[0].pose.pose.position;
  _current_state_msg.velocity = msg->tracks[0].twist.twist.linear;
}

void TrajectoryPredictor::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  /* WARNING multi-track prediciton is not supported yet!
  * Currenlty, the first track is only used!!
  */
 
  // Sanity check
  if (msg->poses.empty())
  {
    ROS_WARN("[TrajectoryPredictor::poseArrayCallback] poses are empty! Skipping prediction");
    return;
  }

  // Make sure we have new measurements
  if ( !(msg->header.stamp.toSec() > _last_meas_t.toSec() ) )
  {
    if (_debug)
      ROS_WARN("[TrajectoryPredictor::poseArrayCallback] No new measuremetns");
    return;
  }
  _last_meas_t = msg->header.stamp;
  
  _current_state_msg.time_from_start = 0.0;
  _current_state_msg.header.stamp = msg->header.stamp;
  _current_state_msg.header.frame_id = msg->header.frame_id;
  _current_state_msg.position = msg->poses[0].position;
  // _current_state_msg.velocity = msg->tracks[0].twist.twist.linear;

  _predicted_traj_msg.header.frame_id = msg->header.frame_id;
  _predicted_traj_msg.header.stamp = ros::Time::now(); // msg->header;

  if (!buildTrajectory()) 
  {
    ROS_WARN("[TrajectoryPredictor::kfTracksCallback] Could not build trajectory");
    return;
  }

  _traj_pub.publish(_predicted_traj_msg);
  pubPredictedPos();
}

bool
TrajectoryPredictor::evaluateTrajectory(void)
{
  
  if(_resampling_dt > _dt)
  {
    ROS_ERROR("[evaluateTrajectory] Resampling dt = %f > prediction dt = %f", _resampling_dt, _dt);
    return false;
  }
  
  // Const vel model
  if(_predicted_poses_const_vel.empty())
    return false;

  if(_predicted_poses_bezier.empty())
    return false;

  if(_predicted_poses_bezier.size() != _predicted_poses_const_vel.size())
  {
    ROS_ERROR("[evaluateTrajectory] Predicted trajectories don't have equal sizes");
    return false;
  }

  // Initialize
  if(_predicted_poses_const_vel_to_evaluate.empty())
    _predicted_poses_const_vel_to_evaluate = _predicted_poses_const_vel;

  if(_predicted_poses_bezier_to_evaluate.empty())
    _predicted_poses_bezier_to_evaluate = _predicted_poses_bezier;

  // Check if we have enough measurements
  _buff_mutex.lock();
  if(_predicted_poses_const_vel_to_evaluate.front().header.stamp.toSec() < _resampeld_measurement_buff.front().time or 
      _predicted_poses_const_vel_to_evaluate.back().header.stamp.toSec() > _resampeld_measurement_buff.back().time)
  {
    if(_debug)
      ROS_WARN("[evaluateTrajectory] No enough measurements in the resampled buffer = %d", (int) _resampeld_measurement_buff.size());
    _buff_mutex.unlock();
    return false;
  }

  //fill the predicted traj msgs (this is a custom msg)
  _evaluated_const_vel_traj_msg.header.stamp = _predicted_poses_const_vel_to_evaluate[0].header.stamp;
  _evaluated_const_vel_traj_msg.header.frame_id = _predicted_poses_const_vel_to_evaluate[0].header.frame_id;
  _evaluated_bezier_traj_msg.header.stamp = _predicted_poses_bezier_to_evaluate[0].header.stamp;
  _evaluated_bezier_traj_msg.header.frame_id = _predicted_poses_bezier_to_evaluate[0].header.frame_id;

  for(int i=0; i<_predicted_poses_const_vel_to_evaluate.size(); i++)
  {
    State x; x.state.resize(NUM_OF_STATES);
    custom_trajectory_msgs::State s;
    
    // const vel
    s.header = _predicted_poses_const_vel_to_evaluate[i].header;
    s.position.x = _predicted_poses_const_vel_to_evaluate[i].pose.position.x;
    s.position.y = _predicted_poses_const_vel_to_evaluate[i].pose.position.y;
    s.position.z = _predicted_poses_const_vel_to_evaluate[i].pose.position.z;
    _evaluated_const_vel_traj_msg.states.push_back(s);

    // bezier
    s.header = _predicted_poses_bezier_to_evaluate[i].header;
    s.position.x = _predicted_poses_bezier_to_evaluate[i].pose.position.x;
    s.position.y = _predicted_poses_bezier_to_evaluate[i].pose.position.y;
    s.position.z = _predicted_poses_bezier_to_evaluate[i].pose.position.z;
    _evaluated_bezier_traj_msg.states.push_back(s);
  }

  // ROS_INFO("[evaluateTrajectory] Preparing interpolated measurements");

  // Do linear interpolation to match the time stamps between the predicted and actual measurements
  std::vector<State> intrp_measurements;
  for(auto it=_predicted_poses_const_vel_to_evaluate.begin(); it != _predicted_poses_const_vel_to_evaluate.end(); it++)
  {
    double pred_t = (*it).header.stamp.toSec();

    // find a previous and later samples for interpolation
    State prev_x , next_x;
    bool found_prev_x = false; bool found_next_x = false;

    // ROS_INFO("[evaluateTrajectory] Finding a previous and later measurements for interpolation...");

    for(auto it2=_resampeld_measurement_buff.begin(); it2 != _resampeld_measurement_buff.end(); it2++)
    {
      // previous sample
      if( (pred_t - (*it2).time) > 0 and  (pred_t - (*it2).time) < 0.8*_dt)
      {
        // ROS_INFO("[evaluateTrajectory] Found previous measurement");
        prev_x = *it2;
        found_prev_x = true;
      }

      if( ((*it2).time - pred_t) > 0 and  ((*it2).time - pred_t) < 0.8*_dt)
      {
        // ROS_INFO("[evaluateTrajectory] Found next measurement");
        next_x = *it2;
        found_next_x = true;
      }

      if(found_prev_x and found_next_x)
      {
        State intrp_x; intrp_x.time = pred_t; intrp_x.state.resize(NUM_OF_STATES);
          for (int i=0; i<NUM_OF_STATES; i++)
            intrp_x.state(i) =  interpolate(prev_x.time, prev_x.state(i), next_x.time, next_x.state(i), pred_t);
        intrp_measurements.push_back(intrp_x);
        break;
      }
    }

  }// done with all prediciton samples
  _buff_mutex.unlock();

  // check if we got all the needed interpolated measuremetns
  if(intrp_measurements.size() != _predicted_poses_const_vel.size())
  {
    if(_debug)
      ROS_WARN("Could not construct interpolated measurments that is of the same length of the predicted sampels");
    return false;
  }
  // ROS_INFO("[evaluateTrajectory] Interpolated measurements are prepared");

  // Compute RMSE
  _rmse_bezier = 0; _rmse_const_vel = 0;

  for(int i = 0; i< intrp_measurements.size(); i++)
  {
    // bezier
    double dx = _predicted_poses_bezier_to_evaluate[i].pose.position.x - intrp_measurements[i].state(0);
    double dy = _predicted_poses_bezier_to_evaluate[i].pose.position.y - intrp_measurements[i].state(2);
    double dz = _predicted_poses_bezier_to_evaluate[i].pose.position.z - intrp_measurements[i].state(4);

    double d = dx*dx + dy*dy + dz*dz;
    _rmse_bezier += d;

     // const vel
     dx = _predicted_poses_const_vel_to_evaluate[i].pose.position.x - intrp_measurements[i].state(0);
     dy = _predicted_poses_const_vel_to_evaluate[i].pose.position.y - intrp_measurements[i].state(2);
     dz = _predicted_poses_const_vel_to_evaluate[i].pose.position.z - intrp_measurements[i].state(4);

    d = dx*dx + dy*dy + dz*dz;
    _rmse_const_vel += d;
  }

  _rmse_const_vel = sqrt(_rmse_const_vel/intrp_measurements.size());
  _rmse_bezier = sqrt(_rmse_bezier/intrp_measurements.size());

  _evaluated_bezier_traj_msg.quality = _rmse_bezier;
  _evaluated_const_vel_traj_msg.quality = _rmse_const_vel;

  return true;

}

double
TrajectoryPredictor::evaluateTrajectory(const std::vector<geometry_msgs::PoseStamped>& predicted_traj, const boost::circular_buffer<State>& observed_traj)
{
  /** @todo
   * Implement
  */
  
  // Root Mean Squared Error
  double rmse=-1.0; // negative value => comparison is not valid

  if(predicted_traj.empty() || observed_traj.empty())
    return rmse;

  if(observed_traj.size() < predicted_traj.size())
    return rmse;

  // In case the last measurement is old, return
  if(predicted_traj.back().header.stamp.toSec() > observed_traj.back().time)
    return rmse;

  ROS_INFO("[evaluateTrajectory] Finding a matching sample...");
  // find a match between the last predicted state and an observed one
  for(auto it=observed_traj.end(); it != observed_traj.begin(); it--)
  {
    int idx = it - observed_traj.begin();
    ROS_INFO("Index of observed measurement = %d", idx);
    // Check if there is close enough measurement 
    double last_pred_dt = predicted_traj.back().header.stamp.toSec();
    ROS_INFO("Time stamps last_pred_dt=%f", last_pred_dt);
    double obs_dt = observed_traj[idx].time;
    ROS_INFO("Time stamps obs_dt=%f",obs_dt);

    double dt = abs(last_pred_dt - obs_dt);
    ROS_INFO("Calculated dt %f", dt);
    if(dt <= 0.5*_dt)
    {
      ROS_INFO("Found a matching sample");
      // we found a matching state in the observed_traj,
      // but we need to check if we have enough samples before this matching sample
      int last_observation_idx = (it - observed_traj.begin());
      ROS_INFO("Calculated the index of the matching sample %d", last_observation_idx);

      if ((last_observation_idx+1) < (int)predicted_traj.size())// no enough observations
        return rmse;

      ROS_INFO("Found enough observations %d", last_observation_idx+1);

      /** @todo compute RMSE */
      ROS_WARN("Computing RMSE");
      rmse=0;
      for(int i=last_observation_idx; i>=0; i--)
      {
        double dx = predicted_traj[i].pose.position.x - observed_traj[i].state(0);
        double dy = predicted_traj[i].pose.position.y - observed_traj[i].state(2);
        double dz = predicted_traj[i].pose.position.z - observed_traj[i].state(4);

        double d = sqrt(dx*dx + dy*dy + dz*dz);
        rmse += d*d;
      }
      rmse = sqrt(rmse/(double)predicted_traj.size());
      return rmse;
    }

    
  }


 // A negative value means could not match the time stamps between trajectories
 return rmse;
}

void
TrajectoryPredictor::predloopCallback(const ros::TimerEvent &event)
{

  if (!buildTrajectory(_current_state_msg.header.stamp, _current_state_msg.header.frame_id)) // this is going to fill prediction in _predicted_traj_msg
  {
    ROS_WARN_THROTTLE(1,"[TrajectoryPredictor::predloopCallback] Could not build trajectory");
    return;
  }

  _traj_pub.publish(_predicted_traj_msg); // can be used in another node for tracking
  
  pubPosHistory(); // RViz path
  pubPredictedPos(); // RViz path
  // pubFittedPaths(); // RViz

  return;
}

void
TrajectoryPredictor::trajEvaluationCallback(const ros::TimerEvent &event)
{
  /** @todo IMPLEMENT 
   * Use latest measurements to find the best model that fits them
  */

  return;
}

void
TrajectoryPredictor::resampleMeasurement(State x)
{
  if(_resampeld_measurement_buff.empty())
  {
    _resampeld_measurement_buff.push_back(x);
    _latest_resampled_state = x;
    _last_sampled_sate_time = x.time;
  }
    
  else{
    double dt = x.time - _resampeld_measurement_buff.back().time;
    // DEBUG ROS_INFO("_resampeld_measurement_buff data sampling rate dt = %f", dt);
    if(dt>_resampling_dt){
    
      // Do linear interploation
      State x_intrp;
      x_intrp.state.resize(NUM_OF_STATES);
      x_intrp.time = _resampeld_measurement_buff.back().time + _resampling_dt;
      unsigned int idx;
      for (unsigned int i=0; i<NUM_OF_STATES; i++)
        x_intrp.state(i) = interpolate(_resampeld_measurement_buff.back().time, _resampeld_measurement_buff.back().state(i), x.time, x.state(i), _resampeld_measurement_buff.back().time+_resampling_dt);
      
      _resampeld_measurement_buff.push_back(x_intrp);
      _latest_resampled_state = x_intrp;
    }
  }
  return;
}