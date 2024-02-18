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


#ifndef CONSTANT_VELOCITY_H
#define CONSTANT_VELOCITY_H

#include "trajectory_prediction/base_model.h"

/*********************************************************************
 * Constant Velocity Model
*********************************************************************/
class ConstantVelocityModel: public PredictionModel
{
private:
  unsigned int _M; /** Number of measurements */
  Eigen::Vector3d _estimated_velocity;      /** Estimated velocity using least square. See fitModel() */
  // LeastSquare           _ls_object;                         /** Object of the least square solver. */

public:
  ConstantVelocityModel()
  {
    setMinPosBuffLength(1); // 1 measurement is enough for this prediction model
  }
  
  // Overriden
  Eigen::VectorXd model(Eigen::VectorXd x)
  {
    // states: [px, vx, py, vy, pz, vz]

    Eigen::MatrixXd A_dt, A;
    A_dt = Eigen::MatrixXd::Identity(2,2); // Construct the smaller A matrix
    A = Eigen::MatrixXd::Identity(NUM_OF_STATES, NUM_OF_STATES);

    A_dt(0,1) = _dt;
    
    A.block(0,0, 2, 2) = A_dt;
    A.block(2,2, 2, 2) = A_dt;
    A.block(4,4, 2, 2) = A_dt;

    if (_debug)
      printf("Calling constant velocity model");

    return A*x;
  }

// Overriden
  // Eigen::VectorXd model(Eigen::VectorXd x)
  // {
  //   // states x= [px, vx, py, vy, pz, vz]

  //   Eigen::Vector3d last_pos(x(0), x(2), x(4));

  //   Eigen::Vector3d pos = last_pos + _dt*Eigen::Matrix3d::Identity(3,3) * _estimated_velocity ;

  //   Eigen::VectorXd new_x; new_x.resize(NUM_OF_STATES);
  //   new_x(0) = pos(0); //x
  //   new_x(1) = _estimated_velocity(0); //vx
  //   new_x(2) = pos(1); //y
  //   new_x(3) = _estimated_velocity(1); //vy
  //   new_x(4) = pos(2); //z
  //   new_x(5) = _estimated_velocity(2); //vz

  //   if (_debug)
  //     printf("Calling constant velocity model");

  //   return new_x;
  // }

// THIS IS NON-SENSE!
bool fitModel(void)
{
  /** @todo Needs implementation */
  // This is to compute execution time of this function
    auto t0 = std::chrono::high_resolution_clock::now();

    if(_debug)
      printf("[ConstantVelocity::fitModel] executing fitModel ...");

    _M = (unsigned int) _state_buff.size();

    // Sanity check
    if (_M < 3)
    {
      printf("[ConstantVelocity::fitModel] Number of measurements _M %u is < 3", _M);
      return false;
    }


  Eigen::MatrixXd A; A.resize(3*(_M-1), 3);
  Eigen::VectorXd b; b.resize(3*(_M-1),1);

  for (int i=1; i< _state_buff.size();i++)
  {
    double dt = _state_buff[i].time - _state_buff[i-1].time;
    A.block(3*(i-1),0, 3,3) = dt*Eigen::MatrixXd::Identity(3,3);

    b(3*(i-1),0) = _state_buff[i].state(0) - _state_buff[i-1].state(0); // delta x
    b(3*(i-1)+1,0) = _state_buff[i].state(2) - _state_buff[i-1].state(2); // delta y
    b(3*(i-1)+2,0) = _state_buff[i].state(4) - _state_buff[i-1].state(4); // delta z
  }

  // Solve Least Square problem
  // result = Alq.colPivHouseholderQr().solve(blq); // slower, more accurate
  _estimated_velocity= A.colPivHouseholderQr().solve(b); // Faster, less accurate

  // std::cout << " Estimated velocity: \n" << _estimated_velocity << "\n";
  // ROS_INFO("Estimated velocity magnitude = %f", _estimated_velocity.norm());

  _fitting_accuracy = (A*_estimated_velocity-b).norm();
  _fitting_accuracy *= _fitting_accuracy;

  // Construct the fitted data
  _fitted_positions.resize(_M,3);

  for (int i=0; i < _M; ++i)
  {
    if(i==0)
    {
      _fitted_positions(0,0) = _state_buff[0].state(0); //x0
      _fitted_positions(0,1) = _state_buff[0].state(2); //y0
      _fitted_positions(0,2) = _state_buff[0].state(4); //z0
    }
    else{
      _fitted_positions(i,0) = _dt*_estimated_velocity(0) + _fitted_positions(i-1,0); //x_i
      _fitted_positions(i,1) = _dt*_estimated_velocity(1) + _fitted_positions(i-1,1); //y_i
      _fitted_positions(i,2) = _dt*_estimated_velocity(2) + _fitted_positions(i-1,2); //z_i
    }
  }

  
  if(_debug)
  {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    printf("[ConstantVelocity::fitModel] FitCurve took %ld milli econd(s)", duration.count());
  }

  return true;
}

};


#endif // CONSTANT_VELOCITY_H