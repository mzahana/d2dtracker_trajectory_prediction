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


#ifndef DUBIN3D_MODEL_H
#define DUBIN3D_MODEL_H

#include "trajectory_prediction/base_model.h"


/*********************************************************************
 * Dubin's path model
*********************************************************************/

/**
 * Dubin's path model with constant speed
 * Inputs: 
 *  Current position in 3D
 * This model requires a sequence of past position measurements in order to estimate
 *  a constant speed that is needed by the prediciton model,
 *  initial heading and pitch angles, theta and gamma, respectively. Needed by the prediciton model
 *  constant heading and pitch rates, needed by the prediciton model
 */
class DubinsPathModel: public PredictionModel
{
private:
  LeastSquare           _ls_object;                         /** Object of the least square solver. */

protected:
  double                _current_speed;                     /** Estimated constant speed used by the Dubin's path model, m/s */
  double                _heading_rate;                      /** Estimated heading rate in rad/sec */
  double                _pitch_rate;                        /** Estimated pitch rate in rad/sec */
  double                _current_heading;                   /** Current estimate of heading angle, rad */
  double                _current_pitch;                     /** Current estimate of pitch angle, rad */

public:
  bool _calc_speed_form_x = false;

  DubinsPathModel()
  {
    setMinPosBuffLength(MAX_STATE_BUFF_LENGTH);
    setDebug(false);
  }

  // Overriden
  Eigen::VectorXd model(Eigen::VectorXd x)
  {
    // states: [px, vx, py, vy, pz, vz]
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(NUM_OF_STATES);

    // Estimate current_speed from the current state
    if (_calc_speed_form_x)
    {
      Eigen::Vector3d v;
      v(0) = x(1); v(1)=x(3); v(2)=x(5);
      _current_speed = v.norm();
    }

    x_new(0) = x(0) + _dt * _current_speed * cosf64(_current_heading) * cosf64(_current_pitch);// x
    x_new(2) = x(2) + _dt * _current_speed * sinf64(_current_heading) * cosf64(_current_pitch);// y
    x_new(4) = x(4) + _dt * _current_speed * sinf64(_current_pitch); // z
    x_new(1) = x(1); // vx
    x_new(3) = x(3); // vy
    x_new(5) = x(5); // vz

    _current_heading = _current_heading + _dt*_heading_rate;
    _current_pitch = _current_pitch + _dt*_pitch_rate;


    if (_debug)
      printf("Calling Dubin's path model");

    return x_new;
  }


  /**
   * @brief Least squares fitting to estimate _current_speed, _pitch_rate, _heading_rate, _current_heading, _current_pitch,
   * using history of  measurements _state_buff.
   */
  bool fitModel(void)
  {
    if (_debug)
    {
      printf("[DubinsPathModel::fitModel] Calling fitModel()");
    }
    // Sanity check
    if (_state_buff.size() < _max_pos_buff)
    {
      printf("[DubinsPathModel::fitModel] Not enough measurements (%d)for the estimation", (int) _state_buff.size());
      return false;
    }

    /** @brief construct least square matrices */
    int Nm = (int) _state_buff.size()-1; // Number of measurements to consider
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(5*Nm,3);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(5*Nm);
    Eigen::VectorXd result;
    double x1,x2,y1,y2,z1,z2;
    double dt, theta_k1, theta_k2, gamma_k1, gamma_k2, x, y, z, xy1_norm, xy2_norm;

    for(int i=1; i<= Nm; i++)
    {
      // time stamps
      dt = _state_buff[i].time -  _state_buff[i-1].time;
      if (dt<0.001) dt=0.001; // set to default _dt if actual dt is too small

      // estimate angles
      x1 = _state_buff[i-1].state(0); y1 = _state_buff[i-1].state(2); z1 = _state_buff[i-1].state(4);
      theta_k1 = atan2f64(y1,x1); 
      if (theta_k1 <0) theta_k1 = 2*M_PI+theta_k1;
      xy1_norm = sqrtf64(x1*x1+y1*y1);
      gamma_k1 = atan2f64(z1,xy1_norm);
      if (gamma_k1<0) gamma_k1 = 2*M_PI+gamma_k1;

      x2 = _state_buff[i].state(0); y2 = _state_buff[i].state(2); z2 = _state_buff[i].state(4);
      theta_k2 = atan2f64(y2,x2);
      if(theta_k2<0) theta_k2 = 2*M_PI+theta_k2;
      xy2_norm = sqrtf64(x2*x2+y2*y2);
      gamma_k2 = atan2f64(z2,xy2_norm);
      if(gamma_k2<0) gamma_k2 = gamma_k2 + 2*M_PI+gamma_k2;

      // compute delta_theta and delta_gamma
      Eigen::Vector3d v1(x1,y1,0);
      Eigen::Vector3d v2(x2,y2,0);
      double nrm=v1.norm()*v2.norm();
      double delta_theta =0;
      if (nrm>0)
        delta_theta = acos(v1.dot(v2)/nrm);

      Eigen::Vector3d v_cross = v1.cross(v2);
      delta_theta = v_cross(2)/abs(v_cross(2))*delta_theta;
      

      // delta gamma
      v1(0)=xy1_norm; v1(1)=z1; v1(2)=0;
      v2(0)=xy2_norm; v2(1)=z2; v2(2)=0;
      nrm=v1.norm()*v2.norm();
      double delta_gamma =0;
      if (nrm>0)
        delta_gamma = acos(v1.dot(v2)/nrm);

      v_cross = v1.cross(v2);
      delta_gamma = v_cross(2)/abs(v_cross(2))*delta_gamma;

      if(_debug)
      {
        std::cout << "------------------\n";
        std::cout << "[DubinsModel::fitModel] theta_1: " << theta_k1 << "theta_k2: "<<  theta_k2 << "\nDelta theta: " << theta_k2-theta_k1 << "\n";
        std::cout << "[DubinsModel::fitModel] gamma_1: " << gamma_k1 << "gamma_k2: "<<  gamma_k2 << "\nDelta gamma: " << gamma_k2-gamma_k1 << "\n";
        std::cout << "------------------\n";
      }

      // compute current measurement vector
      b((i-1)*5+0) = _state_buff[i].state(0) - _state_buff[i-1].state(0); // $$ \Delta x $$
      b((i-1)*5+1) = _state_buff[i].state(2) - _state_buff[i-1].state(2); // $$ \Delta y $$
      b((i-1)*5+2) = _state_buff[i].state(4) - _state_buff[i-1].state(4); // $$ \Delta z $$
      b((i-1)*5+3) = delta_theta; // $$ \Delta \theta $$
      b((i-1)*5+4) = delta_gamma; // $$ \Delta \gamma $$

      // compute A_k
      A((i-1)*5+0,0) = dt*cosf64(theta_k1)*cosf64(gamma_k1);
      A((i-1)*5+1,0) = dt*sinf64(theta_k1)*cosf64(gamma_k1);
      A((i-1)*5+2,0) = dt*sinf64(gamma_k1);
      A((i-1)*5+3,1) = dt;
      A((i-1)*5+4,2) = dt;
    }

    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(3,3);

    if(!_ls_object.isLSInitialized())
    {
      _ls_object.setDebug(_debug);
      Eigen::VectorXd x_l; x_l = Eigen::VectorXd::Zero(3); x_l(0)=-12; x_l(1)=-100; x_l(2)=-100;
      Eigen::VectorXd x_u; x_u = Eigen::VectorXd::Zero(3); x_u(0)=12; x_u(1)=100; x_u(2)=100;
      if(!_ls_object.initLSProblem(3,(unsigned int)b.size(), 3, W, x_l, x_u)) return false;
    }
    if (!_ls_object.updateLSData(A, b)) return false;
    if(!_ls_object.solve()) return false;
    result = _ls_object.getSolution();

    // Solve Least Square problem
    // result = A.householderQr().solve(b); // Faster, less accurate
    // result = A.colPivHouseholderQr().solve(b); // slower, more accurate

    _current_speed = result(0);
    _heading_rate = result(1);
    _pitch_rate = result(2);
    _current_heading = theta_k2;
    _current_pitch = gamma_k2;

    
    if (_debug)
    {
      std::cout << "------------------------------- \n";
      std::cout << "current_speed: " << _current_speed << "\n";
      std::cout << "_current_heading: " << _current_heading << "\n";
      std::cout << "_current_pitch: " << _current_pitch << "\n";
      std::cout << "_heading_rate: " << _heading_rate << "\n";
      std::cout << "_pitch_rate: " << _pitch_rate << "\n";
      std::cout << "------------------------------- \n";
    }

    return true;
  }

};


#endif // DUBIN3D_MODEL_H