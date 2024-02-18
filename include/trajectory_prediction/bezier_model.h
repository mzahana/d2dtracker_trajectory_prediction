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


#ifndef BEZIER_MODEL_H
#define BEZIER_MODEL_H

#include "trajectory_prediction/base_model.h"

/**
 * Bezier curve-based model
 */
class BezierModel: public PredictionModel
{
private:
  bool _estimate_velocity; /** fit velocity curve and predict future ones */
  Eigen::VectorXd _pos_control_pts; /** Bezier curve control points for position*/
  Eigen::VectorXd _vel_control_pts; /** Bezier curve control points velocity */
  unsigned int _deg; /** Curve degree */
  unsigned int _M; /** Number of measurements */
  LeastSquare           _ls_object;                         /** Object of the least square solver. */
  const int            _nx=3;                            /** Dimension of position points. 2: 2D , 3: 3D*/

public:
  BezierModel()
  {
    _deg=3; /** default degree */
    _M = (_deg+1)*_nx; /** default minimum number of measurements */
    setMinPosBuffLength(_M);
    _estimate_velocity = false;
  }

  bool setDeg(unsigned int d)
  {
    if (d<0){
      return false;
    } 
    _deg = d; return true;
  }
  bool estimateVelocity(bool f)
  {
    _estimate_velocity = f;
    return true;
  }

  bool initCntPts(void)
  {
    try{
      _pos_control_pts = Eigen::VectorXd::Zero((_deg+1)*_nx);
      _vel_control_pts = Eigen::VectorXd::Zero((_deg+1)*_nx);
      if(_debug)
      {
        printf("[BezierModel::initCntPts] Done initializing control points");
        std::cout << "_pos_control_pts: \n" << _pos_control_pts << "\n";
        std::cout << "_vel_control_pts: \n" << _vel_control_pts << "\n";
      }
      return true;
    }
    catch(...){
      printf("[BezierModel::initCntPts] Could not intialize control points ");
      return false;
    }
  }
  Eigen::MatrixXd A(double t)
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nx, _nx*(_deg+1));
    if (t<0)
    {
      printf("[BezierModel::A] t <0 ");
      return A;
    }
    for (int i=0; i<(_deg+1); i++)
    {
      A.block(0,_nx*i,_nx,_nx) = binomialCoeff(_deg,i) *pow(1-t, _deg-i) * pow(t,i) * Eigen::MatrixXd::Identity(_nx,_nx);
      // A.block(0,_nx*i,_nx,_nx) = pow(t,i) * Eigen::MatrixXd::Identity(_nx,_nx);
    }

    if(_debug)
      std::cout << "A(t): \n" << A << "\n";
    return A;
  }

  
  Eigen::VectorXd predictPos(double t)
  {
    if(_debug)
      printf("[BezierModel::predictPos] executing predictPos ...");

    return (A(t)*_pos_control_pts);
  }

  Eigen::VectorXd predictVel(double t)
  {
    if(_debug)
      printf("[BezierModel::predictVel] executing predictVel ...");
    return (A(t)*_vel_control_pts);
  }

  Eigen::VectorXd forward(double t)
  {
    if(_debug)
      printf("[BezierModel::forward] executing forward ...");

    Eigen::VectorXd pos =  predictPos(t);
    Eigen::VectorXd vel =  predictVel(t);

    if(_estimate_velocity){
      _x(0) = _x(0) + vel(0)*_dt; //x
      _x(1)=vel(0);//vx
      _x(2) = _x(2) + vel(1)*_dt;// y
      _x(3) = vel(1); // vy
    }
    else{
      _x(0) = pos(0); _x(1)=vel(0);
      _x(2) = pos(1); _x(3) = vel(1);  
    }
    
    if(_nx==2)
    {
      _x(4) = _state_buff[_state_buff.size()-1].state(4); _x(5) = _state_buff[_state_buff.size()-1].state(5);
    }
    else{
      if(_estimate_velocity)
      {
        _x(4) = _x(4) + _dt*vel(2); // z
        _x(5) = vel(2); //vz
      }
      else{
        _x(4) = pos(2); _x(5) = vel(2);
      }
    }

    if(_debug)
      printf("[BezierModel::forward] Done executing forward .");

    return _x;
  }

  bool fitModel(void)
  {
    // This is to compute execution time of this function
    auto t0 = std::chrono::high_resolution_clock::now();

    if(_debug)
      printf("[BezierModel::fitCurve] executing fitCurve ...");

    _M = (unsigned int) _state_buff.size();

    // Sanity check
    if (_M < ((_deg+1)*_nx))
    {
      printf("[BezierModel::fitCurve] Number of measurements _M %u is < (_deg+1) * %d = %u", _M, _nx, (_deg+1)*_nx);
      return false;
    }

    // get minimum time
    double t_min = _state_buff[0].time;
    // max time
    double t_max = _state_buff[_M-1].time;

    if (t_max <= t_min)
    {
      printf("[BezierModel::fitCurve] t_max is not > t_min. Check measurements times!");
      return false;
    }

    // Least square (lq) A matrix
    Eigen::MatrixXd Alq = Eigen::MatrixXd::Zero(_nx*_M, _nx*(_deg+1));
    // least square b matrix
    Eigen::VectorXd b_pos = Eigen::VectorXd::Zero(_nx*_M);
    Eigen::VectorXd b_vel = Eigen::VectorXd::Zero(_nx*_M);

    for (int i=0; i< _M; i++)
    {
      double t = (_state_buff[i].time - t_min)/(t_max-t_min);
      Alq.block(_nx*i,0,_nx,_nx*(_deg+1)) = A(t);

      b_pos(_nx*i+0) = _state_buff[i].state(0); //x
      b_pos(_nx*i+1) = _state_buff[i].state(2); //y
      if (_nx==3)
        b_pos(_nx*i+2) = _state_buff[i].state(4); //z

      if(_estimate_velocity)
      {
        b_vel(_nx*i+0) = _state_buff[i].state(1); //x
        b_vel(_nx*i+1) = _state_buff[i].state(3); //y
        if (_nx==3)
          b_vel(_nx*i+2) = _state_buff[i].state(5); //z
      }
    }

    // Solve Least Square problem
    // result = Alq.colPivHouseholderQr().solve(blq); // slower, more accurate
    // _pos_control_pts = Alq.colPivHouseholderQr().solve(b_pos); // Faster, less accurate
    // if(_estimate_velocity)
    //   _vel_control_pts = Alq.householderQr().solve(b_vel); // Faster, less accurate

    
    //**************** Solve constrained least square problem
    // Number of extra constraints
    unsigned int nc = _nx+_nx*(_deg+1);
    Eigen::VectorXd x_l= Eigen::VectorXd::Ones(nc);
    Eigen::VectorXd x_u = Eigen::VectorXd::Ones(nc);

    // last received position estimate
    // Used to estimate a bound on the predicted positions
    Eigen::Vector3d pos(_state_buff[_state_buff.size()-1].state(0),_state_buff[_state_buff.size()-1].state(2),_state_buff[_state_buff.size()-1].state(4));

    Eigen::Vector3d vel(_state_buff[_state_buff.size()-1].state(1),_state_buff[_state_buff.size()-1].state(3),_state_buff[_state_buff.size()-1].state(5));

    // Linear constraints matrix W
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(nc,_nx*(_deg+1));
    
    if(!_ls_object.isLSInitialized())
    {
      _ls_object.setDebug(_debug);
      // Constraints on bounds of the control points
      W.block(0,0,_nx*(_deg+1),_nx*(_deg+1)) = Eigen::MatrixXd::Identity(_nx*(_deg+1),_nx*(_deg+1));

      // Constraints on the curve value at future time, outside the interval [0,1], (e.g. 2)
      W.block(_nx*(_deg+1),0, _nx, _nx*(_deg+1)) = A(2); // 2 means at future time = 2

      if(!_ls_object.initLSProblem(_nx*(_deg+1),(unsigned int)b_pos.size(), nc, W, x_l, x_u)) return false;
    }

    double bound;

    if (_estimate_velocity){
      bound = 2; // added bound on velocity m/s
      x_l.block(0,0,_nx*(_deg+1),1) = -(vel.norm()+bound)*Eigen::VectorXd::Ones(_nx*(_deg+1));
      x_u.block(0,0,_nx*(_deg+1),1) = (vel.norm()+bound)*Eigen::VectorXd::Ones(_nx*(_deg+1));

      // Update bounds on curve value at future time
      bound = 2;// added bound in m/s
      x_l.block(_nx*(_deg+1),0,_nx,1) = -(vel.norm()+bound)*Eigen::VectorXd::Ones(_nx);
      x_u.block(_nx*(_deg+1),0, _nx,1) = (vel.norm()+bound)*Eigen::VectorXd::Ones(_nx);

      if (!_ls_object.updateLSData(Alq, b_vel, x_l, x_u)) return false;
      if(!_ls_object.solve()) return false;
      _vel_control_pts = _ls_object.getSolution();
    }
    else
    {
      // Update bounds on control points
      bound = 100;// in meters
      x_l.block(0,0,_nx*(_deg+1),1) = -(pos.norm()+bound)*Eigen::VectorXd::Ones(_nx*(_deg+1));
      x_u.block(0,0,_nx*(_deg+1),1) = (pos.norm()+bound)*Eigen::VectorXd::Ones(_nx*(_deg+1));

      // Update bounds on curve value at future time
      bound = 2;// in meters
      x_l.block(_nx*(_deg+1),0,_nx,1) = -(pos.norm()+bound)*Eigen::VectorXd::Ones(_nx);
      x_u.block(_nx*(_deg+1),0, _nx,1) = (pos.norm()+bound)*Eigen::VectorXd::Ones(_nx);

      if (!_ls_object.updateLSData(Alq, b_pos, x_l, x_u)) return false;
      if(!_ls_object.solve()) return false;
      _pos_control_pts = _ls_object.getSolution();
      _fitting_accuracy = (Alq*_pos_control_pts-b_pos).norm();
      _fitting_accuracy *= _fitting_accuracy;
    }

    Eigen::MatrixXd fitted_pos = Alq*_pos_control_pts;
    // _fitted_positions = fitted_pos.
    _fitted_positions.resize(_M,3);
    for(int i=0; i< (int) (fitted_pos.size()/_nx); i++)
    {
      _fitted_positions(i, 0) = fitted_pos(3*i+0);
      _fitted_positions(i, 1) = fitted_pos(3*i+1);
      _fitted_positions(i, 2) = fitted_pos(3*i+2);
    }

    if(_debug){
      printf("[BezierModel::fitCurve] Done executing fitPosCurve .");
      std::cout << "Alq: \n" << Alq << "\n";
      std::cout << "b_pos: \n" << b_pos << "\n";
      std::cout << "_pos_control_pts: \n" << _pos_control_pts << "\n";
      std::cout << "_vel_control_pts: \n" << _vel_control_pts << "\n";
    }

    // std::cout << "Position Control points... \n";
    // std::cout << _pos_control_pts << "\n";
    // std::cout << "b_pos: \n" << b_pos << "\n";
    // std::cout << "b_pos: \n" << x_l << "\n";
    // std::cout << "Velocity Control points... \n";
    // std::cout << _vel_control_pts << "\n";
    // Compute total error
    // double err = (Alq*_pos_control_pts-b_pos).norm(); err *= err;
    // std::cout << "Least square total error : " << err << "\n";
    
    if(_debug)
      printf("[BezierModel::fitCurve] Done executing fitCurve .");

    if(_debug)
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
      printf("[BezierModel::fitCurve] FitCurve took %ld milli econd(s)", duration.count());
    }

    return true;
  }
};

#endif // BEZIER_MODEL_H