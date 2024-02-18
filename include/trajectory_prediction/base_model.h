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


#ifndef BASE_MODEL_H
#define BASE_MODEL_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <math.h>
#include <boost/circular_buffer.hpp>
#include "least_square.h"
#include "utils.h"
#include <chrono>


/**
 * State
 */ 
struct State
{
  double time; // time in seconds
  Eigen::VectorXd state;
};

/*********************************************************************
 * Base class
*********************************************************************/
class PredictionModel
{

/**
* @brief Implements a base class for prediction models. This class should be inherited by specific model classes.
*/

protected:
  bool                  _debug;                 /** Print debug messages */
  Eigen::VectorXd       _x;                     /** state =[px, vx, py, vy, pz, vz] */
  double                _dt;                    /** Sampling time in seconds */
  int                   _N;                     /** Number of prediction steps */

  const int             NUM_OF_STATES=6;        /** Number of states. Size of _x */
  const int             MAX_STATE_BUFF_LENGTH=1000; /** size/length of history */

  boost::circular_buffer<State> _state_buff;

  int                   _min_pos_buff;          /** Minimum number of position measurements */
  int                   _max_pos_buff;          /** Maximum number of position measurements */

  double _fitting_accuracy;                   /** The value of the least square objective function */

  Eigen::MatrixXd _fitted_positions;          /** 3D postitions of the fitted model */

public:
  PredictionModel() : _state_buff(MAX_STATE_BUFF_LENGTH), _fitting_accuracy(-1.0) {_debug=false;}
  ~PredictionModel(){}

  /**
  * @brief Set _debug 
  */
  void setDebug(bool d)
  {
    _debug = d;
  }


  /**
  * @brief Initialize prediction model by setting _dt, _N, _A
  */
  bool init(double dt, int N)
  {
    if(! setDt(dt))
     return false;

    if(!setN(N))
      return false;

    _x.resize(NUM_OF_STATES); _x.setZero();

    return true;
  }

  /**
  * @brief Sets sampling time _dt
  * @param dt type double. Sampling time in seconds
  * @return bool. True if successful
  */
  bool setDt(double dt)
  {
    if(dt <=0 )
    {
      printf("[PredictionModel::setDt] dt should be > 0");
      return false;
    }
    _dt = dt;

    return true;
  }

  /**
  * @brief Sets number of time steps _N
  * @param N type int. Number of steps
  * @return bool. False if N <=0
  */
  bool setN(int N)
  {
    if (N <=0 )
    {
      printf("[PredictionModel::setN] N %d should be > 0", N);
      return false;
    }
    _N = N;

    return true;
  }

  /**
  * @brief Sets current/initial state _x
  * @param x type VectorXd.
  * @return bool. True if successful
  */
  bool setX(Eigen::VectorXd x)
  {
    if (x.size() != NUM_OF_STATES)
    {
      printf("[PredictionModel::setX] Size of x = %d != Number of states=%d", (int)x.size(), NUM_OF_STATES);
      return false;
    }

    _x = x;

    return true;
  }

  /**
   * @brief Inserts state stamped vector x into the circular buffer, _state_buff
   */
  bool insertX(State x)
  {
    if (x.state.size() != NUM_OF_STATES)
      return false;
    // printf("-----State is not empty-----\n");

    if(!_state_buff.empty())
    {
      if(x.time <= _state_buff.back().time)
        return false;
    }
    // printf("-----State is new----\n");
    _x = x.state;
    _state_buff.push_back(x);
    return true;
  }

  /**
   * @brief returns _state_buff
   */
  boost::circular_buffer<State> getStateBuffer()
  {
    return _state_buff;
  }

  bool gotEnoughMeasurements()
  {
    if (_state_buff.size() < _max_pos_buff) return false;

    return true;
  }

  /**
  * @brief Returns current state _x
  * @return VectorXd.
  */
  Eigen::VectorXd getX(void)
  {
    return _x;
  }

  /**
   * State transition model. Takes current state x, and returns next state.
   */
  virtual Eigen::VectorXd model(Eigen::VectorXd x)
  {
    if(_debug)
    {
      printf("[PredictionModel::model] This is the model() function of the parent class. Nothing is implemented");
    }
    return x;
  }

  /**
  * @brief Transitions the state x one step in time using model()
  * @param x Type VectorXd. Initial state =[px, vx, py, vy, pz, vz].
  * @return VectorXd new state. Also updates _x
  */
  Eigen::VectorXd forward(Eigen::VectorXd x)
  {
    if(x.size() != NUM_OF_STATES)
    {
      printf("[PredictionModel::forward] Size of the state x %d != Number of states %d. Returning current sate _x", (int)x.size(), NUM_OF_STATES);
      return _x;
    }
    _x = model(x);

    return _x;
  }

  /**
  * @brief Transitions the internal state _x one step in time using model() and dt. Updates the internal _dt
  * @param dt [double] time step
  * @return VectorXd new state. Also updates _x
  */
  Eigen::VectorXd forward(double dt)
  {
    if (dt<=0)
    {
      printf("[PredictionModel::forward] time step is <=0. %f <=0", dt);
    }

    _dt=dt;

    _x = model(_x);

    return _x;
  }

  Eigen::VectorXd forward()
  {
    if(_x.size() != NUM_OF_STATES)
    {
      printf("[PredictionModel::forward] Size of the state x %d != Number of states %d. Returning current sate _x", (int)_x.size(), NUM_OF_STATES);
      return _x;
    }
    _x = model(_x);

    return _x;
  }

  /**
  * @brief Transitions the state x, N steps in time using model()
  * @param x Type VectorXd. Initial state =[px, vx, py, vy, pz, vz].
  * @param n Type int. Number of transitions.
  * @return VectorXd trajectory. Also updates _x
  */
  Eigen::VectorXd forward(Eigen::VectorXd x, int steps)
  {
    if (steps <= 0)
    {
      printf("[PredictionModel::forward] steps should be > 0");
      return x;
    }
    if(x.size() != NUM_OF_STATES)
    {
      printf("[PredictionModel::forward] Size of the state x %d != Number of states %d. Returning current sate _x", (int)x.size(), NUM_OF_STATES);
      return _x;
    }

    _x = x;
    Eigen::VectorXd traj;
    traj.resize(NUM_OF_STATES*(steps+1));
    traj.segment(0, NUM_OF_STATES) = _x;
    for (int i=1; i< (steps+1); i++)
    {
      _x = model(_x);
      traj.segment(i*NUM_OF_STATES, NUM_OF_STATES) = _x;
    }
    
    return traj;
  }

  /**
   * @brief Sets minimum position buffer length
   * @param L Integer Minimum buffer length
   * @return Bool False if L<3
   */ 
  bool setMinPosBuffLength(int L)
  {
    if (L > MAX_STATE_BUFF_LENGTH)
    {
      printf("[PredictinModel::setMinPosBuffLength] L=%d > MAX_STATE_BUFF_LENGTH=%d", L, MAX_STATE_BUFF_LENGTH);
      _min_pos_buff = MAX_STATE_BUFF_LENGTH;
    }
    else if (L<1){
      printf("[PredictinModel::setMinPosBuffLength] L=%d < 1. Setting to 1", L);
      _min_pos_buff = 1;
    }
    else
      _min_pos_buff = L;
    
    return true;
  }

  /**
  * @brief Set _max_pos_buff 
  */
  void setMaxPosBuffLength(int s)
  {
    if (s >= _min_pos_buff && s > 0)
      _max_pos_buff = s;
    else
    {
      printf("Max buffer length is either 0 or more than %d. Setting to the max %d", MAX_STATE_BUFF_LENGTH, MAX_STATE_BUFF_LENGTH);
      _max_pos_buff = MAX_STATE_BUFF_LENGTH;
    }
    
    _state_buff.rset_capacity(_max_pos_buff);
  }

double getFittingAccuracy(void)
{
  return _fitting_accuracy;
}

Eigen::MatrixXd 
getFittedPositions(void)
{
  return _fitted_positions;
}

};

#endif // BASE_MODEL_H