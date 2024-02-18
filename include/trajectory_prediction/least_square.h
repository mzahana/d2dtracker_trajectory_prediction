/*
BSD 3-Clause License

Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LEAST_SQUARE_H
#define LEAST_SQUARE_H

#include <stdio.h>
#include <Eigen/Dense>
// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>





/**
 * @brief Class for solving constrained least square problem, for trajectory fitting
 * Reference: https://osqp.org/docs/examples/least-squares.html
 */
class LeastSquare
{
private:
   bool                          _debug;                 /** Printing debug messages */
   bool                          _initialized;           /** True if problem/solver is intialized */
   unsigned int                  _NUM_X;              /** Number of Least squares optimization variables (x) */
   unsigned int                  _NUM_VARS;              /** Number of QP optimization variables ( \xi )*/
   unsigned int                  _NUM_LS_CONSTRAINTS;              /** Number of LS contraints from measurements */
   unsigned int                  _NUM_LINEAR_CONSTRAINTS;         /** Numbe rof extra linear constraints on the LS variables (e.g. box constraints)*/
   unsigned int                  _NUM_QP_CONSTRAINTS;              /** Number of QP contraints */
   Eigen::VectorXd               _gradient;              /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
   Eigen::MatrixXd               _A;                     /** The least squares A matrix */
   Eigen::VectorXd               _b;                     /** Least squre b (measurement) matrix */
   Eigen::MatrixXd               _hessian;               /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
   Eigen::SparseMatrix<double>   _hessian_sparse;   /** sparce version of the hessian */
   Eigen::MatrixXd               _C;                    /** Linear constraint  matrix of the QP problem */
   Eigen::SparseMatrix<double>   _C_sparse;       /** Sparse version of C */ 
   Eigen::VectorXd               _qp_lowerBounds;            /** Lower bounds vector of the QP problem */
   Eigen::VectorXd               _ls_lowerBounds;            /** Lower bounds vector of the LS problem */
   Eigen::VectorXd               _qp_upperBounds;           /** Upper bounds vector of the QP problem */
   Eigen::VectorXd               _ls_upperBounds;           /** Upper bounds vector of the LS problem */
   OsqpEigen::Solver             _qpSolver;              /** Object of the quadratic program solver */

   void setA(Eigen::MatrixXd A)
   {
      _A = A;
      _NUM_X = (unsigned int) A.cols();
      _NUM_LS_CONSTRAINTS = (unsigned int) A.rows();
   }
   
   void
   setB(Eigen::VectorXd b)
   {
      _b = b;
   }

   /**
    * @brief Sets the least square data matrices
    * @param A Eigen::MatrixXd A matrix
    * @param b Eigen::VectorXd Least square b (measurement) matrix
   */
   bool
   setLSData(Eigen::MatrixXd A, Eigen::VectorXd b)
   {
      unsigned int b_size = (unsigned int) b.size();
      unsigned int A_rows = (unsigned int) A.rows();

      if (b_size != A_rows)
      {
         printf("[LeastSquare::setLSData] Number of rows in Least square matrix A != size of vector b");
         return false;
      }

      setA(A);
      setB(b);
      return true;
   }


   bool
   setQPNumVars(unsigned int n)
   {
      if (n<1) return false;
      _NUM_VARS  = n;
      return true;
   }

   bool
   setQPNumConstraints(unsigned int n)
   {
      if (n<1) return false;
      _NUM_QP_CONSTRAINTS = n;
      return true;
   }
   
   /**
    * NOT USED
    * @brief Set Hessian matrix of the QP problem
    * @param h Eigen matrix
    * @return bool False if rows or cols != _NUM_VARS
   */
   bool
   setQPHessianMatrix(Eigen::MatrixXd h)
   {
      if (h.rows() != _NUM_VARS || h.cols() != _NUM_VARS)
      {
         printf("[LeastSquare::setHessian] Nu,ber of rows: %ld, or columns: %ld does not equal to number of optimization variables: %d", h.rows(), h.cols(), _NUM_VARS);
         return false;
      }
      _hessian = h;
      return true;      
   }

   /**
    * NOT USED
    * @brief Sets the Constraints matrix
    * @param C Eigen matrix
    * @return bool False if C.rows != _NUM_QP_CONSTRAINTS, or C.cols() != _NUM_VARS
   */
   bool
   setQPConstraintsMatrix(Eigen::MatrixXd C)
   {
      if (C.rows() != _NUM_QP_CONSTRAINTS)
      {
         printf("[LeastSquare::setQPConstraintsMatrix] Number of rows: %ld does not equal to the number of constraints %d", C.rows(), _NUM_QP_CONSTRAINTS);
         return false;
      }

      if (C.cols() != _NUM_VARS)
      {
         printf("[LeastSquare::setQPConstraintsMatrix] Number of columns: %ld does not equal to the number of optimization variables %d", C.cols(), _NUM_VARS);
         return false;
      }
      _C = C;
      return true;
   }

   /**
    * NOT USED
   */
   bool
   setQPLowerBounds(Eigen::VectorXd L)
   {
      if ( (unsigned int) L.size() != _NUM_QP_CONSTRAINTS)
      {
         printf("[LeastSquare::setQPLowerBounds] Size of vector: %d != _NUM_QP_CONSTRAINTS: %d", (unsigned int)L.size(), _NUM_QP_CONSTRAINTS);
         return false;
      }
      _qp_lowerBounds = L;
      return true;
   }

   /**
    * NOT USED
   */
   bool
   setQPUpperBounds(Eigen::VectorXd U)
   {
      if ( (unsigned int) U.size() != _NUM_QP_CONSTRAINTS)
      {
         printf("[LeastSquare::setQPLowerBounds] Size of vector: %d != _NUM_QP_CONSTRAINTS: %d", (unsigned int)U.size(), _NUM_QP_CONSTRAINTS);
         return false;
      }
      _qp_upperBounds = U;
      return true;
   }

   /**
    * NOT USED
    * @brief Sets both lower and upper bounds vectors
    * @param L Eigen::VectorXd Lower bounds vector
    * @param U Eigen::VectorXd Upper bounds vector
   */
   bool
   setQBounds(Eigen::VectorXd L, Eigen::VectorXd U)
   {
      return (setQPUpperBounds(U) || setQPLowerBounds(L));
   }

   /**
    * NOT USED
    * @brief Initialize QP problem data
    * @param ns Number of optimization variables
    * @param nc Number of constraints
    * @param h Eigen::MatrixXd Hessian matrix
    * @param C Eigen::MatrixXd Constraints matrix
    * @param L Eigen::VectorXd Lower bounds vector
    * @param U Eigen::VectorXd Upper bounds vector
   */
   bool
   initQPData(unsigned int ns, unsigned int nc, Eigen::MatrixXd h, Eigen::MatrixXd C, Eigen::VectorXd L, Eigen::VectorXd U)
   {
      if (!setQPNumVars(ns)) return false;
      if(!setQPNumConstraints(nc)) return false;
      if (!setQPHessianMatrix(h)) return false;
      if (!setQPConstraintsMatrix(C)) return false;
      if (!setQPLowerBounds(L)) return false;
      if (!setQPUpperBounds(U)) return false;

      if(_debug)
         printf("[LeastSquare::initQPData] Done initializing problem data");

      return true;
   }

   /**
    * @brief Initialize QP solver
    * @return Bool. False if there is an error
   */
  bool
  initSolver()
  {
   printf(" [LeastSquare::initSolver] Initializing QP solver ...\n");

   _qpSolver.settings()->setVerbosity(_debug);
   _qpSolver.settings()->setWarmStart(true);

   // set the initial data of the QP solver
   _qpSolver.data()->setNumberOfVariables(_NUM_VARS);
   _qpSolver.data()->setNumberOfConstraints(_NUM_QP_CONSTRAINTS);
   _hessian_sparse = _hessian.sparseView();
   _C_sparse = _C.sparseView();
   if(!_qpSolver.data()->setHessianMatrix(_hessian_sparse)) return false;
   Eigen::VectorXd gradient = Eigen::VectorXd::Zero(_NUM_VARS);
   if(!_qpSolver.data()->setGradient(gradient)) return false; // gradient vector is always zero
   if(!_qpSolver.data()->setLinearConstraintsMatrix(_C_sparse)) return false;
   if(!_qpSolver.data()->setLowerBound(_qp_lowerBounds)) return false;
   if(!_qpSolver.data()->setUpperBound(_qp_upperBounds)) return false;

   if(!_qpSolver.initSolver()) return false;


   if(_debug)
   {
      printf("QP solver is initialized.");
   }
   return true;
  }
  
  bool
  updateQPData(void)
  {
   if(!_qpSolver.updateBounds(_qp_lowerBounds, _qp_upperBounds))
   {
      printf("[LeastSquare::updateQPData] _qpSolver.updateBounds failed to update bounds");
      return false;
   }
   if(_debug)
      printf("[LeastSquare::updateQPData] Bounds are updated.");

   _C_sparse = _C.sparseView();
   if (!_qpSolver.updateLinearConstraintsMatrix(_C_sparse))
   {
      printf("[LeastSquare::updateQPData] Could not update contraint matrix");
      return false;
   }
   if(_debug)
      printf("[LeastSquare::updateQPData] constraint matrix is updated.");

   if (_debug)
      printf("[LeastSquare::updateQPData] Problem data is updated.");

   return true;

  }

   
public:
   LeastSquare(): _debug(false) {_NUM_LINEAR_CONSTRAINTS=0; _NUM_LS_CONSTRAINTS=0; _NUM_QP_CONSTRAINTS=0; _NUM_X=0; _NUM_VARS=0; _initialized=false;}
   ~LeastSquare(){}

   void setDebug(bool d)
   {
      _debug = d;
   }
   bool isLSInitialized(void) { return _initialized;}   
   
   /**
    * @brief Initialize the QP matrices using number of variables and constraints of least squares problem
    * @param nx number of variables in the least square problem
    * @param nm number of measurements in the least square problem 
    * @param nc number of linear constraints on the least square optimization variables
    * @param W  Eigen::MatrixXd Matrix of extra linear constraints on the LS variables
    * @param x_l Eigen::VectorXd lower bounds on x the least square optimization variable
    * @param x_u Eigen::VectorXd upper bounds on x
   */
   bool
   initLSProblem(unsigned int nx, unsigned int nm, unsigned int nc, Eigen::MatrixXd W,  Eigen::VectorXd x_l, Eigen::VectorXd x_u)
   {
      if(_debug)
         printf("[LeastSquare::initLSProblem] Initializing least square problem");

      // Sanity checks
      if(nx<1) return false;
      if(nm<=nx) return false;

      _NUM_X = nx;
      _NUM_LS_CONSTRAINTS = nm;
      _NUM_VARS = _NUM_X + _NUM_LS_CONSTRAINTS;
      _NUM_LINEAR_CONSTRAINTS = nc;
      _NUM_QP_CONSTRAINTS = _NUM_LINEAR_CONSTRAINTS + _NUM_LS_CONSTRAINTS;

      // Hessian
      _hessian = Eigen::MatrixXd::Zero(nx+nm, nx+nm);
      _hessian.block(nx,nx,nm,nm) = Eigen::MatrixXd::Identity(nm,nm);
      _hessian_sparse = _hessian.sparseView();

      //Constraints matrix
      _C = Eigen::MatrixXd::Zero(nm+nc, nx+nm);
      // _C.block(0,0,nx,nx) = Eigen::MatrixXd::Identity(nx,nx);
      // _C.block(nx,nx, nm,nm) = -1*Eigen::MatrixXd::Identity(nm,nm);
      _C.block(0,nx, nm,nm) = -1*Eigen::MatrixXd::Identity(nm,nm);
      _C.block(nm,0, nc,nx) = W;
      _C_sparse = _C.sparseView();


      // Upper/lower bounds
      _qp_lowerBounds = Eigen::VectorXd::Zero(nc+nm);
      _qp_lowerBounds.block(nm,0, nc,1) = x_l;
      _qp_upperBounds = Eigen::VectorXd::Zero(nc+nm);
      _qp_upperBounds.block(nm,0, nc,1) = x_u;

      if(!initSolver()) return false;

      _initialized = true;

      if (_debug)
         printf("[LeastSquare::initLSProblem] LS is initialized.");

      return true;
   }

   /**
    * @param A Eigen::MatrixXd Least square A matrix, from measurements
    * @param b Eigen::VectorXd Least square b matrix, from measurements
    * @param W Eigen::MatrixXd Linear constraints matrix on the LS variables
    * @param x_l Eigen::VectorXd Vector of lower bounds on the LS variables
    * @param x_u Eigen::VectorXd Vector of upper bounds on the LS variables
   */
   bool
   updateLSData(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd W, Eigen::VectorXd x_l, Eigen::VectorXd x_u)
   {
      // Sanity checks
      if (_NUM_LS_CONSTRAINTS<=0)
      {
         printf(" _NUM_LS_CONSTRAINTS: %d <=0", _NUM_LS_CONSTRAINTS);
         return false;
      }

      if (_NUM_QP_CONSTRAINTS<=0)
      {
         printf(" _NUM_QP_CONSTRAINTS: %d <=0", _NUM_QP_CONSTRAINTS);
         return false;
      }

      if (_NUM_X<=0)
      {
         printf(" _NUM_X: %d <=0", _NUM_X);
         return false;
      }

      if (_NUM_VARS<=0)
      {
         printf(" _NUM_VARS: %d <=0", _NUM_VARS);
         return false;
      }

      if (_NUM_LINEAR_CONSTRAINTS<=0)
      {
         printf(" _NUM_LINEAR_CONSTRAINTS: %d <=0", _NUM_LINEAR_CONSTRAINTS);
         return false;
      }
      
      if ((unsigned int)A.rows() != _NUM_LS_CONSTRAINTS)
      {
         printf("[LeastSquare::updateLSData] A.rows() != _NUM_LS_CONSTRAINTS");
         return false;
      }
      
      if (A.cols() != (unsigned int) _NUM_X)
      {
         printf("[LeastSquare::updateLSData] A.cols != _NUM_X");
         return false;
      }

      /** @todo implement more sanity checks */

      _C.block(0, 0, _NUM_LS_CONSTRAINTS, _NUM_X) = A;
      _C.block(_NUM_LS_CONSTRAINTS, 0, _NUM_LINEAR_CONSTRAINTS, _NUM_X) = W;

      
      _qp_lowerBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;
      _qp_lowerBounds.block(_NUM_LS_CONSTRAINTS,0,_NUM_LINEAR_CONSTRAINTS,1) = x_l;

      _qp_upperBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;
      _qp_upperBounds.block(_NUM_LS_CONSTRAINTS,0,_NUM_LINEAR_CONSTRAINTS,1) = x_u;

      if(!updateQPData()) return false;

      if(_debug)
         printf("[LeastSquare::updateLSData] LS data is updated.");

      return true;
   }

   bool
   updateLSData(Eigen::MatrixXd A, Eigen::VectorXd b)
   {
      // Sanity checks
      if ((unsigned int)A.rows() != _NUM_LS_CONSTRAINTS)
      {
         printf("[LeastSquare::updateLSData] A.rows() != _NUM_LS_CONSTRAINTS");
         return false;
      }
      if (A.cols() != (unsigned int) _NUM_X)
      {
         printf("[LeastSquare::updateLSData] A.cols != _NUM_X");
         return false;
      }

      /** @todo implement more sanity checks */

      _C.block(0, 0, _NUM_LS_CONSTRAINTS, _NUM_X) = A;
      
      _qp_lowerBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;
      _qp_upperBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;

      if(!updateQPData()) return false;

      if(_debug)
         printf("[LeastSquare::updateLSData] LS data is updated.");

      return true;
   }

   bool
   updateLSData(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd xl, Eigen::VectorXd xu)
   {
      // Sanity checks
      if ((unsigned int)A.rows() != _NUM_LS_CONSTRAINTS)
      {
         printf("[LeastSquare::updateLSData] A.rows() != _NUM_LS_CONSTRAINTS");
         return false;
      }
      if (A.cols() != (unsigned int) _NUM_X)
      {
         printf("[LeastSquare::updateLSData] A.cols != _NUM_X");
         return false;
      }

      /** @todo implement more sanity checks */

      _C.block(0, 0, _NUM_LS_CONSTRAINTS, _NUM_X) = A;
      
      _qp_lowerBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;
      _qp_lowerBounds.block(_NUM_LS_CONSTRAINTS,0,_NUM_LINEAR_CONSTRAINTS,1) = xl;

      _qp_upperBounds.block(0,0, _NUM_LS_CONSTRAINTS,1) = b;      
      _qp_upperBounds.block(_NUM_LS_CONSTRAINTS,0,_NUM_LINEAR_CONSTRAINTS,1) = xu;

      if(!updateQPData()) return false;

      if(_debug)
         printf("[LeastSquare::updateLSData] LS data is updated.");

      return true;
   }

  /**
   * @brief Execute _qpSolver.solve() to solve QP
   * @return bool True if there is a solution. False otherwise
  */
  bool
  solve(void)
  {
   if(_qpSolver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
   {
      printf("[LeastSquare::solve] Unable to solve QP problem");
      auto  qp_status = _qpSolver.getStatus();
      if(qp_status != OsqpEigen::Status::Solved)
         printf("[LeastSquare::solve] Solution is unfeasible. Status = %d", (int)qp_status);

      return false;
   }
   
   // if(!_qpSolver.solve()) return false;
   return true;
  }

  /**
   * @brief Returns the solution of the least square variabels (part of the total optimizaiton vector)
   * @return Eigen::VectorXd (_NUM_X,1) Least square solution
  */
  Eigen::VectorXd
  getSolution(void)
  {
   Eigen::VectorXd sol = _qpSolver.getSolution();
   return sol.block(0,0,_NUM_X,1);
  }

};

#endif //LEAST_SQUARE_H
