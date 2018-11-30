/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libicp.
Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libicp can be used

#include <iostream>
#include "icpPointToPlane.h"
#include "icpPointToPoint.h"
#include "bgs_time.h"

using namespace std;

int main (int argc, char** argv) {

  // define a 3 dim problem with 10000 model points
  // and 10000 template points:
  int32_t dim = 2;
  int32_t num = 50;

  // allocate model and template memory
  double* M = (double*)calloc(dim*num,sizeof(double));
  double* T = (double*)calloc(dim*num,sizeof(double));

  // set model and template points
  cout << endl << "Creating model with 10000 points ..." << endl;
  cout << "Creating template by shifting model by (1,0.5,-1) ..." << endl;
  int32_t k=0;
  
  for( int32_t k = 0; k < num; k++ )
  {
	  M[k*dim+0] = k * 0.05;
	  M[k*dim+1] = -k * 0.06;
	  T[k*dim+0] = M[k*dim+0] - 0.25;
	  T[k*dim+1] = M[k*dim+1] - 0.37;
  }

  // start with identity as initial transformation
  // in practice you might want to use some kind of prediction here
  Matrix R = Matrix::eye(dim);
  Matrix t(dim,1);
  
  // test the local mxm
//   int32_t test_dim = 1000;
//   FLOAT val[test_dim];
//   for( int i = 0; i < test_dim; ++i )
// 	  val[i] = i * 0.37;
//   Matrix a( test_dim, 1, val );
//   Matrix b( 1, 2, val+1 );
//   cout << "local_mxm a*b = " << a*b << endl;
  
  
  // run point-to-plane ICP (-1 = no outlier threshold)
  cout << endl << "Running ICP (point-to-point, no outliers)" << endl;
  
  BgsTime time = BgsTime::get_current_time();
  IcpPointToPoint icp(M,num,dim);
  double residual = icp.fit(T,num,R,t,-1);
  
  std::cout << "time : " << (BgsTime::get_current_time() - time).DebugString() << std::endl;

  // results
  cout << endl << "Transformation results:" << endl;
  cout << "R:" << endl << R << endl << endl;
  cout << "t:" << endl << t << endl << endl;
  cout << "Residual:"<<residual << endl;

  // free memory
  free(M);
  free(T);

  // success
  return 0;
}

