#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//--------------------------------------------------------------------------
// LICENSE INFORMATION
//
// 1.  For academic/research users:
//
// This program is free for academic/research purpose:   you can redistribute
// it and/or modify  it under the terms of the GNU General Public License as 
// published by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// Under this academic/research condition,  this program is distributed in 
// the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along 
// with this program. If not, see <http://www.gnu.org/licenses/>.
//
// 2.  For commercial OEMs, ISVs and VARs:
// 
// For OEMs, ISVs, and VARs who distribute/modify/use this software 
// (binaries or source code) with their products, and do not license and 
// distribute their source code under the GPL, please contact NICTA 
// (www.nicta.com.au), and NICTA will provide a flexible OEM Commercial 
// License. 
//
//---------------------------------------------------------------------------
//
// Modified by Filip Srajer (filip.srajer at fel.cvut.cz)
// 12/06/2015 (day/month/year)
// Changes:
// 0) Inserted into the new VS 2013 project (5point).
// 1) Added ifndef, define, endif wrapper.
// 2) Wrapped in fivepoint namespace
//
//---------------------------------------------------------------------------

#ifndef FIVEPOINT_HIDDEN6_H
#define FIVEPOINT_HIDDEN6_H

#ifdef BUILD_MEX
#   include <mex.h>
#   define printf mexPrintf
#endif

namespace fivepoint
{

// Dimensions of the matrices that we will be using
const int Nrows = 10;
const int Ncols = 10;
const int Maxdegree = 20;

// For holding polynomials of matrices
typedef double PolyMatrix[Nrows][Ncols][Maxdegree + 1];
typedef int PolyDegree[Nrows][Ncols];
typedef double Matches[][3];

// We need to be able to solve matrix equations up to this dimension
typedef double BMatrix[Maxdegree + 1][Maxdegree + 1];

// Forward declarations
// void print_equation_set (EquationSet A, int maxdegree = 3);
void print_polymatrix(PolyMatrix A,int maxdegree = 3);

void polyquotient(
  double *a,int sa,
  double *b,double *t,int sb,
  double *q,int &sq,
  BMatrix B,int &current_size
  );

void find_polynomial_determinant(
  PolyMatrix &Q,
  PolyDegree deg,
  int rows[Nrows],  // This keeps the order of rows pivoted on.
  int dim = Nrows
  );

void det_preprocess_6pt(
  PolyMatrix &Q,
  PolyDegree degree,
  int n_zero_roots	// Number of roots known to be zero
  );

void do_scale(
  PolyMatrix &Q,
  PolyDegree degree,
  double &scale_factor,	// Factor that x is multiplied by
  bool degree_by_row,
  int dim = Nrows
  );

}

#endif FIVEPOINT_HIDDEN6_H