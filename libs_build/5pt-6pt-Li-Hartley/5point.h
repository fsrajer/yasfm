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
// 0) File created.
// 1) compute_E_matrices declaration taken from Ematrix_5pt.cc.
// 2) Matches and Ematrix definition taken from hidden6.h and Ematrix_5pt.cc respectively.
// 3) Added Maxsolutions based on Maxdegree in hidden6.h.
// 4) Wrapped in fivepoint namespace
//
//---------------------------------------------------------------------------

#ifndef FIVEPOINT_5POINT_H
#define FIVEPOINT_5POINT_H

namespace fivepoint
{

typedef double Matches[][3];
typedef double Ematrix[3][3];
const int Maxsolutions = 20;

// Input Matches of size 5.
// Optimized version runs highly optimized Nister's 5 point.
// Other version is generic version of the solver by Li and Hartley.
void compute_E_matrices(
  const Matches q,const Matches qp,
  Ematrix Ematrices[10],
  int &nroots,
  bool optimized = true 
  );

}

#endif // FIVEPOINT_5POINT_H