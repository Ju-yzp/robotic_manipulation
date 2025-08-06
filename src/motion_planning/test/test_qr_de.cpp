#include <motion_planning/util.hpp>

int main()
{
using namespace motion_planning;
Eigen::Matrix<double,4,3> A;
Eigen::Vector<double,4> B;
A<<1,2,3,
   4,5,6,
   7,8,9,
   10,11,12;
SVDSolver solver;
solver.solve(A,B);
}