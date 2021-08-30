#include<iostream>
using namespace std;

#include<ctime>
#include<Eigen/Core>
#include<Eigen/Dense>
using namespace Eigen;

int main(int argc ,char** argv)
{
   Matrix<double,Dynamic,Dynamic> A=MatrixXd::Random(100,100);
   A=A*A.transpose();
   Matrix<double,Dynamic,1> B=MatrixXd::Random(100,1);
   
    //求解AX=B
   //直接分解
   Matrix<double,Dynamic,1> X1=A.inverse()*B;

   //QR分解
   Matrix<double,Dynamic,1> X2;
   X2=A.colPivHouseholderQr().solve(B);

   //cholesky分解
   Matrix<double,Dynamic,1> X3;
   X3=A.ldlt().solve(B);

   cout<<"X1"<<X1.transpose()<<endl;
   cout<<"X2"<<X2.transpose()<<endl;
   cout<<"X3"<<X3.transpose()<<endl;

   return 0;
}
