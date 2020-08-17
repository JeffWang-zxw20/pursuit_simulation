#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;
int main()
{
    //1.rotation vector to  rotation matrix
    cout<<"------------------------"<<endl;
    AngleAxisd rotationVector(M_PI/4,Vector3d(0,0,1));
    Matrix3d rotationMatrix=Matrix3d::Identity();
    rotationMatrix=rotationVector.toRotationMatrix();
    cout<<"rotationMatrix \n"<<rotationMatrix<<endl;

    //2.rotation vector to quaterniond
    cout<<"------------------------"<<endl;
    Quaterniond q=Quaterniond( rotationVector );
    cout<<"rotation quaterniond \n"<<q.coeffs()<<endl;

    //3.rotaion vector to eulerAngles
    cout<<"------------------------"<<endl;
    Vector3d eulerAngle=rotationVector.matrix().eulerAngles(0,1,2);
    cout<<"eulerAngle roll pitch yaw\n"<<180*eulerAngle/M_PI<<endl;


    //4.EulerAngles to RotationMatrix
    cout<<"4.------------------------"<<endl;
    double yaw, pitch, roll;
    yaw = 90.0*M_PI/180.0;
    pitch = 0;
    roll = 60.0*M_PI/180.0;
    Vector3d ea0(yaw,pitch,roll);

    Matrix3d R; //ZYX EulerAngles
    R = AngleAxisd(ea0[0], Vector3d::UnitZ())
        * AngleAxisd(ea0[1], Vector3d::UnitY())
        * AngleAxisd(ea0[2], Vector3d::UnitX());
 
    cout << "RotationMatrix:"<<endl<<R<<endl;

    //5.RotationMatrix to Quaterniond
    cout<<"5.------------------------"<<endl; 
    q = R;    
    cout << "Quaterniond:"<<endl<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<", "<<endl;
    
    //6.Quaterniond to RotationMatrix
    cout<<"6.------------------------"<<endl;
    Matrix3d Rx = q.toRotationMatrix();
    cout << "RotationMatrix:"<<endl<<Rx<<endl;

    //7.Quaterniond to EulerAngles
    cout<<"7.------------------------"<<endl; //The output order is: yaw/ pitch/ roll/
    Vector3d eulerAngle1=q.matrix().eulerAngles(2,1,0);
    cout << "EulerAngles:"<<endl<<eulerAngle1<<endl;
    
    //8.RotationMatrix to EulerAngles
    cout<<"8.------------------------"<<endl;
    Vector3d ea1 = Rx.eulerAngles(2,1,0);//EulerOrder: ZYX
    cout << "EulerAngles:"<<endl<<ea1/M_PI*180 << endl << endl;

    //9.Point Transform
    cout<<"9.------------------------"<<endl;
    Vector3d Pointi(1,1,1);
    Vector3d t(5,10,15);
    Vector3d Pointj;
    Pointj = q*Pointi + t;
    cout<<"Pointj:"<<endl<<Pointj.transpose()<<endl;

    //proportion
    Vector3d Pointj1;
    Vector3d eulerAngle2 = q.matrix().eulerAngles(2,1,0);
    cout<<"eulerAngle2: "<<eulerAngle2.transpose()/M_PI*180<<endl;
    
    Vector3d TheEuler = eulerAngle2*0.5;
    cout<<"The Euler: "<<TheEuler.transpose()/M_PI*180<<endl;
    Quaterniond Theq;
    Theq = AngleAxisd(TheEuler(0), Vector3d::UnitZ()) 
        * AngleAxisd(TheEuler(1), Vector3d::UnitY()) 
        * AngleAxisd(TheEuler(2), Vector3d::UnitX());

    //Vector3d point_start = Theq.inverse() * (point_frame-t*s);
    Vector3d HalfEulerAngles = Theq.matrix().eulerAngles(2,1,0);
    cout << "HalfEulerAngles:"<<endl<<HalfEulerAngles/M_PI*180<<endl;

    //Inverse
    Vector3d inversePointi;
    inversePointi = q.inverse()*(Pointj - t);
    cout<<"inversePoint:"<<endl<<inversePointi.transpose()<<endl;


    //10.Matrix Assignment
    Matrix<double, 6, 6> J;
    for(int i = 0; i < 6; i++){
        for(int j =0; j < 6; j++){
            J(i,j) = 0;
        }
    }
    Vector3d Z1(1,2,3);
    Vector3d Z2(4,5,6);
    J.block(0,0,3,1) = Z1;
    J.block(3,0,3,1) = Z2;
    cout<<endl<<"J:"<<endl<<J<<endl;

    Vector3d Zc = Z1.cross(Z2);
    cout<<"Zc:"<<endl<<Zc<<endl;


    return 0;
}

