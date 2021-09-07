#ifndef _GLIBCXX_IOSTREAM
#include <iostream>
#endif

#ifndef _GLIBCXX_CMATH
#include <cmath>
#endif

#ifndef _GLIBCXX_VECTOR
#include <vector>
#endif

#include <Eigen/Core>

#define PI 3.1415926
#define l1 0.0838
#define l2 0.2
#define l3 0.2
#define L
#define W


double degToRad(double deg){
	double r = deg*PI/180;
	return r;
}
double radToDeg(double rad){
	double r = rad*180/PI;
	return r;
}





//FK

Eigen::Vector4d legFK(Eigen::Vector3d jointAngle, const int legParity){
	//jointAngle : (q1, q2, q3)
	//legParity : -1 if left, 1 if right
	
	Eigen::Matrix4d H1, H2, H3;
	H1 << cos(jointAngle[0]), -sin(jointAngle[0]), 0, -l1*legParity*cos(jointAngle[0]),
	      sin(jointAngle[0]), cos(jointAngle[0]),  0, -l1*legParity*sin(jointAngle[0]),
	      0, 0, 1, 0,
	      0, 0, 0, 1;
	H2 << 1, 0, 0, 0,
	      0, cos(jointAngle[1]), -sin(jointAngle[1]), -l2*cos(jointAngle[1]),
	      0, sin(jointAngle[1]), cos(jointAngle[1]), -l2*sin(jointAngle[1]),
	      0, 0, 0, 1;
	H3 << 1, 0, 0, 0,
	      0, cos(jointAngle[2]), -sin(jointAngle[2]), -l3*cos(jointAngle[2]),
	      0, sin(jointAngle[2]), cos(jointAngle[2]), -l3*sin(jointAngle[2]),
	      0, 0, 0, 1; 
	Eigen::Vector4d id(0,0,0,1);
	return H1*H2*H3*id;
}

/*Eigen::Vector4d bodyFK(Eigen::Vector3d jointAngle, const int legID);
	Eigen::Matrix4d transMat;
	transMat << 1, 0, 0, 0,
		    0, 1, 0, legID,
		    0, 0, 1, ,
		    0, 0, 0, 1;
	return legFK(jointAngle, leg)

}*/






//IK

Eigen::Vector3d legIK(Eigen::Vector4d legPosition, const int legParity, const int q3Parity){
	//legPosition : position of the end-effector with respect to each hip joint
	//legParity : -1 if left, 1 if right
	//q3Parity : decides L3 configuration
	
	double R1, R2, R3;
	double x(legPosition[0]), y(legPosition[1]), z(legPosition[2]);

	Eigen::Vector3d IKResult;	//IKResult : (q1, q2, q3)
	
	R1 = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));		//R1 : y2 distance from hip joint to end-effector
	R2 = sqrt(pow(R1, 2) + pow(z, 2)); 			//R2 : distance from hip joint to end-effector
	R3 = (pow(R2, 2)-pow(l2, 2)-pow(l3, 2))/(2*l2*l3);

	if(R3 > 1 || R3 < -1) (R3 > 0)?(R3 = 1):(R3 = -1);
	IKResult[2] = q3Parity*acos(R3);						// q3
	IKResult[1] = atan2(-z, R1)-atan2(l3*sin(IKResult[2]),l2+l3*cos(IKResult[2]));	// q2
	IKResult[0] = atan2(y, x)+atan2(R1, -l1*legParity);				// q1
	
	return IKResult; 
}


/*Eigen::Vectord bodyIK(Eigen::Vector4d bodyPosition, const int lr, const int parity3){
	//bodyPosition : position of the end-effector w.r.t the COM of the body (x', y', z' 1)
	//parity3 : the same as legIK
	
	double 
}*/



//VK

