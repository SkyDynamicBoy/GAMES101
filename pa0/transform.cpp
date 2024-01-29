# include<eigen3/Eigen/Core>
# include<iostream>
# include<cmath>

using namespace std;
using namespace Eigen;

#define PI 3.1415926

int main(){
	Vector3d p(2.0, 1.0, 1.0);

	Matrix3d R, T;
	float sin45 = sin(45.0/180.0*PI);
	float cos45 = sin(45.0/180.0*PI);
	cout << sin45 << ":::" << cos45 << endl;
	R << cos45, -sin45, 0, sin45, cos45,0, 0, 0, 1;	//Rotate 45 degree counterclockwise
	T << 1, 0, 1, 0, 1, 2, 0, 0, 1;  //Tranlate (1,2)
	cout << "\nthe rotation matrix is:\n" << R << endl;
	cout << "\nthe translation matrix is:\n" <<T << endl;
	Vector3d result = T*R*p;
	cout << result << endl;
}

