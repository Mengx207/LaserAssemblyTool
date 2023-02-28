
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#define n 3
 
using namespace std; 

int dotProduct(int vect_A[], int vect_B[]);
void crossProduct(int vect_A[], int vect_B[], int cross_P[]);
 
int main()
{
	//Known normal vectors for two planes
    int N1[] = { 10,8,3 }; 
    int N2[] = { 2,6,5 };
	//Known one point on each plane
	int point1[] = { 10,5,5 };
	int point2[] = { 2,2,2 };

	double a1,b1,c1,a2,b2,c2;
	a1 = N1[0];
	b1 = N1[1];
	c1 = N1[2];
	a2 = N2[0];
	b2 = N2[1];
	c2 = N2[2];	
    int cross_P[n];
	//find the plane equations
	double x,y,z;
	//a1*(x-point1[0])+b1*(y-point1[0])+c1*(z-point1[0]) = 0;
	//a2*(x-point2[0])+b2*(y-point2[0])+c2*(z-point2[0]) = 0;
	//a1*x+b1*y+c1*z = (a1*point1[0]+b1*point1[0]+c1*point1[0]);
	//a2*x+b2*y+c2*z = (a2*point2[0]+b2*point2[0]+c2*point2[0]);	
	cout<<"Plane1 equation: "<<a1<<"(x-"<<point1[0]<<")+"<<b1<<"*(y-"<<point1[1]<<")+"<<c1<<"*(z-"<<point1[2]<<") = 0"<<endl;
	cout<<"Plane2 equation: "<<a2<<"(x-"<<point2[0]<<")+"<<b2<<"*(y-"<<point2[1]<<")+"<<c2<<"*(z-"<<point2[2]<<") = 0"<<endl;
    // dotProduct function call
    cout << "Dot product:";
    cout << dotProduct(N1, N2) << endl;
    // crossProduct function call
    crossProduct(N1, N2, cross_P);
	cout<<"Nomal vector cross product: v=("<<cross_P[0]<<","<<cross_P[1]<<","<<cross_P[2]<<")";
	// To find a point on intersection line, use two plane equations and set z=0
	// a1*x+b1*y = (a1*point1[0]+b1*point1[0]+c1*point1[0]);
	// a2*x+b2*y = (a2*point2[0]+b2*point2[0]+c2*point2[0]);
	c1 = a1*point1[0]+b1*point1[0]+c1*point1[0];
	c2 = a2*point2[0]+b2*point2[0]+c2*point2[0];
	x = (c1*b2-b1*c2)/(a1*b2-b1*a2);
	y = (a1*c2-c1*a2)/(a1*b2-b1*a2);
	cout<<endl<<"One point on intersection line: r0 = ("<<x<<","<<y<<",0)"<<endl;
	
	// Plug v and r0 into vector equation
	// r = (x*i + y*j + 0*k) + t*(cross_P[0]*i+cross_P[1]*j+cross_P[2]*k)
	// r = (x+t*cross_P[0])*i + (y+t*cross_P[1])*j + (t*cross_P[2])*k
	double a,b,c;
	char t;
	a = x+t*cross_P[0];
	b = y+t*cross_P[1];
	c = t*cross_P[2];
	cout<<"Intersection line(vector equation) of two planes:"<<endl<<"r= a*i+b*j+c*k"<<endl;
	cout<<"a="<<x<<"+t*"<<cross_P[0]<<endl;
	cout<<"b="<<y<<"+t*"<<cross_P[1]<<endl;
	cout<<"c=t*"<<cross_P[2]<<endl;
	cout<<"r=("<<x<<"+t*"<<cross_P[0]<<")*i+("<<y<<"+t*"<<cross_P[1]<<")*j+("<<cross_P[2]<<")*k"<<endl;
	
	return 0;
}

int dotProduct(int vect_A[], int vect_B[])
{
    int product = 0;
    for (int i = 0; i < n; i++)
	{
        product = product + vect_A[i] * vect_B[i];
	}
    return product;
}
 
// cross product of two vector array.
void crossProduct(int vect_A[], int vect_B[], int cross_P[])
 
{
 
    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}