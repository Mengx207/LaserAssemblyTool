// #include <iostream>
// #include <vector>

// using namespace std;

// struct Plane {
//     vector<double> normal;
//     double d;
// };

// vector<double> crossProduct(vector<double> a, vector<double> b) {
//     vector<double> result(3);
//     result[0] = a[1]*b[2] - a[2]*b[1];
//     result[1] = a[2]*b[0] - a[0]*b[2];
//     result[2] = a[0]*b[1] - a[1]*b[0];
//     return result;
// }

// vector<double> solveLinearEquations(vector<vector<double>> A, vector<double> b) {
//     int n = A.size();
//     vector<vector<double>> augmented(n, vector<double>(n+1));
//     for (int i = 0; i < n; i++) {
//         for (int j = 0; j < n; j++) {
//             augmented[i][j] = A[i][j];
//         }
//         augmented[i][n] = b[i];
//     }
//     for (int i = 0; i < n; i++) {
//         int maxRow = i;
//         for (int j = i+1; j < n; j++) {
//             if (abs(augmented[j][i]) > abs(augmented[maxRow][i])) {
//                 maxRow = j;
//             }
//         }
//         swap(augmented[i], augmented[maxRow]);
//         for (int j = i+1; j <= n; j++) {
//             augmented[i][j] /= augmented[i][i];
//         }
//         for (int j = 0; j < n; j++) {
//             if (j != i) {
//                 for (int k = i+1; k <= n; k++) {
//                     augmented[j][k] -= augmented[j][i] * augmented[i][k];
//                 }
//             }
//         }
//     }
//     vector<double> x(n);
//     for (int i = 0; i < n; i++) {
//         x[i] = augmented[i][n];
//     }
//     return x;
// }

// void findIntersectionLine(Plane p1, Plane p2) {
//     vector<double> dir = crossProduct(p1.normal, p2.normal);
//     vector<vector<double>> A = {p1.normal, p2.normal, dir};
//     vector<double> b = {-p1.d, -p2.d, 0};
//     vector<double> point = solveLinearEquations(A, b);
//     cout << "The intersection line is: ";
//     cout << "x = " << point[0] << " + " << dir[0] << "t, ";
//     cout << "y = " << point[1] << " + " << dir[1] << "t, ";
//     cout << "z = " << point[2] << " + " << dir[2] << "t" << endl;
// }

// int main() {
//     Plane p1 = {{7, 4, -2}, -12};
//     Plane p2 = {{3, -3, -1}, 8};
// 	findIntersectionLine(p1,p1);

// }

// #include <iostream>
// #include <cmath>

// using namespace std;

// struct Plane {
//     double a, b, c, d; // coefficients of the plane equation
// };

// struct Line {
//     double x, y, z; // point on the line
//     double l, m, n; // direction vector of the line
// };

// Line intersectPlanes(Plane p1, Plane p2) {
//     Line line;

//     // calculate the direction vector of the line
//     line.l = p1.b*p2.c - p1.c*p2.b;
//     line.m = p1.c*p2.a - p1.a*p2.c;
//     line.n = p1.a*p2.b - p1.b*p2.a;

//     // calculate a point on the line
//     double det = p1.a*p2.b*p2.d + p1.b*p2.c*p2.d + p1.c*p2.a*p2.d
//                - p1.a*p2.c*p1.d - p1.b*p2.a*p1.d - p1.c*p2.b*p1.d;
//     line.x = (p1.b*p2.d - p1.d*p2.b)*p1.c - p1.b*(p2.c*p1.d - p1.c*p2.d);
//     line.y = (p1.d*p2.a - p1.a*p2.d)*p1.c - p1.d*(p2.c*p1.a - p1.c*p2.a);
//     line.z = (p1.a*p2.d - p1.d*p2.a)*p1.b - p1.a*(p2.b*p1.d - p1.b*p2.d);
//     line.x /= det;
//     line.y /= det;
//     line.z /= det;

//     return line;
// }

// int main() {
//     // define two planes

//     Plane p1 = {7, 4, -2, 12};
//     Plane p2 = {3, -3, -1, -8};

//     // find the intersection line of the planes
//     Line line = intersectPlanes(p1, p2);

//     // print the equation of the line
//     cout << "Equation of the intersection line: ";
//     cout << line.x << " + " << line.l << "t, ";
//     cout << line.y << " + " << line.m << "t, ";
//     cout << line.z << " + " << line.n << "t\n";

//     return 0;
// }

#include <iostream>
#include <cmath>

using namespace std;

// Define a structure for a point
struct Point {
    double x, y, z;
};

// Define a structure for a plane
struct Plane {
    double a, b, c, d;
};

// Define a function to find the intersection line
void intersection_line(Plane plane1, Plane plane2) {
    // Calculate the direction vector of the intersection line
    double dir_x = plane1.b * plane2.c - plane1.c * plane2.b;
    double dir_y = plane1.c * plane2.a - plane1.a * plane2.c;
    double dir_z = plane1.a * plane2.b - plane1.b * plane2.a;

    // Calculate a point on the intersection line
    double point_x = 0;
    double point_y = 0;
    double point_z = 0;

    if (abs(dir_x) > 0.0001) {
        point_x = -(plane1.d * plane2.c - plane1.c * plane2.d) / dir_x;
    }
    else if (abs(dir_y) > 0.0001) {
        point_y = -(plane1.d * plane2.a - plane1.a * plane2.d) / dir_y;
    }
    else if (abs(dir_z) > 0.0001) {
        point_z = -(plane1.d * plane2.b - plane1.b * plane2.d) / dir_z;
    }

    // Print the equation of the intersection line
    cout << "Equation of the intersection line:" << endl;
    cout << "(x - " << point_x << ") / " << dir_x << " = (y - " << point_y << ") / " << dir_y << " = (z - " << point_z << ") / " << dir_z << endl;
}

int main() {
    // Define two planes
    Plane p1 = {7, 4, -2, -12};
    Plane p2 = {3, -3, -1, 8};


    // Find the intersection line
    intersection_line(p1, p2);

    return 0;
}