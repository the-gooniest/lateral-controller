/*This is a basic tutorial going over some examples of
how to use Eigen library. I'll try my best to be 
thorough enough for this project*/

#include <iostream>     /* includes cin, cout, and some
                        other functions like printf*/
#include "eigen3/Eigen/Dense"  /*this is an external library but
                        doesn't work with angular brackets
                        for some reason. This includes 
                        all matrices/arrays and linear algebra*/
#include <math.h>

using namespace Eigen;  /*typically we use "Eigen::" before
                        using any of the functions such as 
                        MatrixXf but we don't have to now. */
using namespace std;    /* same as before but now with
                        "std::" for cout, cin and others. */

int main()  //This is where we put our code to run stuff
{
    /*I want to preface that there is a difference between
    a matrix and an array which I will go in more detail soon */

    Matrix<float, 2, 2> m;  /*This creates a 2x2 float type matrix called m.
                            When creating matrices/arrays, it is important 
                            to keep the data type the same because we cannot
                            convert from int to float or double. */
    
    Matrix2f m1;    //This creates the same 2x2 float type matrix like previously

    m << 1,2,3,4;   //This is one way to input data into a known sized matrix.

    m1(0,0) = 5;    //This is another way to input data
    m1(0,1) = 6;    //into a matrix with indices
    m1(1,0) = 7;
    m1(1,1) = 8;

    cout << "This is what matrix m looks like: " << endl << m << endl;
    cout << "This is matrix m1: " << endl << m1 << endl;

    /*Hopefully, we just learned about known sized matrices, next up
    are dynamic sized matrices which can be resized any time while
    the ones we learned previously (static) cannot be resized*/

    Matrix<double, Dynamic, 2> m2;  /*This creates a double type matrix that 
                                    can change the amount of rows but 2 columns 
                                    are known so that cannot be changed */
    
    MatrixXf m3;    /*This creates a float type matrix which is dynamic
                    both in rows and columns */
    
    /* NOTE: m2 = m is not possible due to different data type
    i.e double and float. */

    m3 = m; //This is possible due to same data type.

    cout << "This the first m3: " << endl << m3 <<endl;

    m3.resize(3,3); //This resizes m3 to be a 3x3 matrix

    cout << "This is resized m3 before any established elements: " << endl << m3 << endl;

    m3 << 0,1,2,3,4,5,6,7,8;

    cout << "This is resized m3 after established elements: " << endl << m3 << endl;

    // Next step is to learn how to create arrays

    Array<float,2,2> a;     //This creates a 2x2 float type array

    Array22f a1;    //This creates another 2x2 float type array

    ArrayXXf a2;    //A dynamic both in rows and columns array

    a << 1,2,3,4;   //Same as matrix

    a1(0,0) = 5;    //This is another way to input data
    a1(0,1) = 6;    //into an array with indices
    a1(1,0) = 7;
    a1(1,1) = 8;

    cout << "This is array a: " << endl << a << endl;   //outputs the same way
    cout << "This is array a1: " << endl << a1 << endl;

    /*Establishing matrices and arrays are the same, but the difference
    happens with operations */

    Matrix2f bob = m*m1;    //if you want to establish another matrix
    Array22f bill = a*a1;   //or array, you need to establish the
                            //data type (i.e MatrixXi). Make sure 
                            //that everything is the same data type

    cout << "This is matrix multiplication with m and m1:" << endl
    << bob << endl;
    cout << "This is array multiplication with a and a1:" << endl
    << bill << endl;

    /*Notice how array multiplication multiplies element of one array
    to the same element of the 2nd array (i.e a(0,0) * a1(0,0)) 
    You can still do this with matrices with a function called
    cwiseProduct*/

    cout << "This is the matrices multiplied together with cwiseProduct:"
    << endl << m.cwiseProduct(m1) << endl;

    //There are other useful functions we will be using.

    cout << "This is the transpose of matrix m:" << endl
    << m.transpose() << endl;   

    cout << "This is the inverse of matrix m:" << endl
    << m.inverse() << endl;

    /*You can do the exact some functions above for both arrays
    and matrices */
    /*Division and multiplication with a scalar work the same 
    for both arrays and matrices*/

    cout << "This is matrix m multiplied by 10 then divided by 5:" << endl
    << (m*10)/5 << endl;  

    cout << "This is array a multiplied by 100 then divided by 10:" << endl
    << (a*100)/10 << endl;

    /*They differ with with square roots, squares, cubic functions.
    One has to have an array to do so. This is how to convert to an array/matrix
    and also use some neat functions */

    cout << "This is array a and a1 converted to matrices, then multiplied:" << endl
    << a.matrix() * a1.matrix() << endl;  
    //Notice that the result is the same as m * m1

    cout << "This is array m converted to array, then squared:" << endl
    << m.array().square() << endl << "Now square rooted:" << endl << m.array().sqrt()
    << endl << "Now cubed:" << endl << m.array().cube() << endl; 

    //Next is vectors and some more functions

    Vector2f v = m.row(0);  /*creates a 2x1 column vector with elements 
                            of the matrix m's first row */
    RowVector2f v1 = m.col(1);   /*creates a 1x2 row vector with elements
                                of matrix m's 2nd column*/

    cout << "This is vector v: " << endl << v << endl << "This vector v1" 
    << endl << v1 << endl;

    int columns = v.cols(); //retrieves v's number of columns
    int rows = v.rows();    //retrieves v's number of rows
    cout << "Vector v has " << rows << " row(s) and " << columns << " column(s)" << endl;

    return 0;
}