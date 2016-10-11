#include <iostream>
#include <vector>
#include "MathDefs.h"

int test_poly()
{
    Vector2s x1(1,4), x2(2,1), x3(6,2);
    // x1 << 3,1; x2 << 5,9; x3<<5,3;
    Vector2s dx1(1,4), dx2(3,5), dx3(3,7);
    // dx1 << 4,1; dx2 << 2,6; dx3 << 5,8;
    double r1 = 1;
    double r2 = 2;

    std::vector<double> position_polynomial;
    std::vector<double> alpha_greater_than_zero_polynomial;
    std::vector<double> alpha_less_than_one_polynomial;

    // Add Theme 2 Milestone 2 code here.

    // Your implementation here should fill the polynomials with right coefficients

    {
        double xx32 = (x3-x2).dot(x3-x2);
        double xx21 = (x2-x1).dot(x2-x1);
        double xd21 = (dx2-dx1).dot(x2-x1);
        double dd21 = (dx2-dx1).dot(dx2-dx1);
        double xd32 = (x3-x2).dot(dx3-dx2);
        double dd32 = (dx3-dx2).dot(dx3-dx2);
        double x21x32 = (x2-x1).dot(x3-x2);
        double xd3221 = (dx3-dx2).dot(x2-x1) + (dx2-dx1).dot(x3-x2);
        double dx21dx32 = (dx3-dx2).dot(dx2-dx1);

        // A: (x3t - x2t)^2
        // B: (x2t - x1t)^2
        // C: (x2t - x1t).dot(x3t - x2t)
        // = A*(r1+r2)^2 - A*B + C^2
        double a0=0, a1=0, a2=0, a3=0, a4=0;

        { // C = a + b*t + c*t^2
            double a = x21x32, b = xd3221, c = dx21dx32;
            // C^2 = a*a + (2*a*b)t + (2*a*c+b*b)t^2 + (2*b*c)t^3+ (c*c) t^4
            a0+=a*a; a1+=2*a*b; a2+=(2*a*c+b*b);
            a3+=2*b*c; a4+=c*c;
        }{
            double a = xx32, b = 2*xd32, c = dd32; // A
            double d = xx21, e = 2*xd21, f = dd21; // B
            // A*B = a*d + (b*d+a*e)t + (c*d+b*e+a*f)t^2 + (c*e+b*f)t^3 + (c*f)t^4
            a0-=a*d; a1-=b*d+a*e; a2-=c*d+b*e+a*f; a3-=c*e+b*f; a4-=c*f;
            double rr = (r1+r2)*(r1+r2);
            a0+=a*rr; a1+=b*rr; a2+=c*rr;
        }
        position_polynomial.push_back(a4);
        position_polynomial.push_back(a3);
        position_polynomial.push_back(a2);
        position_polynomial.push_back(a1);
        position_polynomial.push_back(a0);
        std::cout << a4 << ' ' << a3 << ' ' << a2 << ' ' << a1 << ' ' << a0 << std::endl;
    }

    return 0;
}
