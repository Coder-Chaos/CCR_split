#include <iostream>
#include <math.h>
#include <iomanip>
constexpr double DEG_TO_RAD_LOCAL = 3.1415926535897932 / 180.0;

void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN)
{
    double lon = longitude;
    double lat = latitude;
    // unit: km
    // variable
    double a = 6378.137;
    double e = 0.0818192;
    double k0 = 0.9996;
    double E0 = 500;
    double N0 = 0;
    //calc zoneNumber
    double zoneNumber = floor(lon/6) + 31;
    //calc lambda0
    double lambda0 = (zoneNumber - 1) * 6 - 180 + 3; //deg
    lambda0 = lambda0 * DEG_TO_RAD_LOCAL; //radian
    //calc phi and lambda (lat and lon)
    double phi = lat * DEG_TO_RAD_LOCAL;
    double lambda = lon * DEG_TO_RAD_LOCAL;

    // Formula START
    double v = 1 / sqrt(1 - pow(e*sin(phi), 2));
    double A = (lambda - lambda0) * cos(phi);
    double T = pow(tan(phi), 2);
    double C = pow(e, 2) / (1 - pow(e, 2)) * pow(cos(phi), 2);
    double s = (1 - pow(e, 2)/4 - 3*pow(e, 4)/64 - 5*pow(e, 6)/256)*phi - (3*pow(e, 2)/8 + 3*pow(e, 4)/32 + 45*pow(e, 6)/1024)*sin(2*phi) + (15*pow(e, 4)/256 + 45*pow(e, 6)/1024)*sin(4*phi) - 35*pow(e, 6)/3072*sin(6*phi);

    UTME = E0 + k0*a*v * (A + (1-T+C)*pow(A, 3)/6 + (5-18*T+T*T)*pow(A, 5)/120);
    UTMN = N0 + k0*a * (s + v*tan(phi) * (pow(A, 2)/2 + (5-T+9*C+4*C*C)*pow(A, 4)/24 + (61-58*T+T*T)*pow(A, 6)/720));

    UTME *= 1000;
    UTMN *= 1000;
}

int main()
{
    std::cout << "Hello world!" << std::endl;
    double lon = 114.034502;
    double lat = 22.362537;
    double x;
    double y;
    LonLat2UTM(lon, lat, x, y);
    std::cout<<setiosflags(std::ios::fixed)<<"x:"<<x<<std::endl;
    std::cout<<setiosflags(std::ios::fixed)<<"y:"<<y<<std::endl;
    return 0;
}

