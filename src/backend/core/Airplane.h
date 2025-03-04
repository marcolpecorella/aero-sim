//
// Created by fdp on 25/02/25.
//

#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <string>
#include <array>
#include <vector>
#include <cmath>

#include "Math.h"

/**
 * ref: https://apps.dtic.mil/sti/tr/pdf/ADA588839.pdf
 */
constexpr double TABLE4[8][4]={
    00000 , -0.0065 , 288.150 , 1.01325000000000E+5,
    11000 , 0.0000 , 216.650 , 2.26320639734629E+4 ,
    20000 , 0.0010 , 216.650 , 5.47488866967777E+3 ,
    32000 , 0.0028 , 228.650 , 8.68018684755228E+2 ,
    47000 , 0.0000 , 270.650 , 1.10906305554966E+2 ,
    51000 , -0.0028 , 270.650 , 6.69388731186873E+1 ,
    71000 , -0.0020 , 214.650 , 3.95642042804073E+0 ,
    84852 , 0.0000 , 186.946 , 3.73383589976215E-1
    };

class Airplane {
public:
    Airplane() = default;
    explicit Airplane(const std::string &);

    void calculateGravityForce();
    void calculateDragForce();
    void calculateThrustForce();
    void calculateLiftForce();

    void calculateAirDensity();

    void calculateAoa();

    void calculateForce();
    void calculateVelocity();
    void calculatePositions();

    void calcLiftC();
    void calcDragC();

    void getRotationMatrix(double R[3][3]) const;


    void step();

    /*
     * Methods to evaluate cos and sin -> this is to check if it's 0
     */
    static double cos(const double &rad);
    static double sin(const double &rad);

    static double getTemperature(const double &z);
    static double getPressure(const double &z);

    double dt; // delta time ~ generally 1 sec
    double a_r; // wing ratio
    double a_o_a; // Angle of Attack in radians
    double lft, drg, grvt, trst; // N
    double velocity; // m/s
    Vector3 force; // Force Vector
    Vector3 position; // Position Vector
    Vector3 speed; // Speed vector
    double c_d, c_l; // Drag Coefficient, Lift Coefficient
    double p; // Air Density
    double yaw, pitch, roll; // radians
    double percent_of_thrust; // normalized value from [0,1] for evaluating max thrust
    max_value_for_dt max_values;
    AircraftPhysicsInformation params;
};

#endif //AIRPLANE_H