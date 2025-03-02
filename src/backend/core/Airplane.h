//
// Created by fdp on 25/02/25.
//

#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <string>
#include <array>
#include <vector>
#include <cmath>

/*
 * Usefully CONSTANTS
 */

#define G 9.80665

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

struct max_value_for_dt {
    double thrust;
    double pitch;
    double roll;
};

// 3D Vector representation for simplifying calculations
struct Vector3 {
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Vector3 operator-(const Vector3& v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 cross(const Vector3& v) const {
        return {
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        };
    }

    double dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3 normalize() const {
        double mag = magnitude();
        if (mag > 0) {
            return Vector3(x / mag, y / mag, z / mag);
        }
        return *this;
    }
};

class Airplane {
public:
    Airplane() = default;
    explicit Airplane(const std::string &);
    void simpleStep();
    void rigidBodyStep();  // New method for rigid body simulation


protected:
    void calculateGravityForce();
    void calculateDragForce();
    void calculateThrustForce();
    void calculateLiftForce();
    void calculateAirDensity();
    void calculateDragAndLiftCoefficient();
    void calculateInitialDragAndLiftCoefficient();
    void calculateAoa();
    void calculateVelocity();
    void calculateVelocities();
    void calculatePositions();

    // New methods for rigid body physics
    void initializeInertiaMatrix();
    void calculateForceApplication();
    void calculateTorques();
    void updateAngularVelocity();
    void updateOrientation();
    void applyControlSurfaces();

    std::tuple<double, double, double> calculateVxVyVz();
    void getRotationMatrix(double R[3][3]) const;

    // Updated rotation matrix that uses quaternions for better stability
    void getQuaternionRotationMatrix(double R[3][3]) const;

    /*
     * Methods to evaluate cos and sin -> this is to check if it's 0
     */
    static double cos(const double &rad);
    static double sin(const double &rad);

    static double getTemperature(const double &z);
    static double getPressure(const double &z);

protected:
    double dt; // delta time ~ generally 1 sec

    double w_a; // Wing Area: m^2
    double w_s; // Wing Span: m
    double a_r; // Aspect Ratio: is s^2 (span squared) / a (wing surface area)

    double mass; // kg
    double a_o_a; // angle of attack

    double lft, drg, grvt, trst; // N

    double fx, fy, fz; // N
    double dx, dy, dz; // m
    double vx, vy, vz; // m/s
    double velocity;   // m/s

    double c_d_o, c_l_o; // Cl0 and Cd0
    double c_d, c_l; // drag and lift coefficient

    double p; // air density
    double e; // efficient factor
    double yaw, pitch, roll; // radians

    max_value_for_dt max_values;

    // New members for rigid body physics

    // Inertia tensor (3x3 matrix)
    double inertia_tensor[3][3] = {{0}};
    double inverse_inertia_tensor[3][3] = {{0}};

    // Quaternion for rotation (avoids gimbal lock)
    double quaternion[4] = {1, 0, 0, 0}; // w, x, y, z

    // Angular velocity (rad/s)
    Vector3 angular_velocity;

    // Forces and torques
    Vector3 total_force;       // Net force in world coordinates
    Vector3 total_torque;      // Net torque in body coordinates

    // Force application points (relative to center of mass)
    struct ForcePoint {
        Vector3 position;      // Position relative to center of mass
        Vector3 force;         // Force in body coordinates
        Vector3 direction;     // Direction of force
    };

    std::vector<ForcePoint> force_points;

    // Control surfaces
    double aileron_deflection = 0;   // -1 to 1 (left to right)
    double elevator_deflection = 0;  // -1 to 1 (down to up)
    double rudder_deflection = 0;    // -1 to 1 (left to right)
    double flap_deflection = 0;      // 0 to 1 (retracted to extended)

    // Additional aircraft physical properties
    double length;             // Aircraft length (m)
    double height;             // Aircraft height (m)
    double wing_chord;         // Wing chord (m)
    double wing_dihedral;      // Wing dihedral angle (radians)

    // Aerodynamic coefficients
    double c_l_alpha;          // Lift coefficient per alpha (1/rad)
    double c_m_alpha;          // Pitching moment coefficient per alpha (1/rad)
    double c_l_q;              // Lift coefficient per pitch rate
    double c_m_q;              // Pitching moment coefficient per pitch rate
    double c_y_beta;           // Side force coefficient per sideslip
    double c_n_beta;           // Yawing moment coefficient per sideslip
    double c_roll_beta;        // Rolling moment coefficient per sideslip

    // Control effectiveness
    double c_l_delta_a;        // Lift coefficient per aileron
    double c_roll_delta_a;     // Rolling moment coefficient per aileron
    double c_m_delta_e;        // Pitching moment coefficient per elevator
    double c_l_delta_e;        // Lift coefficient per elevator
    double c_n_delta_r;        // Yawing moment coefficient per rudder
    double c_y_delta_r;        // Side force coefficient per rudder
};

#endif //AIRPLANE_H