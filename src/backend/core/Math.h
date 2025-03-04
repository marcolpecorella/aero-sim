//
// Created by fdp on 03/03/25.
//

#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <map>
#include <memory>

// CONSTANTS
#define G 9.80665

// STRUCTS


struct max_value_for_dt {
    double thrust;
    double pitch;
    double roll;
};

/*
 * AircraftPhysicsInformation
 * TODO: Implement Documentation
 *
 */
typedef struct AircraftPhysicsInformation{
    double mass;
    double wing_area;
    double wing_span;
    double thrust;
    double pitch;
    double roll;
    double efficient_factor;
    double cl0;
    double cl_alpha;
    double alpha_stall;
    double cl_max;
    double alpha_zero_lift;
    double flap_cl_inc;

    static std::map <std::string, AircraftPhysicsInformation> get_values();
};

/*
 * Vector3
 * TODO: Implement Documentation
 *
 */
struct Vector3 {
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}

    Vector3(const double &x, const double &y, const double &z) : x(x), y(y), z(z) {}


    Vector3 operator+(const Vector3& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Vector3 operator-(const Vector3& v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vector3 operator*(const double &scalar) const {
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
        if (const double mag = magnitude(); mag > 0) {
            return {x / mag, y / mag, z / mag};
        }
        return *this;
    }
};

/*
 * ForcePoint
 * TODO: Implement Documentation
 *
 */
struct ForcePoint {
    Vector3 position;
    Vector3 force;
    Vector3 direction;
};

#endif //MATH_H
