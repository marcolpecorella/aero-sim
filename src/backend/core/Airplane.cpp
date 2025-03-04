#include "Airplane.h"
#include "Math.h"
#include <algorithm>


Airplane::Airplane(const std::string& config) {
    params = AircraftPhysicsInformation::get_values()[config];
    dt = 1;
    a_r = params.wing_span * params.wing_span / params.wing_area;
    a_o_a = pitch = roll = yaw = 0.0;
    lft = trst = drg = grvt = 0.0;
    velocity = 0.0;
    speed = force = position = Vector3();
    c_l = c_d = 0.0;
    p = 0;
    percent_of_thrust = 0;
    max_values = {0.0,0.0,0.0}; // TODO to be defined -> this is usefull for the autopilot
}


void Airplane::calculateGravityForce() {
    grvt = params.mass * G;
}

void Airplane::calculateDragForce() {
    drg = 0.5 * p * velocity * velocity * params.wing_area * c_d;
}

void Airplane::calculateThrustForce() {
    trst = params.thrust*percent_of_thrust;
}

void Airplane::calculateLiftForce() {
    lft = 0.5 * p * velocity * velocity * params.wing_area * c_l;
}

void Airplane::calculateAirDensity() {
    double T;          // Temperature (Kelvin)
    double pressure;   // Pressure (Pascals)

    if (position.z < 11000) {
        T = 288.15 - 0.0065 * position.z;
        pressure = 101325 * std::pow(T / 288.15, 5.2561);
    } else {
        T = 216.65;
        pressure = 22632.1;
    }

    p = pressure / (287.058 * T);
}


void Airplane::calculateAoa() {
    a_o_a = pitch; // TODO the be defined
}

void Airplane::calculateForce() {
    /*
     * Evaluating thrust like 0 if going forward (pi/2) -> so Sin(thrust) is equal to max thrust on the x axes
     * same for roll pi/2 it's a plane allied to the ground
     *
     */
    const Vector3 gravity = Vector3(0, 0, -grvt);
    const Vector3 lift = Vector3(0, 0, sin(roll)*lft);
    const Vector3 thrust = Vector3(sin(yaw)*trst, cos(yaw)*trst + cos(roll)*lft, 0);
    const Vector3 drag = Vector3(-sin(yaw)*drg, -cos(yaw)*drg, 0);

    force = thrust + gravity + lift + drag;
}

void Airplane::calculateVelocity() {
    velocity = speed.magnitude(); // Constant calculated from the speed on the x, y, z axis
}

void Airplane::calculatePositions() {
    position = position + speed*dt;
}

void Airplane::calcLiftC() {
    c_l = params.cl0 + params.cl_alpha * a_o_a;
    if (a_o_a > params.alpha_stall) {
        const auto stall_excess = (a_o_a - params.alpha_stall) / 10.0;
        c_l = std::max(0.1, params.cl_max - stall_excess);
    }

    if (a_o_a < -params.alpha_stall*0.7) {
        const auto neg_stall_excess = (a_o_a + params.alpha_stall*0.7) / 10.0;
        c_l = std::min(-0.1, -params.cl_max*0.6 + neg_stall_excess);
    }

    if (a_o_a <= params.alpha_stall && a_o_a >= -params.alpha_stall*0.7) {
        c_l = std::min(c_l, params.cl_max);
    }
}

void Airplane::calcDragC() {
    constexpr auto c_d_0 = 0.0;  // TODO to be defined;
    c_d = c_d_0 + std::pow(c_l, 2) / (M_PI*a_r*params.efficient_factor);
}


void Airplane::step() {
    // Calculate aerodynamic forces
    calcLiftC();
    calcDragC();
    calculateAirDensity();
    calculateAoa();
    calculateGravityForce();
    calculateDragForce();
    calculateLiftForce();
    calculateThrustForce();
    calculateForce();

    // Update linear motion
    const double ax = force.x / params.mass;
    const double ay = force.y / params.mass;
    const double az = force.z / params.mass;

    speed.x += ax * dt;
    speed.y += ay * dt;
    speed.z += az * dt;

    // Update positions
    calculatePositions();

    // Update velocity magnitude
    calculateVelocity();
}

void Airplane::getRotationMatrix(double R[3][3]) const {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

double Airplane::cos(const double &rad) {
    return std::cos(rad) < 1e-7 ? 0.0 : std::cos(rad);
}

double Airplane::sin(const double &rad) {
    return std::sin(rad) < 1e-7 ? 0.0 : std::sin(rad);
}
