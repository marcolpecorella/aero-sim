#include <algorithm>
#include "Airplane.h"

// Gravity: simply set the gravitational force magnitude.
void Airplane::calculateGravityForce() {
    // Gravity force magnitude (in Newtons)
    grvt = mass * G;
    // (This value is later rotated into the body frame in calculateForceApplication.)
}

// Drag: computed from the drag equation.
void Airplane::calculateDragForce() {
    // Drag force magnitude: F_drag = 0.5 * ρ * v² * S * C_d
    drg = 0.5 * p * velocity * velocity * w_a * c_d;
}

// Thrust: for simplicity, we use a constant thrust value.
void Airplane::calculateThrustForce() {
    // A constant thrust value (e.g. 15,000 N) is applied.
    // In a more advanced simulation, this might depend on throttle input.
    trst = 15000.0;
}

// Lift: computed from the lift equation.
void Airplane::calculateLiftForce() {
    // Lift force magnitude: F_lift = 0.5 * ρ * v² * S * C_l
    lft = 0.5 * p * velocity * velocity * w_a * c_l;
}

// Air density: using a simplified ISA model (only below 11km is considered here).
void Airplane::calculateAirDensity() {
    double altitude = dz; // assuming dz represents altitude in meters
    double T;          // Temperature (Kelvin)
    double pressure;   // Pressure (Pascals)

    if (altitude < 11000) {
        T = 288.15 - 0.0065 * altitude;
        pressure = 101325 * std::pow(T / 288.15, 5.2561);
    } else {
        // For altitudes above 11 km, a constant value is used for simplicity.
        T = 216.65;
        pressure = 22632.1;
    }

    // Air density: ρ = p / (R_specific * T), with R_specific ≈ 287.058 J/(kg·K)
    p = pressure / (287.058 * T);
}

// Drag and Lift Coefficients: update them based on the angle of attack.
void Airplane::calculateDragAndLiftCoefficient() {
    // Update lift coefficient: C_l = C_l0 + C_l_alpha * (angle of attack)
    c_l = c_l_o + c_l_alpha * a_o_a;

    // If the aspect ratio is not set, calculate it from wing span and area.
    if (a_r <= 0) {
        a_r = (w_s * w_s) / w_a;
    }

    // Update drag coefficient using a parabolic drag polar:
    // C_d = C_d0 + (C_l^2 / (π * AR * e))
    c_d = c_d_o + (c_l * c_l) / (M_PI * a_r * e);
}

// Initialize the drag and lift coefficients to typical baseline values.
void Airplane::calculateInitialDragAndLiftCoefficient() {
    // Baseline coefficients for a typical small aircraft
    c_l_o = 0.2;
    c_d_o = 0.02;

    // Initialize the current coefficients
    c_l = c_l_o;
    c_d = c_d_o;
}

// Angle of Attack (AoA): computed from the velocity components in the body frame.
void Airplane::calculateAoa() {
    // Assuming vx is the forward component and vz is the vertical component,
    // AoA = arctan(vz / vx) (with a safeguard against division by zero)
    if (std::abs(vx) > 0.001) {
        a_o_a = std::atan2(vz, vx);
    } else {
        a_o_a = 0.0;
    }
}

// Velocity magnitude: computed from its Cartesian components.
void Airplane::calculateVelocity() {
    velocity = std::sqrt(vx * vx + vy * vy + vz * vz);
}

// Velocities update: for this simple model, we simply refresh the velocity magnitude.
void Airplane::calculateVelocities() {
    calculateVelocity();
}

// Positions update: integrate the velocity to update positions.
void Airplane::calculatePositions() {
    dx += vx * dt;
    dy += vy * dt;
    dz += vz * dt;
}

void Airplane::rigidBodyStep() {
    // Calculate aerodynamic forces
    calculateAirDensity();
    calculateAoa();
    calculateDragAndLiftCoefficient();
    calculateGravityForce();
    calculateDragForce();
    calculateLiftForce();
    calculateThrustForce();

    // Apply control surfaces
    applyControlSurfaces();

    // Calculate force application and resulting torques
    calculateForceApplication();
    calculateTorques();

    // Update linear motion
    total_force = Vector3(fx, fy, fz);
    double ax = total_force.x / mass;
    double ay = total_force.y / mass;
    double az = total_force.z / mass;

    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    // Update angular motion
    updateAngularVelocity();
    updateOrientation();

    // Update positions
    calculatePositions();

    // Update velocity magnitude
    calculateVelocity();
}

inline void Airplane::initializeInertiaMatrix() {
    // Estimated inertia values based on aircraft dimensions and mass
    // These would typically be calculated from a detailed model or measured

    // Assume the aircraft is roughly ellipsoidal
    double Ixx = mass * (height*height + w_s*w_s) / 12.0;  // Roll inertia
    double Iyy = mass * (length*length + height*height) / 12.0;  // Pitch inertia
    double Izz = mass * (length*length + w_s*w_s) / 12.0;  // Yaw inertia

    // Simplified inertia tensor (assuming symmetry, so off-diagonal terms are 0)
    inertia_tensor[0][0] = Ixx;
    inertia_tensor[1][1] = Iyy;
    inertia_tensor[2][2] = Izz;

    // Calculate inverse inertia tensor (for simpler symmetrical case)
    inverse_inertia_tensor[0][0] = 1.0 / Ixx;
    inverse_inertia_tensor[1][1] = 1.0 / Iyy;
    inverse_inertia_tensor[2][2] = 1.0 / Izz;
}

inline void Airplane::calculateForceApplication() {
    force_points.clear();

    // Define key force application points

    // Main wing lift
    ForcePoint wing_lift;
    wing_lift.position = Vector3(0, 0, 0);  // Typically near center of mass
    wing_lift.force = Vector3(0, lft, 0);   // Lift in Y direction (body frame)
    force_points.push_back(wing_lift);

    // Engine thrust
    ForcePoint engine_thrust;
    engine_thrust.position = Vector3(length * 0.2, 0, 0);  // Engine position
    engine_thrust.force = Vector3(trst, 0, 0);  // Thrust in X direction
    force_points.push_back(engine_thrust);

    // Gravity (applied at center of mass)
    ForcePoint gravity;
    gravity.position = Vector3(0, 0, 0);

    // Rotate gravity into body frame
    double R[3][3];
    getRotationMatrix(R);
    double gx = -R[0][1] * grvt;  // Gravity is in -Y direction in world frame
    double gy = -R[1][1] * grvt;
    double gz = -R[2][1] * grvt;

    gravity.force = Vector3(gx, gy, gz);
    force_points.push_back(gravity);

    // Drag force (applied at center of pressure, slightly behind center of mass)
    ForcePoint drag_force;
    drag_force.position = Vector3(-wing_chord * 0.25, 0, 0);  // Slightly behind CM

    // Drag is opposite to velocity direction
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (speed > 0.001) {
        Vector3 vel_dir(-vx/speed, -vy/speed, -vz/speed);
        // Transform to body coordinates
        Vector3 drag_dir(
            R[0][0] * vel_dir.x + R[0][1] * vel_dir.y + R[0][2] * vel_dir.z,
            R[1][0] * vel_dir.x + R[1][1] * vel_dir.y + R[1][2] * vel_dir.z,
            R[2][0] * vel_dir.x + R[2][1] * vel_dir.y + R[2][2] * vel_dir.z
        );
        drag_force.force = drag_dir * drg;
    } else {
        drag_force.force = Vector3(0, 0, 0);
    }
    force_points.push_back(drag_force);
}

void Airplane::calculateTorques() {
    total_torque = Vector3(0, 0, 0);

    // Calculate torque from each force
    for (const auto& point : force_points) {
        Vector3 torque = point.position.cross(point.force);
        total_torque = total_torque + torque;
    }

    // Add control-induced torques from aerodynamic surfaces
    // These come from aileron, elevator, and rudder deflections

    // Aileron-induced roll torque
    double roll_torque = 0.5 * p * velocity * velocity * w_a * w_s * c_roll_delta_a * aileron_deflection;
    total_torque.x += roll_torque;

    // Elevator-induced pitch torque
    double pitch_torque = 0.5 * p * velocity * velocity * w_a * wing_chord * c_m_delta_e * elevator_deflection;
    total_torque.y += pitch_torque;

    // Rudder-induced yaw torque
    double yaw_torque = 0.5 * p * velocity * velocity * w_a * w_s * c_n_delta_r * rudder_deflection;
    total_torque.z += yaw_torque;
}

void Airplane::updateAngularVelocity() {
    // Apply torque to change angular velocity (I^-1 * (torque - ω × (I * ω)))

    // Calculate I * ω
    double Iw[3] = {
        inertia_tensor[0][0] * angular_velocity.x +
        inertia_tensor[0][1] * angular_velocity.y +
        inertia_tensor[0][2] * angular_velocity.z,

        inertia_tensor[1][0] * angular_velocity.x +
        inertia_tensor[1][1] * angular_velocity.y +
        inertia_tensor[1][2] * angular_velocity.z,

        inertia_tensor[2][0] * angular_velocity.x +
        inertia_tensor[2][1] * angular_velocity.y +
        inertia_tensor[2][2] * angular_velocity.z
    };

    // Calculate ω × (I * ω)
    Vector3 w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Vector3 Iw_vec(Iw[0], Iw[1], Iw[2]);
    Vector3 wxIw = w.cross(Iw_vec);

    // Calculate torque - ω × (I * ω)
    Vector3 effective_torque = total_torque - wxIw;

    // Apply I^-1 to get angular acceleration
    double alpha[3] = {
        inverse_inertia_tensor[0][0] * effective_torque.x +
        inverse_inertia_tensor[0][1] * effective_torque.y +
        inverse_inertia_tensor[0][2] * effective_torque.z,

        inverse_inertia_tensor[1][0] * effective_torque.x +
        inverse_inertia_tensor[1][1] * effective_torque.y +
        inverse_inertia_tensor[1][2] * effective_torque.z,

        inverse_inertia_tensor[2][0] * effective_torque.x +
        inverse_inertia_tensor[2][1] * effective_torque.y +
        inverse_inertia_tensor[2][2] * effective_torque.z
    };

    // Update angular velocity
    angular_velocity.x += alpha[0] * dt;
    angular_velocity.y += alpha[1] * dt;
    angular_velocity.z += alpha[2] * dt;

    // Apply damping to angular velocity to simulate air resistance
    const double damping = 0.98;
    angular_velocity.x *= damping;
    angular_velocity.y *= damping;
    angular_velocity.z *= damping;
}

void Airplane::updateOrientation() {
    // Update orientation using quaternion integration
    // This prevents gimbal lock and provides smoother rotation

    // Convert current angular velocity to quaternion rate
    double q[4] = {quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
    double omega[4] = {
        0,
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z
    };

    // Quaternion derivative: q_dot = 0.5 * q * omega
    // where * represents quaternion multiplication
    double q_dot[4] = {
        -0.5 * (q[1] * omega[1] + q[2] * omega[2] + q[3] * omega[3]),
        0.5 * (q[0] * omega[1] + q[2] * omega[3] - q[3] * omega[2]),
        0.5 * (q[0] * omega[2] + q[3] * omega[1] - q[1] * omega[3]),
        0.5 * (q[0] * omega[3] + q[1] * omega[2] - q[2] * omega[1])
    };

    // Update quaternion: q += q_dot * dt
    quaternion[0] += q_dot[0] * dt;
    quaternion[1] += q_dot[1] * dt;
    quaternion[2] += q_dot[2] * dt;
    quaternion[3] += q_dot[3] * dt;

    // Normalize quaternion to prevent drift
    double q_mag = std::sqrt(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]
    );

    quaternion[0] /= q_mag;
    quaternion[1] /= q_mag;
    quaternion[2] /= q_mag;
    quaternion[3] /= q_mag;

    // Extract Euler angles for compatibility with existing code
    // Roll (x-axis rotation)
    roll = std::atan2(
        2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
        1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
    );

    // Pitch (y-axis rotation)
    double sinp = 2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    yaw = std::atan2(
        2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
        1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])
    );
}

void Airplane::applyControlSurfaces() {
    // Apply control inputs within limits
    aileron_deflection = std::max(-1.0, std::min(1.0, aileron_deflection));
    elevator_deflection = std::max(-1.0, std::min(1.0, elevator_deflection));
    rudder_deflection = std::max(-1.0, std::min(1.0, rudder_deflection));
    flap_deflection = std::max(0.0, std::min(1.0, flap_deflection));

    // Flaps increase lift coefficient and drag
    if (flap_deflection > 0.01) {
        c_l += 0.5 * flap_deflection;  // Additional lift from flaps
        c_d += 0.2 * flap_deflection;  // Additional drag from flaps
    }

    // Elevator affects angle of attack and thus lift distribution
    if (std::abs(elevator_deflection) > 0.01) {
        // Elevator changes effective angle of attack
        a_o_a += 0.2 * elevator_deflection;  // Adjust AoA based on elevator
    }
}

void Airplane::getQuaternionRotationMatrix(double R[3][3]) const {
    // Convert quaternion to rotation matrix
    // This is more stable than Euler angles rotation matrix
    double w = quaternion[0];
    double x = quaternion[1];
    double y = quaternion[2];
    double z = quaternion[3];

    // Calculate rotation matrix elements
    R[0][0] = 1 - 2*y*y - 2*z*z;
    R[0][1] = 2*x*y - 2*w*z;
    R[0][2] = 2*x*z + 2*w*y;

    R[1][0] = 2*x*y + 2*w*z;
    R[1][1] = 1 - 2*x*x - 2*z*z;
    R[1][2] = 2*y*z - 2*w*x;

    R[2][0] = 2*x*z - 2*w*y;
    R[2][1] = 2*y*z + 2*w*x;
    R[2][2] = 1 - 2*x*x - 2*y*y;
}

// Constructor implementation with additional rigid body parameters
Airplane::Airplane(const std::string& config) {
    // Initialize existing parameters

    // Add initialization for rigid body parameters
    length = 10.0;  // Aircraft length in meters
    height = 3.0;   // Aircraft height in meters
    wing_chord = 2.0;  // Wing chord in meters
    wing_dihedral = 0.05;  // Wing dihedral angle in radians

    // Aerodynamic coefficients
    c_l_alpha = 5.0;     // Lift coefficient per alpha
    c_m_alpha = -1.0;    // Pitching moment coefficient per alpha
    c_l_q = 7.0;         // Lift coefficient per pitch rate
    c_m_q = -10.0;       // Pitching moment coefficient per pitch rate
    c_y_beta = -1.0;     // Side force coefficient per sideslip
    c_n_beta = 0.12;     // Yawing moment coefficient per sideslip
    c_roll_beta = -0.1;  // Rolling moment coefficient per sideslip

    // Control effectiveness
    c_l_delta_a = 0.0;     // Lift coefficient per aileron
    c_roll_delta_a = 0.15; // Rolling moment coefficient per aileron
    c_m_delta_e = -1.5;    // Pitching moment coefficient per elevator
    c_l_delta_e = 0.3;     // Lift coefficient per elevator
    c_n_delta_r = 0.1;     // Yawing moment coefficient per rudder
    c_y_delta_r = 0.2;     // Side force coefficient per rudder

    // Initialize quaternion to identity rotation
    quaternion[0] = 1.0;  // w
    quaternion[1] = 0.0;  // x
    quaternion[2] = 0.0;  // y
    quaternion[3] = 0.0;  // z
    // Calculate inertia tensor
    initializeInertiaMatrix();
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
