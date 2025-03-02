#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include "core/Airplane.h"

// Simple struct to record state for analysis/visualization
struct StateRecord {
    double time;
    double x, y, z;
    double vx, vy, vz;
    double roll, pitch, yaw;
    double angular_velocity_x, angular_velocity_y, angular_velocity_z;
};

// Free fall airplane simulation with initial horizontal velocity
int main() {
    // Create output file for recording simulation data
    std::ofstream dataFile("airplane_free_fall.csv");
    dataFile << "Time,X,Y,Z,VX,VY,VZ,Speed,Roll,Pitch,Yaw,AngVelX,AngVelY,AngVelZ\n";
    
    // Create a custom airplane for our simulation
    Airplane plane("default_config");
    
    // Class extension to access private members for simulation
    // In a real application, you would add proper getters/setters
    class SimulationPlane : public Airplane {
    public:
        SimulationPlane(const std::string& config) : Airplane(config) {}
        
        // Initialize for free fall with initial velocity
        void initializeFreeFall(double initialAltitude, double initialVelocityX) {
            // Access private members using this technique for demo purposes
            // A better design would expose these properly
            dt = 0.01; // Smaller time step for accuracy
            mass = 5000.0; // 5000 kg aircraft
            
            // Initial position (origin with altitude)
            dx = 0.0;
            dy = initialAltitude;
            dz = 0.0;
            
            // Initial velocity (horizontal only)
            vx = initialVelocityX;
            vy = 0.0;
            vz = 0.0;
            velocity = initialVelocityX;
            
            // Wings and aerodynamic properties
            w_a = 30.0;   // 30 m² wing area
            w_s = 15.0;   // 15 m wingspan
            a_r = w_s * w_s / w_a;  // Aspect ratio
            
            // Initial orientation (level)
            roll = 0.0;
            pitch = 0.0;
            yaw = 0.0;
            quaternion[0] = 1.0; // w
            quaternion[1] = 0.0; // x
            quaternion[2] = 0.0; // y
            quaternion[3] = 0.0; // z
            
            // Initial angular velocity (zero)
            angular_velocity.x = 0.0;
            angular_velocity.y = 0.0;
            angular_velocity.z = 0.0;
            
            // Aerodynamic coefficients
            c_d_o = 0.025; // Zero-lift drag coefficient
            c_l_o = 0.3;   // Zero-alpha lift coefficient
            e = 0.8;       // Efficiency factor
            
            // Engine is off for free fall
            trst = 0.0;
            
            // Initialize inertia matrix
            initializeInertiaMatrix();
            
            // Calculate initial air density at altitude
            calculateAirDensity();
        }
        
        // Get current state for recording
        StateRecord getState(double time) {
            StateRecord state;
            state.time = time;
            state.x = dx;
            state.y = dy;
            state.z = dz;
            state.vx = vx;
            state.vy = vy;
            state.vz = vz;
            state.roll = roll;
            state.pitch = pitch;
            state.yaw = yaw;
            state.angular_velocity_x = angular_velocity.x;
            state.angular_velocity_y = angular_velocity.y;
            state.angular_velocity_z = angular_velocity.z;
            return state;
        }
        
        // Randomly perturb the aircraft to demonstrate rigid body dynamics
        void randomPerturb(double time) {
            // At 2 seconds, apply a small roll perturbation
            if (std::abs(time - 2.0) < 0.01) {
                angular_velocity.x += 0.2;  // Small roll rate
                std::cout << "Applied roll perturbation at t = " << time << "s\n";
            }
            
            // At 4 seconds, apply elevator deflection
            if (std::abs(time - 4.0) < 0.01) {
                elevator_deflection = 0.3;  // Pull up slightly
                std::cout << "Applied elevator deflection at t = " << time << "s\n";
            }
            
            // At 6 seconds, apply aileron deflection
            if (std::abs(time - 6.0) < 0.01) {
                aileron_deflection = 0.5;  // Roll right
                std::cout << "Applied aileron deflection at t = " << time << "s\n";
            }
        }
        
        // Ground collision detection
        bool isGroundCollision() {
            return dy <= 0.1;  // 10cm above ground level
        }
    };
    
    // Create the simulation plane
    SimulationPlane simPlane("default_config");
    
    // Initialize with 1000m altitude and 50 m/s forward speed
    simPlane.initializeFreeFall(1000.0, 50.0);
    
    // Simulation parameters
    const double maxTime = 60.0;  // 60 seconds max simulation time
    const double dt = 0.01;       // 10ms time step
    double currentTime = 0.0;
    
    std::cout << "Starting airplane free fall simulation...\n";
    
    // Main simulation loop
    while (currentTime < maxTime) {
        // Apply random perturbations at specific times
        simPlane.randomPerturb(currentTime);
        
        // Perform rigid body physics step
        simPlane.rigidBodyStep();
        
        // Get and record current state
        StateRecord state = simPlane.getState(currentTime);
        
        // Write data to CSV
        dataFile << state.time << ","
                 << state.x << "," << state.y << "," << state.z << ","
                 << state.vx << "," << state.vy << "," << state.vz << ","
                 << std::sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz) << ","
                 << state.roll << "," << state.pitch << "," << state.yaw << ","
                 << state.angular_velocity_x << "," << state.angular_velocity_y << "," << state.angular_velocity_z
                 << "\n";
        
        // Print progress every second
        if (std::fmod(currentTime, 1.0) < dt) {
            std::cout << "Simulation time: " << currentTime 
                      << "s, Altitude: " << state.y 
                      << "m, Speed: " << std::sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz)
                      << " m/s\n";
        }
        
        // Check for ground collision
        if (simPlane.isGroundCollision()) {
            std::cout << "Ground collision detected at time " << currentTime << "s\n";
            break;
        }
        
        // Increment time
        currentTime += dt;
    }
    
    dataFile.close();
    std::cout << "Simulation completed. Data saved to 'airplane_free_fall.csv'\n";
    std::cout << "You can visualize this data using plotting tools like Python with matplotlib.\n";
    
    // Optional: Add a simple visualization script
    std::ofstream pythonScript("visualize_airplane.py");
    pythonScript << R"(
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the data
df = pd.read_csv('airplane_free_fall.csv')

# Create figure with subplots
fig = plt.figure(figsize=(15, 10))

# 3D trajectory
ax1 = fig.add_subplot(221, projection='3d')
ax1.plot(df['X'], df['Z'], df['Y'])
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Z (m)')
ax1.set_zlabel('Y (m) - Altitude')
ax1.set_title('Airplane Trajectory')

# Altitude vs Time
ax2 = fig.add_subplot(222)
ax2.plot(df['Time'], df['Y'])
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')
ax2.set_title('Altitude vs Time')
ax2.grid(True)

# Speed vs Time
ax3 = fig.add_subplot(223)
ax3.plot(df['Time'], df['Speed'])
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Speed (m/s)')
ax3.set_title('Speed vs Time')
ax3.grid(True)

# Attitude (Roll, Pitch, Yaw)
ax4 = fig.add_subplot(224)
ax4.plot(df['Time'], df['Roll'], label='Roll')
ax4.plot(df['Time'], df['Pitch'], label='Pitch')
ax4.plot(df['Time'], df['Yaw'], label='Yaw')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Attitude (rad)')
ax4.set_title('Aircraft Attitude')
ax4.legend()
ax4.grid(True)

plt.tight_layout()
plt.savefig('airplane_free_fall.png')
plt.show()
)";
    pythonScript.close();
    
    std::cout << "Created visualization script 'visualize_airplane.py'.\n";
    std::cout << "To visualize the data, run: python visualize_airplane.py\n";
    
    return 0;
}