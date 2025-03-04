#include <iostream>
#include <string>
#include <csignal>

#include "core/Airplane.h"
#include "module/CommunicationHandler.h"

/*
static CommunicationHandler* g_handler = nullptr;

void signalHandler(int signal) {
    std::cout << "Received signal " << signal << std::endl;
    if (g_handler != nullptr) {
        std::cout << "Stopping server..." << std::endl;
        g_handler->stop();
    }
}

int main() {
    // Set up signal handling for graceful shutdown
    g_handler = new CommunicationHandler("http://localhost:8080");

    std::signal(SIGINT, signalHandler);  // Handle Ctrl+C

    std::cout << "Starting server..." << std::endl;
    g_handler->start();

    std::cout << "Server running. Press Enter to stop." << std::endl;
    std::string line;
    std::getline(std::cin, line);

    // Clean shutdown
    if (g_handler != nullptr) {
        g_handler->stop();
        delete g_handler;
        g_handler = nullptr;
    }

    std::cout << "Server stopped." << std::endl;
    return 0;
}
*/



int main() {
    Airplane plane("A320");
    plane.pitch = 0.0;
    plane.roll = 0.0;
    plane.yaw = 0.0;
    plane.position = Vector3(0.0, 0.0, 30000.0);
    plane.speed = Vector3(130.0, 0.0, 0.0);
    plane.calculateVelocity();
    std::cout << "Initial position: "
              << "x = " << plane.position.x << ", "
              << "y = " << plane.position.y << ", "
              << "z = " << plane.position.z << std::endl;

    for (int i = 0; i < 50; ++i) {
        plane.step();
        std::cout << "Step " << i+1 << " position: "
                  << "x = " << plane.position.x << ", "
                  << "y = " << plane.position.y << ", "
                  << "z = " << plane.position.z << std::endl;
    }

    return 0;
}
