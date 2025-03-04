//
// Created by fdp on 03/03/25.
//

#include "Math.h"

std::map<std::string, AircraftPhysicsInformation> AircraftPhysicsInformation::get_values() {
    std::map <std::string, AircraftPhysicsInformation> filled_map;
    filled_map["A320"] = {
        7300,
        122.6,
        34.10,
        147,
        30,
        40,
        0.783,
        0.23,
        5.7,
        15.0,
        1.8,
        -2.5,
        0.9
    };

    return filled_map;
}


