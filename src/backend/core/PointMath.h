//
// Created by fdp on 21/02/25.
//

#ifndef POINTMATH_H
#define POINTMATH_H

/*
 * Usefull definition needed by all classes using Points
 *
 */

#define EARTH_RADIUS_IN_METERS 6378137
#define KNOTS_TO_METERS_ON_SECONDS 0.514444
#define PI_ON_360 0.0087
#define FL_TO_METERS 30.48
#include <tuple>

/*
 * The point struct
 *
 */

struct Point {
    double lat;     // in decimal form
    double lng;     // in decimal form
    double height;  // in meters
    double speed;   // in m/s
    bool asap;      // request to reach speed and level as soon as possible
};

/*
 * Interface containing all the related methods for the Point struct
 *
 */

class PointMath {
public:
    static double normalizeAngle(double angle);
    static double fastHeversine(const Point &from, const Point &to);
    static double convertDegreesToRadians(const double &degree);
    static double convertRadiansToDegrees(const double &radians);
    static double calculateBearing(const Point &from, const Point &to);
    static bool isTargetBehind(const Point &currentPos, double currentYaw, const Point &targetPos, double refLat);
    /* This function return a new point given the start, the bearing, and the distance */
    static Point calculateNewPoint(const Point &from, const double &bearing, const double &distance);

    /* Usefull mathod to convert and unpack a point in radians lat and radians long */
    static std::tuple<double, double> convertPointToRadians(const Point &point);

};



#endif //POINTMATH_H
