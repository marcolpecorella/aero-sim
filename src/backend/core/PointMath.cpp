//
// Created by fdp on 21/02/25.
//

#include "PointMath.h"
#include <cmath>

static constexpr double DEG_TO_RAD   = M_PI / 180.0;
static constexpr double DEG_LAT_TO_M   = 111320.0;    // Roughly 1 deg lat = 111.32 km

double PointMath::fastHeversine(const Point &from, const Point &to) {
    const double cLat = std::cos((from.lat + from.lat) * PI_ON_360);
    const double dLat = (to.lat - from.lat) * PI_ON_360;
    const double dLon = (to.lng - from.lng) * PI_ON_360;
    const double f = dLat * dLat + cLat * cLat * dLon * dLon;
    const double c = 2 * std::atan2(std::sqrt(f), std::sqrt(1 - f));
    return EARTH_RADIUS_IN_METERS * c;
}

double PointMath::normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}

double PointMath::convertDegreesToRadians(const double &degree) {
    return degree * M_PI / 180.0; /* radiants */
}

double PointMath::convertRadiansToDegrees(const double &radians) {
    return radians * 180.0 / M_PI; /* degrees */
}

double PointMath::calculateBearing(const Point &from, const Point &to) {
    auto [r_f_lat, r_f_lng] = convertPointToRadians(from);
    auto [r_t_lat, r_t_lng] = convertPointToRadians(to);

    const double d_lng = r_t_lng - r_f_lng;
    const double y = std::sin(d_lng) * std::cos(r_f_lat);
    const double x = std::cos(r_f_lat) * std::sin(r_t_lat) - std::sin(r_f_lat) * std::cos(r_t_lat) * d_lng;

    return atan2(y, x);
}

Point PointMath::calculateNewPoint(const Point &from, const double &bearing, const double &distance) {
    auto [r_f_lat, r_f_lng] = convertPointToRadians(from);
    const double angular_distance = distance/EARTH_RADIUS_IN_METERS;
    const double sin_A = std::sin(angular_distance);
    const double cos_A = std::cos(angular_distance);

    /* Calculate new lat in radians */
    const double new_lat = std::asin((std::sin(r_f_lat) * cos_A)
                                     + (std::cos(r_f_lat) * sin_A * std::cos(bearing)));

    /* Calculate new lng in radians */
    const double new_lng = r_f_lng +
                           std::atan2(std::sin(bearing) * sin_A * std::cos(r_f_lat),
                                      cos_A - std::sin(r_f_lat) * std::sin(new_lat)) ;

    return Point{convertRadiansToDegrees(new_lat), convertRadiansToDegrees(new_lng), from.height};
}

std::tuple<double, double> PointMath::convertPointToRadians(const Point &point) {
    return {convertDegreesToRadians(point.lat), convertDegreesToRadians(point.lng)};
}

bool PointMath::isTargetBehind(const Point &currentPos,
                               double       currentYaw,     // rad
                               const Point &targetPos,
                               double       refLat) {
    double cosLat    = std::cos(refLat * DEG_TO_RAD);
    double degLonToM = (cosLat < 1e-12) ? 1e-12 : cosLat * DEG_LAT_TO_M;

    double cx = (currentPos.lat - refLat) * DEG_LAT_TO_M;
    double cy = (currentPos.lng - 0.0)     * degLonToM; // reference lng=0 for local?
    // Actually, let's do an offset for the target, too:
    double tx = (targetPos.lat - refLat)  * DEG_LAT_TO_M;
    double ty = (targetPos.lng - 0.0)      * degLonToM;

    double dx = tx - cx;
    double dy = ty - cy;

    // heading from aircraft to target
    double headingToTarget = std::atan2(dy, dx);

    // difference from currentYaw
    double headingError = headingToTarget - currentYaw;
    while (headingError >  M_PI) headingError -= 2.0*M_PI;
    while (headingError < -M_PI) headingError += 2.0*M_PI;

    // behind if > 90 deg
    return (std::fabs(headingError) > M_PI_2);
}