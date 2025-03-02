//
// Created by fdp on 21/02/25.
//

#ifndef LINEBUILDER_H
#define LINEBUILDER_H

#include <vector>

#include "PointMath.h"

class LineBuilder {
    public:
    LineBuilder();
    explicit LineBuilder(const std::vector<Point>& points);

    ~LineBuilder();

    /*
     * @Method: testNextPointReachingCapacity
     * @brief: this method test if with the performance given is possible to reach the next point,
     * if so return a tuple with three values:
     * bool reachable -> True if the next point is reachable.
     * double distance -> distance to the next point.
     * Point closest -> if not reachable return the closest point to the target.
     *
     */
    std::tuple<bool, double, Point> testNextPointReachingCapacity();
    private:
    int m_current_index;
    double m_current_heading;
    std::vector<Point> m_points;
};



#endif //LINEBUILDER_H
