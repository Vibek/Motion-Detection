#ifndef naorobots_apf_h
#define naorobots_apf_h

/**
 * Defines the artificial potential field (APF) obstacle avoidance algorithm.
 */

#include <list>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

#include "obstacle_avoidance.h"

/** The distance at which to switch from conical to quadratic goal attraction. */
#define D_GOAL 1.0

/** The distance at which to start paying attention to obstacles. */
#define D_OBS 1.0

/** Goal gain (constant factor). */
#define K_GOAL 1.0

/** Obstacle gain (constant factor). */
#define K_OBS 1.0

/**
 * APF implementation of the ObstacleAvoider interface.
 */
class APF : public ObstacleAvoider {
public:

    /**
     * {@inheritDoc}
     */
    virtual Polar nav(sensor_msgs::LaserScan scan);

protected:

    /** The last command given by nav(). */
    Polar cmdPrev;

    /**
     * Converts a laser scan to a list of polar coordinates.
     *
     * @param scan  The laser scan to convert.
     * @returns     The list of polar coords.
     */
    std::list<Polar> scanToList(sensor_msgs::LaserScan scan);

    /**
     * Finds the local minima (by distance) in a list of polar coords.
     * It is assumed the coords are sorted by angle.
     *
     * @param scan  The laser scan to use.
     * @returns     A list of minima in polar coordinates.
     */
    std::list<Polar> findLocalMinima(std::list<Polar> points);

    /**
     * Finds "objects" in a list of polar coords. An object is defined as
     * a series of points whose distance doesn't vary by more than OBJ_DIST
     * from point to point. The returned coord for the object is the minimal
     * point in the object.
     *
     * @param points    The list of points to start with.
     * @returns         The nearest point of each object.
     */
    std::list<Polar> findObjects(std::list<Polar> points);

};

#endif /* naorobots_apf_h */
