#ifndef LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_
#define LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <Eigen/Core>
#include <cstdint>

namespace pcl {

struct LivoxPointXyzrtlt {
        PCL_ADD_POINT4D; // This adds the members x, y, z, and _padding.

        float intensity; /**< intensity   */
        std::uint8_t tag;     /**< Livox point tag   */
        std::uint8_t line;    /**< Laser line id     */
        double timestamp; /**< Timestamp         */

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EIGEN_ALIGN16;
};

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointXyzrtlt,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint8_t, tag, tag)
                                  (std::uint8_t, line, line)
                                  (double, timestamp, timestamp)
)

#endif // LIVOX_LASER_SIMULATION_LIVOX_POINT_XYZRTL_H_
