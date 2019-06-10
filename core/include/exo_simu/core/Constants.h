///////////////////////////////////////////////////////////////////////////////
/// WanderBrain
///
/// \brief    Contains constants used in the code.
///
/// \details  Constants are physical constants and conversion factors,
///           which should not change in the time, contrary to
///           configurations parameters which store constants that
///           depends on our system and may be tweaked.
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_UTIL_CONSTANTS_H
#define WDC_UTIL_CONSTANTS_H

#include <math.h>

#include "exo_simu/core/Types.h"

// TODO: Check that all these constants are still in use

namespace exo_simu
{
    ///////////////////////////////////////////////////////////////
    ///              PARAMETERS FOR STRING LENGTH
    ///////////////////////////////////////////////////////////////
    uint32_t const DEFAULT_TEXTLOG_LINE_LENGTH = 512U;
    uint32_t const TIMESTAMP_LENGTH = 17U;  ///< The exact length of a timestamp in our iso8601 format.
    uint32_t const FILENAME_SUFFIX_LENGTH = 20U;

    ///////////////////////////////////////////////////////////////
    ///              SI FACTOR AND DIVIDERS
    ///////////////////////////////////////////////////////////////
    static float64_t const ONE         =  1.0;   ///<  1 constant
    static float64_t const TWO         =  2.0;   ///<  2 constant
    static float64_t const THREE       =  3.0;   ///<  3 constant
    static float64_t const FOUR        =  4.0;   ///<  4 constant
    static float64_t const FIVE        =  5.0;   ///<  5 constant
    static float64_t const SIX         =  6.0;   ///<  6 constant
    static float64_t const EIGHT       =  8.0;   ///<  8 constant
    static float64_t const TWELVE      = 12.0;   ///< 12 constant
    static float64_t const FOURTEEN    = 14.0;   ///< 14 constant
    static float64_t const SIXTEEN     = 16.0;   ///< 16 constant
    static float64_t const TWENTY      = 20.0;   ///< 20 constant
    static float64_t const THIRTY      = 30.0;   ///< 30 constant
    static float64_t const KILO   = 1000.0;   ///< kilo prefix
    static float64_t const MEGA   = 1000000.0;   ///< Mega prefix
    static int32_t   const DIVIDER_TO_THOUSANDS  = 1000;
    static uint32_t  const UDIVIDER_TO_THOUSANDS = 1000U;
    static float64_t const FDIVIDER_TO_THOUSANDS = KILO;

    ///////////////////////////////////////////////////////////////
    ///              TIME CONSTANTS
    ///////////////////////////////////////////////////////////////
    static uint32_t const MINUTES_TO_SECONDS     = 60;  // Conversion factor from minutes to seconds
    static uint64_t const MILLISECONDS_IN_SECOND = 1000;

    static uint64_t const MICROSECONDS_IN_MILLISECOND = 1000;
    static uint64_t const MICROSECONDS_IN_SECOND      = 1000000;
    static uint64_t const MICROSECONDS_IN_HOUR        = 3600 * MICROSECONDS_IN_SECOND;

    static uint64_t const NANOSECONDS_IN_MICROSECOND = 1000;
    static uint64_t const NANOSECONDS_IN_MILLISECOND = 1000000;
    static uint64_t const NANOSECONDS_IN_SECOND      = 1000000000;

    static float64_t const SECONDS_IN_NANOSECOND  = 0.000000001;
    static float64_t const SECONDS_IN_MICROSECOND = 0.000001;
    static float64_t const SECONDS_IN_MILLISOND   = 0.001;

    ///////////////////////////////////////////////////////////////
    ///              SIZE CONSTANTS
    ///////////////////////////////////////////////////////////////
    static uint32_t const KB_IN_BYTES        = 1024U;                ///< 1 KB in bytes.
    static uint32_t const MB_IN_BYTES        = 1024U * KB_IN_BYTES;  ///< 1 MB in bytes.
    static uint32_t const GB_IN_BYTES        = 1024U * MB_IN_BYTES;  ///< 1 GB in bytes.

    ///////////////////////////////////////////////////////////////
    ///              FUNDAMENTAL PHYSICAL CONSTANTS
    ///////////////////////////////////////////////////////////////
    static int32_t const MILLIVOLTS_IN_VOLT = 1000;
    static float32_t const KELVIN_CONSTANT = 273.15f;
    static float64_t const EARTH_SURFACE_GRAVITY = 9.81;  // In m/s^2.

    ///////////////////////////////////////////////////////////////
    ///              GEOMETRIC CONSTANTS
    ///////////////////////////////////////////////////////////////
    static float64_t const PI     = 3.14159265358979323846;  ///< Pi constant
    static float64_t const TWO_PI = 2.0 * PI;  ///< 2 * Pi constant
    static float64_t const SQRT_2 = 1.41421356237309504880;   ///< sqrt(2) constant
    static float64_t const DEGREE_TO_RAD = PI / 180.0;  ///< Conversion from degrees to rad.
    static float64_t const RAD_TO_DEGREE = 180.0 / PI;  ///< Conversion from rad to degrees.


    ///////////////////////////////////////////////////////////////
    ///              DEFINITION OF AXES ORDER
    ///////////////////////////////////////////////////////////////
    namespace spacecoordinates
    {
        enum Enum
        {
            X,   ///< Index of the X axis
            Y,   ///< Index of the y axis
            Z,   ///< Index of the z axis
            N_COORDINATES  ///< Number of dimensions.
        };
    }  // namespace spacecoordinates


    namespace planecoordinates
    {
        enum Enum
        {
            X,  ///< Index of the x axis
            Y,  ///< Index of the y axis
            N_COORDINATES  ///< Number of dimensions
        };
    }   // namespace planecoordinates
}

#endif  // WDC_UTIL_CONSTANTS_H
