///////////////////////////////////////////////////////////////////////////////
/// \brief    Contains types used in the optimal module.
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_OPTIMAL_TYPES_H
#define WDC_OPTIMAL_TYPES_H

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace exo_simu
{
    // wdc types
    typedef bool   bool_t;
    typedef char   char_t;
    typedef float  float32_t;
    typedef double float64_t;

    // Import types.
    typedef float64_t real_t;
    typedef float64_t scalar_t;
    typedef char_t const* const const_cstr_t;

    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrixN_t;
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>              vectorN_t;

    typedef Eigen::Matrix<float64_t, Eigen::Dynamic, Eigen::Dynamic> matrixN_double_t;
    typedef Eigen::Matrix<float64_t, Eigen::Dynamic, 1>              vectorN_double_t;

    // Define math types.
    typedef Eigen::Quaternion<real_t> quaternion_t;
}

#endif  // WDC_OPTIMAL_TYPES_H
