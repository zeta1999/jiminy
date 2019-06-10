///////////////////////////////////////////////////////////////////////////////
/// \brief    Contains types used in the optimal module.
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_OPTIMAL_TYPES_H
#define WDC_OPTIMAL_TYPES_H

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "exo_simu/core/Types.h"

namespace exo_simu
{
    // Import types.
    typedef exo_simu::float64_t real_t;
    typedef exo_simu::float64_t scalar_t;

    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrixN_t;
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>              vectorN_t;

    typedef Eigen::Matrix<float64_t, Eigen::Dynamic, Eigen::Dynamic> matrixN_double_t;
    typedef Eigen::Matrix<float64_t, Eigen::Dynamic, 1>              vectorN_double_t;

    // Define math types.
    typedef Eigen::Quaternion<real_t> quaternion_t;
    typedef Eigen::Matrix<real_t, 3, 1> vector3_t;
    typedef Eigen::Matrix<real_t, 3, 3> matrix3_t;
    typedef Eigen::Matrix<real_t, 3, 3> rotation_t;
    typedef Eigen::Matrix<real_t, 4, 1> vector4_t;

    template<typename key_T, typename value_T>
    struct map
    {
        typedef std::map<key_T, value_T> type;
    };

    typedef char_t const* const const_cstr;
    typedef float64_t real_t;
    typedef uint32_t error_t;
    typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> VectorN;
    typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> MatrixN;
    typedef Eigen::Matrix<real_t, 3, 1> Vector3;
    typedef Eigen::Matrix<real_t, 6, 1> Vector6;
    typedef Eigen::Matrix<real_t, 3, 3> Matrix3;
    typedef Eigen::Matrix<real_t, 6, Eigen::Dynamic> Matrix6N;
    typedef Eigen::SparseMatrix<real_t> SparseMatrix;
    typedef Eigen::Triplet<real_t> Triplet;
}

#endif  // WDC_OPTIMAL_TYPES_H
