///////////////////////////////////////////////////////////////////////////////
/// \brief    Contains types used in the optimal module.
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_OPTIMAL_TYPES_H
#define WDC_OPTIMAL_TYPES_H

#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/variant.hpp>

namespace exo_simu
{
    class AbstractSensorBase;
    
    // wdc types
    typedef bool   bool_t;
    typedef char   char_t;
    typedef float  float32_t;
    typedef double float64_t;

    // other "standard" types
    typedef char_t const* const const_cstr_t;

    // import types.
    typedef float64_t real_t;
    typedef float64_t scalar_t;

    // math types.
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrixN_t;
    typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>              vectorN_t;
    typedef Eigen::Matrix<scalar_t, 1, Eigen::Dynamic>              rowN_t;

    typedef Eigen::Block<matrixN_t const, Eigen::Dynamic, Eigen::Dynamic> constBlockXpr;
    typedef Eigen::Block<matrixN_t, Eigen::Dynamic, Eigen::Dynamic> blockXpr;

    typedef Eigen::Quaternion<real_t> quaternion_t;

    // exo_simu specific type
    enum class result_t : int32_t
    {
        SUCCESS = 1,
        ERROR_GENERIC = -1,
        ERROR_BAD_INPUT = -2,
        ERROR_INIT_FAILED = -3
    };
    
    typedef boost::make_recursive_variant<bool_t, uint32_t, int32_t, real_t, std::string, vectorN_t, matrixN_t, 
                                          std::map<std::string, boost::recursive_variant_> >::type configField_t;
    typedef std::map<std::string, configField_t> configHolder_t;

    typedef std::map<std::string, std::shared_ptr<AbstractSensorBase> > sensorsHolder_t;
    typedef std::map<std::string, sensorsHolder_t> sensorsGroupHolder_t;
}

#endif  // WDC_OPTIMAL_TYPES_H