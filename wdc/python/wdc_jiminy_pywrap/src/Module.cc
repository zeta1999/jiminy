///////////////////////////////////////////////////////////////////////////////
///
/// \brief             Python module implementation for ExoSimulator.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>

#include "jiminy/wdc/python/Jiminy.h"


#if PY_VERSION_HEX >= 0x03000000
    static void* init_numpy() {
        import_array();
        return NULL;
    }
#else
    static void init_numpy() {
        import_array();
    }
#endif


namespace jiminy
{
namespace python
{
    namespace bp = boost::python;

    BOOST_PYTHON_MODULE(libwdc_jiminy_pywrap)
    {
        // Requirement to create Py arrays and Eigen variables
        eigenpy::enableEigenPy();
        init_numpy();

        jiminy::wdc::python::PyExoModelVisitor::expose();
        jiminy::wdc::python::PyExoControllerVisitor::expose();
    }
}
}
