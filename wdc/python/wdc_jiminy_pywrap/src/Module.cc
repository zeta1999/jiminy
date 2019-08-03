///////////////////////////////////////////////////////////////////////////////
///
/// \brief             Python module implementation for ExoSimulator.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

#include "jiminy/wdc/python/Jiminy.h"


#if PY_VERSION_HEX >= 0x03000000
    static void* initNumpyC() {
        import_array();
        return NULL;
    }
#else
    static void initNumpyC() {
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
        // Requirement to handle numpy::ndarray, and create PyArrays<->Eigen automatic converters
        eigenpy::enableEigenPy();
        bp::numpy::initialize();
        initNumpyC();

        jiminy::wdc::python::PyExoModelVisitor::expose();
        jiminy::wdc::python::PyExoControllerVisitor::expose();
    }
}
}
