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


namespace jiminy
{
namespace python
{
    namespace bp = boost::python;

    BOOST_PYTHON_MODULE(libwdc_jiminy_pywrap)
    {
        // Requirement to create Py arrays and Eigen variables
        eigenpy::enableEigenPy();
        import_array();

        jiminy::wdc::python::PyExoModelVisitor::expose();
        jiminy::wdc::python::PyExoControllerVisitor::expose();
    }
}
}
