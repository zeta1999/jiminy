///////////////////////////////////////////////////////////////////////////////
///
/// \brief             Python module implementation for ExoSimulator.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>

// Define the maximum number of sensor types that can accept the 'ControllerFunctor' Python bindings
#define  PYTHON_CONTROLLER_FUNCTOR_MAX_SENSOR_TYPES 6

#include "jiminy/core/Types.h"
#include "jiminy/python/Utilities.h"
#include "jiminy/python/Jiminy.h"


namespace jiminy
{
namespace python
{
    namespace bp = boost::python;

    BOOST_PYTHON_MODULE(libjiminy_pywrap)
    {
        // Requirement to create Py arrays and Eigen variables
        eigenpy::enableEigenPy();
        import_array();

        // Interfaces for result_t enum
        bp::enum_<result_t>("result_t")
        .value("SUCCESS", result_t::SUCCESS)
        .value("ERROR_GENERIC", result_t::ERROR_GENERIC)
        .value("ERROR_BAD_INPUT", result_t::ERROR_BAD_INPUT)
        .value("ERROR_INIT_FAILED", result_t::ERROR_INIT_FAILED);

        // Enable some automatic C++ to Python converters
        bp::to_python_converter<std::vector<std::string>, stdVectorToListPyConverter<std::string> >();
        bp::to_python_converter<std::vector<int32_t>, stdVectorToListPyConverter<int32_t> >();
        bp::to_python_converter<std::vector<vectorN_t>, stdVectorToListPyConverter<vectorN_t> >();
        bp::to_python_converter<std::vector<matrixN_t>, stdVectorToListPyConverter<matrixN_t> >();
        bp::to_python_converter<std::vector<matrixN_t>, stdVectorToListPyConverter<matrixN_t> >();

        // Expose classes
        jiminy::python::PyModelVisitor::expose();
        jiminy::python::PySensorVisitor::expose();
        jiminy::python::PyAbstractControllerVisitor::expose();
        bp::def("controller_functor",
                PyControllerFunctorNFactory,
                (bp::arg("command_handle"), "internal_dynamics_handle", bp::arg("nb_sensor_types")=-1),
                bp::return_value_policy<bp::manage_new_object>());
        jiminy::python::PyEngineVisitor::expose();
    }
}
}
