///////////////////////////////////////////////////////////////////////////////
/// \brief             Python exposition functions for ExoSimulator.
///
/// \copyright         Wandercraft
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_EXO_SIMULATOR_PYTHON_H
#define WDC_EXO_SIMULATOR_PYTHON_H

#include <boost/python.hpp>
#include <boost/python/def.hpp>
#include <boost/python/dict.hpp>

#include "exo_simu/engine/ExoSimulator.hpp"
#include "exo_simu/python/Utilities.h"

namespace exo_simu
{
namespace python
{
    namespace bp = boost::python;

    struct controllerPyWrapper {
    public:
        controllerPyWrapper(bp::object const& objPy) : funcPyPtr_(objPy) {}
        void operator() (const float64_t & t,
                           const vectorN_t & x,
                           const matrixN_t & optoforces,
                           const matrixN_t & IMUs,
                                 vectorN_t & u)
        {
            u = bp::extract<vectorN_t>(funcPyPtr_(t, x, optoforces, IMUs));
        }
    private:
        bp::object funcPyPtr_;
    };

    struct callbackPyWrapper {
    public:
        callbackPyWrapper(bp::object const& objPy) : funcPyPtr_(objPy) {}
        bool operator() (const float64_t & t, const vectorN_t & x)
        {
            return bp::extract<bool>(funcPyPtr_(t, x));
        }
    private:
        bp::object funcPyPtr_;
    };

    struct ExoSimulatorVisitor
        : public bp::def_visitor<ExoSimulatorVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass& cl) const
        {
            cl
                .def("init", &ExoSimulatorVisitor::init, 
                             (bp::arg("self"), "urdf_path"))
                .def("simulate", &ExoSimulatorVisitor::simulate, 
                                 (bp::arg("self"), "x0", "t0", "tf", "dt", "controller"))
                .def("simulate", &ExoSimulatorVisitor::simulate_with_callback, 
                                 (bp::arg("self"), "x0", "t0", "tf", "dt", "controller", "callback"))
                .def("get_log", &ExoSimulatorVisitor::getLog)
                .def("get_urdf_path", &ExoSimulator::getUrdfPath, 
                                      bp::return_value_policy<bp::return_by_value>())
                .def("set_urdf_path", &ExoSimulator::setUrdfPath)
                .def("get_model_options", &ExoSimulatorVisitor::getModelOptions, 
                                          bp::return_value_policy<bp::return_by_value>())
                .def("set_model_options", &ExoSimulatorVisitor::setModelOptions)
                .def("get_simulation_options", &ExoSimulatorVisitor::getSimulationOptions, 
                                               bp::return_value_policy<bp::return_by_value>())
                .def("set_simulation_options", &ExoSimulatorVisitor::setSimulationOptions)
                ;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Run the simulation
        ///////////////////////////////////////////////////////////////////////////////
        static void init(ExoSimulator& self,
                         std::string const& urdf_path)
        {
            self.init(urdf_path);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Run the simulation
        ///////////////////////////////////////////////////////////////////////////////
        static void simulate(ExoSimulator& self,
                             vectorN_t const& x0,
                             float64_t const& t0,
                             float64_t const& tf,
                             float64_t const& dt,
                             bp::object const& controllerPy)
        {
            controllerPyWrapper controller = controllerPyWrapper(controllerPy);
            self.simulate(x0, t0, tf, dt, controller);
        }

        static void simulate_with_callback(ExoSimulator& self,
                                           vectorN_t const& x0,
                                           float64_t const& t0,
                                           float64_t const& tf,
                                           float64_t const& dt,
                                           bp::object const& controllerPy,
                                           bp::object const& callbackPy)
        {
            controllerPyWrapper controller = controllerPyWrapper(controllerPy);
            callbackPyWrapper callback = callbackPyWrapper(callbackPy);
            self.simulate(x0, t0, tf, dt, controller, callback);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Getters and Setters
        ///////////////////////////////////////////////////////////////////////////////

        static std::vector<std::vector<float64_t> > getLog(ExoSimulator& self)
        {
            return self.log;
        }

        static bp::dict getModelOptions(ExoSimulator& self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.getModelOptions(), configPy);
            return configPy;
        }

        static void setModelOptions(ExoSimulator& self, bp::dict const& configPy)
        {
            ConfigHolder config = self.getDefaultModelOptions();
            loadConfigHolder(configPy, config);
            self.setModelOptions(config);
        }

        static bp::dict getSimulationOptions(ExoSimulator& self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.getSimulationOptions(), configPy);
            return configPy;
        }

        static void setSimulationOptions(ExoSimulator& self, bp::dict const& configPy)
        {
            ConfigHolder config = self.getDefaultSimulationOptions();
            loadConfigHolder(configPy, config);
            self.setSimulationOptions(config);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::to_python_converter<std::vector<float64_t>, VecToList<float64_t> >();
            bp::to_python_converter<std::vector<vectorN_t>, VecToList<vectorN_t> >();
            bp::to_python_converter<std::vector<matrixN_t>, VecToList<matrixN_t> >();
            bp::to_python_converter<std::vector<std::vector<float64_t> >, VecToList<std::vector<float64_t> > >();
            bp::class_<ExoSimulator>("ExoSimulator", "ExoSimulator class").def(ExoSimulatorVisitor());
        }
    };
}  // End of namespace python.
}  // End of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_H
