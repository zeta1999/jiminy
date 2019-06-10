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

    struct ExoSimulatorVisitor
        : public bp::def_visitor<ExoSimulatorVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \brief Expose C++ API through the visitor.
        ///
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass& cl) const
        {
            cl
                .def("init", &ExoSimulatorVisitor::init, (bp::arg("self"), "urdf_path"))
                .def("simulate", &ExoSimulatorVisitor::simulate, (bp::arg("self"), "x0", "t0", "tf", "dt", "controller"))
                .add_property("urdf_path", &ExoSimulator::getUrdfPath,  &ExoSimulator::setUrdfPath)
                .add_property("simulation_options", &ExoSimulatorVisitor::getSimulationOptions,  &ExoSimulatorVisitor::setSimulationOptions)
                .add_property("model_options", &ExoSimulatorVisitor::getModelOptions,  &ExoSimulatorVisitor::setModelOptions)
                ;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Run the simulation
        ///////////////////////////////////////////////////////////////////////////////
        static void init(ExoSimulator& self,
                         std::string const& urdf_path)
        {
            //self.simulate(x0, t0, tf, dt, FctHandleProxy(controller));
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Run the simulation
        ///////////////////////////////////////////////////////////////////////////////
        static void simulate(ExoSimulator& self,
                             vectorN_t const& x0,
                             float64_t const& t0,
                             float64_t const& tf,
                             float64_t const& dt,
                             bp::object const& controller)
        {
            //self.simulate(x0, t0, tf, dt, FctHandleProxy(controller));
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Getters and Setters
        ///////////////////////////////////////////////////////////////////////////////
        static void getSimulationOptions(void)
        {

        }

        static void setSimulationOptions(void)
        {
                
        }

        static void getModelOptions(void)
        {

        }

        static void setModelOptions(void)
        {
                
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::to_python_converter<std::vector<int32_t>, VecToList<int32_t> >();
            bp::to_python_converter<std::vector<vectorN_t>, VecToList<vectorN_t> >();
            bp::to_python_converter<std::vector<matrixN_t>, VecToList<matrixN_t> >();
            bp::to_python_converter<std::vector<std::vector<matrixN_t> >, VecToList<std::vector<matrixN_t> > >();
            bp::class_<ExoSimulator>("ExoSimulator", "ExoSimulator class")
                .def(ExoSimulatorVisitor());
        }
    };
}  // End of namespace python.
}  // End of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_H
