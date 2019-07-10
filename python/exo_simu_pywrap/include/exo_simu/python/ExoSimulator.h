///////////////////////////////////////////////////////////////////////////////
/// \brief             Python exposition functions for ExoSimulator project.
///
/// \copyright         Wandercraft
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_EXO_SIMULATOR_PYTHON_H
#define WDC_EXO_SIMULATOR_PYTHON_H

#include <boost/weak_ptr.hpp>
#include <boost/python.hpp>
#include <boost/python/def.hpp>
#include <boost/python/dict.hpp>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"
#include "exo_simu/python/Utilities.h"


namespace exo_simu
{
namespace python
{
    namespace bp = boost::python;

    typedef std::function<void(float64_t const & /*t*/,
                               vectorN_t const & /*q*/,
                               vectorN_t const & /*v*/,
                               matrixN_t const & /*forceSensorsData*/,
                               matrixN_t const & /*imuSensorsData*/,
                               matrixN_t const & /*encoderSensorsData*/,
                               vectorN_t       & /*u*/)> commandFct_t;

    typedef std::function<bool(float64_t const & /*t*/,
                               vectorN_t const & /*x*/)> callbackFct_t;

    class PyEngine // Composition over inheritance
    {
    public:
        PyEngine(void) :
        urdfPath_(),
        model_(),
        controller_(),
        simulator_()
        {
            // Empty.
        }

        ~PyEngine(void)
        {
            // Empty.
        }

        result_t initialize(std::string urdfPath)
        {
            result_t returnCode = result_t::SUCCESS;

            urdfPath_ = urdfPath;
            returnCode = model_.initialize(urdfPath_);

            return returnCode;
        }

        result_t simulate(vectorN_t     const & x_init,
                          float64_t     const & end_time, 
                          commandFct_t          commandFct,
                          callbackFct_t         callbackFct)
        {
            result_t returnCode = result_t::SUCCESS;

            returnCode = controller_.initialize(commandFct);

            if (returnCode == result_t::SUCCESS)
            {
                returnCode = simulator_.initialize(model_, controller_, callbackFct);
            }

            if (returnCode == result_t::SUCCESS)
            {
                returnCode = simulator_.simulate(x_init, end_time);
            }

            return returnCode;
        }

        std::string const & getUrdfPath(void) const
        {
            return urdfPath_;
        }

        static boost::shared_ptr<PyEngine> pyEngineFactory(void)
        {
            if (pyEnginePtr_.use_count())
            {
                return pyEnginePtr_.lock(); 
            } 
            else 
            {
                boost::shared_ptr<PyEngine> strong = boost::shared_ptr<PyEngine>(new PyEngine());
                pyEnginePtr_ = strong;
                return strong;
            }
        }

    public:
        std::string urdfPath_;
        ExoModel model_;
        ExoController controller_;
        Engine simulator_;

    private:
        static boost::weak_ptr<PyEngine> pyEnginePtr_;
    };

    boost::weak_ptr<PyEngine> PyEngine::pyEnginePtr_ = boost::weak_ptr<PyEngine>();

    struct controllerPyWrapper {
    public:
        controllerPyWrapper(bp::object const& objPy) : funcPyPtr_(objPy) {}
        void operator() (float64_t const & t,
                         vectorN_t const & q,
                         vectorN_t const & v,
                         matrixN_t const & forceSensorsData,
                         matrixN_t const & imuSensorsData,
                         matrixN_t const & encoderSensorsData,
                         vectorN_t       & u)
        {
            u = bp::extract<vectorN_t>(funcPyPtr_(t, q, v, forceSensorsData, imuSensorsData, encoderSensorsData));
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

    struct PyEngineVisitor
        : public bp::def_visitor<PyEngineVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass& cl) const
        {
            cl
                .def("__init__", bp::make_constructor(&PyEngine::pyEngineFactory))
                .def("init", &PyEngineVisitor::init, 
                             (bp::arg("self"), "urdf_path"))
                .def("simulate", &PyEngineVisitor::simulate, 
                                 (bp::arg("self"), "x_init", "end_time", "controller_handle"))
                .def("simulate", &PyEngineVisitor::simulate_with_callback, 
                                 (bp::arg("self"), "x_init", "end_time", "controller_handle", "callback_handle"))
                .def("get_log", &PyEngineVisitor::getLog)
                .def("write_log", &PyEngineVisitor::writeLog, 
                                 (bp::arg("self"), "filename", bp::arg("isModeBinary")=false))
                .def("get_urdf_path", &PyEngine::getUrdfPath, 
                                      bp::return_value_policy<bp::return_by_value>())
                .def("get_model_options", &PyEngineVisitor::getModelOptions, 
                                          bp::return_value_policy<bp::return_by_value>())
                .def("set_model_options", &PyEngineVisitor::setModelOptions)
                .def("get_controller_options", &PyEngineVisitor::getControllerOptions, 
                                               bp::return_value_policy<bp::return_by_value>())
                .def("set_controller_options", &PyEngineVisitor::setControllerOptions)
                .def("get_simulation_options", &PyEngineVisitor::getSimulationOptions, 
                                               bp::return_value_policy<bp::return_by_value>())
                .def("set_simulation_options", &PyEngineVisitor::setSimulationOptions)
                ;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Initialize the model
        ///////////////////////////////////////////////////////////////////////////////
        static void init(PyEngine          & self,
                         std::string const & urdf_path)
        {
            self.initialize(urdf_path);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Run the simulation
        ///////////////////////////////////////////////////////////////////////////////
        static void simulate(PyEngine         & self,
                             vectorN_t  const & x_init,
                             float64_t  const & end_time,
                             bp::object const & controllerPy)
        {
            controllerPyWrapper controllerFct = controllerPyWrapper(controllerPy);
            callbackFct_t callbackFct = [](float64_t const & t, 
                                           vectorN_t const & x) -> bool
            {
                return true;
            };
            self.simulate(x_init, end_time, controllerFct, callbackFct);
        }

        static void simulate_with_callback(PyEngine         & self,
                                           vectorN_t  const & x_init,
                                           float64_t  const & end_time,
                                           bp::object const & controllerPy,
                                           bp::object const & callbackPy)
        {
            controllerPyWrapper controllerFct = controllerPyWrapper(controllerPy);
            callbackPyWrapper callbackFct = callbackPyWrapper(callbackPy);
            self.simulate(x_init, end_time, controllerFct, callbackFct);
        }

        static void writeLog(PyEngine          & self, 
                             std::string const & filename,
                             bool        const & isModeBinary)
        {
            if (isModeBinary)
            {
                self.simulator_.writeLogBinary(filename);
            }
            else
            {
                self.simulator_.writeLogTxt(filename);
            }
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Getters and Setters
        ///////////////////////////////////////////////////////////////////////////////

        static bp::tuple getLog(PyEngine & self)
        {
            std::vector<std::string> header;
            matrixN_t log;
            self.simulator_.getLogData(header, log);
            return bp::make_tuple(header, log);
        }

        static bp::dict getModelOptions(PyEngine & self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.model_.getOptions(), configPy);
            return configPy;
        }

        static void setModelOptions(PyEngine       & self, 
                                    bp::dict const & configPy)
        {
            configHolder_t config = self.model_.getDefaultOptions();
            loadConfigHolder(configPy, config);
            self.model_.setOptions(config);
        }

        static bp::dict getControllerOptions(PyEngine & self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.controller_.getOptions(), configPy);
            return configPy;
        }

        static void setControllerOptions(PyEngine       & self, 
                                         bp::dict const & configPy)
        {
            configHolder_t config = self.controller_.getDefaultOptions();
            loadConfigHolder(configPy, config);
            self.controller_.setOptions(config);
        }

        static bp::dict getSimulationOptions(PyEngine & self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.simulator_.getOptions(), configPy);
            return configPy;
        }

        static void setSimulationOptions(PyEngine       & self, 
                                         bp::dict const & configPy)
        {
            configHolder_t config = self.simulator_.getDefaultOptions();
            loadConfigHolder(configPy, config);
            self.simulator_.setOptions(config);
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
            bp::class_<PyEngine, boost::shared_ptr<PyEngine>, boost::noncopyable>("simulator").def(PyEngineVisitor());
        }
        
    };
}  // End of namespace python.
}  // End of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_H
