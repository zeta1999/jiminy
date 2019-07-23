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
        model_(),
        controller_(),
        engine_()
        {
            // Empty.
        }

        ~PyEngine(void)
        {
            // Empty.
        }

        result_t initialize(std::string urdfPath)
        {
            return model_.initialize(urdfPath);
        }

        result_t simulate(vectorN_t     const & x_init,
                          float64_t     const & end_time,
                          commandFct_t          commandFct,
                          callbackFct_t         callbackFct)
        {
            result_t returnCode = result_t::SUCCESS;

            returnCode = controller_.initialize(model_, commandFct);

            if (returnCode == result_t::SUCCESS)
            {
                returnCode = engine_.initialize(model_, controller_, callbackFct);
            }

            if (returnCode == result_t::SUCCESS)
            {
                returnCode = engine_.simulate(x_init, end_time);
            }

            return returnCode;
        }

        static boost::shared_ptr<PyEngine> pyEngineFactory(void)
        {
            if (pyEnginePtr_.use_count())
            {
                boost::shared_ptr<PyEngine> pyEnginePtrStrong = pyEnginePtr_.lock();
                pyEnginePtrStrong->engine_.reset(true);
                pyEnginePtrStrong->controller_.reset(true);
                return pyEnginePtrStrong;
            }
            else
            {
                boost::shared_ptr<PyEngine> strong = boost::shared_ptr<PyEngine>(new PyEngine());
                pyEnginePtr_ = strong;
                return strong;
            }
        }

    public:
        ExoModel model_;
        ExoController controller_;
        Engine engine_;

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
                         vectorN_t       & uCommand)
        {
            // Pass the arguments by reference (be careful const qualifiers are lost)
            bp::handle<> qPy(getNumpyReferenceFromEigenVector(q));
            bp::handle<> vPy(getNumpyReferenceFromEigenVector(v));
            bp::handle<> forceSensorsDataPy(getNumpyReferenceFromEigenMatrix(forceSensorsData));
            bp::handle<> imuSensorsDataPy(getNumpyReferenceFromEigenMatrix(imuSensorsData));
            bp::handle<> encoderSensorsDataPy(getNumpyReferenceFromEigenMatrix(encoderSensorsData));
            bp::handle<> uCommandPy(getNumpyReferenceFromEigenVector(uCommand));
            funcPyPtr_(t, qPy, vPy, forceSensorsDataPy, imuSensorsDataPy, encoderSensorsDataPy, uCommandPy);
        }
    private:
        bp::object funcPyPtr_;
    };

    template<typename T>
    PyObject * getNumpyReference(T & data)
    {
        return getNumpyReferenceFromScalar(data);
    }

    template<>
    PyObject * getNumpyReference<vector3_t>(vector3_t & data)
    {
        return getNumpyReferenceFromEigenVector(data);
    }

    template<typename T>
    struct timeStateFctPyWrapper {
    public:
        // Disable the copy of the class
        timeStateFctPyWrapper & operator = (timeStateFctPyWrapper const & other) = delete;

    public:
        timeStateFctPyWrapper(bp::object const& objPy) :
        funcPyPtr_(objPy),
        outPtr_(new T),
        outPyPtr_()
        {
            outPyPtr_ = getNumpyReference(*outPtr_);
        }

        // Copy constructor, same as the normal constructor
        timeStateFctPyWrapper(timeStateFctPyWrapper const & other) :
        funcPyPtr_(other.funcPyPtr_),
        outPtr_(new T),
        outPyPtr_()
        {
            *outPtr_ = *(other.outPtr_);
            outPyPtr_ = getNumpyReference(*outPtr_);
        }

        // Move constructor, takes a rvalue reference &&
        timeStateFctPyWrapper(timeStateFctPyWrapper&& other) :
        funcPyPtr_(other.funcPyPtr_),
        outPtr_(nullptr),
        outPyPtr_(nullptr)
        {
            // Steal the resource from "other"
            outPtr_ = other.outPtr_;
            outPyPtr_ = other.outPyPtr_;

            /* "other" will soon be destroyed and its destructor will
               do nothing because we null out its resource here */
            other.outPtr_ = nullptr;
            other.outPyPtr_ = nullptr;
        }

        // Destructor
        ~timeStateFctPyWrapper()
        {
            delete outPtr_;
        }

        // Move assignment, takes a rvalue reference &&
        timeStateFctPyWrapper& operator=(timeStateFctPyWrapper&& other)
        {
            /* "other" is soon going to be destroyed, so we let it destroy our current resource
               instead and we take "other"'s current resource via swapping */
            std::swap(funcPyPtr_, other.funcPyPtr_);
            std::swap(outPtr_, other.outPtr_);
            std::swap(outPyPtr_, other.outPyPtr_);
            return *this;
        }

        T operator() (float64_t const & t,
                      vectorN_t const & x)
        {
            // Pass the arguments by reference (be careful const qualifiers are lost)
            bp::handle<> xPy(getNumpyReferenceFromEigenVector(x));
            bp::handle<> outPy(bp::borrowed(outPyPtr_));
            funcPyPtr_(t, xPy, outPy);
            return *outPtr_;
        }

    private:
        bp::object funcPyPtr_;
        T * outPtr_;
        PyObject * outPyPtr_; // Its lifetime in managed by boost::python
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
                .def("run", &PyEngineVisitor::simulate,
                            (bp::arg("self"), "x_init", "end_time", "controller_handle"))
                .def("run", &PyEngineVisitor::simulateWithCallback,
                            (bp::arg("self"), "x_init", "end_time", "controller_handle", "callback_handle"))
                .def("get_log", &PyEngineVisitor::getLog,
                                bp::return_value_policy<bp::return_by_value>())
                .def("write_log", &PyEngineVisitor::writeLog,
                                 (bp::arg("self"), "filename", bp::arg("isModeBinary")=false))
                .def("removes_entries", &PyEngineVisitor::removeEntries)
                .def("register_entry", &PyEngineVisitor::registerNewEntry,
                                       (bp::arg("self"), "fieldname", "value"))
                .def("register_entry", &PyEngineVisitor::registerNewVectorEntry)
                .def("register_force_impulse", &PyEngineVisitor::registerForceImpulse,
                                               (bp::arg("self"), "frame_name", "t", "dt", "F"))
                .def("register_force_profile", &PyEngineVisitor::registerForceProfile,
                                               (bp::arg("self"), "frame_name", "force_handle"))
                .def("get_pinocchio_model", &PyEngineVisitor::getPinocchioModel,
                                            bp::return_value_policy<bp::return_by_value>())
                .def("get_urdf_path", &PyEngineVisitor::getUrdfPath,
                                      bp::return_value_policy<bp::return_by_value>())
                .def("get_motors_names", &PyEngineVisitor::getMotorsNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_joints_names", &PyEngineVisitor::getJointsNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_frames_names", &PyEngineVisitor::getFramesNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_model_options", &PyEngineVisitor::getModelOptions,
                                          bp::return_value_policy<bp::return_by_value>())
                .def("set_model_options", &PyEngineVisitor::setModelOptions)
                .def("get_sensors_options", &PyEngineVisitor::getSensorsOptions,
                                            bp::return_value_policy<bp::return_by_value>())
                .def("set_sensors_options", &PyEngineVisitor::setSensorsOptions)
                .def("get_controller_options", &PyEngineVisitor::getControllerOptions,
                                               bp::return_value_policy<bp::return_by_value>())
                .def("set_controller_options", &PyEngineVisitor::setControllerOptions)
                .def("get_engine_options", &PyEngineVisitor::getEngineOptions,
                                           bp::return_value_policy<bp::return_by_value>())
                .def("set_engine_options", &PyEngineVisitor::setEngineOptions)
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

        static void simulateWithCallback(PyEngine         & self,
                                         vectorN_t  const & x_init,
                                         float64_t  const & end_time,
                                         bp::object const & controllerPy,
                                         bp::object const & callbackPy)
        {
            controllerPyWrapper controllerFct(controllerPy);
            timeStateFctPyWrapper<bool> callbackFct(callbackPy);
            self.simulate(x_init, end_time, controllerFct, std::move(callbackFct));
        }

        static void writeLog(PyEngine          & self,
                             std::string const & filename,
                             bool        const & isModeBinary)
        {
            if (isModeBinary)
            {
                self.engine_.writeLogBinary(filename);
            }
            else
            {
                self.engine_.writeLogTxt(filename);
            }
        }

        static void removeEntries(PyEngine & self)
        {
            self.controller_.reset(true);
        }

        static void registerNewEntry(PyEngine          & self,
                                     std::string const & fieldName,
                                     PyObject          * dataPy)
        {
            float64_t const * data = (float64_t *) PyArray_DATA(reinterpret_cast<PyArrayObject *>(dataPy));
            self.controller_.registerNewEntry(fieldName, *data);
        }

        static void registerNewVectorEntry(PyEngine       & self,
                                           bp::list const & fieldNamesPy,
                                           PyObject       * dataPy) // Const qualifier is not supported by PyArray_DATA anyway
        {
            std::vector<std::string> fieldNames = listPyToStdVector<std::string>(fieldNamesPy);
            Eigen::Map<vectorN_t> data((float64_t *) PyArray_DATA(reinterpret_cast<PyArrayObject *>(dataPy)), fieldNames.size());
            self.controller_.registerNewVectorEntry(fieldNames, data);
        }

        static void registerForceImpulse(PyEngine          & self,
                                         std::string const & frameName,
                                         float64_t   const & t,
                                         float64_t   const & dt,
                                         vector3_t   const & F)
        {
            self.engine_.registerForceImpulse(frameName, t, dt, F);
        }

        static void registerForceProfile(PyEngine          & self,
                                         std::string const & frameName,
                                         bp::object  const & forcePy)
        {
            timeStateFctPyWrapper<vector3_t> forceFct(forcePy);
            self.engine_.registerForceProfile(frameName, std::move(forceFct));
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Getters and Setters
        ///////////////////////////////////////////////////////////////////////////////

        static bp::tuple getLog(PyEngine & self)
        {
            std::vector<std::string> header;
            matrixN_t log;
            self.engine_.getLogData(header, log);
            return bp::make_tuple(header, log);
        }

        static std::string getUrdfPath(PyEngine & self)
        {
            return self.model_.getUrdfPath();
        }

        static pinocchio::Model getPinocchioModel(PyEngine & self)
        {
            return self.model_.pncModel_;
        }

        static std::vector<std::string> getMotorsNames(PyEngine & self)
        {
            return self.model_.getMotorsNames();
        }

        static std::vector<std::string> getJointsNames(PyEngine & self)
        {
            return self.model_.getRigidJointsNames();
        }

        static std::vector<std::string> getFramesNames(PyEngine & self)
        {
            pinocchio::container::aligned_vector<pinocchio::Frame> frames =
                self.model_.pncModel_.frames;
            std::vector<std::string> framesNames;
            for (pinocchio::Frame const & frame : frames)
            {
                framesNames.push_back(frame.name);
            }
            return framesNames;
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
            configHolder_t config = self.model_.getOptions();
            loadConfigHolder(configPy, config);
            self.model_.setOptions(config);
        }

        static bp::dict getSensorsOptions(PyEngine & self)
        {
            configHolder_t config;
            bp::dict configPy;
            self.model_.getSensorsOptions(config);
            convertConfigHolderPy(config, configPy);

            return configPy;
        }

        static void setSensorsOptions(PyEngine       & self,
                                      bp::dict const & configPy)
        {
            configHolder_t config;
            self.model_.getSensorsOptions(config);
            loadConfigHolder(configPy, config);
            self.model_.setSensorsOptions(config);
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
            configHolder_t config = self.controller_.getOptions();
            loadConfigHolder(configPy, config);
            self.controller_.setOptions(config);
        }

        static bp::dict getEngineOptions(PyEngine & self)
        {
            bp::dict configPy;
            convertConfigHolderPy(self.engine_.getOptions(), configPy);
            return configPy;
        }

        static void setEngineOptions(PyEngine       & self,
                                     bp::dict const & configPy)
        {
            configHolder_t config = self.engine_.getOptions();
            loadConfigHolder(configPy, config);
            self.engine_.setOptions(config);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            import_array(); // Required to create Py arrays
            bp::to_python_converter<std::vector<std::string>, stdVectorToListPyConverter<std::string> >();
            bp::to_python_converter<std::vector<vectorN_t>, stdVectorToListPyConverter<vectorN_t> >();
            bp::to_python_converter<std::vector<matrixN_t>, stdVectorToListPyConverter<matrixN_t> >();
            bp::class_<PyEngine, boost::shared_ptr<PyEngine>, boost::noncopyable>("simulator").def(PyEngineVisitor());
        }

    };
}  // End of namespace python.
}  // End of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_H
