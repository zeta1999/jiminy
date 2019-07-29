///////////////////////////////////////////////////////////////////////////////
/// \brief             Python exposition functions for ExoSimulator project.
///
/// \copyright         Wandercraft
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_SIMULATOR_PYTHON_H
#define WDC_SIMULATOR_PYTHON_H

#include <boost/weak_ptr.hpp>

#include <boost/python.hpp>
#include <boost/python/def.hpp>
#include <boost/python/dict.hpp>

#include "jiminy/wdc/ExoModel.h"
#include "jiminy/wdc/ExoController.h"
#include "jiminy/python/Jiminy.h"


namespace jiminy
{
namespace wdc
{
namespace python
{
    using namespace jiminy::python;
    namespace bp = boost::python;

    // ***************************** PyExoModelVisitor ***********************************

    // Overloading must be resolved manually
    result_t (ExoModel::*ExoModelInit)(std::string const &) = &ExoModel::initialize;

    struct PyExoModelVisitor
        : public bp::def_visitor<PyExoModelVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass& cl) const
        {
            cl
                .def("initialize", ExoModelInit,
                                   (bp::arg("self"), "urdf_path"))
                .def("get_urdf_path", &PyModelVisitor::getUrdfPath,
                                      bp::return_value_policy<bp::return_by_value>())
                .def("get_pinocchio_model", &PyModelVisitor::getPinocchioModel,
                                            bp::return_value_policy<bp::return_by_value>())
                .def("get_motors_names", &PyModelVisitor::getMotorsNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_joints_names", &PyModelVisitor::getJointsNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_frames_names", &PyModelVisitor::getFramesNames,
                                         bp::return_value_policy<bp::return_by_value>())
                .def("get_model_options", &PyExoModelVisitor::getModelOptions,
                                          bp::return_value_policy<bp::return_by_value>())
                .def("set_model_options", &PyExoModelVisitor::setModelOptions)
                .def("get_sensors_options", &PyModelVisitor::getSensorsOptions,
                                            bp::return_value_policy<bp::return_by_value>())
                .def("set_sensors_options", &PyModelVisitor::setSensorsOptions)
                ;
        }

        static bp::dict getModelOptions(ExoModel & self)
        {
            // It is necessary to call it manually to resolve shadowing

            bp::dict configPy;
            convertConfigHolderPy(self.getOptions(), configPy);
            return configPy;
        }

        static void setModelOptions(ExoModel       & self,
                                    bp::dict const & configPy)
        {
            // It is necessary to call it manually to resolve shadowing

            configHolder_t config = self.getOptions();
            loadConfigHolder(configPy, config);
            self.setOptions(config);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<ExoModel, bp::bases<Model>,
                       boost::shared_ptr<ExoModel>,
                       boost::noncopyable>("exo_model")
                .def(PyExoModelVisitor());
        }
    };


    // ***************************** PyExoControllerVisitor ***********************************

    struct PyExoControllerVisitor
        : public bp::def_visitor<PyExoControllerVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass& cl) const
        {
            cl
                .def("initialize", &PyExoControllerVisitor::initialize,
                                   (bp::arg("self"), "model", "controller_handle"))
                .def("register_entry", &PyAbstractControllerVisitor::registerNewEntry,
                                       (bp::arg("self"), "fieldname", "value"))
                .def("register_entry", &PyAbstractControllerVisitor::registerNewVectorEntry)
                .def("remove_entries", &PyAbstractControllerVisitor::removeEntries)
                .def("get_controller_options", &PyAbstractControllerVisitor::getControllerOptions,
                                               bp::return_value_policy<bp::return_by_value>())
                .def("set_controller_options", &PyAbstractControllerVisitor::setControllerOptions)
                ;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief      Initialize the model
        ///////////////////////////////////////////////////////////////////////////////
        static void initialize(ExoController       & self,
                               Model               & model,
                               bp::object    const & commandPy)
        {
            ControllerFctWrapperN<3> commandFct(commandPy);
            self.initialize(model, std::move(commandFct));
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<ExoController, bp::bases<AbstractController>,
                       boost::shared_ptr<ExoController>,
                       boost::noncopyable>("exo_controller")
                .def(PyExoControllerVisitor());
        }
    };
}  // End of namespace python.
}  // End of namespace wdc.
}  // End of namespace jiminy.

#endif  // WDC_SIMULATOR_PYTHON_H
