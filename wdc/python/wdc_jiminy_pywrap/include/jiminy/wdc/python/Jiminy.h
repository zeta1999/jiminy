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

                .def("get_model_options", &PyExoModelVisitor::getModelOptions,
                                          bp::return_value_policy<bp::return_by_value>())
                .def("set_model_options", &PyExoModelVisitor::setModelOptions)
                .def("get_sensors_options", &PyModelVisitor::getSensorsOptions,
                                            bp::return_value_policy<bp::return_by_value>())
                .def("set_sensors_options", &PyModelVisitor::setSensorsOptions)
                .def("get_sensor", &PyModelVisitor::getSensor,
                                   (bp::arg("self"), "sensor_type", "sensor_name"),
                                   bp::return_value_policy<bp::reference_existing_object>())

                .add_property("pinocchio_model", &PyModelVisitor::getPinocchioModel)
                .add_property("frames_names", &PyModelVisitor::getFramesNames)

                .add_property("is_initialized", bp::make_function(&Model::getIsInitialized,
                                                bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("has_freeflyer", bp::make_function(&Model::getHasFreeFlyer,
                                               bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("urdf_path", bp::make_function(&Model::getUrdfPath,
                                           bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("motors_names", bp::make_function(&Model::getMotorsNames,
                                              bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joints_names", bp::make_function(&Model::getRigidJointsNames,
                                               bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("contact_frames_idx", bp::make_function(&Model::getContactFramesIdx,
                                                    bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("motors_position_idx", bp::make_function(&Model::getMotorsPositionIdx,
                                                     bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("motors_velocity_idx", bp::make_function(&Model::getMotorsVelocityIdx,
                                                     bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("position_fieldnames", bp::make_function(&Model::getPositionFieldNames,
                                                     bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("velocity_fieldnames", bp::make_function(&Model::getVelocityFieldNames,
                                                     bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("acceleration_fieldnames", bp::make_function(&Model::getAccelerationFieldNames,
                                                         bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("motor_torque_fieldnames", bp::make_function(&Model::getMotorTorqueFieldNames,
                                                         bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("nq", bp::make_function(&Model::nq,
                                    bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("nv", bp::make_function(&Model::nv,
                                    bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("nx", bp::make_function(&Model::nx,
                                    bp::return_value_policy<bp::copy_const_reference>()))
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
                .def("get_options", &PyAbstractControllerVisitor::getOptions,
                                    bp::return_value_policy<bp::return_by_value>())
                .def("set_options", &PyAbstractControllerVisitor::setOptions)
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
