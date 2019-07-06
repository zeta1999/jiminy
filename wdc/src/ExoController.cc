#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Sensor.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"


namespace exo_simu
{
    ExoController::ExoController(void) :
    AbstractController(),
    commandFct_([](float64_t const & t,
                   vectorN_t const & q,
                   vectorN_t const & v,
                   matrixN_t const & forceSensorsData,
                   matrixN_t const & imuSensorsData,
                   matrixN_t const & encoderSensorsData,
                   vectorN_t       & u) -> void
                {
                   u = vectorN_t::Zero(12);
                })
    {
        // Empty.
    }

    ExoController::~ExoController(void)
    {
        // Empty.
    }

    AbstractController* ExoController::clone(void)
    {
        return new ExoController(*this);
    }

    result_t ExoController::initialize(commandFct_t commandFct)
    {
        result_t returnCode = result_t::SUCCESS;

        /* TODO: Check that the command function is working as expected,
           but to do so it requires a way to provide mock sensor data. */
        commandFct_ = commandFct;
        
        isInitialized_ = true;

        return returnCode;
    }

    void ExoController::compute_command(Model     const & model,
                                        float64_t const & t,
                                        vectorN_t const & q,
                                        vectorN_t const & v,
                                        vectorN_t       & u)
    {
        matrixN_t const & forceSensorsData = model.getSensorsData(ForceSensor::type_);
        matrixN_t const & imuSensorsData = model.getSensorsData(ImuSensor::type_);
        matrixN_t const & encoderSensorsData = model.getSensorsData(EncoderSensor::type_);
        commandFct_(t, q, v, forceSensorsData, imuSensorsData, encoderSensorsData, u);
    }

    void ExoController::internalDynamics(Model     const & model,
                                         float64_t const & t,
                                         vectorN_t const & q,
                                         vectorN_t const & v,
                                         vectorN_t       & u)
    {
        ExoModel const & exoModel = static_cast<ExoModel const &>(model);
        ExoModel::exoJointOptions_t const & exoJointOptions_ = exoModel.exoMdlOptions_->joints;

        std::vector<int32_t> jointsVelocityIdx = exoModel.getJointsVelocityIdx();
        for (uint32_t i = 0; i < jointsVelocityIdx.size(); i++)
        {
            float64_t jointId = jointsVelocityIdx[i];
            u(jointId) = -exoJointOptions_.frictionViscous(i)*v(jointId) - exoJointOptions_.frictionDry(i) * \
                saturateSoft(v(jointId) / exoJointOptions_.dryFrictionVelEps,-1.0,1.0,0.7);
        }
    }
}