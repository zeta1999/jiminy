#include "jiminy/core/Utilities.h"
#include "jiminy/core/Sensor.h"
#include "jiminy/core/Model.h"
#include "jiminy/wdc/ExoModel.h"
#include "jiminy/wdc/ExoController.h"


namespace jiminy
{
namespace wdc
{
    ExoController::ExoController(void) :
    AbstractController(),
    exoModel_(nullptr),
    commandFct_([](float64_t const & t,
                   vectorN_t const & q,
                   vectorN_t const & v,
                   matrixN_t const & forceSensorsData,
                   matrixN_t const & imuSensorsData,
                   matrixN_t const & encoderSensorsData,
                   vectorN_t       & u) -> void
                {
                   u.setZero(); // Do not check nor enforce the size
                }),
    forceSensorsData_(),
    imuSensorsData_(),
    encoderSensorsData_()
    {
        // Empty.
    }

    ExoController::~ExoController(void)
    {
        // Empty.
    }

    result_t ExoController::initialize(Model        const & model,
                                       commandFct_t       & commandFct)
    {
        result_t returnCode = result_t::SUCCESS;

        commandFct_ = commandFct;
        exoModel_ = static_cast<ExoModel const *>(&model);
        AbstractController::initialize(model);

        return returnCode;
    }

    result_t ExoController::initialize(Model        const &  model,
                                       commandFct_t       && commandFct)
    {
        result_t returnCode = result_t::SUCCESS;

        commandFct_ = std::move(commandFct);
        exoModel_ = static_cast<ExoModel const *>(&model);
        AbstractController::initialize(model);

        return returnCode;
    }

    result_t ExoController::fetchSensors(void)
    {
        result_t returnCode = result_t::SUCCESS;

        returnCode = model_->getSensorsData(ForceSensor::type_, forceSensorsData_);
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = model_->getSensorsData(ImuSensor::type_, imuSensorsData_);
        }
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = model_->getSensorsData(EncoderSensor::type_, encoderSensorsData_);
        }

        return returnCode;
    }

    result_t ExoController::computeCommand(float64_t const & t,
                                           vectorN_t const & q,
                                           vectorN_t const & v,
                                           vectorN_t       & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - ExoController::computeCommand - The model is not initialized." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            returnCode = fetchSensors();
        }
        if (returnCode == result_t::SUCCESS)
        {
            commandFct_(t, q, v, forceSensorsData_, imuSensorsData_, encoderSensorsData_, u);
        }

        return returnCode;
    }

    result_t ExoController::internalDynamics(float64_t const & t,
                                             vectorN_t const & q,
                                             vectorN_t const & v,
                                             vectorN_t       & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - ExoController::internalDynamics - The model is not initialized." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            ExoModel::exoJointOptions_t const & exoJointOptions_ = exoModel_->exoMdlOptions_->joints;

            // Add viscous friction to the joints
            std::vector<int32_t> const & motorsVelocityIdx = exoModel_->getMotorsVelocityIdx();
            for (uint32_t i = 0; i < motorsVelocityIdx.size(); i++)
            {
                float64_t jointId = motorsVelocityIdx[i];
                u(jointId) += -exoJointOptions_.frictionViscous(i) * v(jointId) - exoJointOptions_.frictionDry(i) * \
                    saturateSoft(v(jointId) / exoJointOptions_.dryFrictionVelEps,-1.0,1.0,0.7);
            }

            // Add viscous friction to the toes to avoid numerical instabilities
            float64_t const frictionViscous = 1e-4;
            for (int32_t const & jointId : exoModel_->getToesVelocityIdx())
            {
                u(jointId) += -frictionViscous * v(jointId);
            }
        }

        return returnCode;
    }
}
}