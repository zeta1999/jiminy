#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Sensor.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"


namespace exo_simu
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
                   u = vectorN_t::Zero(12);
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
                                       commandFct_t         commandFct)
    {
        result_t returnCode = result_t::SUCCESS;

        commandFct_ = commandFct;
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
                u(jointId) = -exoJointOptions_.frictionViscous(i)*v(jointId) - exoJointOptions_.frictionDry(i) * \
                    saturateSoft(v(jointId) / exoJointOptions_.dryFrictionVelEps,-1.0,1.0,0.7);
            }

            // Add viscous friction to the toes to avoid numerical instabilities
            float64_t const frictionViscous = 1e-4;
            for (int32_t const & jointId : exoModel_->getToesVelocityIdx())
            {
                u(jointId) = -frictionViscous * v(jointId);
            }

            // Compute the flexibilities
            std::vector<int32_t> const & jointPositionId = exoModel_->getFlexibleJointsPositionIdx();
            std::vector<int32_t> const & jointVelocityId = exoModel_->getFlexibleJointsVelocityIdx();
            for (uint32_t i=0; i<jointVelocityId.size(); ++i)
            {
                float64_t theta;
                quaternion_t quat(q.segment<4>(jointPositionId[i]).data()); // Only way to initialize with [x,y,z,w] order
                vectorN_t axis = pinocchio::quaternion::log3(quat, theta);
                u.segment<3>(jointVelocityId[i]) = - exoModel_->exoMdlOptions_->dynamics.flexibleJointsStiffness[i].array() * axis.array()
                    - exoModel_->exoMdlOptions_->dynamics.flexibleJointsDamping[i].array() * v.segment<3>(jointVelocityId[i]).array();
            }
        }

        return returnCode;
    }
}