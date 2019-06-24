#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/AbstractSensor.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"

namespace exo_simu
{
    ExoController::ExoController(void) :
    AbstractController(),
    commandFct_([](float64_t const & t,
                   vectorN_t const & q,
                   vectorN_t const & v,
                   matrixN_t const & optoforces,
                   matrixN_t const & IMUs,
                   vectorN_t       & u)->void{ u = vectorN_t::Zero(12); })
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

        // TODO: Check that the command function is working as expected
        commandFct_ = commandFct;
        
        isInitialized_ = true;

        return returnCode;
    }

    void ExoController::compute_command(Engine    const & engine,
                                        float64_t const & t,
                                        vectorN_t const & q,
                                        vectorN_t const & v,
                                        vectorN_t       & u)
    {
        sensorsGroupMap_t const * sensors = engine.getModel().getSensors();
        std::map<std::string, matrixN_t> sensorsData = getSensorsData(sensors);
        commandFct_(t, q, v, sensorsData.at("IMUSensor"), sensorsData.at("ForceSensor"), u);
    }

    void ExoController::internalDynamics(Engine    const & engine,
                                         float64_t const & t,
                                         vectorN_t const & q,
                                         vectorN_t const & v,
                                         vectorN_t       & u)
    {
        ExoModel const & exoModel = static_cast<ExoModel const &>(engine.getModel());
        ExoModel::exoJointOptions_t const & exoJointOptions_ = exoModel.exoMdlOptions_->joints;

        std::vector<int32_t> jointsVelocityIdx = exoModel.getJointsVelocityIdx();
        for (uint32_t i = 0; i < jointsVelocityIdx.size(); i++)
        {
            float64_t jointId = jointsVelocityIdx[i];
            u(jointId) = -exoJointOptions_.frictionViscous(i)*v(jointId) - exoJointOptions_.frictionDry(i) * \
                saturateSoft(v(jointId) / exoJointOptions_.dryFrictionVelEps,-1.0,1.0,0.7);
        }
    }

     std::map<std::string, matrixN_t> ExoController::getSensorsData(sensorsGroupMap_t const * sensors)
     {
        std::map<std::string, matrixN_t> sensorsData;
        for (auto const & sensorGroup : *sensors)
        {
            if (!sensorGroup.second.empty())
            {
                auto sensorIt = sensorGroup.second.begin();
                vectorN_t sensorDataTmp = sensorIt->second->get();
                matrixN_t sensorsGroupData = matrixN_t::Zero(sensorDataTmp.size(), sensorGroup.second.size());
                sensorsGroupData.col(0) = sensorDataTmp;
                sensorIt++;
                for (uint32_t i = 1; i < sensorGroup.second.size() ; i++)
                {
                    sensorsGroupData.col(i) = sensorIt->second->get();
                    sensorIt++;
                }
                sensorsData[sensorGroup.first] = sensorsGroupData;
            }   
        }
        return sensorsData;
     }
}