#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/core/Sensor.h"

namespace exo_simu
{
    // ===================== ImuSensor =========================

    ImuSensor::ImuSensor(std::string              const & name, 
                         std::vector<std::string> const & headerSuffixes) :
    AbstractSensor(name, headerSuffixes),
    imuSensorOptions_(nullptr),
    data_(vectorN_t::Zero(7)),
    dataStrings_(),
    framesIdx_()
    {
        setOptions(getDefaultOptions());
    }

    ImuSensor::~ImuSensor(void)
    {
        // Empty.
    }

    AbstractSensor* ImuSensor::clone(void)
    {
        return new ImuSensor(*this);
    }

    void ImuSensor::initialize(int32_t const & framesIdx)
    {
        framesIdx_ = framesIdx;
        isInitialized_ = true;
    }

    void ImuSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        imuSensorOptions_ = std::shared_ptr<imuSensorOptions_t const>(new imuSensorOptions_t(sensorOptionsHolder_));
    }

    int32_t ImuSensor::getFrameIdx(void) const
    {
        return framesIdx_;
    }

    result_t ImuSensor::set(Engine    const & engine,
                            float64_t const & t,
                            vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t const & a,
                            vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - ImuSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            Model model = engine.getModel();
            Eigen::Matrix4d const tformIMU = model.pncData_.oMf[framesIdx_].toHomogeneousMatrix();
            Eigen::Matrix3d const rotIMU = tformIMU.topLeftCorner<3,3>();
            quaternion_t const quatIMU(rotIMU); // Convert a rotation matrix to a quaternion
            data_.head(4) = quatIMU.coeffs(); // (x,y,z,w)
            pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model.pncModel_,model.pncData_,framesIdx_);
            Eigen::Vector3d omegaIMU = motionIMU.angular();
            data_.tail(3) = omegaIMU;
        }

        return returnCode;
    }

    std::vector<std::string> const & ImuSensor::getDataStrings(void)
    {
        toVectorString(data_, dataStrings_);
        return dataStrings_;
    }

    // ===================== ForceSensor =========================

    ForceSensor::ForceSensor(std::string              const & name, 
                             std::vector<std::string> const & headerSuffixes) :
    AbstractSensor(name, headerSuffixes),
    forceSensorOptions_(nullptr),
    data_(vectorN_t::Zero(3)),
    dataStrings_(),
    framesIdx_()
    {
        setOptions(getDefaultOptions());
    }

    ForceSensor::~ForceSensor(void)
    {
        // Empty.
    }

    AbstractSensor* ForceSensor::clone(void)
    {
        return new ForceSensor(*this);
    }

    void ForceSensor::initialize(int32_t const & framesIdx)
    {
        framesIdx_ = framesIdx;
        isInitialized_ = true;
    }

    void ForceSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        forceSensorOptions_ = std::shared_ptr<forceSensorOptions_t const>(new forceSensorOptions_t(sensorOptionsHolder_));
    }

    int32_t ForceSensor::getFrameIdx(void) const
    {
        return framesIdx_;
    }

    result_t ForceSensor::set(Engine    const & engine,
                              float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t const & a,
                              vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - ForceSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            pinocchio::Force const fFrame(engine.contactDynamics(framesIdx_));
            data_ = fFrame.linear();
        }

        return returnCode;
    }

    std::vector<std::string> const & ForceSensor::getDataStrings(void)
    {
        toVectorString(data_, dataStrings_);
        return dataStrings_;
    }
}