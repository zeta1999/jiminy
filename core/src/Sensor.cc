#include <algorithm>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Sensor.h"

namespace exo_simu
{
    // ===================== ImuSensor =========================

    template<>
    std::vector<uint32_t> SensorDataHolder<ImuSensor>::sensorsCopyCounter_ = std::vector<uint32_t>();
    template<>
    std::vector<SensorDataHolder<ImuSensor> *> SensorDataHolder<ImuSensor>::sensorsHolder_ = std::vector<SensorDataHolder<ImuSensor> *>();
    template<>
    matrixN_t SensorDataHolder<ImuSensor>::dataHolder_ = matrixN_t();
    template<>
    uint32_t const SensorDataHolder<ImuSensor>::sizeOf_(7);
    template<>
    uint32_t SensorDataHolder<ImuSensor>::sensorNb_(0);

    ImuSensor::ImuSensor(std::string const & name) :
    AbstractSensor(name),
    SensorDataHolder(name),
    imuSensorOptions_(nullptr),
    frameIdx_()
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

    void ImuSensor::initialize(int32_t const & frameIdx)
    {
        frameIdx_ = frameIdx;
        isInitialized_ = true;
    }

    void ImuSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        imuSensorOptions_ = std::make_shared<imuSensorOptions_t const>(sensorOptionsHolder_);
    }

    int32_t ImuSensor::getFrameIdx(void) const
    {
        return frameIdx_;
    }

    result_t ImuSensor::set(Model     const & model,
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
            Eigen::Matrix4d const tformIMU = model.pncData_.oMf[frameIdx_].toHomogeneousMatrix();
            Eigen::Matrix3d const rotIMU = tformIMU.topLeftCorner<3,3>();
            quaternion_t const quatIMU(rotIMU); // Convert a rotation matrix to a quaternion
            data().head(4) = quatIMU.coeffs(); // (x,y,z,w)
            pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model.pncModel_,model.pncData_,frameIdx_);
            Eigen::Vector3d omegaIMU = motionIMU.angular();
            data().tail(3) = omegaIMU;
        }

        return returnCode;
    }

    // ===================== ForceSensor =========================

    template<>
    std::vector<uint32_t> SensorDataHolder<ForceSensor>::sensorsCopyCounter_ = std::vector<uint32_t>();
    template<>
    std::vector<SensorDataHolder<ForceSensor> *> SensorDataHolder<ForceSensor>::sensorsHolder_ = std::vector<SensorDataHolder<ForceSensor> *>();
    template<>
    matrixN_t SensorDataHolder<ForceSensor>::dataHolder_ = matrixN_t();
    template<>
    uint32_t const SensorDataHolder<ForceSensor>::sizeOf_(3);
    template<>
    uint32_t SensorDataHolder<ForceSensor>::sensorNb_(0);

    ForceSensor::ForceSensor(std::string const & name) :
    AbstractSensor(name),
    SensorDataHolder(name),
    forceSensorOptions_(nullptr),
    frameIdx_()
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

    void ForceSensor::initialize(int32_t const & frameIdx)
    {
        frameIdx_ = frameIdx;
        isInitialized_ = true;
    }

    void ForceSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        forceSensorOptions_ = std::make_shared<forceSensorOptions_t const>(sensorOptionsHolder_);
    }

    int32_t ForceSensor::getFrameIdx(void) const
    {
        return frameIdx_;
    }

    result_t ForceSensor::set(Model     const & model,
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
            std::vector<int32_t> const & contactFramesIdx = model.getContactFramesIdx();
            std::vector<int32_t>::const_iterator it = std::find(contactFramesIdx.begin(), contactFramesIdx.end(), frameIdx_);
            data() = model.contactForces_[*it].linear();
        }

        return returnCode;
    }

    // ===================== EncoderSensor =========================

    template<>
    std::vector<uint32_t> SensorDataHolder<EncoderSensor>::sensorsCopyCounter_ = std::vector<uint32_t>();
    template<>
    std::vector<SensorDataHolder<EncoderSensor> *> SensorDataHolder<EncoderSensor>::sensorsHolder_ = std::vector<SensorDataHolder<EncoderSensor> *>();
    template<>
    matrixN_t SensorDataHolder<EncoderSensor>::dataHolder_ = matrixN_t();
    template<>
    uint32_t const SensorDataHolder<EncoderSensor>::sizeOf_(2);
    template<>
    uint32_t SensorDataHolder<EncoderSensor>::sensorNb_(0);

    EncoderSensor::EncoderSensor(std::string const & name) :
    AbstractSensor(name),
    SensorDataHolder(name),
    encoderSensorOptions_(nullptr),
    jointPositionIdx_(),
    jointVelocityIdx_()
    {
        setOptions(getDefaultOptions());
    }

    EncoderSensor::~EncoderSensor(void)
    {
        // Empty.
    }

    AbstractSensor* EncoderSensor::clone(void)
    {
        return new EncoderSensor(*this);
    }

    void EncoderSensor::initialize(int32_t const & jointPositionIdx,
                                   int32_t const & jointVelocityIdx)
    {
        jointPositionIdx_ = jointPositionIdx;
        jointVelocityIdx_ = jointVelocityIdx;
        isInitialized_ = true;
    }

    void EncoderSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        encoderSensorOptions_ = std::make_shared<encoderSensorOptions_t const>(sensorOptionsHolder_);
    }

    int32_t EncoderSensor::getJointPositionIdx(void) const
    {
        return jointPositionIdx_;
    }

    int32_t EncoderSensor::getJointVelocityIdx(void) const
    {
        return jointVelocityIdx_;
    }

    result_t EncoderSensor::set(Model     const & model,
                                float64_t const & t,
                                vectorN_t const & q,
                                vectorN_t const & v,
                                vectorN_t const & a,
                                vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - EncoderSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            data().head(1) = q.segment<1>(jointPositionIdx_);
            data().tail(1) = v.segment<1>(jointVelocityIdx_);
        }

        return returnCode;
    }
}