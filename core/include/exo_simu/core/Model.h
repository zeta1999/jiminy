#ifndef SIMU_MODEL_H
#define SIMU_MODEL_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/circular_buffer.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    std::string const JOINT_PREFIX_BASE("current");
    std::string const FREE_FLYER_PREFIX_BASE_NAME(JOINT_PREFIX_BASE + "FreeFlyer");

    class Engine;
    class AbstractSensorBase;
    class TelemetryData;
    struct SensorDataHolder_t;

    class Model
    {
        friend Engine;

    public:
        // Disable the copy of the class
        Model(Model const & model) = delete;
        Model & operator = (Model const & other) = delete;

    public:
        virtual configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundsFromUrdf"] = true; // Must be true since boundsMin and boundsMax are undefined
            config["boundsMin"] = vectorN_t();
            config["boundsMax"] = vectorN_t();

            return config;
        };

        struct jointOptions_t
        {
            bool      const boundsFromUrdf;
            vectorN_t const boundsMin;
            vectorN_t const boundsMax;

            jointOptions_t(configHolder_t const & options) :
            boundsFromUrdf(boost::get<bool>(options.at("boundsFromUrdf"))),
            boundsMin(boost::get<vectorN_t>(options.at("boundsMin"))),
            boundsMax(boost::get<vectorN_t>(options.at("boundsMax")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultDynamicsOptions()
        {
            // Add extra options or update default values
            configHolder_t config;
            config["inertiaBodiesBiasStd"] = 0.0;
            config["massBodiesBiasStd"] = 0.0;
            config["centerOfMassPositionBodiesBiasStd"] = 0.0;
            config["relativePositionBodiesBiasStd"] = 0.0;
            config["isFlexibleModel"] = true;
            config["flexibleJointNames"] = std::vector<std::string>();

            return config;
        };

        struct dynamicsOptions_t
        {
            float64_t                const inertiaBodiesBiasStd;
            float64_t                const massBodiesBiasStd;
            float64_t                const centerOfMassPositionBodiesBiasStd;
            float64_t                const relativePositionBodiesBiasStd;
            bool                     const isFlexibleModel;
            std::vector<std::string> const flexibleJointNames;

            dynamicsOptions_t(configHolder_t const & options) :
            inertiaBodiesBiasStd(boost::get<float64_t>(options.at("inertiaBodiesBiasStd"))),
            massBodiesBiasStd(boost::get<float64_t>(options.at("massBodiesBiasStd"))),
            centerOfMassPositionBodiesBiasStd(boost::get<float64_t>(options.at("centerOfMassPositionBodiesBiasStd"))),
            relativePositionBodiesBiasStd(boost::get<float64_t>(options.at("relativePositionBodiesBiasStd"))),
            isFlexibleModel(boost::get<bool>(options.at("isFlexibleModel"))),
            flexibleJointNames(boost::get<std::vector<std::string> >(options.at("flexibleJointNames")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["dynamics"] = getDefaultDynamicsOptions();
            config["joints"] = getDefaultJointOptions();

            return config;
        };

        struct modelOptions_t
        {
            dynamicsOptions_t const dynamics;
            jointOptions_t const joints;

            modelOptions_t(configHolder_t const & options) :
            dynamics(boost::get<configHolder_t>(options.at("dynamics"))),
            joints(boost::get<configHolder_t>(options.at("joints")))
            {
                // Empty.
            }
        };

    public:
        Model(void);
        virtual ~Model(void);

        result_t initialize(std::string              const & urdfPath,
                            std::vector<std::string> const & contactFramesNames,
                            std::vector<std::string> const & motorsNames,
                            bool                     const & hasFreeflyer = true);
        virtual void reset(bool const & resetTelemetry = false);

        template<typename TSensor>
        result_t addSensor(std::string              const & sensorName,
                           std::shared_ptr<TSensor>       & sensor);
        result_t removeSensor(std::string const & sensorType,
                              std::string const & sensorName);
        result_t removeSensors(std::string const & sensorType);

        configHolder_t getOptions(void) const;
        result_t setOptions(configHolder_t mdlOptions); // Make a copy !
        result_t getSensorOptions(std::string    const & sensorType,
                                  std::string    const & sensorName,
                                  configHolder_t       & sensorOptions) const;
        result_t getSensorsOptions(std::string    const & sensorType,
                                   configHolder_t       & sensorsOptions) const;
        result_t getSensorsOptions(configHolder_t & sensorsOptions) const;
        result_t setSensorOptions(std::string    const & sensorType,
                                  std::string    const & sensorName,
                                  configHolder_t const & sensorOptions);
        result_t setSensorsOptions(std::string    const & sensorType,
                                   configHolder_t const & sensorsOptions);
        result_t setSensorsOptions(configHolder_t const & sensorsOptions);
        bool getIsInitialized(void) const;
        bool getIsTelemetryConfigured(void) const;
        std::string getUrdfPath(void) const;
        bool getHasFreeFlyer(void) const;
        std::map<std::string, std::vector<std::string> > getSensorsNames(void) const;
        result_t getSensorsData(std::string const & sensorType,
                                matrixN_t         & data) const;
        result_t getSensorData(std::string const & sensorType,
                               std::string const & sensorName,
                               vectorN_t         & data) const;
        void setSensorsData(float64_t const & t,
                            vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t const & a,
                            vectorN_t const & u);
        void updateTelemetry(void);
        std::vector<int32_t> const & getContactFramesIdx(void) const;
        std::vector<std::string> const & getMotorsName(void) const;
        std::vector<int32_t> const & getMotorsPositionIdx(void) const;
        std::vector<int32_t> const & getMotorsVelocityIdx(void) const;
        std::vector<std::string> const & getPositionFieldNames(void) const;
        std::vector<std::string> const & getVelocityFieldNames(void) const;
        std::vector<std::string> const & getAccelerationFieldNames(void) const;
        std::vector<std::string> const & getMotorTorqueFieldNames(void) const;
        uint32_t nq(void) const; // no get keyword for consistency with pinocchio C++ API
        uint32_t nv(void) const;
        uint32_t nx(void) const;

        virtual result_t configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData);

        template<typename TSensor>
        result_t getSensor(std::string              const & sensorType,
                           std::string              const & sensorName,
                           std::shared_ptr<TSensor>       & sensor);

        result_t loadUrdfModel(std::string const & urdfPath,
                               bool        const & hasFreeflyer);
        result_t getFrameIdx(std::string const & frameName,
                             int32_t           & frameIdx) const;
        result_t getFramesIdx(std::vector<std::string> const & framesNames,
                              std::vector<int32_t>           & framesIdx) const;
        result_t getJointIdx(std::string const & motorName,
                             int32_t           & motorPositionIdx,
                             int32_t           & motorVelocityIdx) const;
        result_t getJointsIdx(std::vector<std::string> const & motorsNames,
                              std::vector<int32_t>           & motorsPositionIdx,
                              std::vector<int32_t>           & motorsVelocityIdx) const;

    public:
        pinocchio::Model pncModel_;
        pinocchio::Data pncData_;
        std::unique_ptr<modelOptions_t const> mdlOptions_;
        pinocchio::container::aligned_vector<pinocchio::Force> contactForces_; // Buffer to store the contact forces

    protected:
        bool isInitialized_;
        bool isTelemetryConfigured_;
        std::string urdfPath_;
        bool hasFreeflyer_;
        configHolder_t mdlOptionsHolder_;

        std::shared_ptr<TelemetryData> telemetryData_;
        sensorsGroupHolder_t sensorsGroupHolder_;

        std::vector<std::string> contactFramesNames_;
        std::vector<std::string> motorsNames_;
        std::vector<int32_t> contactFramesIdx_;  // Indices of the contact frame in the model
        std::vector<int32_t> motorsPositionIdx_; // Indices of the actuated joints in the configuration representation
        std::vector<int32_t> motorsVelocityIdx_; // Indices of the actuated joints in the velocity vector representation
        std::vector<std::string> positionFieldNames_;
        std::vector<std::string> velocityFieldNames_;
        std::vector<std::string> accelerationFieldNames_;
        std::vector<std::string> motorTorqueFieldNames_;

    private:
        std::map<std::string, std::shared_ptr<SensorDataHolder_t> > sensorsDataHolder_;
        uint32_t nq_;
        uint32_t nv_;
        uint32_t nx_;
    };

    struct SensorDataHolder_t
    {
        SensorDataHolder_t(void) :
        time_(),
        data_(),
        counters_(),
        sensors_(),
        num_()
        {
            // Empty.
        };

        ~SensorDataHolder_t(void)
        {
            // Empty.
        };

        boost::circular_buffer_space_optimized<float64_t> time_;
        boost::circular_buffer_space_optimized<matrixN_t> data_;
        std::vector<uint32_t> counters_;
        std::vector<AbstractSensorBase *> sensors_;
        uint32_t num_;
    };
}

#include "exo_simu/core/Model.tcc"

#endif //end of SIMU_MODEL_H
