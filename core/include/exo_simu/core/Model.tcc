#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    template<typename TSensor>
    result_t Model::addSensor(std::string              const & sensorName, 
                              std::shared_ptr<TSensor>       & sensor)
    {
        // The sensor name must be unique, even if there types are different
        result_t returnCode = result_t::SUCCESS;

        for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
        {
            sensorsHolder_t::const_iterator it = sensorGroup.second.find(sensorName);
            if (it != sensorGroup.second.end())
            {
                std::cout << "Error - Model::addSensor - Sensor with the same name already exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            std::string const & sensorType = TSensor::type_;
            
            sensorsGroupHolder_t::iterator sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                // Create a new sensor data holder
                sensorsDataHolder_[sensorType] = std::make_shared<SensorDataHolder_t>();
            }

            // Create the sensor and add it to its group
            sensorsGroupHolder_[sensorType][sensorName] = 
                std::shared_ptr<AbstractSensor>(new TSensor(*this, 
                                                            sensorsDataHolder_.at(sensorType), 
                                                            sensorName));

            // Get a pointer to the sensor
            sensor = getSensor<TSensor>(sensorType, sensorName);
        }

        return returnCode;
    }

    template<typename TSensor>
    std::shared_ptr<TSensor> Model::getSensor(std::string const & sensorType,
                                              std::string const & sensorName)
    {
        /* Casting is almost always unecessary since the base class methods are 
           virtuals and therefore the derived implementations are called, even
           using a downcasted pointer to the base class. */

        return std::static_pointer_cast<TSensor>(sensorsGroupHolder_.at(sensorType).at(sensorName));
    }
}