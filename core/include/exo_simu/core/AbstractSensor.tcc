
namespace exo_simu
{
    template <typename T>
    AbstractSensorTpl<T>::AbstractSensorTpl(Model                               const & model,
                                            std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                                            std::string                         const & name) :
    AbstractSensorBase(model, name),
    dataHolder_(dataHolder),
    sensorId_(dataHolder_->num_)
    {
        ++dataHolder_->num_;
        dataHolder_->data_.conservativeResize(dataHolder_->num_, sizeOf_);
        data() = vectorN_t::Zero(sizeOf_);
        dataHolder_->sensors_.push_back(this);
        dataHolder_->counters_.push_back(1);
    }

    template <typename T>
    AbstractSensorTpl<T>::~AbstractSensorTpl(void)
    {
        --dataHolder_->counters_[sensorId_];
        if (!dataHolder_->counters_[sensorId_])
        {
            // Remove associated row in the global data buffer
            if(sensorId_ < dataHolder_->num_ - 1)
            {
                dataHolder_->data_.block(sensorId_, 0, dataHolder_->num_ - sensorId_ - 1, sizeOf_) = 
                    dataHolder_->data_.block(sensorId_ + 1, 0, dataHolder_->num_ - sensorId_ - 1, sizeOf_).eval(); // eval to avoid aliasing
            }
            dataHolder_->data_.conservativeResize(dataHolder_->num_ - 1, sizeOf_);

            // Shift the sensor ids
            for (uint32_t i=sensorId_ + 1; i < dataHolder_->num_; i++)
            {
                AbstractSensorTpl<T> * sensor = static_cast<AbstractSensorTpl<T> *>(dataHolder_->sensors_[i]);
                --sensor->sensorId_;
            }

            // Remove the deprecated elements of the global containers
            dataHolder_->sensors_.erase(dataHolder_->sensors_.begin() + sensorId_);
            dataHolder_->counters_.erase(dataHolder_->counters_.begin() + sensorId_);

            // Update the total number of sensors left
            --dataHolder_->num_;
        }
    }

    template <typename T>
    void AbstractSensorTpl<T>::reset(void)
    {
        AbstractSensorBase::reset();
        bias_ = randVectorNormal(sizeOf_, sensorOptions_->biasMean, sensorOptions_->biasStd);
    }

    template <typename T>
    void AbstractSensorTpl<T>::setOptionsAll(configHolder_t const & sensorOptions)
    {
        for (AbstractSensorBase * sensor : dataHolder_->sensors_)
        {
            sensor->setOptions(sensorOptions);
        }
    }
    
    template <typename T>
    std::vector<std::string> const & AbstractSensorTpl<T>::getFieldNames(void) const
    {
        if(sensorOptions_->enablePostProccess)
        {
            return fieldNamesPostProcess_;
        }
        else
        {
            return fieldNamesPreProcess_;
        }
    }

    template <typename T>
    matrixN_t::ConstRowXpr AbstractSensorTpl<T>::get(void) const
    {
        return matrixN_t::ConstRowXpr(dataHolder_->data_.derived(), sensorId_, 0, 1, sizeOf_);
    }

    template <typename T>
    matrixN_t const & AbstractSensorTpl<T>::getAll(void) const
    {
        return dataHolder_->data_;
    }

    template <typename T>
    result_t AbstractSensorTpl<T>::setAll(float64_t const & t,
                                       vectorN_t const & q,
                                       vectorN_t const & v,
                                       vectorN_t const & a,
                                       vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS; 

        for (AbstractSensorBase * sensor : dataHolder_->sensors_)
        {
            if (returnCode == result_t::SUCCESS)
            {
                returnCode = sensor->set(t, q, v, a, u);
            }
        }

        return returnCode;
    }

    template <typename T>
    void AbstractSensorTpl<T>::updateTelemetryAll(void)
    {
        for (AbstractSensorBase * sensor : dataHolder_->sensors_)
        {
            sensor->updateTelemetry();
        }
    }

    template <typename T>
    matrixN_t::RowXpr AbstractSensorTpl<T>::data(void)
    {
        return matrixN_t::RowXpr(dataHolder_->data_.derived(), sensorId_, 0, 1, sizeOf_);
    }
}