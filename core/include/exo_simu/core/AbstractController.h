#ifndef SIMU_ABSTRACT_CONTROLLER_H
#define SIMU_ABSTRACT_CONTROLLER_H

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    class Engine;

    class AbstractController
    {
        friend class Engine;

    public:
        virtual configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["updatePeriod"] = 0.0;

            return config;
        };

        struct controllerOptions_t
        {
            float64_t const updatePeriod;

            controllerOptions_t(configHolder_t const & options) :
            updatePeriod(boost::get<float64_t>(options.at("updatePeriod")))
            {
                // Empty.
            }
        };

    public:
        AbstractController(void);
        virtual ~AbstractController(void);
        virtual AbstractController* clone(void) = 0;

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & ctrlOptions);
        bool getIsInitialized(void) const;

    protected:
        // It assumes that the engine internal state is consistent with other input arguments
        result_t compute_efforts(Engine    const & engine,
                                 float64_t const & t,
                                 vectorN_t const & q,
                                 vectorN_t const & v,
                                 vectorN_t       & u);

        virtual void compute_command(Engine    const & engine,
                                     float64_t const & t,
                                     vectorN_t const & q,
                                     vectorN_t const & v,
                                     vectorN_t       & u) = 0;

        virtual void internalDynamics(Engine    const & engine,
                                      float64_t const & t,
                                      vectorN_t const & q,
                                      vectorN_t const & v,
                                      vectorN_t       & u) = 0;

    public:
        std::shared_ptr<controllerOptions_t const> ctrlOptions_;

    protected:
        bool isInitialized_;
        configHolder_t ctrlOptionsHolder_;
    };
}

#endif //end of SIMU_ABSTRACT_CONTROLLER_H