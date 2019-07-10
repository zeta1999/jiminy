#ifndef SIMU_ABSTRACT_CONTROLLER_H
#define SIMU_ABSTRACT_CONTROLLER_H

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    class Model;

    class AbstractController
    {
    public:
        // Disable the copy of the class
        AbstractController(AbstractController const & controller) = delete;
        AbstractController & operator = (AbstractController const & controller) = delete;

    public:
        virtual configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            // No default configuration parameter

            return config;
        };

        struct controllerOptions_t
        {
            controllerOptions_t(configHolder_t const & options)
            {
                // Empty.
            }
        };

    public:
        AbstractController(void);
        virtual ~AbstractController(void);

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & ctrlOptions);
        bool getIsInitialized(void) const;

        // It assumes that the model internal state is consistent with other input arguments
        virtual void compute_command(Model     const & model,
                                     float64_t const & t,
                                     vectorN_t const & q,
                                     vectorN_t const & v,
                                     vectorN_t       & u) = 0;

        virtual void internalDynamics(Model     const & model,
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