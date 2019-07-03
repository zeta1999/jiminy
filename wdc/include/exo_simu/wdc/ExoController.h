#ifndef WDC_EXO_CONTROLLER_H
#define WDC_EXO_CONTROLLER_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    class ExoController : public AbstractController
    {
    protected:
        typedef std::function<void(float64_t const &/*t*/,
                                   vectorN_t const &/*q*/,
                                   vectorN_t const &/*v*/,
                                   matrixN_t const &/*forceSensorsData*/,
                                   matrixN_t const &/*imuSensorsData*/,
                                   matrixN_t const &/*encoderSensorsData*/,
                                   vectorN_t       &/*u*/)> commandFct_t;

    public:
        ExoController(void);
        ~ExoController(void);
        AbstractController* clone(void) override;

        result_t initialize(commandFct_t commandFct);

        void compute_command(Model     const & model,
                             float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t       & u) override;

        void internalDynamics(Model     const & model,
                              float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t       & u) override;

    private:
        commandFct_t commandFct_;
    };
}

#endif //end of WDC_EXO_CONTROLLER_H