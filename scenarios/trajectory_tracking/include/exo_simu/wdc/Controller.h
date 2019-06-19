#ifndef WDC_EXO_CONTROLLER_H
#define WDC_EXO_CONTROLLER_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    class Engine;

    class Controller : public AbstractController
    {
    public:
        vectorN_t const Kp = (vectorN_t(12) << 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0,
                                               41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0).finished();
        vectorN_t const Kd = (vectorN_t(12) << 500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 
                                               500.0, 160.0, 120.0, 270.0, 15.0, 20.0).finished();

        Controller(void);
        ~Controller(void);
        AbstractController* clone(void);

        result_t initialize(void);

    protected:
        void compute_command(float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t       & u);

        void internalDynamics(Engine    const & engine,
                              float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t       & u);
    };
}

#endif //end of WDC_EXO_CONTROLLER_H