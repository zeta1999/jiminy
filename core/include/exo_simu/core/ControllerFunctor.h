#ifndef WDC_CONTROLLER_FUNCTOR_H
#define WDC_CONTROLLER_FUNCTOR_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    template<typename F1, typename F2>
    class ControllerFunctor : public AbstractController
    {
    public:
        // Disable the copy of the class
        ControllerFunctor(ControllerFunctor const & controller) = delete;
        ControllerFunctor & operator = (ControllerFunctor const & controller) = delete;

    public:
        ControllerFunctor(F1 & commandFct,
                          F2 & internalDynamicsFct);
        ~ControllerFunctor(void);

        result_t configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData);

        result_t computeCommand(float64_t const & t,
                                vectorN_t const & q,
                                vectorN_t const & v,
                                vectorN_t       & u) override;
        result_t internalDynamics(float64_t const & t,
                                  vectorN_t const & q,
                                  vectorN_t const & v,
                                  vectorN_t       & u) override;

    private:
        F1 commandFct_;
        F2 internalDynamicsFct_;
        std::vector<matrixN_t> sensorsData_;
    };

    template<typename F1, typename F2>
    ControllerFunctor<F1, F2> createControllerFunctor(F1 & commandFct,
                                                      F2 & internalDynamicsFct)
    {
        return {commandFct, internalDynamicsFct};
    }
}

#include "exo_simu/core/ControllerFunctor.tcc"

#endif //end of WDC_CONTROLLER_FUNCTOR_H