#ifndef WDC_EXO_CONTROLLER_H
#define WDC_EXO_CONTROLLER_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    class ExoController : public AbstractController
    {
    public:
        // Disable the copy of the class
        ExoController(ExoController const & controller) = delete;
        ExoController & operator = (ExoController const & controller) = delete;

    protected:
        typedef std::function<void(float64_t const & /*t*/,
                                   vectorN_t const & /*q*/,
                                   vectorN_t const & /*v*/,
                                   matrixN_t const & /*forceSensorsData*/,
                                   matrixN_t const & /*imuSensorsData*/,
                                   matrixN_t const & /*encoderSensorsData*/,
                                   vectorN_t       & /*u*/)> commandFct_t;

    public:
        ExoController(void);
        ~ExoController(void);

        result_t initialize(Model        const & model,
                            commandFct_t         commandFct);
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
        result_t fetchSensors(void);

        using AbstractController::initialize; // Discourage the use

    private:
        ExoModel const * exoModel_; // Raw pointer to avoid managing its deletion
        commandFct_t commandFct_;
        matrixN_t forceSensorsData_;
        matrixN_t imuSensorsData_;
        matrixN_t encoderSensorsData_;
    };
}

#endif //end of WDC_EXO_CONTROLLER_H