#ifndef WDC_EXO_CONTROLLER_H
#define WDC_EXO_CONTROLLER_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    class Engine;

    class ExoController : public AbstractController
    {
    protected:
        typedef std::function<void(float64_t const &/*t*/,
                                   vectorN_t const &/*q*/,
                                   vectorN_t const &/*v*/,
                                   matrixN_t const &/*optoforces*/,
                                   matrixN_t const &/*IMUs*/,
                                   vectorN_t       &/*u*/)> commandFct_t;

    public:
        ExoController(void);
        ~ExoController(void);
        AbstractController* clone(void);

        result_t initialize(commandFct_t commandFct);

    protected:
        void compute_command(Engine    const & engine,
                             float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t       & u);

        void internalDynamics(Engine    const & engine,
                              float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t       & u);

    private:
        static std::map<std::string, matrixN_t> getSensorsData(sensorsGroupMap_t const * sensors);

    private:
        commandFct_t commandFct_;
    };
}

#endif //end of WDC_EXO_CONTROLLER_H