#ifndef WDC_EXO_SIMULATOR_H
#define WDC_EXO_SIMULATOR_H

#include <string>
#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "exo_simu/engine/Types.h"
#include "exo_simu/engine/ConfigHolder.h"


using namespace boost::numeric::odeint;

namespace exo_simu
{
    class ExoSimulator
    {
    ////////////////Public typedefs//////////////////
    public:
        typedef std::vector<float64_t> state_t;
        typedef std::vector<state_t> log_t;
        typedef std::function<void(const float64_t &/*t*/,
                                   const vectorN_t &/*x*/,
                                   const matrixN_t &/*optoforces*/,
                                   const matrixN_t &/*IMUs*/,
                                         vectorN_t &/*u*/)> controller_t;
        typedef std::function<bool(const float64_t &/*t*/,
                                   const vectorN_t &/*x*/)> callbackFct_t;

        enum class result_t : int32_t
        {
            SUCCESS = 1,
            ERROR_GENERIC = -1,
            ERROR_BAD_INPUT = -2,
            ERROR_INIT_FAILED = -3
        };

        static ConfigHolder getDefaultContactOptions()
        {
            ConfigHolder config;
            config.addOption<float64_t>("frictionViscous", 0.8);
            config.addOption<float64_t>("frictionDry", 1.0);
            config.addOption<float64_t>("dryFrictionVelEps", 1.0e-2);
            config.addOption<float64_t>("stiffness", 5.0e5);
            config.addOption<float64_t>("damping", 5.0e3);
            config.addOption<float64_t>("transitionEps", 1.0e-3);

            return config;
        };

        struct contactOptions_t
        {
            const float64_t frictionViscous;
            const float64_t frictionDry;
            const float64_t dryFrictionVelEps;
            const float64_t stiffness;
            const float64_t damping;
            const float64_t transitionEps;

            contactOptions_t(ConfigHolder const& options):
            frictionViscous(options.get<float64_t>("frictionViscous")),
            frictionDry(options.get<float64_t>("frictionDry")),
            dryFrictionVelEps(options.get<float64_t>("dryFrictionVelEps")),
            stiffness(options.get<float64_t>("stiffness")),
            damping(options.get<float64_t>("damping")),
            transitionEps(options.get<float64_t>("transitionEps"))
            {
            }
        };

        static ConfigHolder getDefaultJointOptions()
        {
            ConfigHolder config;
            config.addOption<bool>("boundsFromUrdf", true);
            config.addOption<vectorN_t>("boundsMin", -(vectorN_t(12) << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished());
            config.addOption<vectorN_t>("boundsMax", (vectorN_t(12) << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished());
            config.addOption<vectorN_t>("frictionViscous", 0*(vectorN_t(12) << 100.0,100.0,100.0,100.0,20.0,20.0,100.0,100.0,100.0,100.0,20.0,20.0).finished());
            config.addOption<vectorN_t>("frictionDry", 0*(vectorN_t(12) << 10.0,10.0,10.0,10.0,2.0,2.0,10.0,10.0,10.0,10.0,2.0,2.0).finished());
            config.addOption<float64_t>("dryFrictionVelEps", 1.0e-2);
            config.addOption<float64_t>("boundStiffness", 5.0e5);
            config.addOption<float64_t>("boundDamping", 5.0e2);
            config.addOption<float64_t>("boundTransitionEps", 2.0e-3);

            return config;
        };

        struct jointOptions_t
        {
            const bool      boundsFromUrdf;
            const vectorN_t boundsMin;
            const vectorN_t boundsMax;
            const vectorN_t frictionViscous;
            const vectorN_t frictionDry;
            const float64_t dryFrictionVelEps;
            const float64_t boundStiffness;
            const float64_t boundDamping;
            const float64_t boundTransitionEps;

            jointOptions_t(ConfigHolder const& options):
            boundsFromUrdf(options.get<bool>("boundsFromUrdf")),
            boundsMin(options.get<vectorN_t>("boundsMin")),
            boundsMax(options.get<vectorN_t>("boundsMax")),
            frictionViscous(options.get<vectorN_t>("frictionViscous")),
            frictionDry(options.get<vectorN_t>("frictionDry")),
            dryFrictionVelEps(options.get<float64_t>("dryFrictionVelEps")),
            boundStiffness(options.get<float64_t>("boundStiffness")),
            boundDamping(options.get<float64_t>("boundDamping")),
            boundTransitionEps(options.get<float64_t>("boundTransitionEps"))
            {
            }
        };

        static ConfigHolder getDefaultModelOptions()
        {
            ConfigHolder config;
            config.addOption<ConfigHolder>("joints", getDefaultJointOptions());
            config.addOption<ConfigHolder>("contacts", getDefaultContactOptions());
            config.addOption<vectorN_t>("gravity", (vectorN_t(6) << 0.0,0.0,-9.81,0.0,0.0,0.0).finished());

            return config;
        };

        struct modelOptions_t
        {
            const jointOptions_t   joints;
            const contactOptions_t contacts;
            const vectorN_t        gravity;

            modelOptions_t(ConfigHolder const& options):
            joints(options.get<ConfigHolder>("joints")),
            contacts(options.get<ConfigHolder>("contacts")),
            gravity(options.get<vectorN_t>("gravity"))
            {
            }
        };

        static ConfigHolder getDefaultSimulationOptions()
        {
            ConfigHolder config;
            config.addOption<float64_t>("tolAbs", 1.0e-6);
            config.addOption<float64_t>("tolRel", 1.0e-6);
            config.addOption<bool>("logController", true);
            config.addOption<bool>("logOptoforces", true);
            config.addOption<bool>("logIMUs", true);

            return config;
        };

        struct simulationOptions_t
        {
            const float64_t tolAbs;
            const float64_t tolRel;
            const bool      logController;
            const bool      logOptoforces;
            const bool      logIMUs;

            simulationOptions_t(ConfigHolder const& options):
            tolAbs(options.get<float64_t>("tolAbs")),
            tolRel(options.get<float64_t>("tolRel")),
            logController(options.get<bool>("logController")),
            logOptoforces(options.get<bool>("logOptoforces")),
            logIMUs(options.get<bool>("logIMUs"))
            {
            }
        };

    ////////////////Protected typedefs//////////////////
    protected:
        typedef runge_kutta_dopri5<state_t> stepper_t;

    /////////////////Public methods///////////////////
    public:
        //Constructor & destructor
        ExoSimulator(void);

        ExoSimulator(const std::string urdfPath);

        ExoSimulator(const std::string urdfPath,
                     const ConfigHolder & mdlOptions);

        ExoSimulator(const std::string urdfPath,
                     const ConfigHolder & mdlOptions,
                     const ConfigHolder & simOptions);

        ~ExoSimulator(void);

        void init(const std::string urdfPath);

        void init(const std::string urdfPath,
                  const ConfigHolder & mdlOptions);

        void init(const std::string urdfPath,
                  const ConfigHolder & mdlOptions,
                  const ConfigHolder & simOptions);

        //Simulate functions
        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tf,
                          const float64_t & dt,
                          controller_t controller);

        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tf,
                          const float64_t & dt,
                          controller_t controller,
                          callbackFct_t callbackFct);

        //Accessors
        std::string getUrdfPath(void);
        void setUrdfPath(const std::string & urdfPath);
        ConfigHolder getModelOptions(void);
        void setModelOptions(const ConfigHolder & mdlOptions);
        ConfigHolder getSimulationOptions(void);
        void setSimulationOptions(const ConfigHolder & simOptions);

    ////////////////Protected methods/////////////////
    protected:
        bool checkCtrl(controller_t controller);

        void dynamicsCL(const state_t & x,
                        state_t & xDot,
                        const float64_t t);
        void internalDynamics(const vectorN_t & q,
                              const vectorN_t & dq,
                                    vectorN_t & u);
        vectorN_t contactDynamics(const int32_t & frameId);

        float64_t saturateSoft(const float64_t in,
                               const float64_t mi,
                               const float64_t ma,
                               const float64_t r);

    ////////////////Public attributes/////////////////
    public:
        log_t log;

    //////////////Protected attributes////////////////
    protected:
        bool isInitialized_;
        std::string urdfPath_;
        controller_t controller_;
        ConfigHolder mdlOptionsHolder_;
        ConfigHolder simOptionsHolder_;
        std::shared_ptr<modelOptions_t> mdlOptions_;
        std::shared_ptr<simulationOptions_t> simOptions_;
        pinocchio::Model model_;
        pinocchio::Data data_;

        const int64_t nq_;
        const int64_t ndq_;
        const int64_t nx_;
        const int64_t nu_;

        const int64_t nqFull_;
        const int64_t ndqFull_;
        const int64_t nxFull_;
        const int64_t nuFull_;

        bool tesc_;

        const std::vector<std::string> contactFramesNames_;
        const std::vector<std::string> imuFramesNames_;
        const std::vector<std::string> jointsNames_;

        std::vector<int32_t> contactFramesIdx_;
        std::vector<int32_t> imuFramesIdx_;
        std::vector<int32_t> jointsIdx_;
    };
}

#endif //end of WDC_EXO_SIMULATOR_H
