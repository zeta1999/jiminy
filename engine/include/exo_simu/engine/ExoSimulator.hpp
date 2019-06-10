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

        typedef Eigen::Matrix<float64_t,6,1> Vector6d;
        typedef Eigen::Matrix<float64_t,12,1> Vector12d;
        typedef std::vector<float64_t> state_t;
        typedef std::vector<state_t> log_t;

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
            config.addOption<float64_t>("dryFictionVelEps", 1.0e-3);
            config.addOption<float64_t>("stiffness", 5.0e5);
            config.addOption<float64_t>("damping", 5.0e3);
            config.addOption<float64_t>("transitionEps", 1.0e-3);

            return config;
        };

        static ConfigHolder getDefaultJointOptions()
        {
            ConfigHolder config;
            config.addOption<vectorN_t>("frictionViscous", (Vector12d() << 100.0,100.0,100.0,100.0,20.0,20.0,100.0,100.0,100.0,100.0,20.0,20.0).finished());
            config.addOption<vectorN_t>("frictionDry", (Vector12d() << 10.0,10.0,10.0,10.0,2.0,2.0,10.0,10.0,10.0,10.0,2.0,2.0).finished());
            config.addOption<vectorN_t>("boundsMin", -(Vector12d() << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished());
            config.addOption<vectorN_t>("boundsMax", (Vector12d() << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished());
            config.addOption<bool>("boundsFromUrdf", true);
            config.addOption<float64_t>("dryFictionVelEps", 1.0e-2);
            config.addOption<float64_t>("boundStiffness", 5.0e5);
            config.addOption<float64_t>("boundDamping", 5.0e2);
            config.addOption<float64_t>("boundTransitionEps", 2.0e-3);

            return config;
        };


        static ConfigHolder getDefaultModelOptions()
        {
            ConfigHolder config;
            config.addOption<ConfigHolder>("joints", getDefaultJointOptions());
            config.addOption<ConfigHolder>("contacts", getDefaultContactOptions());
            config.addOption<vectorN_t>("gravity", (Vector6d() << 0.0,0.0,-9.81,0.0,0.0,0.0).finished());

            return config;
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

        ~ExoSimulator(void);

        void init(const std::string urdfPath);

        void init(const std::string urdfPath,
                  const ConfigHolder & mdlOptions);

        //Simulate functions
        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tend,
                          const float64_t & dt,
                          std::function<void(const float64_t /*t*/,
                                             const vectorN_t &/*x*/,
                                             const matrixN_t &/*optoforces*/,
                                             const matrixN_t &/*IMUs*/,
                                                   vectorN_t &/*u*/)> controller);

        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tend,
                          const float64_t & dt,
                          std::function<void(const float64_t /*t*/,
                                             const vectorN_t &/*x*/,
                                             const matrixN_t &/*optoforces*/,
                                             const matrixN_t &/*IMUs*/,
                                                   vectorN_t &/*u*/)> controller,
                          std::function<bool(const float64_t /*t*/,
                                             const vectorN_t &/*x*/)> monitorFun);

        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tend,
                          const float64_t & dt,
                          std::function<void(const float64_t /*t*/,
                                             const vectorN_t &/*x*/,
                                             const matrixN_t &/*optoforces*/,
                                             const matrixN_t &/*IMUs*/,
                                                   vectorN_t &/*u*/)> controller,
                          const ConfigHolder & simOptions);

        result_t simulate(const vectorN_t & x0,
                          const float64_t & t0,
                          const float64_t & tend,
                          const float64_t & dt,
                          std::function<void(const float64_t /*t*/,
                                             const vectorN_t &/*x*/,
                                             const matrixN_t &/*optoforces*/,
                                             const matrixN_t &/*IMUs*/,
                                                   vectorN_t &/*u*/)> controller,
                          std::function<bool(const float64_t /*t*/,
                                             const vectorN_t &/*x*/)> monitorFun,
                          const ConfigHolder & simOptions);

        //Accessors
        std::string getUrdfPath(void);
        ConfigHolder getModelOptions(void);

    ////////////////Protected methods/////////////////
    protected:
        void setUrdfPath(const std::string & urdfPath);
        void setModelOptions(const ConfigHolder & mdlOptions);
        bool checkCtrl(std::function<void(const float64_t /*t*/,
                                          const vectorN_t &/*x*/,
                                          const matrixN_t &/*optoforces*/,
                                          const matrixN_t &/*IMUs*/,
                                                vectorN_t &/*u*/)> controller);

        void dynamicsCL(const state_t & x,
                        state_t & xDot,
                        const float64_t t);
        void internalDynamics(const vectorN_t & q,
                              const vectorN_t & dq,
                                    vectorN_t & u);
        Vector6d contactDynamics(const int32_t & frameId);

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
        std::function<void(const float64_t /*t*/,
                           const vectorN_t &/*x*/,
                           const matrixN_t &/*optoforces*/,
                           const matrixN_t &/*IMUs*/,
                                 vectorN_t &/*u*/)> controller_;
        ConfigHolder mdlOptions_;
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
