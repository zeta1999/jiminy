#ifndef WDC_EXO_SIMULATOR_H
#define WDC_EXO_SIMULATOR_H

#include <map>
#include <string>
#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/variant.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "exo_simu/engine/Types.h"


using namespace boost::numeric::odeint;

namespace exo_simu
{
    typedef boost::make_recursive_variant<bool_t, int32_t, real_t, std::string, vectorN_t, matrixN_t, std::map<std::string, boost::recursive_variant_> >::type configField_t;
    typedef std::map<std::string, configField_t> configHolder_t;
    typedef std::vector<float64_t> state_t;
    typedef std::vector<state_t> log_t;
        
    class ExoSimulator
    {
    ////////////////Public typedefs//////////////////
    public:
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

        static configHolder_t getDefaultContactOptions()
        {
            configHolder_t config;
            config["frictionViscous"] = 0.8;
            config["frictionDry"] = 1.0;
            config["dryFrictionVelEps"] = 1.0e-2;
            config["stiffness"] = 5.0e5;
            config["damping"] = 5.0e3;
            config["transitionEps"] = 1.0e-3;

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

            contactOptions_t(configHolder_t const & options):
            frictionViscous(boost::get<float64_t>(options.at("frictionViscous"))),
            frictionDry(boost::get<float64_t>(options.at("frictionDry"))),
            dryFrictionVelEps(boost::get<float64_t>(options.at("dryFrictionVelEps"))),
            stiffness(boost::get<float64_t>(options.at("stiffness"))),
            damping(boost::get<float64_t>(options.at("damping"))),
            transitionEps(boost::get<float64_t>(options.at("transitionEps")))
            {
            }
        };

        static configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundsFromUrdf"] = true;
            config["boundsMin"] = (vectorN_t) (-M_PI * vectorN_t::Ones(12));
            config["boundsMax"] = (vectorN_t) ( M_PI * vectorN_t::Ones(12));
            config["frictionViscous"] = (vectorN_t) (0*(vectorN_t(12) << 100.0,100.0,100.0,100.0,20.0,20.0,100.0,100.0,100.0,100.0,20.0,20.0).finished());
            config["frictionDry"] = (vectorN_t) (0*(vectorN_t(12) << 10.0,10.0,10.0,10.0,2.0,2.0,10.0,10.0,10.0,10.0,2.0,2.0).finished());
            config["dryFrictionVelEps"] = 1.0e-2;
            config["boundStiffness"] = 5.0e5;
            config["boundDamping"] = 5.0e2;
            config["boundTransitionEps"] = 2.0e-3;

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

            jointOptions_t(configHolder_t const & options):
            boundsFromUrdf(boost::get<bool>(options.at("boundsFromUrdf"))),
            boundsMin(boost::get<vectorN_t>(options.at("boundsMin"))),
            boundsMax(boost::get<vectorN_t>(options.at("boundsMax"))),
            frictionViscous(boost::get<vectorN_t>(options.at("frictionViscous"))),
            frictionDry(boost::get<vectorN_t>(options.at("frictionDry"))),
            dryFrictionVelEps(boost::get<float64_t>(options.at("dryFrictionVelEps"))),
            boundStiffness(boost::get<float64_t>(options.at("boundStiffness"))),
            boundDamping(boost::get<float64_t>(options.at("boundDamping"))),
            boundTransitionEps(boost::get<float64_t>(options.at("boundTransitionEps")))
            {
            }
        };

        static configHolder_t getDefaultModelOptions()
        {
            configHolder_t config;
            config["joints"] = getDefaultJointOptions();
            config["contacts"] = getDefaultContactOptions();
            config["gravity"] = (vectorN_t(6) << 0.0,0.0,-9.81,0.0,0.0,0.0).finished();

            return config;
        };

        struct modelOptions_t
        {
            const jointOptions_t   joints;
            const contactOptions_t contacts;
            const vectorN_t        gravity;

            modelOptions_t(configHolder_t const & options):
            joints(boost::get<configHolder_t>(options.at("joints"))),
            contacts(boost::get<configHolder_t>(options.at("contacts"))),
            gravity(boost::get<vectorN_t>(options.at("gravity")))
            {
            }
        };

        static configHolder_t getDefaultSimulationOptions()
        {
            configHolder_t config;
            config["tolAbs"] = 1.0e-6;
            config["tolRel"] = 1.0e-6;
            config["logController"] = true;
            config["logOptoforces"] = true;
            config["logIMUs"] = true;

            return config;
        };

        struct simulationOptions_t
        {
            const float64_t tolAbs;
            const float64_t tolRel;
            const bool      logController;
            const bool      logOptoforces;
            const bool      logIMUs;

            simulationOptions_t(configHolder_t const & options):
            tolAbs(boost::get<float64_t>(options.at("tolAbs"))),
            tolRel(boost::get<float64_t>(options.at("tolRel"))),
            logController(boost::get<bool>(options.at("logController"))),
            logOptoforces(boost::get<bool>(options.at("logOptoforces"))),
            logIMUs(boost::get<bool>(options.at("logIMUs")))
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
                     const configHolder_t & mdlOptions);

        ExoSimulator(const std::string urdfPath,
                     const configHolder_t & mdlOptions,
                     const configHolder_t & simOptions);

        ~ExoSimulator(void);

        void init(const std::string urdfPath);

        void init(const std::string urdfPath,
                  const configHolder_t & mdlOptions);

        void init(const std::string urdfPath,
                  const configHolder_t & mdlOptions,
                  const configHolder_t & simOptions);

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
        configHolder_t getModelOptions(void);
        void setModelOptions(const configHolder_t & mdlOptions);
        configHolder_t getSimulationOptions(void);
        void setSimulationOptions(const configHolder_t & simOptions);

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
        configHolder_t mdlOptionsHolder_;
        configHolder_t simOptionsHolder_;
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
