#ifndef SIMU_ENGINE_H
#define SIMU_ENGINE_H

#include <string>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Logger.h"


namespace exo_simu
{
    using namespace boost::numeric::odeint;

    class AbstractController;
        
    class Engine
    {
    protected:
        typedef std::function<void(float64_t const &/*t*/,
                                   vectorN_t const &/*x*/,
                                   matrixN_t const &/*optoforces*/,
                                   matrixN_t const &/*IMUs*/,
                                   vectorN_t       &/*u*/)> controller_t;

        typedef std::function<bool(float64_t const &/*t*/,
                                   vectorN_t const &/*x*/)> callbackFct_t;

        typedef runge_kutta_dopri5<state_t> stepper_t;

    protected:
        virtual configHolder_t getDefaultContactOptions()
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
            float64_t const frictionViscous;
            float64_t const frictionDry;
            float64_t const dryFrictionVelEps;
            float64_t const stiffness;
            float64_t const damping;
            float64_t const transitionEps;

            contactOptions_t(configHolder_t const & options):
            frictionViscous(boost::get<float64_t>(options.at("frictionViscous"))),
            frictionDry(boost::get<float64_t>(options.at("frictionDry"))),
            dryFrictionVelEps(boost::get<float64_t>(options.at("dryFrictionVelEps"))),
            stiffness(boost::get<float64_t>(options.at("stiffness"))),
            damping(boost::get<float64_t>(options.at("damping"))),
            transitionEps(boost::get<float64_t>(options.at("transitionEps")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundStiffness"] = 5.0e5;
            config["boundDamping"] = 5.0e2;
            config["boundTransitionEps"] = 2.0e-3;

            return config;
        };

        struct jointOptions_t
        {
            float64_t const boundStiffness;
            float64_t const boundDamping;
            float64_t const boundTransitionEps;

            jointOptions_t(configHolder_t const & options):
            boundStiffness(boost::get<float64_t>(options.at("boundStiffness"))),
            boundDamping(boost::get<float64_t>(options.at("boundDamping"))),
            boundTransitionEps(boost::get<float64_t>(options.at("boundTransitionEps")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultStepperOptions()
        {
            configHolder_t config;
            config["tolAbs"] = 1.0e-6;
            config["tolRel"] = 1.0e-6;

            return config;
        };

        struct stepperOptions_t
        {
            float64_t const tolAbs;
            float64_t const tolRel;

            stepperOptions_t(configHolder_t const & options):
            tolAbs(boost::get<float64_t>(options.at("tolAbs"))),
            tolRel(boost::get<float64_t>(options.at("tolRel")))
            {
                // Empty.
            }
        };

    public:
        virtual configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["stepper"] = getDefaultStepperOptions();
            config["joints"] = getDefaultJointOptions();
            config["contacts"] = getDefaultContactOptions();
            config["gravity"] = (vectorN_t(6) << 0.0,0.0,-9.81,0.0,0.0,0.0).finished();

            return config;
        };


        struct engineOptions_t
        {
            stepperOptions_t const stepper;
            jointOptions_t   const joints;
            contactOptions_t const contacts;
            vectorN_t        const gravity;

            engineOptions_t(configHolder_t const & options):
            stepper(boost::get<configHolder_t>(options.at("stepper"))),
            joints(boost::get<configHolder_t>(options.at("joints"))),
            contacts(boost::get<configHolder_t>(options.at("contacts"))),
            gravity(boost::get<vectorN_t>(options.at("gravity")))
            {
                // Empty.
            }
        };
        
    public:
        Engine(void);
        virtual ~Engine(void);

        result_t initialize(Model              & model,
                            AbstractController & controller);

        vectorN_t const & contactDynamics(int32_t const & frameId) const;

        result_t simulate(vectorN_t    const & x0,
                          float64_t    const & tf,
                          float64_t    const & dt);
        result_t simulate(vectorN_t     const & x0,
                          float64_t     const & tf,
                          float64_t     const & dt,
                          callbackFct_t         callbackFct);

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & engineOptions);
        bool getIsInitialized(void) const;
        std::string getLog(bool const & isHeaderEnable = true, 
                           bool const & isDataEnable = true) const;
        Model const & getModel(void) const;
        std::vector<vectorN_t> const & getContactForces(void) const;

    protected:
        void dynamicsCL(float64_t const & t,
                        state_t   const & x,
                        state_t         & xDot);

    public:
        std::shared_ptr<engineOptions_t const> engineOptions_;

    protected:
        bool isInitialized_;
        Model model_;
        std::shared_ptr<AbstractController> controller_;
        Logger logger_;
        configHolder_t engineOptionsHolder_;
    };
}

#endif //end of SIMU_ENGINE_H
