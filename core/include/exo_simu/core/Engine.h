#ifndef SIMU_ENGINE_H
#define SIMU_ENGINE_H

#include <string>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/TelemetrySender.h"


namespace exo_simu
{
    std::string const ENGINE_OBJECT_NAME("Global");

    using namespace boost::numeric::odeint;

    class AbstractController;
    class TelemetryData;
    class TelemetryRecorder;

    class Engine
    {
    protected:
        typedef std::function<bool(float64_t const & /*t*/,
                                   vectorN_t const & /*x*/)> callbackFct_t;

        typedef runge_kutta_dopri5<vectorN_t, float64_t, vectorN_t, float64_t, vector_space_algebra> stepper_t;

    public:
        // Disable the copy of the class
        Engine(Engine const & engine) = delete;
        Engine & operator = (Engine const & other) = delete;

    public:
        configHolder_t getDefaultContactOptions()
        {
            configHolder_t config;
            config["frictionViscous"] = 0.8;
            config["frictionDry"] = 1.0;
            config["dryFrictionVelEps"] = 1.0e-2;
            config["stiffness"] = 1.0e6;
            config["damping"] = 2.0e3;
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

        configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundStiffness"] = 1.0e5;
            config["boundDamping"] = 1.0e2;
            config["boundTransitionEps"] = 1.0e-2; // about 0.55 degrees

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

        configHolder_t getDefaultWorldOptions()
        {
            configHolder_t config;
            config["gravity"] = (vectorN_t(6) << 0.0,0.0,-9.81,0.0,0.0,0.0).finished();

            return config;
        };

        struct worldOptions_t
        {
            vectorN_t const gravity;

            worldOptions_t(configHolder_t const & options) :
            gravity(boost::get<vectorN_t>(options.at("gravity")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultStepperOptions()
        {
            configHolder_t config;
            config["tolAbs"] = 1.0e-5;
            config["tolRel"] = 1.0e-4;
            config["sensorsUpdatePeriod"] = 0.0;
            config["controllerUpdatePeriod"] = 0.0;
            config["randomSeed"] = 0;

            return config;
        };

        struct stepperOptions_t
        {
            float64_t const tolAbs;
            float64_t const tolRel;
            float64_t const sensorsUpdatePeriod;
            float64_t const controllerUpdatePeriod;
            int32_t const randomSeed;

            stepperOptions_t(configHolder_t const & options):
            tolAbs(boost::get<float64_t>(options.at("tolAbs"))),
            tolRel(boost::get<float64_t>(options.at("tolRel"))),
            sensorsUpdatePeriod(boost::get<float64_t>(options.at("sensorsUpdatePeriod"))),
            controllerUpdatePeriod(boost::get<float64_t>(options.at("controllerUpdatePeriod"))),
            randomSeed(boost::get<int32_t>(options.at("randomSeed")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultTelemetryOptions()
        {
            configHolder_t config;
            config["logConfiguration"] = true;
            config["logVelocity"] = true;
            config["logAcceleration"] = true;
            config["logCommand"] = true;
            return config;
        };

        struct telemetryOptions_t
        {
            bool const logConfiguration;
            bool const logVelocity;
            bool const logAcceleration;
            bool const logCommand;

            telemetryOptions_t(configHolder_t const & options):
            logConfiguration(boost::get<bool>(options.at("logConfiguration"))),
            logVelocity(boost::get<bool>(options.at("logVelocity"))),
            logAcceleration(boost::get<bool>(options.at("logAcceleration"))),
            logCommand(boost::get<bool>(options.at("logCommand")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["telemetry"] = getDefaultTelemetryOptions();
            config["stepper"] = getDefaultStepperOptions();
            config["world"] = getDefaultWorldOptions();
            config["joints"] = getDefaultJointOptions();
            config["contacts"] = getDefaultContactOptions();

            return config;
        };

        struct engineOptions_t
        {
            telemetryOptions_t const telemetry;
            stepperOptions_t const stepper;
            worldOptions_t   const world;
            jointOptions_t   const joints;
            contactOptions_t const contacts;

            engineOptions_t(configHolder_t const & options) :
            telemetry(boost::get<configHolder_t>(options.at("telemetry"))),
            stepper(boost::get<configHolder_t>(options.at("stepper"))),
            world(boost::get<configHolder_t>(options.at("world"))),
            joints(boost::get<configHolder_t>(options.at("joints"))),
            contacts(boost::get<configHolder_t>(options.at("contacts")))
            {
                // Empty.
            }
        };

    protected:
        struct stepperState_t
        {
        // Internal state for the integration loop

        public:
            stepperState_t(void):
            iterLast(),
            tLast(),
            qLast(),
            vLast(),
            aLast(),
            uLast(),
            uCommandLast(),
            qNames(),
            vNames(),
            aNames(),
            uCommandNames(),
            x(),
            dxdt(),
            uControl(),
            fext(),
            uBounds(),
            uInternal(),
            energyLast(0.0),
            isInitialized()
            {
                // Empty.
            }

            bool getIsInitialized(void) const
            {
                return isInitialized;
            }


            result_t initialize(Model const & model)
            {
                vectorN_t x_init = vectorN_t::Zero(model.nx());
                return initialize(model, x_init);
            }

            result_t initialize(Model     const & model,
                                vectorN_t const & x_init)
            {
                result_t returnCode = result_t::SUCCESS;

                if (!model.getIsInitialized())
                {
                    std::cout << "Error - stepperState_t::initialize - Something is wrong with the Model." << std::endl;
                    returnCode = result_t::ERROR_INIT_FAILED;
                }

                if (returnCode == result_t::SUCCESS)
                {
                    iterLast = 0;
                    tLast = 0;
                    qLast = x_init.head(model.nq());
                    vLast = x_init.tail(model.nv());
                    aLast = vectorN_t::Zero(model.nv());
                    uLast = vectorN_t::Zero(model.nv());
                    uCommandLast = vectorN_t::Zero(model.getJointsVelocityIdx().size());

                    qNames = defaultVectorFieldnames("q", qLast.size());
                    vNames = defaultVectorFieldnames("v", vLast.size());
                    aNames = defaultVectorFieldnames("a", aLast.size());
                    uCommandNames = defaultVectorFieldnames("uCommand", uCommandLast.size());

                    x = x_init;
                    dxdt = vectorN_t::Zero(model.nx());
                    uControl = vectorN_t::Zero(model.nv());

                    fext = pinocchio::container::aligned_vector<pinocchio::Force>(model.pncModel_.joints.size(),
                                                                                  pinocchio::Force::Zero());
                    uBounds = vectorN_t::Zero(model.nv());
                    uInternal = vectorN_t::Zero(model.nv());

                    isInitialized = true;
                }

                return returnCode;
            }

            void updateLast(float64_t const & t,
                            vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t const & a,
                            vectorN_t const & u,
                            vectorN_t const & uCommand,
                            float64_t const & energy)
            {
                tLast = t;
                qLast = q;
                vLast = v;
                aLast = a;
                uLast = u;
                uCommandLast = uCommand;
                energyLast = energy;
                ++iterLast;
            }

        public:
            // State information about the last iteration
            uint32_t iterLast;
            float64_t tLast;
            vectorN_t qLast;
            vectorN_t vLast;
            vectorN_t aLast;
            vectorN_t uLast;
            vectorN_t uCommandLast;
            float64_t energyLast; ///< Energy (kinetic + potential) of the system at the last state.

            std::vector<std::string> qNames;
            std::vector<std::string> vNames;
            std::vector<std::string> aNames;
            std::vector<std::string> uCommandNames;

            // Internal buffers required for the adaptive step computation and system dynamics
            vectorN_t x;
            vectorN_t dxdt;
            vectorN_t uControl;

            // Internal buffers to speed up the evaluation of the system dynamics
            pinocchio::container::aligned_vector<pinocchio::Force> fext;
            vectorN_t uBounds;
            vectorN_t uInternal;

        private:
            bool isInitialized;
        };

    public:
        Engine(void);
        ~Engine(void);

        result_t initialize(Model              & model,
                            AbstractController & controller,
                            callbackFct_t        callbackFct);

        result_t simulate(vectorN_t const & x_init,
                          float64_t const & end_time);

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & engineOptions);
        bool getIsInitialized(void) const;
        Model const & getModel(void) const;
        std::vector<vectorN_t> const & getContactForces(void) const;
        void getLogData(std::vector<std::string> & header,
                        matrixN_t                & logData);
        void writeLogTxt(std::string const & filename);
        void writeLogBinary(std::string const & filename);

    protected:
        void systemDynamics(float64_t const & t,
                            vectorN_t const & x,
                            vectorN_t       & dxdt);
        void boundsDynamics(vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t       & u);
        vectorN_t contactDynamics(int32_t const & frameId) const;

    public:
        std::unique_ptr<engineOptions_t const> engineOptions_;

    protected:
        bool isInitialized_;
        Model * model_;
        AbstractController * controller_;
        configHolder_t engineOptionsHolder_;
        callbackFct_t callbackFct_;

    private:
        TelemetrySender telemetrySender_;
        std::shared_ptr<TelemetryData> telemetryData_;
        std::unique_ptr<TelemetryRecorder> telemetryRecorder_;
        stepperState_t stepperState_; // Internal state for the integration loop
    };
}

#endif //end of SIMU_ENGINE_H
