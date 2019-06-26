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


namespace exo_simu
{
    using namespace boost::numeric::odeint;

    class AbstractController;
        
    class Engine
    {
    protected:
        typedef std::function<bool(float64_t const &/*t*/,
                                   vectorN_t const &/*x*/)> callbackFct_t;

        typedef runge_kutta_dopri5<vectorN_t, float64_t, vectorN_t, float64_t, vector_space_algebra> stepper_t;

        struct stepperState_t
        {
            // Internal state for the integration loop

        public:
            stepperState_t(void):
            iterPrev(),
            tPrev(),
            qPrev(),
            vPrev(),
            aPrev(),
            uPrev(),
            qNext(),
            fext(),
            uCommand(),
            uInternal(),
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
                result_t returnCode = result_t::SUCCESS;

                if (!model.getIsInitialized())
                {
                    std::cout << "Error - stepperState_t::initialize - Something is wrong with the Model." << std::endl;
                    returnCode = result_t::ERROR_INIT_FAILED;
                }
                
                if (returnCode == result_t::SUCCESS)
                {
                    iterPrev = 0;
                    tPrev = 0;
                    qPrev = vectorN_t::Zero(model.nq());
                    vPrev = vectorN_t::Zero(model.nv());
                    aPrev = vectorN_t::Zero(model.nv());
                    uPrev = vectorN_t::Zero(model.nv());
                    qNext = vectorN_t::Zero(model.nq());
                    fext = pinocchio::container::aligned_vector<pinocchio::Force>(model.pncModel_.joints.size(), 
                                                                                  pinocchio::Force::Zero());
                    uCommand = vectorN_t::Zero(model.getJointsVelocityIdx().size());
                    uInternal = vectorN_t::Zero(model.nv());
                    isInitialized = true;
                }

                return returnCode;
            }

            void updatePrev(float64_t const & t,
                            vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t const & a,
                            vectorN_t const & u)
            {
                tPrev = t;
                qPrev = q;
                vPrev = v;
                aPrev = a;
                uPrev = u;
                ++iterPrev;
            }

        public:
            uint32_t iterPrev;
            float64_t tPrev;
            vectorN_t qPrev;
            vectorN_t vPrev;
            vectorN_t aPrev;
            vectorN_t uPrev;
            vectorN_t qNext;
            pinocchio::container::aligned_vector<pinocchio::Force> fext;
            vectorN_t uCommand;
            vectorN_t uInternal;

        private:
            bool isInitialized;
        };
        
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

        configHolder_t getDefaultStepperOptions()
        {
            configHolder_t config;
            config["tolAbs"] = 1.0e-5;
            config["tolRel"] = 1.0e-4;

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

        configHolder_t getDefaultOptions()
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
        ~Engine(void);

        result_t initialize(Model              & model,
                            AbstractController & controller,
                            callbackFct_t        callbackFct);

        vectorN_t contactDynamics(int32_t const & frameId) const;

        result_t simulate(vectorN_t         x0, /* Make a copy */
                          float64_t const & tf,
                          float64_t const & dt);

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & engineOptions);
        bool getIsInitialized(void) const;
        Model const & getModel(void) const;
        std::vector<vectorN_t> const & getContactForces(void) const;

    protected:
        void dynamicsCL(float64_t const & t,
                        vectorN_t const & xVect,
                        vectorN_t       & xVectDot);

        void internalDynamics(float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t       & u);

    public:
        matrixN_t log; // TODO: To be removed
        std::shared_ptr<engineOptions_t const> engineOptions_;

    protected:
        bool isInitialized_;
        std::shared_ptr<Model> model_;
        std::shared_ptr<AbstractController> controller_;
        configHolder_t engineOptionsHolder_;
        callbackFct_t callbackFct_;

    private:
        stepperState_t stepperState_; // Internal state for the integration loop
    };
}

#endif //end of SIMU_ENGINE_H
