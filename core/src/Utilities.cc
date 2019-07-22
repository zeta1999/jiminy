#include <math.h>
#include <climits>
#include <stdlib.h>     /* srand, rand */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/TelemetrySender.h"
#include "exo_simu/core/Model.h"


namespace exo_simu
{
    // ************************* Timer **************************

    Timer::Timer(void) :
    t0(),
    tf(),
    dt(0)
    {
        tic();
    }

    void Timer::tic(void)
    {
        t0 = Time::now();
    }

    void Timer::toc(void)
    {
        tf = Time::now();
        std::chrono::duration<float64_t> timeDiff = tf - t0;
        dt = timeDiff.count();
    }

    // ***************** Random number generator *****************
    // Based on Ziggurat generator by Marsaglia and Tsang (JSS, 2000)

    std::mt19937 generator;
    std::uniform_real_distribution<float32_t> distUniform(0.0,1.0);

    uint32_t kn[128];
    float32_t fn[128];
    float32_t wn[128];

    void r4_nor_setup(void)
    {
        float64_t const m1 = 2147483648.0;
        float64_t const vn = 9.91256303526217E-03;
        float64_t dn = 3.442619855899;
        float64_t tn = 3.442619855899;

        float64_t q = vn / exp (-0.5 * dn * dn);

        kn[0] = (uint32_t) ((dn / q) * m1);
        kn[1] = 0;

        wn[0] = static_cast<float32_t>(q / m1);
        wn[127] = static_cast<float32_t>(dn / m1);

        fn[0] = 1.0f;
        fn[127] = static_cast<float32_t>(exp(-0.5 * dn * dn));

        for (uint8_t i=126; 1 <= i; i--)
        {
            dn = sqrt (-2.0 * log(vn / dn + exp(-0.5 * dn * dn)));
            kn[i+1] = static_cast<uint32_t>((dn / tn) * m1);
            tn = dn;
            fn[i] = static_cast<float32_t>(exp(-0.5 * dn * dn));
            wn[i] = static_cast<float32_t>(dn / m1);
        }
    }

    float32_t r4_uni(void)
    {
        return distUniform(generator);
    }

    float32_t r4_nor(void)
    {
        float32_t const r = 3.442620f;
        int32_t hz;
        uint32_t iz;
        float32_t x;
        float32_t y;

        hz = static_cast<int32_t>(generator());
        iz = (hz & 127U);

        if (fabs(hz) < kn[iz])
        {
            return static_cast<float32_t>(hz) * wn[iz];
        }
        else
        {
            while(true)
            {
                if (iz == 0)
                {
                    while(true)
                    {
                        x = - 0.2904764f * log(r4_uni());
                        y = - log(r4_uni());
                        if (x * x <= y + y)
                        {
                            break;
                        }
                    }

                    if (hz <= 0)
                    {
                        return - r - x;
                    }
                    else
                    {
                        return + r + x;
                    }
                }

                x = static_cast<float32_t>(hz) * wn[iz];

                if (fn[iz] + r4_uni() * (fn[iz-1] - fn[iz]) < exp (-0.5f * x * x))
                {
                    return x;
                }

                hz = static_cast<int32_t>(generator());
                iz = (hz & 127);

                if (fabs(hz) < kn[iz])
                {
                    return static_cast<float32_t>(hz) * wn[iz];
                }
            }
        }
    }

    // ************** Random number generator utilities ****************

	void resetRandGenerators(uint32_t seed)
	{
		srand(seed); // Eigen relies on srand for genering random matrix
        generator.seed(seed);
        r4_nor_setup();
	}

	float64_t randUniform(float64_t const & lo,
	                      float64_t const & hi)
    {
        return lo + r4_uni() * (hi - lo);
    }

	float64_t randNormal(float64_t const & mean,
	                     float64_t const & std)
    {
        return mean + r4_nor() * std;
    }

    vectorN_t randVectorNormal(uint32_t  const & size,
                               float64_t const & mean,
                               float64_t const & std)
    {
        if (std > 0.0)
        {
            return vectorN_t::NullaryExpr(size,
                                        [&mean, &std] (vectorN_t::Index const &) -> float64_t
                                        {
                                            return randNormal(mean, std);
                                        });
        }
        else
        {
            return vectorN_t::Constant(size, mean);
        }
    }

    vectorN_t randVectorNormal(uint32_t  const & size,
                               float64_t const & std)
    {
        return randVectorNormal(size, 0, std);
    }

    vectorN_t randVectorNormal(vectorN_t const & mean,
                               vectorN_t const & std)
    {
        return vectorN_t::NullaryExpr(std.size(),
                                      [&mean, &std] (vectorN_t::Index const & i) -> float64_t
                                      {
                                          return randNormal(mean[i], std[i]);
                                      });
    }

    vectorN_t randVectorNormal(vectorN_t const & std)
    {
        return vectorN_t::NullaryExpr(std.size(),
                                      [&std] (vectorN_t::Index const & i) -> float64_t
                                      {
                                          return randNormal(0, std[i]);
                                      });
    }

    // ******************* Telemetry utilities **********************

    void registerNewVectorEntry(TelemetrySender                & telemetrySender,
                                std::vector<std::string> const & fieldNames,
                                vectorN_t                const & initialValues)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        for (uint32_t i=0; fieldIt != fieldEnd; ++fieldIt, ++i)
        {
            (void) telemetrySender.registerNewEntry<float64_t>(*fieldIt, initialValues[i]);
        }
    }

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           vectorN_t                const & values)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        for (uint32_t i=0; fieldIt != fieldEnd; ++fieldIt, ++i)
        {
            telemetrySender.updateValue(*fieldIt, values[i]);
        }
    }

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           matrixN_t::ConstColXpr           values)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        for (uint32_t i=0; fieldIt != fieldEnd; ++fieldIt, ++i)
        {
            telemetrySender.updateValue(*fieldIt, values[i]);
        }
    }

    std::vector<std::string> defaultVectorFieldnames(std::string const & baseName,
                                                     uint32_t    const & size)
    {
        std::vector<std::string> fieldnames;
        fieldnames.reserve(size);
        for (uint32_t i=0; i<size; i++)
        {
            fieldnames.emplace_back(baseName + std::to_string(i)); // TODO: MR going to support "." delimiter
        }
        return fieldnames;
    }

    std::vector<std::string> removeFieldnamesSuffix(std::vector<std::string>         fieldnames,
                                                    std::string              const & suffix)
    {
        std::transform(fieldnames.begin(),
                       fieldnames.end(),
                       fieldnames.begin(),
                       [&suffix](std::string name) -> std::string
                       {
                           if (!name.compare(name.size() - suffix.size(), suffix.size(), suffix))
                           {
                               name.erase(name.size() - suffix.size(), name.size());
                           }
                           return name;
                       });
        return fieldnames;
    }

    // ********************** Pinocchio utilities **********************

    result_t getJointNameFromPositionId(Model       const & model,
                                        int32_t     const & idIn,
                                        std::string       & jointNameOut)
    {
        result_t returnCode = result_t::ERROR_GENERIC;

        // Iterate over all joints.
        for (int32_t i = 0; i < model.pncModel_.njoints; i++)
        {
            // Get joint starting and ending index in position vector.
            int32_t startIndex = model.pncModel_.joints[i].idx_q();
            int32_t endIndex = startIndex + model.pncModel_.joints[i].nq();

            // If inIn is between start and end, we found the joint we were looking for.
            if(startIndex <= idIn && endIndex > idIn)
            {
                jointNameOut = model.pncModel_.names[i];
                returnCode = result_t::SUCCESS;
                break;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            std::cout << "Error - Utilities::getJointNameFromVelocityId - Position index out of range." << std::endl;
        }

        return returnCode;
    }

    result_t getJointNameFromVelocityId(Model       const & model,
                                        int32_t     const & idIn,
                                        std::string       & jointNameOut)
    {
        result_t returnCode = result_t::ERROR_GENERIC;

        // Iterate over all joints.
        for(int32_t i = 0; i < model.pncModel_.njoints; i++)
        {
            // Get joint starting and ending index in velocity vector.
            int32_t startIndex = model.pncModel_.joints[i].idx_v();
            int32_t endIndex = startIndex + model.pncModel_.joints[i].nv();

            // If inIn is between start and end, we found the joint we were looking for.
            if(startIndex <= idIn && endIndex > idIn)
            {
                jointNameOut = model.pncModel_.names[i];
                returnCode = result_t::SUCCESS;
                break;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            std::cout << "Error - Utilities::getJointNameFromVelocityId - Velocity index out of range." << std::endl;
        }

        return returnCode;
    }

    result_t getJointTypeFromId(Model     const & model,
                                int32_t   const & idIn,
                                joint_t         & jointTypeOut)
    {
        result_t returnCode = result_t::SUCCESS;

        if(model.pncModel_.njoints < idIn - 1)
        {
            std::cout << "Error - Utilities::getJointTypeFromId - Joint id out of range." << std::endl;
            returnCode = result_t::ERROR_GENERIC;
        }

        if (returnCode == result_t::SUCCESS)
        {
            auto joint = model.pncModel_.joints[idIn];

            if (joint.shortname() == "JointModelFreeFlyer")
            {
                jointTypeOut = joint_t::FREE;
            }
            else if (joint.shortname() == "JointModelSpherical")
            {
                jointTypeOut = joint_t::SPHERICAL;
            }
            else if (joint.shortname() == "JointModelPlanar")
            {
                jointTypeOut = joint_t::PLANAR;
            }
            else if (joint.shortname() == "JointModelPX" ||
                     joint.shortname() == "JointModelPY" ||
                     joint.shortname() == "JointModelPZ")
            {
                jointTypeOut = joint_t::LINEAR;
            }
            else if (joint.shortname() == "JointModelRX" ||
                     joint.shortname() == "JointModelRY" ||
                     joint.shortname() == "JointModelRZ")
            {
                jointTypeOut = joint_t::ROTARY;
            }
            else
            {
                // Unknown joint, throw an error to avoid any wrong manipulation.
                jointTypeOut = joint_t::NONE;
                std::cout << "Error - Utilities::getJointTypeFromId - Unknown joint type." << std::endl;
                returnCode = result_t::ERROR_GENERIC;
            }
        }

        return returnCode;
    }

    result_t getJointTypePositionSuffixes(joint_t                  const & jointTypeIn,
                                          std::vector<std::string>       & jointTypeSuffixesOut)
    {
        result_t returnCode = result_t::SUCCESS;

        jointTypeSuffixesOut = std::vector<std::string>({std::string("")}); // If no extra discrimination is needed
        switch (jointTypeIn)
        {
        case joint_t::LINEAR:
            break;
        case joint_t::ROTARY:
            break;
        case joint_t::PLANAR:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("TransX"),
                                                             std::string("TransY"),
                                                             std::string("TransZ")});
            break;
        case joint_t::SPHERICAL:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("QuatX"),
                                                             std::string("QuatY"),
                                                             std::string("QuatZ"),
                                                             std::string("QuatW")});
            break;
        case joint_t::FREE:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("TransX"),
                                                             std::string("TransY"),
                                                             std::string("TransZ"),
                                                             std::string("QuatX"),
                                                             std::string("QuatY"),
                                                             std::string("QuatZ"),
                                                             std::string("QuatW")});
            break;
        case joint_t::NONE:
        default:
            std::cout << "Error - Utilities::getJointFieldnamesFromType - Joints of type 'NONE' do not have fieldnames." << std::endl;
            returnCode = result_t::ERROR_GENERIC;
        }

        return returnCode;
    }

    result_t getJointTypeVelocitySuffixes(joint_t                  const & jointTypeIn,
                                          std::vector<std::string>       & jointTypeSuffixesOut)
    {
        result_t returnCode = result_t::SUCCESS;

        jointTypeSuffixesOut = std::vector<std::string>({std::string("")}); // If no extra discrimination is needed
        switch (jointTypeIn)
        {
        case joint_t::LINEAR:
            break;
        case joint_t::ROTARY:
            break;
        case joint_t::PLANAR:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("LinX"),
                                                             std::string("LinY"),
                                                             std::string("LinZ")});
            break;
        case joint_t::SPHERICAL:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("AngX"),
                                                             std::string("AngY"),
                                                             std::string("AngZ")});
            break;
        case joint_t::FREE:
            jointTypeSuffixesOut = std::vector<std::string>({std::string("LinX"),
                                                             std::string("LinY"),
                                                             std::string("LinZ"),
                                                             std::string("AngX"),
                                                             std::string("AngY"),
                                                             std::string("AngZ")});
            break;
        case joint_t::NONE:
        default:
            std::cout << "Error - Utilities::getJointFieldnamesFromType - Joints of type 'NONE' do not have fieldnames." << std::endl;
            returnCode = result_t::ERROR_GENERIC;
        }

        return returnCode;
    }

    // ********************** Math utilities *************************

    float64_t saturateSoft(float64_t const & in,
                           float64_t const & mi,
                           float64_t const & ma,
                           float64_t const & r)
    {
        float64_t uc, range, middle, bevelL, bevelXc, bevelYc, bevelStart, bevelStop, out;
        float64_t const alpha = M_PI/8;
        float64_t const beta = M_PI/4;

        range = ma - mi;
        middle = (ma + mi)/2;
        uc = 2*(in - middle)/range;

        bevelL = r * tan(alpha);
        bevelStart = 1 - cos(beta)*bevelL;
        bevelStop = 1 + bevelL;
        bevelXc = bevelStop;
        bevelYc = 1 - r;

        if (uc >= bevelStop)
        {
            out = ma;
        }
        else if (uc <= -bevelStop)
        {
            out = mi;
        }
        else if (uc <= bevelStart && uc >= -bevelStart)
        {
            out = in;
        }
        else if (uc > bevelStart)
        {
            out = sqrt(r * r - (uc - bevelXc) * (uc - bevelXc)) + bevelYc;
            out = 0.5 * out * range + middle;
        }
        else if (uc < -bevelStart)
        {
            out = -sqrt(r * r - (uc + bevelXc) * (uc + bevelXc)) - bevelYc;
            out = 0.5 * out * range + middle;
        }
        else
        {
            out = in;
        }
        return out;
    }
}