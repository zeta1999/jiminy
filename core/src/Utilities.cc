#include <math.h>
#include <climits>
#include <stdlib.h>     /* srand, rand */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/TelemetrySender.h"


namespace exo_simu
{
    // *********************** Timer **************************

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

    // **************** Random number generator *****************
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

    // ***************** Random number generator utilities **********************

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

    matrixN_t::RowXpr addRandNormal(matrixN_t::RowXpr         vector, 
                                    float64_t         const & mean,
                                    float64_t         const & std)
    {
        vectorN_t noise(vector.size());
        vector += noise.unaryExpr([mean, std](float64_t x) -> float64_t 
                                  {
                                      return randNormal(mean, std);
                                  }); // unaryExpr returns a view. It does not update the original data
        return vector;
    }

    // ********************** Telemetry utilities **********************

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
                           matrixN_t::ConstRowXpr           values)
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
            fieldnames.emplace_back(baseName + "_" + std::to_string(i)); // TODO: MR going to support "." delimiter
        }
        return fieldnames;
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