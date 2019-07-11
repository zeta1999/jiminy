#include <math.h>
#include <stdlib.h>     /* srand, rand */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/TelemetrySender.h"


namespace exo_simu
{
    std::default_random_engine generator; // Global random number generator
    std::uniform_real_distribution<float64_t> distributionNormal(0.0, 1.0);
    std::uniform_real_distribution<float64_t> distributionUniform(0.0, 1.0);

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

	void resetRandGenerators(uint32_t seed)
	{
		srand(seed); // Eigen relies on srand for genering random matrix
		generator.seed(seed);
	}

	float64_t randUniform(float64_t const & lo, 
	                      float64_t const & hi)
    {
        return lo + distributionUniform(generator) * (hi - lo);
    }

	float64_t randNormal(float64_t const & mean, 
	                     float64_t const & std)
    {
        return mean + distributionNormal(generator) * std;
    }

    matrixN_t::RowXpr addWhiteNoise(matrixN_t::RowXpr vector, 
                                    float64_t const & std)
    {
        vectorN_t noise(vector.size());
        vector += noise.unaryExpr([std](float64_t x) -> float64_t 
                                  {
                                      return randNormal(0, std);
                                  }); // unaryExpr returns a view. It does not update the original data
        return vector;
    }

    void registerNewVectorEntry(TelemetrySender                & telemetrySender,
                                std::vector<std::string> const & fieldNames,
                                vectorN_t                const & initialValues)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        float64_t const * valueIt = initialValues.data();
        for (; fieldIt != fieldEnd; ++fieldIt, ++valueIt)
        {
            (void) telemetrySender.registerNewEntry<float64_t>(*fieldIt, *valueIt);
        }
    }

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           vectorN_t                const & values)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        float64_t const * valueIt = values.data();
        for (; fieldIt != fieldEnd; ++fieldIt, ++valueIt)
        {
            telemetrySender.updateValue(*fieldIt, *valueIt);
        }
    }

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           matrixN_t::ConstRowXpr           values)
    {
        std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
        std::vector<std::string>::const_iterator fieldEnd = fieldNames.end();
        float64_t const * valueIt = values.data();
        for (; fieldIt != fieldEnd; ++fieldIt, ++valueIt)
        {
            telemetrySender.updateValue(*fieldIt, *valueIt);
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