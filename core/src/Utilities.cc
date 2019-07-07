#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/TelemetrySender.h"


namespace exo_simu
{
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
            fieldnames.emplace_back(baseName + "." + std::to_string(i));
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