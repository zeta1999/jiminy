///////////////////////////////////////////////////////////////////////////////
/// WanderBrain
///
/// \brief  Contains useful methods and mathematical tools
///
/// \version 1
/// \copyright Wandercraft
///
///////////////////////////////////////////////////////////////////////////////


// Associated header
#include "exo_simu/core/Error.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Logger.h"

// System includes
#include <cstdio>
#include <sys/file.h>
#include <sstream>
#include <string>

namespace exo_simu
{
namespace core
{
    static int32_t lockFilefd = -1;

    hresult_t acquireLockFile(std::string const & fileIn)
    {
        hresult_t returnCode(error::S_OK);

        lockFilefd = open(fileIn.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
        if (lockFilefd < 0)
        {
            returnCode = error::errnoToHresult(errno);
        }

        return returnCode;
    }


    hresult_t releaseLockFile(std::string const & fileIn)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);
        if (lockFilefd >= 0)
        {
            int32_t const rcClose = std::remove(fileIn.c_str());
            if (0 == rcClose)
            {
                lockFilefd = -1;
                returnCode = error::S_OK;
            }
        }
        else
        {
            returnCode = error::S_OK; // silent pass
        }

        return returnCode;
    }


    std::string convertIntToString(int32_t const value)
    {
        std::ostringstream tmpStream;
        tmpStream << value;
        return tmpStream.str();
    }


    std::string buildFirmwareVersion(uint32_t const version)
    {
        // Decode the fw version vX.Y.Z from the uint32_t field.
        // The decimal value of the field is: xx0yyy0zzz. ie 0100020003 is v1.2.3
        uint32_t const vZ = version % 1000U;
        uint32_t const vY = (version / 10000U) % 1000U;
        uint32_t const vX = (version / 100000000U) % 100U;
        std::ostringstream s;
        s << "v" << vX << "." << vY << "." << vZ;
        return s.str();
    }


    std::string convertFloatToString(float64_t const value)
    {
        std::ostringstream tmpStream;
        tmpStream << value;
        return tmpStream.str();
    }


    int32_t convertStringToInt(std::string const & stringIn)
    {
        int32_t integer(0);
        std::istringstream convertedValue(stringIn);
        if (not (convertedValue >> integer))
        {
            integer = 0;
        }
        return integer;
    }


    float64_t ticksToRad(int32_t const ticks, float64_t encoderResolution)
    {
        return ( static_cast<float64_t>(ticks) * 2.0 * M_PI) / encoderResolution ;
    }


    int32_t radToTicks(float64_t const angle, float64_t encoderResolution)
    {
        return static_cast<int32_t>((angle * encoderResolution) / (2.0 * M_PI));
    }


    template<> bool_t isApprox(float64_t const first, float64_t const second, float64_t const maxDiff);
}
}
