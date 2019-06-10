///////////////////////////////////////////////////////////////////////////////
///
/// \brief This file contains time related functions.
///
/// \copyright Wandercraft
///
///////////////////////////////////////////////////////////////////////////////

// system header
#include <errno.h>
#include <cstring>

// Associated header
#include "exo_simu/core/Error.h"
#include "exo_simu/core/os/Time.h"
#include "exo_simu/core/os/SharedMemory.h"
#include "exo_simu/core/Logger.h"


namespace exo_simu
{
namespace core
{
    struct SharedTime
    {
        uint64_t startTimeUs;
        uint64_t currentTimeUs;
    };


    struct SharedTime* getSharedTime()
    {
        static SharedMemory shm("WanderTime", sizeof(struct SharedTime));

        hresult_t const rcCreate = shm.create();
        WDC_REPORT_ERROR(rcCreate);

        struct SharedTime* shmTime = static_cast<struct SharedTime*>(shm.address());
        return shmTime;
    }


    void resetStartTime()
    {
        getSharedTime()->startTimeUs = getCurrentTimeUs();
    }


    void setSharedTime(uint64_t currentTimeUs)
    {
        getSharedTime()->currentTimeUs = currentTimeUs;
    }


    hresult_t sleepMs(uint64_t msTime)
    {
        return sleepUs(msTime * MICROSECONDS_IN_MILLISECOND);
    }


#ifdef WDC_TARGET_GAZEBO
    hresult_t sleepMsSim(uint64_t msTime)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);
        uint64_t targetSimTime = getCurrentTimeMs() + msTime;
        while (targetSimTime >= getCurrentTimeMs())
        {
            returnCode = sleepUs(500U);
            if (returnCode != error::S_OK)
            {
                break;
            }
        }

        return returnCode;
    }
#endif /* WDC_TARGET_GAZEBO */


    hresult_t sleepUs(uint64_t usTime)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);

        // Store remaining time to wait.
        timespec remainingTime = convertUsToTimespec(usTime);
        for (;;)
        {
            // New required time (requiredTime) is the last remaining time (remainingTime)
            timespec requiredTime = remainingTime;

            // Call nanosleep with required time (requiredTime), tell it to store
            // potentially remaining time in remainingTime. result is EINTR if nanosleep got
            // a signal exit and remainingTime is then set.
            int32_t const result = clock_nanosleep(CLOCK_REALTIME, 0, &requiredTime, &remainingTime);
            if (0 == result)
            {
                return error::S_OK;
            }
            else
            {
                if (EINTR == result)
                {
                    // Nanosleep was interrupted by a POSIX signal: must sleep again.
                    continue;
                }
                else
                {
                    returnCode = error::errnoToHresult(result);
                    break;
                }
            }
        }

        return returnCode;
    }


    uint32_t getElapsedTimeMs(uint64_t const & startTime)
    {
        uint64_t const currentTime = getCurrentTimeUs();
        uint64_t const diff = currentTime - startTime ;

        return static_cast<uint32_t>(diff / MICROSECONDS_IN_MILLISECOND);
    }


    uint64_t getElapsedTimeUs(uint64_t const& startTime)
    {
        return (getCurrentTimeUs() - startTime);
    }


    float64_t getElapsedTimeSeconds(uint64_t const & startTime)
    {
        float64_t elapsedTime = static_cast<float64_t>(getElapsedTimeUs(startTime));
        return (elapsedTime / static_cast<float64_t>(MICROSECONDS_IN_SECOND));
    }


    uint64_t getCurrentTimeUs()
    {
#ifdef WDC_TARGET_GAZEBO
        return getSharedTime()->currentTimeUs;
#else
        timespec now = {0, 0};
        uint64_t nowInUs = 0U;

        // System call
        int32_t const rcClockGetTime = clock_gettime(CLOCK_REALTIME, &now);
        if (-1 == rcClockGetTime)
        {
            WDC_REPORT_EMERGENCY(error::errnoToHresult(errno));
        }
        else
        {
            nowInUs = now.tv_sec * MICROSECONDS_IN_SECOND +
                        now.tv_nsec / NANOSECONDS_IN_MICROSECOND;
        }

        return nowInUs;
#endif
    }


    uint64_t getCurrentTimeMs()
    {
        return (getCurrentTimeUs() / MICROSECONDS_IN_MILLISECOND);
    }


    uint64_t getStartTimeUs()
    {
        return getSharedTime()->startTimeUs;
    }


    uint64_t getStartTimeMs()
    {
        return (getStartTimeUs() / MICROSECONDS_IN_SECOND);
    }


    void convertWdcTimeToTimespec(uint64_t const& timeIn, timespec & timeOut)
    {
        timeOut.tv_sec  = static_cast<std::time_t>(timeIn / MICROSECONDS_IN_SECOND);

        uint64_t nsec   = timeIn * NANOSECONDS_IN_MICROSECOND;
        nsec -= timeOut.tv_sec * NANOSECONDS_IN_SECOND;
        timeOut.tv_nsec = static_cast<int32_t>(nsec);
    }


    void getTimestamp(std::string& timeString, uint64_t timeMsIn)
    {
        // Default value to handle error.
        timeString = "00001122T334455Z";

        std::time_t logNameTime = static_cast<std::time_t>(timeMsIn);
        char_t buffer[TIMESTAMP_LENGTH];

        // Paste timestamp (YYYYMMDDTHHMMSSZ) following iso8601
        std::tm brokenDownTime;
        if(NULL == gmtime_r(&logNameTime, &brokenDownTime))
        {
            return;
        }

        std::size_t const rcStrftime = std::strftime(
            buffer,
            TIMESTAMP_LENGTH,
            "%Y%m%dT%H%M%SZ",
            &brokenDownTime);
        if (0U == rcStrftime)
        {
            return;
        }

        timeString = buffer;
    }


    uint64_t convertTimespecToUs(timespec const& timeIn)
    {
        uint64_t time = timeIn.tv_sec * MICROSECONDS_IN_SECOND
                    + timeIn.tv_nsec / NANOSECONDS_IN_MICROSECOND;
        return time;
    }


    timespec convertUsToTimespec(uint64_t time)
    {
        timespec ts;
        ts.tv_sec = time / MICROSECONDS_IN_SECOND;

        time = time % MICROSECONDS_IN_SECOND;
        ts.tv_nsec = time * NANOSECONDS_IN_MICROSECOND;

        return ts;
    }


    hresult_t waitFlagUs(bool_t& flag, uint64_t deadlineUs, uint64_t pauseDurationUs)
    {
        uint64_t const now = getCurrentTimeUs();
        while (not flag)
        {
            if (getElapsedTimeUs(now) > deadlineUs)
            {
                return error::E_ETIMEDOUT;
            }

            sleepUs(pauseDurationUs);
        }

        // Reset flag.
        flag = false;
        return error::S_OK;
    }


    hresult_t waitFlagMs(bool_t& flag, uint64_t deadlineMs, uint64_t pauseDurationMs)
    {
        return waitFlagUs(flag,
                            deadlineMs * MICROSECONDS_IN_MILLISECOND,
                            pauseDurationMs * MICROSECONDS_IN_MILLISECOND);
    }

    float64_t ms2Sec(float64_t msTime)
    {
        return SECONDS_IN_MILLISOND * msTime;
    }
}
}
