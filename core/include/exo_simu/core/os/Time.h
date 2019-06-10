///////////////////////////////////////////////////////////////////////////////
///
/// \brief This file contains time related functions.
///
/// \copyright Wandercraft
///
//////////////////////////////////////////////////////////////////////////////
#ifndef WDC_UTIL_TIME_H_
#define WDC_UTIL_TIME_H_

// system library
#include <ctime>
#include <string>

#include "exo_simu/core/Error.h"
#include "exo_simu/core/Constants.h"


namespace exo_simu
{
namespace core
{
    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Reset the shared start time.
    ///////////////////////////////////////////////////////////////////////////
    void resetStartTime();

    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Set the current time in the shared memory (Gazebo only!).
    ///
    /// \param[in]  currentTimeUs  the number of microseconds since UNIX Epoch.
    ///////////////////////////////////////////////////////////////////////////
    void setSharedTime(uint64_t currentTimeUs);

    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Pause the calling thread.
    ///
    /// \param[in]  msTime  the number of milliseconds to sleep
    ///
    /// \return     error::S_OK on success, another error otherwise.
    ///////////////////////////////////////////////////////////////////////////
    hresult_t sleepMs(uint64_t msTime);

#ifdef WDC_TARGET_GAZEBO
    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Pause the calling thread.
    ///
    /// \param[in]  msTime  the number of milliseconds to sleep in simulation time.
    ///
    /// \return     error::S_OK on success, another error otherwise.
    ///////////////////////////////////////////////////////////////////////////
    hresult_t sleepMsSim(uint64_t msTime);
#endif /* WDC_TARGET_GAZEBO */

    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Pause the calling thread.
    ///
    /// \param[in]  usTime  the number of microseconds to sleep
    ///
    /// \return     error::S_OK on success, another error otherwise.
    ///////////////////////////////////////////////////////////////////////////
    hresult_t sleepUs(uint64_t usTime);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Get time elapsed since startTime in microseconds
    ///
    /// \param[in]   startTime  The time since which we measure time elapsed.
    ///
    /// \return   elasped time in us
    ///////////////////////////////////////////////////////////////////////
    uint64_t getElapsedTimeUs(uint64_t const& startTime);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Get time elapsed since startTime in milliseconds
    ///
    /// \param[in]   startTime  The time since which we measure time elapsed.
    ///
    /// \return   elasped time in ms
    ///////////////////////////////////////////////////////////////////////
    uint32_t getElapsedTimeMs(uint64_t const& startTime);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Get time elapsed since startTime in seconds
    ///
    /// \param[in]   startTime  The time since which we measure time elapsed.
    ///
    /// \return   elasped time in seconds
    ///////////////////////////////////////////////////////////////////////
    float64_t getElapsedTimeSeconds(uint64_t const & startTime);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Get time elapsed since UNIX reference time.
    ///
    /// \return   The time in microseconds since UNIX epoch.
    ///////////////////////////////////////////////////////////////////////
    uint64_t getCurrentTimeUs();

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Get time elapsed since UNIX reference time.
    ///
    /// \return   The time in milliseconds since UNIX epoch.
    ///////////////////////////////////////////////////////////////////////
    uint64_t getCurrentTimeMs();

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Get process startup time since UNIX reference time.
    ///
    /// \return   The process start time in microseconds since UNIX epoch.
    ///////////////////////////////////////////////////////////////////////
    uint64_t getStartTimeUs();

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Get process startup time in since UNIX reference time.
    ///
    /// \return   The process start time in milliseconds since UNIX epoch.
    ///////////////////////////////////////////////////////////////////////
    uint64_t getStartTimeMs();

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Convert wdc_time_t to POSIX timespec format
    ///
    /// \param[in]  timeIn     time to convert (in WDC format)
    /// \param[out] timeOut    converted time  (in POSIX timespec format)
    ///////////////////////////////////////////////////////////////////////
    void convertWdcTimeToTimespec(uint64_t const& timeIn, timespec & timeOut);

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Convert POSIX timespec format to micro seconds
    ///
    /// \param[in]  timeIn     time to convert (in POSIX timespec format)
    /// \retval                converted time  (in micro seconds)
    ///////////////////////////////////////////////////////////////////////
    uint64_t convertTimespecToUs(timespec const& timeIn);

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Convert micro seconds to POSIX timespec format.
    ///
    /// \param[in]  timeIn     time to convert (in micro seconds)
    /// \retval                converted time  (in POSIX timespec format)
    ///////////////////////////////////////////////////////////////////////
    timespec convertUsToTimespec(uint64_t time);

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Convert milliseconds to seconds.
    ///
    /// \param[in]  timeIn     time to convert (in milliseconds)
    /// \retval                converted time  (in seconds)
    ///////////////////////////////////////////////////////////////////////
    float64_t ms2Sec(float64_t msTime);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Creates an iso8601 compliant timestamp.
    ///
    ///            This function stores in the argument an iso8601 compliant
    ///            timestamp in the form YYYYMMDDTHHMMSSZ where T is the
    ///            separator between date and hour and Z indicates the time
    ///            is given in UTC.
    ///
    /// \param[out] timeString the pointer where to store the timestamp.
    ///////////////////////////////////////////////////////////////////////
    void getTimestamp(std::string& timeString, uint64_t timeMsIn = getStartTimeMs());

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Wait a flag to be true by polling it. The flag is reset to false
    ///           after a successful detection to true.
    ///
    /// \param  flag             flag to check
    /// \param  deadlineUs       deadline in microseconds
    /// \param  pauseDurationUs  pause between flag reading in microseconds
    ///
    /// \retval  error::S_OK is successful
    /// \retval  error::E_ETIMEDOUT if the flag wasn't detected to true in the elapsed time.
    ///////////////////////////////////////////////////////////////////////
    hresult_t waitFlagUs(bool_t& flag, uint64_t deadlineUs, uint64_t pauseDurationUs);

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Wait a flag to be true by polling it. The flag is reset to false
    ///           after a successful detection to true.
    ///
    /// \param  flag             flag to check
    /// \param  deadlineMs       deadline in milliseconds
    /// \param  pauseDurationMs  pause between flag reading in milliseconds
    ///
    /// \retval  error::S_OK is successful
    /// \retval  error::E_ETIMEDOUT if the flag wasn't detected to true in the elapsed time.
    ///////////////////////////////////////////////////////////////////////
    hresult_t waitFlagMs(bool_t& flag, uint64_t deadlineMs, uint64_t pauseDurationMs);
}
}

#endif  // WDC_UTIL_TIME_H_
