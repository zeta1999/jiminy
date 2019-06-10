///////////////////////////////////////////////////////////////////////////////
/// WanderBrain
///
/// \brief Contains useful methods and mathematical tools
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_WANDERBRAIN_UTILITIES_H_
#define WDC_WANDERBRAIN_UTILITIES_H_

// System includes
#include <ctime>
#include <cstring>
#include <limits>
#include <fstream>
#include <string>
#include <cmath>

// Internal includes
#include "exo_simu/core/Error.h"
#include "exo_simu/core/Constants.h"

namespace exo_simu
{
namespace core
{
    ///////////////////////////////////////////////////////////////////////
    /// \brief     Opens a uniquely named file called "/tmp/wdc_lock"
    ///
    ///            This function allows an application to run only once
    ///            by acquiring a lock on a file.
    ///            The application must call releaseLockFile() on exit.
    ///
    ///
    /// \retval    error::S_OK  on success
    /// \retval    error::E_UTIL if lock is already opened
    ///////////////////////////////////////////////////////////////////////
    hresult_t acquireLockFile(std::string const & fileIn);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     release the lock file properly so that the next instance can run.
    ///
    ///
    /// \retval    error::S_OK  on success
    /// \retval    error::E_UTIL if lock is already opened
    /// \see       acquireLockFile()
    ///////////////////////////////////////////////////////////////////////
    hresult_t releaseLockFile(std::string const & fileIn);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Write the a POD type value to a buffer
    ///
    /// \details   This function will do raw write of the memory value
    ///            of the data to a buffer
    ///            It must be used only on POD (plain old data) type variables.
    ///
    ///
    /// \param[in] buff the buffer to which store the data.
    ///
    /// \param[in] posInOut the position in the buffer where to store the data, it is incremented by
    ///            the size written.
    /// \param[in] maxSizeIn the size of the buffer. To avoid overflow.
    /// \param[in] valueIn the value (struct or basic type) to store in the buffer.
    /// \retval  error::E_UTIL if the buffer is full or null
    /// \retval  error::S_OK  on success
    /////////////////////////////////////////////////////////////////////
    template<typename T>
    hresult_t writePODToBuff(char_t* const buff,
                                std::size_t& posInOut,
                                std::size_t const maxSizeIn,
                                T const valueIn);

    ///////////////////////////////////////////////////////////////////////
    /// \brief     Write the a POD type value to file as binary
    ///
    /// \details   This function will do raw write of the memory value
    ///            of the data to a file.
    ///            It must be used only on POD (plain old data) type variables.
    ///
    ///
    /// \param[in]   file the file to write the data to.
    ///
    /// \param[in]   valueIn  the value to store in the file.
    ///
    /// \retval  error::E_UTIL if the file is invalid.
    /// \retval  error::S_OK  on success
    /////////////////////////////////////////////////////////////////////
    template<typename T>
    hresult_t writePODToFile(std::ofstream& file, T const valueIn);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// \brief Convert int to a string.
    /// \retval string contraining the formated value
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string convertIntToString(int32_t const value);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// \brief Convert float to a string.
    /// \retval string contraining the formated value
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string convertFloatToString(float64_t const value);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// \brief Convert string to a int.
    /// \retval int contraining the formated value
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    int32_t convertStringToInt(std::string const & stringIn);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /// \brief Convert a firmware version formatted on an uint32_t
    ///        Firmware version from ankle and pelvis boards are encoded inside an uint32_t sdo.
    ///        The decimal value is XX0YY0ZZZ for a firmware version vXX.YY.ZZZ
    /// \retval the string representation of firmware version in format "vXX.YY.ZZZ"
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string buildFirmwareVersion(uint32_t const version);

    /// \brief Util methods for easy conversion between radians and degrees.
    constexpr float64_t rad2Deg(float64_t radians) {return (radians * 180.0) / M_PI;}
    constexpr float64_t deg2Rad(float64_t degrees) {return (degrees * M_PI) / 180.0;}

    /// \brief Convert motor position in ticks to radians.
    float64_t ticksToRad(int32_t const ticks, float64_t encoderResolution);

    /// \brief Convert motor position in radians to ticks.
    int32_t radToTicks(float64_t const angle, float64_t encoderResolution);

    //////////////////////////////////////////////////////////////////////////////////////////////
    /// \brief Compare two scalar values for approximate equality.
    ///
    /// \param[in] first First value to compare.
    /// \param[in] second Seconf value to compare.
    /// \param[in] tolerance Tolerance for value comparison.
    ///
    /// \details If first and second value are too close, tolerance
    ///          argument is used as an absolute tolerance. Otherwise,
    ///          it is used as a relative tolerance.
    ///
    /// \retval true if first and second are approximately equal.
    /// \retval false otherwise
    ///
    /// \ref
    /// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename Scalar_T> bool_t isApprox(
        Scalar_T const first,
        Scalar_T const second,
        Scalar_T const tolerance = std::numeric_limits<Scalar_T>::epsilon());
}
}

#include "Utilities.tpp"

#endif  // WDC_WANDERBRAIN_UTILITIES_H_
