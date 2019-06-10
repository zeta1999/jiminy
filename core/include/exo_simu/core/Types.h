///////////////////////////////////////////////////////////////////////////////
///
/// \file              Types.h
/// \brief             Define types.
/// \details           Define main types for use in wandercraft code.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_DEFINITION_TYPES_H
#define WDC_DEFINITION_TYPES_H

#include <cstdint>  // C99 fixed size types.
#include <cstddef>   // Various standard specific types (size_t, NULL, nullptr, etc).

namespace exo_simu
{
    //////////////////////////////////////////////////////////////////////////////
    ///
    /// \typedef Define main types for use in wandercraft code.
    ///
    //////////////////////////////////////////////////////////////////////////////
    typedef bool            bool_t;
    typedef char            char_t;
    typedef float           float32_t;
    typedef double          float64_t;

    //////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Type definition for hresult_t.
    ///
    /// \details When viewed as a hexadecimal number, hresult_t is
    ///          divided into three parts: The most significant nibble
    ///          is set to 0 in case of success or 8 in case of error,
    ///          the three following nibbles identify the facility,
    ///          and the last four nibbles identify specific error for
    ///          the facility. The facility codes are unique and
    ///          listed in Facilities.h. The error codes for a given
    ///          facility are listed in each module or class file.
    ///
    //////////////////////////////////////////////////////////////////////////////
    typedef uint32_t hresult_t;

    //////////////////////////////////////////////////////////////////////////////
    /// \brief Disable the copy of the class
    /// \param[in] className  Name of the class on which to disable copy.
    //////////////////////////////////////////////////////////////////////////////
    #define DISABLE_COPY(className) \
        className(className const&) = delete; \
        className& operator=(className const&) = delete
}

#endif  // WDC_DEFINITION_TYPES_H
