///////////////////////////////////////////////////////////////////////////////
/// WanderBrain
///
/// \brief Contains useful methods and mathematical tools
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_WANDERBRAIN_UTILITIES_TPP_
#define WDC_WANDERBRAIN_UTILITIES_TPP_

namespace exo_simu
{
namespace core
{
    ///////////////////////////////////////////////////////////////////////////
    /// \brief      Constrain a value to be in a specific interval
    ///
    /// \param[in]  valueIn   value to force in a specific interval
    /// \param[in]  bound1    target interval lower bound
    /// \param[in]  bound2    target interval upper bound
    /// \param[out] clampedValue  The constrained value.
    /// \remark     If bound2 is lower than bound1, they are exchanged in the
    ///             function body, for retro-compatibility and safety.
    ///             However, one should not rely on this behaviour.
    /// \retval  error::S_OK  If clamped value equals original value.
    /// \retval  error::E_CLAMPED  If the value was clamped to the limits.
    /// \retval  error::E_NOT_A_NUMBER  If input value was NaN.
    /// \retval  error::E_UTIL  the default error code.
    ///
    ///////////////////////////////////////////////////////////////////////////
    template <typename T>
    void clamp(
        T const& valueIn,
        T const& bound1,
        T const& bound2,
        T& clampedValue)
    {
        // Handle the case where bound1 > bound2 and bound2 < bound1
        T valueMin;
        T valueMax;

        if (bound1 < bound2)
        {
            valueMin = bound1;
            valueMax = bound2;
        }
        else
        {
            valueMin = bound2;
            valueMax = bound1;
        }

        // Filter the output accordingly
        // We use this formulation of inequality so that we can
        // use the generic function for floats too.
        if ((valueMax >= valueIn) && (valueMin <= valueIn))
        {
            clampedValue = valueIn;
        }
        else if (valueMin > valueIn)
        {
            clampedValue = valueMin;
        }
        else if (valueMax < valueIn)
        {
            clampedValue = valueMax;
        }
    }


    template<typename T>
    hresult_t writePODToBuff(
        char_t* const buff,
        std::size_t& posInOut,
        std::size_t const maxSizeIn,
        T const valueIn)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);
        if ((NULL!= buff) && (maxSizeIn >= (posInOut + sizeof(T))))
        {
            std::memcpy(&buff[posInOut], &valueIn, sizeof(T));
            posInOut += sizeof(T);
            returnCode = error::S_OK;
        }
        return returnCode;
    }


    template<typename T>
    hresult_t writePODToFile(std::ofstream& file, T const valueIn)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);
        if (file.good())
        {
            file.write(reinterpret_cast<char_t const * const>(&valueIn), sizeof(T));
            returnCode = error::S_OK;
        }
        return returnCode;
    }


    template<typename ElementType_T, uint32_t BufferSize_T>
    hresult_t snprintfArray(ElementType_T*  values,
                            uint32_t const& size,
                            char_t   const* formatString,
                            char_t          (&buffer)[BufferSize_T])
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);

        if (NULL != values)
        {
            uint32_t nWritten(0U);
            nWritten += snprintf(buffer + nWritten, BufferSize_T - nWritten, "[");

            for(uint32_t i = 0U; i < size; i++)
            {
                if (BufferSize_T > nWritten)
                {
                    nWritten += snprintf(buffer + nWritten,
                                            BufferSize_T - nWritten,
                                            formatString,
                                            values[i]);
                }

                if (size - 1U != i)
                {
                    if (BufferSize_T > nWritten)
                    {
                        nWritten += snprintf(buffer + nWritten,
                                                BufferSize_T - nWritten,
                                                ", ");
                    }
                }
            }

            if (BufferSize_T > nWritten)
            {
                nWritten += snprintf(buffer + nWritten, BufferSize_T - nWritten, "]");
            }

            returnCode = error::S_OK;
        }
        else
        {
            // Write an empty string into the output buffer.
            buffer[0] = '\0';

            returnCode = error::E_EINVAL;
        }

        return returnCode;
    }


    template<typename Scalar_T> bool_t isApprox(Scalar_T const first, Scalar_T const second, Scalar_T const tolerance)
    {
        // Find the largest number.
        Scalar_T const firstAbs = fabs(first);
        Scalar_T const secondAbs = fabs(second);
        Scalar_T largest;
        if (secondAbs > firstAbs)
        {
            largest = secondAbs;
        }
        else
        {
            largest = firstAbs;
        }

        // Check if the numbers are really close (absolute
        // difference).
        Scalar_T const diff = fabs(first - second);
        if (diff < tolerance)
        {
            return true;
        }

        // If the numbers are not too close, compare them using
        // relative difference.
        if (diff < largest * tolerance)
        {
            return true;
        }

        return false;
    }
}
}

#endif  // WDC_WANDERBRAIN_UTILITIES_TPP_
