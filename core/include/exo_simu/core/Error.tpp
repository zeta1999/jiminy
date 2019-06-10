///////////////////////////////////////////////////////////////////////////////
/// \copyright         Wandercraft
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_DEFINITION_ERROR_TPP
#define WDC_DEFINITION_ERROR_TPP

#include <sstream>
#include <iomanip>
#include <cstring>


namespace exo_simu
{
    template<typename T1, typename T2, typename T3, typename T4>
    void reportError(error::level::Enum levelIn, std::string const& whereIn, hresult_t errorIn, int32_t lineIn,
                     std::string const& legend1, T1 val1,
                     std::string const& legend2, T2 val2,
                     std::string const& legend3, T3 val3,
                     std::string const& legend4, T4 val4)
    {
        std::string level;
        uint32_t counter;
        if (not error::processError(levelIn, errorIn, level, counter))
        {
            return;
        }

        // Truncate where string if needed.
        std::string where(whereIn);
        if (whereIn.size() > error::WHERE_LENGTH)
        {
            where = whereIn.substr(whereIn.size() - error::WHERE_LENGTH);
        }

        std::stringstream customValues;
        customValues << std::setprecision(6);
        customValues << legend1 << " " << val1;
        customValues << " - ";
        customValues << legend2 << " " << val2;
        customValues << " - ";
        customValues << legend3 << " " << val3;
        customValues << " - ";
        customValues << legend4 << " " << val4;

        char_t errorMessage[error::ERROR_MESSAGE_LENGTH];
        std::fill_n(errorMessage, error::ERROR_MESSAGE_LENGTH, ' ');
        //std::memset(errorMessage, ' ', error::ERROR_MESSAGE_LENGTH);
        errorMessage[error::ERROR_MESSAGE_LENGTH-1] = '\0';

        int32_t end = snprintf(errorMessage, error::ERROR_MESSAGE_LENGTH,
                                "\n#### %s #### <%04d> - hex: 0x%04x - str: %s\n"
                                "    Reported by: %s:%d\n"
                                "    Details:\n    %s\n",
                                level.c_str(), counter, errorIn, error::getErrorString(errorIn).c_str(),
                                where.c_str(), lineIn,
                                customValues.str().c_str()
                                );
        errorMessage[end] = ' '; // we want to record *always* error::ERROR_MESSAGE_LENGTH bytes.

        error::printError(errorMessage);
    }
}

#endif

