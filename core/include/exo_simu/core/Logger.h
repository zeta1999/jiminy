#ifndef SIMU_LOGGER_H
#define SIMU_LOGGER_H

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    class Model;

    namespace logger
    {
        std::string const SEPARATOR = ", ";
    }

    class Logger
    {
    public:
        Logger(void);
        ~Logger(void);

        result_t initialize(Model const & model);

        std::string get(bool const & isHeaderEnable = true, 
                        bool const & isDataEnable = true) const;
        result_t fetch(void);
        void clear(void);

        bool getIsInitialized(void) const;

    private:
        static void fetchStringVector(std::vector<std::string> const & strings, 
                                      std::stringstream              & stream);

    private:
        bool isInitialized_;
        std::stringstream data_;
        std::stringstream headers_;
        std::vector<uint32_t> headersSizes_;
        std::shared_ptr<sensorsMap_t const> sensorsPtr_;
    };
}

#endif //end of SIMU_LOGGER_H