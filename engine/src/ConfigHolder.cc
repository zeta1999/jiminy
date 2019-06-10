#include <climits>

#include "exo_simu/engine/ConfigHolder.h"


namespace exo_simu
{
    namespace configholder
    {
        Option::Option()
            : type(optiontype::NOT_A_TYPE)
        {
            // Empty.
        }


        Option::Option(optiontype type)
            : type(type)
        {
            // Empty.
        }


        Option::~Option()
        {
            // Empty.
        }
    }  // End of namespace configholder.


    ConfigHolder::ConfigHolder()
        : optionNames_(),
          options_()
    {
        // Empyt.
    }


    ConfigHolder::ConfigHolder(ConfigHolder const& other)
        : optionNames_(other.optionNames_),
          options_()
    {
        for (map_t::const_iterator it = other.options_.begin(); it != other.options_.end(); it++)
        {
            options_[it->first] = it->second->clone();
        }
    }


    ConfigHolder& ConfigHolder::operator=(ConfigHolder const& other)
    {
        optionNames_ = other.optionNames_;

        for (map_t::const_iterator it = other.options_.begin(); it != other.options_.end(); it++)
        {
            options_[it->first] = it->second->clone();
        }

        return *this;
    }


    ConfigHolder::~ConfigHolder()
    {
        for (map_t::iterator it = options_.begin();
             it != options_.end();
             it++)
        {
            delete it->second;
        }
    }


    configholder::optiontype ConfigHolder::getType(std::string const& name) const
    {
        map_t::const_iterator it = options_.find(name);
        WDC_ASSERT(it != options_.end() && "Unknown option");

        return it->second->type;
    }


    std::vector<std::string> const& ConfigHolder::getOptionNames() const
    {
        return optionNames_;
    }


    int32_t ConfigHolder::getSize() const
    {
        size_t size = optionNames_.size();
        WDC_ASSERT(size < std::numeric_limits<int32_t>::max() and "Config holder size overflow");
        return static_cast<int32_t>(size);
    }
}