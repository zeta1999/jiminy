///////////////////////////////////////////////////////////////////////////////
/// \brief    Class to hold config data of heterogeneous types.
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_EXO_SIMULATOR_CONFIG_HOLDER_H
#define WDC_EXO_SIMULATOR_CONFIG_HOLDER_H

#include "exo_simu/engine/Types.h"

namespace exo_simu
{
    namespace configholder
    {
        ///////////////////////////////////////////////////////////////////////
        /// \brief      Available types for the options.
        ///////////////////////////////////////////////////////////////////////
        enum class optiontype
        {
            NOT_A_TYPE,
            BOOLEAN,
            INTEGER,
            REAL,
            STRING,
            EIGEN_VECTOR,
            EIGEN_MATRIX
        };

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Struct used internally by the ConfigHolder to hold the value of an option.
        ///
        /// \details    This is the abstract class, derived class implement option for concrete types.
        ///////////////////////////////////////////////////////////////////////
        struct Option
        {
            optiontype type;

            Option();
            Option(optiontype type);
            virtual ~Option();
            virtual Option* clone() const = 0;
        };

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Struct used internally by the ConfigHolder to hold the value of an option.
        ///
        /// \details    This templated class allows implementing option for concrete types.
        ///////////////////////////////////////////////////////////////////////
        template<typename ValueType_T>
        struct OptionTpl : public Option
        {
            ValueType_T value;  ///< Option value.

            OptionTpl(optiontype type, ValueType_T const& value);
            virtual Option* clone() const;
        };
    }  // End of namespace configholder.

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Config holder.
    ///////////////////////////////////////////////////////////////////////
    class ConfigHolder
    {
    public:
        ///////////////////////////////////////////////////////////////////////
        /// \brief      Construtor.
        ///////////////////////////////////////////////////////////////////////
        ConfigHolder();

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Copy construtor.
        ///////////////////////////////////////////////////////////////////////
        ConfigHolder(ConfigHolder const& other);

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Copy-assignment operator.
        ///////////////////////////////////////////////////////////////////////
        ConfigHolder& operator=(ConfigHolder const& other);

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Destructor.
        ///////////////////////////////////////////////////////////////////////
        ~ConfigHolder();

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Create an option.
        ///
        /// \param[in]  name            Name of the option.
        /// \param[in]  defaultValue    Default value of the option.
        ///////////////////////////////////////////////////////////////////////
        template<typename ValueType_T>
        void addOption(std::string const& name, ValueType_T const& defaultValue);

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get the value of an option.
        ///
        /// \param[in]  name            Name of the option.
        /// \return                     Value of the option.
        ///////////////////////////////////////////////////////////////////////
        template<typename ValueType_T>
        ValueType_T& get(std::string const& name);

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get a reference to the value of an option.
        ///
        /// \param[in]  name            Name of the option.
        /// \return                     Reference to the value of the option.
        ///////////////////////////////////////////////////////////////////////
        template<typename ValueType_T>
        ValueType_T const& get(std::string const& name) const;

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get the type of an option.
        ///
        /// \param[in]  name            Name of the option.
        /// \return                     Type of the option.
        ///////////////////////////////////////////////////////////////////////
        configholder::optiontype getType(std::string const& name) const;

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get the names of all the options.
        ///
        /// \return     Option names.
        ///////////////////////////////////////////////////////////////////////
        std::vector<std::string> const& getOptionNames() const;

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get the number of options.
        ///
        /// \return     Number of options.
        ///////////////////////////////////////////////////////////////////////
        int32_t getSize() const;

    private:
        typedef std::map<std::string, configholder::Option*> map_t;
        std::vector<std::string> optionNames_;  ///< List of option names.
        map_t options_;  ///< Option values by name.
    };
}

#include "exo_simu/engine/ConfigHolder.tcc"

#endif  // WDC_EXO_SIMULATOR_CONFIG_HOLDER_H
