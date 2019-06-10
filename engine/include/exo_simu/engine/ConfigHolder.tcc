///////////////////////////////////////////////////////////////////////////////
/// \brief    Implementation of the templated interface defined in ConfigHolder.h
///
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#include "exo_simu/core/Error.h"

namespace exo_simu
{
	namespace configholder
	{
		template<typename ValueType_T>
		struct traits
		{
			static const optiontype type = optiontype::NOT_A_TYPE;
		};

		template<>
		struct traits<bool_t>
		{
			static const optiontype type = optiontype::BOOLEAN;
		};

		template<>
		struct traits<int32_t>
		{
			static const optiontype type = optiontype::INTEGER;
		};

		template<>
		struct traits<real_t>
		{
			static const optiontype type = optiontype::REAL;
		};

		template<>
		struct traits<std::string>
		{
			static const optiontype type = optiontype::STRING;
		};

		template<>
		struct traits<VectorN>
		{
			static const optiontype type = optiontype::EIGEN_VECTOR;
		};

		template<>
		struct traits<MatrixN>
		{
			static const optiontype type = optiontype::EIGEN_MATRIX;
		};


		template<typename ValueType_T>
		OptionTpl<ValueType_T>::OptionTpl(optiontype type, ValueType_T const& value)
			: Option(type),
			  value(value)
		{
			// Empty.
		}


		template<typename ValueType_T>
		Option* OptionTpl<ValueType_T>::clone() const
		{
			return new OptionTpl<ValueType_T>(*this);
		}
	}  // End of namespace configholder.


	template<typename ValueType_T>
	void ConfigHolder::addOption(std::string const& name, ValueType_T const& defaultValue)
	{
		map_t::const_iterator it = options_.find(name);
		WDC_ASSERT(it == options_.end() && "Error: option already exists.");

		optionNames_.push_back(name);
		options_[name] = new configholder::OptionTpl<ValueType_T>(configholder::traits<ValueType_T>::type, defaultValue);
	}


	template<typename ValueType_T>
	ValueType_T& ConfigHolder::get(std::string const& name)
	{
		map_t::iterator it = options_.find(name);
		WDC_ASSERT(it != options_.end() && "Unknown option");
		WDC_ASSERT(it->second->type == configholder::traits<ValueType_T>::type && "Error: option used with wrong type");

		configholder::OptionTpl<ValueType_T>* opt = static_cast< configholder::OptionTpl<ValueType_T>* >(it->second);
		WDC_ASSERT(opt != NULL && "Error: option used with the wrong type");

		return opt->value;
	}


	template<typename ValueType_T>
	ValueType_T const& ConfigHolder::get(std::string const& name) const
	{
		map_t::const_iterator it = options_.find(name);
		WDC_ASSERT(it != options_.end() && "Unknown option");
		WDC_ASSERT(it->second->type == configholder::traits<ValueType_T>::type && "Error: option used with wrong type");

		configholder::OptionTpl<ValueType_T>* opt = static_cast< configholder::OptionTpl<ValueType_T>* >(it->second);
		WDC_ASSERT(opt != NULL && "Error: option used with the wrong type");

		return opt->value;
	}
}