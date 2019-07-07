#ifndef SIMU_UTILITIES_H
#define SIMU_UTILITIES_H

#include <chrono>
#include <vector>

#include "exo_simu/core/Types.h"


namespace exo_simu
{
	class TelemetrySender;

	class Timer
	{
		typedef std::chrono::high_resolution_clock Time;

	public:
		Timer(void);
		void tic(void);
		void toc(void);

	public:
		std::chrono::time_point<Time> t0;
		std::chrono::time_point<Time> tf;
		float32_t dt;
	};

    void registerNewVectorEntry(TelemetrySender                & telemetrySender,
                                std::vector<std::string> const & fieldNames,
                                vectorN_t                const & initialValues);

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           vectorN_t                const & values);
    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           matrixN_t::ConstRowXpr           values);

    std::vector<std::string> defaultVectorFieldnames(std::string const & baseName, 
                                                     uint32_t    const & size);

    float64_t saturateSoft(float64_t const & in,
                           float64_t const & mi,
                           float64_t const & ma,
                           float64_t const & r);

	template<typename KeyType, typename ValueType>
	std::vector<ValueType> getMapValues(std::map<KeyType, ValueType> m);

	template<typename typeOut, typename typeIn>
	std::vector<std::shared_ptr<typeOut>> staticCastSharedPtrVector(std::vector<std::shared_ptr<typeIn>> vIn);
}

#include "exo_simu/core/Utilities.tcc"

#endif  // SIMU_UTILITIES_H