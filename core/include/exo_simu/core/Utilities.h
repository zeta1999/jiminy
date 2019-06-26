#ifndef SIMU_UTILITIES_H
#define SIMU_UTILITIES_H

#include <chrono>

#include "exo_simu/core/Types.h"


namespace exo_simu
{
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
	} ;

	void toVectorString(vectorN_t                & data, 
	                    std::vector<std::string> & dataString);

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