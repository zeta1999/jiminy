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
}

#endif  // SIMU_UTILITIES_H