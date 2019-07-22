#ifndef SIMU_UTILITIES_H
#define SIMU_UTILITIES_H

#include <chrono>
#include <vector>
#include <random>

#include "exo_simu/core/Types.h"


namespace exo_simu
{
	class TelemetrySender;
	class Model;

    // ************************ Timer *****************************

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

    // ************ Random number generator utilities **************

	void resetRandGenerators(uint32_t seed);

	float64_t randUniform(float64_t const & lo,
	                      float64_t const & hi);

	float64_t randNormal(float64_t const & mean,
	                     float64_t const & std);

	vectorN_t randVectorNormal(uint32_t  const & size,
                               float64_t const & mean,
                               float64_t const & std);

	vectorN_t randVectorNormal(uint32_t  const & size,
                               float64_t const & std);

	vectorN_t randVectorNormal(vectorN_t const & std);

	vectorN_t randVectorNormal(vectorN_t const & mean,
                               vectorN_t const & std);

    // ******************* Telemetry utilities **********************

    void registerNewVectorEntry(TelemetrySender                & telemetrySender,
                                std::vector<std::string> const & fieldNames,
                                vectorN_t                const & initialValues);

    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           vectorN_t                const & values);
    void updateVectorValue(TelemetrySender                & telemetrySender,
                           std::vector<std::string> const & fieldNames,
                           matrixN_t::ConstColXpr           values);

    std::vector<std::string> defaultVectorFieldnames(std::string const & baseName,
                                                     uint32_t    const & size);

	std::vector<std::string> removeFieldnamesSuffix(std::vector<std::string>         fieldnames, // Make a copy
                                                       std::string              const & suffix);

    // ******************** Pinocchio utilities *********************

    // Pinocchio joint types
    enum class joint_t : int32_t
    {
        // CYLINDRICAL are not available so far

        NONE = 0,
		LINEAR = 1,
        ROTARY = 2,
		PLANAR = 3,
        SPHERICAL = 4,
        FREE = 5,
    };

    result_t getJointNameFromPositionId(Model       const & model,
                                        int32_t     const & idIn,
                                        std::string       & jointNameOut);

    result_t getJointNameFromVelocityId(Model       const & model,
                                        int32_t     const & idIn,
                                        std::string       & jointNameOut);

    result_t getJointTypeFromId(Model     const & model,
                                int32_t   const & idIn,
                                joint_t         & jointTypeOut);

	result_t getJointTypePositionSuffixes(joint_t                  const & jointTypeIn,
								          std::vector<std::string>       & jointTypeSuffixesOut);

	result_t getJointTypeVelocitySuffixes(joint_t                  const & jointTypeIn,
							              std::vector<std::string>       & jointTypeSuffixesOut);

    // ********************** Math utilities *************************

    float64_t saturateSoft(float64_t const & in,
                           float64_t const & mi,
                           float64_t const & ma,
                           float64_t const & r);

    // *********************** Miscellaneous **************************

	template<typename KeyType, typename ValueType>
	std::vector<ValueType> getMapValues(std::map<KeyType, ValueType> m);

	template<typename typeOut, typename typeIn>
	std::vector<std::shared_ptr<typeOut>> staticCastSharedPtrVector(std::vector<std::shared_ptr<typeIn>> vIn);

	template<class F, class dF=std::decay_t<F> >
	auto not_f(F&& f);
}

#include "exo_simu/core/Utilities.tcc"

#endif  // SIMU_UTILITIES_H