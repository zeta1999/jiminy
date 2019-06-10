///////////////////////////////////////////////////////////////////////////////
///
/// \brief             Python module implementation for ExoSimulator.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>

#include "exo_simu/python/ExoSimulator.h"

BOOST_PYTHON_MODULE(libexo_simu_pywrap)
{
    eigenpy::enableEigenPy();

    exo_simu::python::ExoSimulatorVisitor::expose();
}