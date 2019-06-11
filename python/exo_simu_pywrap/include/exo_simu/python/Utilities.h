///////////////////////////////////////////////////////////////////////////////
///
/// \file              python/Utilities.h
/// \brief             Python utility functions for wdc::dynamicsteller.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_EXO_SIMULATOR_PYTHON_UTILITIES_H
#define WDC_EXO_SIMULATOR_PYTHON_UTILITIES_H

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <exo_simu/engine/ConfigHolder.h>

namespace exo_simu
{
namespace python
{
    namespace bp = boost::python;

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Convert Python list to std::vector.
    ///
    /// \tparam T The object type in the list and the vector.
    ///
    /// \param[in] listIn The Python list.
    ///
    /// \return The converted vector.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    template<typename T>
    inline
    std::vector<T> toStdVector(const bp::list& listPy)
    {
        std::vector<T> v;
        v.reserve(len(listPy));
        for (int32_t i = 0; i < len(listPy); i++)
        {
            v.push_back(bp::extract<T>(listPy[i]));
        }

        return v;
    }
    
    template<typename T>
    inline
    std::vector<std::vector<T>> toStdVectorVector(const bp::list& listPy)
    {
        std::vector<std::vector<T>> v;
        v.reserve(len(listPy));
        for (int32_t i = 0; i < len(listPy); i++)
        {
            v.push_back(toStdVector<T>(bp::extract<bp::list>(listPy[i])));
        }

        return v;
    }

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Convert std::vector to Python list.
    ///
    /// \tparam T The object type in the list and the vector.
    ///
    /// \param[in] vectorIn The std vector.
    ///
    /// \return The converted list.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    template<class T>
    struct VecToList
    {
        static PyObject* convert(const std::vector<T>& vec)
        {
            boost::python::list* l = new boost::python::list();
            for(size_t i = 0; i < vec.size(); i++)
            {
                l->append(vec[i]);
            }

            return l->ptr();
        }
    };

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Convert a 1D python list into an Eigen vector.
    ///////////////////////////////////////////////////////////////////////
    VectorN listPyToVector(bp::list const& listPy)
    {
        VectorN x(len(listPy));
        for (int32_t i = 0; i < len(listPy); i++)
        {
            x(i) = bp::extract<real_t>(listPy[i]);
        }

        return x;
    }

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Convert a 2D python list into an Eigen matrix.
    ///////////////////////////////////////////////////////////////////////
    MatrixN listPyToMatrix(bp::list const& listPy)
    {
        int32_t const nRows = len(listPy);
        assert(nRows > 0 && "empty list");

        int32_t const nCols = len(bp::extract<bp::list>(listPy[0]));
        assert(nCols > 0 && "empty row");

        MatrixN M(nRows, nCols);
        for (int32_t i = 0; i < nRows; i++)
        {
            bp::list const row = bp::extract<bp::list>(listPy[i]);
            assert(len(row) == nCols && "wrong number of columns");
            M.row(i) = listPyToVector(row).transpose();
        }

        return M;
    }

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Convert config holder into a python dictionary.
    ///////////////////////////////////////////////////////////////////////
    void convertConfigHolderPy(ConfigHolder const& config, bp::dict& configPy)
    {
        std::vector<std::string> const options = config.getOptionNames();
        for (int32_t i = 0; i < options.size(); i++)
        {
            std::string const name = options[i];
            switch (config.getType(name))
            {
                case configholder::optiontype::BOOLEAN:
                {
                    configPy[name] = config.get<bool_t>(name);
                    break;
                }
                case configholder::optiontype::INTEGER:
                {
                    configPy[name] = config.get<int32_t>(name);
                    break;
                }
                case configholder::optiontype::REAL:
                {
                    configPy[name] = config.get<real_t>(name);
                    break;
                }
                case configholder::optiontype::STRING:
                {
                    configPy[name] = config.get<std::string>(name);
                    break;
                }
                case configholder::optiontype::EIGEN_VECTOR:
                {
                    VectorN const v = config.get<VectorN>(name);
                    bp::list l;

                    for (int32_t i = 0; i < v.rows(); i++)
                    {
                        l.append(v(i));
                    }
                    configPy[name] = l;
                    break;
                }
                case configholder::optiontype::EIGEN_MATRIX:
                {
                    MatrixN const M = config.get<MatrixN>(name);
                    bp::list l;

                    for (int32_t i = 0; i < M.rows(); i++)
                    {
                        bp::list row;
                        for (int32_t j = 0; j < M.cols(); j++)
                        {
                            row.append(M(i, j));
                        }
                        l.append(row);
                    }
                    configPy[name] = l;
                    break;
                }
                case configholder::optiontype::CONFIG_HOLDER:
                {
                    bp::dict configPyTmp;
                    convertConfigHolderPy(config.get<ConfigHolder>(name), configPyTmp);
                    configPy[name] = configPyTmp;
                    break;
                }
                case configholder::optiontype::NOT_A_TYPE:
                default:
                {
                    assert(false && "Unsupported type");
                    break;
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Load a config holder from a python dictionary.
    ///////////////////////////////////////////////////////////////////////
    void loadConfigHolder(bp::dict const& configPy, ConfigHolder& config)
    {
        std::vector<std::string> const options = config.getOptionNames();
        for (int32_t i = 0; i < options.size(); i++)
        {
            std::string const name = options[i];
            switch (config.getType(name))
            {
                case configholder::optiontype::BOOLEAN:
                {
                    config.get<bool_t>(name) = bp::extract<bool_t>(configPy[name]);
                    break;
                }
                case configholder::optiontype::INTEGER:
                {
                    config.get<int32_t>(name) = bp::extract<int32_t>(configPy[name]);
                    break;
                }
                case configholder::optiontype::REAL:
                {
                    config.get<real_t>(name) = bp::extract<real_t>(configPy[name]);
                    break;
                }
                case configholder::optiontype::STRING:
                {
                    config.get<std::string>(name) = bp::extract<std::string>(configPy[name]);
                    break;
                }
                case configholder::optiontype::EIGEN_VECTOR:
                {
                    config.get<VectorN>(name) = listPyToVector(bp::extract<bp::list>(configPy[name]));
                    break;
                }
                case configholder::optiontype::EIGEN_MATRIX:
                {
                    config.get<MatrixN>(name) = listPyToMatrix(bp::extract<bp::list>(configPy[name]));
                    break;
                }
                case configholder::optiontype::CONFIG_HOLDER:
                {
                    loadConfigHolder(bp::extract<bp::dict>(configPy[name]), config.get<ConfigHolder>(name));
                    break;
                }
                case configholder::optiontype::NOT_A_TYPE:
                default:
                {
                    assert(false && "Unsupported type");
                    break;
                }
            }
        }
    }
}  // end of namespace python.
}  // end of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_UTILITIES_H
