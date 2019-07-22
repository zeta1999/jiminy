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
    std::vector<T> toStdVector(bp::list const & listPy)
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
    std::vector<std::vector<T>> toStdVectorVector(bp::list const & listPy)
    {
        std::vector<std::vector<T>> v;
        v.reserve(len(listPy));
        for (int32_t i = 0; i < len(listPy); i++)
        {
            v.push_back(toStdVector<T>(bp::extract<bp::list>(listPy[i])));
        }

        return v;
    }


    /////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Utilities to convert scalars and Eigen vector/matrix to Python Numpy
    ///        array by reference.
    ///
    /////////////////////////////////////////////////////////////////////////////////////

    enum pyVector_t
    {
        vector,
        matrixCol,
        matrixRow
    };

    int getPyType(bool & data)
    {
        return NPY_BOOL;
    }

    int getPyType(float64_t & data)
    {
        return NPY_DOUBLE;
    }

    template<typename T>
    PyObject * getPyReferenceFromScalar(T & data)
    {
        npy_intp dims[1] = {npy_intp(1)};
        return PyArray_SimpleNewFromData(1, dims, getPyType(data), &data);
    }

    PyObject * getPyReferenceFromVector(vectorN_t  & data,
                                        pyVector_t   type = pyVector_t::vector)
    {
        if (type == pyVector_t::vector)
        {
            npy_intp dims[1] = {npy_intp(data.size())};
            return PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, data.data());
        }
        else
        {
            npy_intp dims[2] = {npy_intp(1), npy_intp(data.size())};
            PyObject * pyData = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, data.data());

            if (type == pyVector_t::matrixCol)
            {
                return PyArray_Transpose(reinterpret_cast<PyArrayObject *>(pyData), NULL);
            }
            else
            {
                return pyData;
            }
        }
    }

    PyObject * getPyReferenceFromVector(vectorN_t  const & data,
                                        pyVector_t         type = pyVector_t::vector)
    {
        if (type == pyVector_t::vector)
        {
            npy_intp dims[1] = {npy_intp(data.size())};
            return PyArray_SimpleNewFromData(1, dims, NPY_DOUBLE, const_cast<float64_t *>(data.data()));
        }
        else
        {
            npy_intp dims[2] = {npy_intp(1), npy_intp(data.size())};
            PyObject * pyData = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, const_cast<float64_t *>(data.data()));

            if (type == pyVector_t::matrixCol)
            {
                return pyData;
            }
            else
            {
                return PyArray_Transpose(reinterpret_cast<PyArrayObject *>(pyData),NULL);
            }
        }
    }

    PyObject * getPyReferenceFromMatrix(matrixN_t & data)
    {
        npy_intp dims[2] = {npy_intp(data.cols()), npy_intp(data.rows())};
        return PyArray_Transpose(reinterpret_cast<PyArrayObject *>(
            PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, data.data())),NULL);
    }

    PyObject * getPyReferenceFromMatrix(matrixN_t const & data)
    {
        npy_intp dims[2] = {npy_intp(data.cols()), npy_intp(data.rows())};
        return  PyArray_Transpose(reinterpret_cast<PyArrayObject *>(
            PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, const_cast<float64_t *>(data.data()))),NULL);
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
        static PyObject* convert(std::vector<T> const & vec)
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
    vectorN_t listPyToVector(bp::list const & listPy)
    {
        vectorN_t x(len(listPy));
        for (int32_t i = 0; i < len(listPy); i++)
        {
            x(i) = bp::extract<real_t>(listPy[i]);
        }

        return x;
    }

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Convert a 2D python list into an Eigen matrix.
    ///////////////////////////////////////////////////////////////////////
    matrixN_t listPyToMatrix(bp::list const & listPy)
    {
        int32_t const nRows = len(listPy);
        assert(nRows > 0 && "empty list");

        int32_t const nCols = len(bp::extract<bp::list>(listPy[0]));
        assert(nCols > 0 && "empty row");

        matrixN_t M(nRows, nCols);
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
    void convertConfigHolderPy(configHolder_t const & config,
                               bp::dict             & configPy)
    {
        std::vector<std::string> options;
        for(auto const& it : config)
        {
            options.push_back(it.first);
        }

        for (uint32_t i = 0; i < options.size(); i++)
        {
            std::string const name = options[i];
            const std::type_info & optionType = config.at(name).type();

            if (optionType == typeid(bool_t))
            {
                configPy[name] = boost::get<bool_t>(config.at(name));
            }
            else if (optionType == typeid(int32_t))
            {
                configPy[name] = boost::get<int32_t>(config.at(name));
            }
            else if (optionType == typeid(real_t))
            {
                configPy[name] = boost::get<real_t>(config.at(name));
            }
            else if (optionType == typeid(std::string))
            {
                configPy[name] = boost::get<std::string>(config.at(name));
            }
            else if (optionType == typeid(vectorN_t))
            {
                vectorN_t const v = boost::get<vectorN_t>(config.at(name));
                bp::list l;

                for (int32_t j = 0; j < v.rows(); j++)
                {
                    l.append(v(j));
                }
                configPy[name] = l;
            }
            else if (optionType == typeid(matrixN_t))
            {
                matrixN_t const M = boost::get<matrixN_t>(config.at(name));
                bp::list l;

                for (int32_t j = 0; j < M.rows(); j++)
                {
                    bp::list row;
                    for (int32_t k = 0; k < M.cols(); k++)
                    {
                        row.append(M(j, k));
                    }
                    l.append(row);
                }
                configPy[name] = l;
            }
            else if (optionType == typeid(configHolder_t))
            {
                bp::dict configPyTmp;
                convertConfigHolderPy(boost::get<configHolder_t>(config.at(name)), configPyTmp);
                configPy[name] = configPyTmp;
            }
            else
            {
                assert(false && "Unsupported type");
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    /// \brief      Load a config holder from a python dictionary.
    ///////////////////////////////////////////////////////////////////////
    void loadConfigHolder(bp::dict       const & configPy,
                          configHolder_t       & config)
    {
        std::vector<std::string> options;
        for(auto const& it : config)
        {
            options.push_back(it.first);
        }

        for (uint32_t i = 0; i < options.size(); i++)
        {
            std::string const name = options[i];
            const std::type_info & optionType = config[name].type();

            if (optionType == typeid(bool_t))
            {
                boost::get<bool_t>(config.at(name)) = bp::extract<bool_t>(configPy[name]);
            }
            else if (optionType == typeid(int32_t))
            {
                boost::get<int32_t>(config.at(name)) = bp::extract<int32_t>(configPy[name]);
            }
            else if (optionType == typeid(real_t))
            {
                boost::get<real_t>(config.at(name)) = bp::extract<real_t>(configPy[name]);
            }
            else if (optionType == typeid(std::string))
            {
                 boost::get<std::string>(config.at(name)) = bp::extract<std::string>(configPy[name]);
            }
            else if (optionType == typeid(vectorN_t))
            {
                boost::get<vectorN_t>(config.at(name)) = listPyToVector(bp::extract<bp::list>(configPy[name]));
            }
            else if (optionType == typeid(matrixN_t))
            {
                boost::get<matrixN_t>(config.at(name)) = listPyToMatrix(bp::extract<bp::list>(configPy[name]));
            }
            else if (optionType == typeid(configHolder_t))
            {
                loadConfigHolder(bp::extract<bp::dict>(configPy[name]), boost::get<configHolder_t>(config.at(name)));
            }
            else
            {
                assert(false && "Unsupported type");
            }
        }
    }
}  // end of namespace python.
}  // end of namespace exo_simu.

#endif  // WDC_EXO_SIMULATOR_PYTHON_UTILITIES_H
