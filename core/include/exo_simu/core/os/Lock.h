///////////////////////////////////////////////////////////////////////////////
/// Wanderbrain
///
/// \brief This file contains the Mutex Lock class.
///
/// The Lock class is used to encapsulate a mutex.
/// \copyright Wandercraft
//////////////////////////////////////////////////////////////////////////////
#ifndef WDC_WANDERBRAIN_MUTEX_LOCK_H_
#define WDC_WANDERBRAIN_MUTEX_LOCK_H_

#include <pthread.h>

// wdc libraries includes.
#include "exo_simu/core/Error.h"

namespace exo_simu
{
namespace core
{
    class Lock
    {
    public:
        ///////////////////////////////////////////////////////////////////////
        /// \brief       Default constructor.
        ///////////////////////////////////////////////////////////////////////
        explicit Lock();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Destructor
        ///////////////////////////////////////////////////////////////////////
        ~Lock();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Lock the pthread_mutex
        /// \retval      error::S_OK on success.
        /// \retval      error::E_MUTEX by default.
        /// \retval      error::E_AGAIN (see Neutrino doc).
        /// \retval      error::E_DEADLK (see Neutrino doc).
        /// \retval      error::E_FAULT (see Neutrino doc).
        /// \retval      error::E_INVAL (see Neutrino doc).
        /// \retval      error::E_TIMEDOUT (see Neutrino doc).
        ///////////////////////////////////////////////////////////////////////
        hresult_t lock();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Unlock the pthread_mutex
        /// \retval      error::S_OK on success.
        /// \retval      error::E_MUTEX by default.
        /// \retval      error::E_INVAL (see Neutrino doc).
        /// \retval      error::E_PERM (see Neutrino doc).
        ///////////////////////////////////////////////////////////////////////
        hresult_t unlock();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Lock the pthread_mutex (non-blocking)
        /// \retval      error::S_OK on success.
        /// \retval      error::E_MUTEX by default.
        /// \retval      error::E_AGAIN (see Neutrino doc).
        /// \retval      error::E_INVAL (see Neutrino doc).
        /// \retval      error::E_BUSY (see Neutrino doc)
        ///////////////////////////////////////////////////////////////////////
        hresult_t tryLock();

    private:
        /// The POSIX mutex protecting the value.
        pthread_mutex_t lock_;

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Translates error codes from Neutrino errors (int) to hresult
        ///
        /// \details     This function takes as input a neutrino error code and translates it
        ///              to a usual hresul_t. This function is static as it doesn't depend on
        ///              the function members.
        ///
        /// \param[in]   errorIn the error to translate.
        /// \retval      error::S_OK when encountering EOK.
        /// \retval      error::E_MUTEX when encountering unknown error code.
        /// \retval      error::E_AGAIN when encountering EAGAIN (see Neutrino doc).
        /// \retval      error::E_DEADLK when encountering EDEADLK (see Neutrino doc).
        /// \retval      error::E_FAULT when encountering EFAULT (see Neutrino doc).
        /// \retval      error::E_INTR when encountering EINTR (see Neutrino doc).
        /// \retval      error::E_INVAL when encountering EINVAL (see Neutrino doc).
        /// \retval      error::E_TIMEDOUT when encountering ETIMEDOUT (see Neutrino doc).
        /// \retval      error::E_PERM when encountering EPERM (see Neutrino doc).
        /// \retval      error::E_BUSY when encountering EBUSY (see Neutrino doc).
        ///////////////////////////////////////////////////////////////////////
        static hresult_t translateMutexError(int32_t const& errorIn);
    };


    class LockGuard
    {
    public:
        ///////////////////////////////////////////////////////////////////////
        /// \brief       Constructor.
        ///
        /// \param       lock    mutex to be acquired by the locker
        ///////////////////////////////////////////////////////////////////////
        explicit LockGuard(Lock& lock);

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Destructor.
        ///////////////////////////////////////////////////////////////////////
        ~LockGuard();

    private:
        Lock& lock_;  ///< Reference to the lock managed by the guard
    };

}
}
#endif
