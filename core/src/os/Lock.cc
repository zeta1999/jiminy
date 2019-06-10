///////////////////////////////////////////////////////////////////////////////
/// Wanderbrain
///
/// \brief This file contains the Mutex class.
///
/// Implementation of the mutex Lock class
///
/// \copyright Wandercraft
//////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <errno.h>

#include "exo_simu/core/os/Lock.h"

namespace exo_simu
{
namespace core
{
    Lock::Lock()
        : lock_()
    {
        (void) pthread_mutex_init(&lock_, NULL);
    }


    Lock::~Lock()
    {
        (void) pthread_mutex_destroy(&lock_);
    }


    hresult_t Lock::lock()
    {
        for (;;)
        {
            hresult_t returnCode = error::errnoToHresult(pthread_mutex_lock(&lock_));

            if (error::E_EINTR == returnCode)
            {
                // Pthread mutex lock was interrupted by a POSIX signal: this a correct behavior since
                // wanderbrain is using signals to manages threads ticks and we must re-lock the mutex.
                continue;
            }
            else
            {
                // any other return code is either a real error that must be managed by the client,
                // either a success.
                return returnCode;
            }
        }
    }


    hresult_t Lock::unlock()
    {
        return error::errnoToHresult(pthread_mutex_unlock(&lock_));
    }


    hresult_t Lock::tryLock()
    {
        return error::errnoToHresult(pthread_mutex_trylock(&lock_));
    }


    LockGuard::LockGuard(Lock & lock)
        : lock_(lock)
    {
        (void) lock_.lock();
    }


    LockGuard::~LockGuard()
    {
        (void) lock_.unlock();
    }
}
}
