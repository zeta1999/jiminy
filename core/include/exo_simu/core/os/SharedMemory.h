///////////////////////////////////////////////////////////////////////////////
///
/// \brief This file contains the SharedMemory class.
///
/// \copyright Wandercraft
///
//////////////////////////////////////////////////////////////////////////////
#ifndef WDC_WANDERBRAIN_SHARED_MEMORY_H
#define WDC_WANDERBRAIN_SHARED_MEMORY_H

// wdc libraries includes.
#include "exo_simu/core/Error.h"


namespace exo_simu
{
namespace core
{
    class SharedMemory
    {
    public:
        ///////////////////////////////////////////////////////////////////////
        /// \brief       Constructor.
        ///
        /// \param  name  Name of the shared memory.
        /// \param  size  Size of the shared memory in bytes.
        ///////////////////////////////////////////////////////////////////////
        SharedMemory(std::string const& name, std::size_t size);

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Destructor
        ///////////////////////////////////////////////////////////////////////
        ~SharedMemory();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Create the shm context.
        /// \details     Create the shm if required, open it, map it and set
        /// the rights to use it.
        ///
        /// \retval definition::S_OK if successful.
        /// \retval the corresponding errno otherwise.
        ///////////////////////////////////////////////////////////////////////
        hresult_t create();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Destroy the shm context..
        /// \details     Destroy the shm if no other client are mapped.
        ///
        /// \retval definition::S_OK if successful.
        /// \retval the corresponding errno otherwise.
        ///////////////////////////////////////////////////////////////////////
        hresult_t destroy();

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Getter on the mapped shm.
        ///
        /// \return The address of the shm in this process.
        ///////////////////////////////////////////////////////////////////////
        void* address();


        ///////////////////////////////////////////////////////////////////////
        /// \brief       Getter on the shm size.
        ///
        /// \return The size in bytes of the shm.
        ///////////////////////////////////////////////////////////////////////
        std::size_t size() const;

    private:
        ///////////////////////////////////////////////////////////////////////
        /// \brief       Copy constructor disabled.
        ///////////////////////////////////////////////////////////////////////
        SharedMemory(SharedMemory const& shm);

        ///////////////////////////////////////////////////////////////////////
        /// \brief       Copy operator disabled.
        ///////////////////////////////////////////////////////////////////////
        SharedMemory& operator=(SharedMemory const& shm);

        std::string name_;  ///< Name of the shared memory.
        std::size_t size_;  ///< Size in bytes of the sahred memory.
        void* shmAddress_;  ///< Address of the shared memory in this processus.
        int32_t fd_;        ///< File descriptor of the shared memory.
    };
}
}

#endif  // WDC_WANDERBRAIN_MUTEX_H_
