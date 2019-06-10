///////////////////////////////////////////////////////////////////////////////
///
/// \brief This file contains the SharedMemory class.
///
/// \copyright Wandercraft
///
//////////////////////////////////////////////////////////////////////////////

// shm POSIX
#include <sys/mman.h>
#include <sys/stat.h> // For mode constants
#include <fcntl.h>    // For O_* constants
#include <unistd.h>   // ftruncate
#include <sys/types.h>
#include <cstring>
#include <errno.h>

#include "exo_simu/core/Error.h"
#include "exo_simu/core/os/SharedMemory.h"

namespace exo_simu
{
namespace core
{
    SharedMemory::SharedMemory(std::string const& name, std::size_t size)
        : name_(name)
        , size_(size)
        , shmAddress_(NULL)
        , fd_(-1)
    {
        // Empty on purpose.
    }


    SharedMemory::~SharedMemory()
    {
        if (-1 != fd_)
        {
            destroy();
        }
    }


    hresult_t SharedMemory::create()
    {
        if (shmAddress_ != NULL)
        {
            return error::S_OK;
        }

        // Create the shared memory, set its mode to R/W and get the R/W access rights.
        fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, S_IRWXU | S_IRWXG | S_IRWXO);
        if (-1 == fd_)
        {
            hresult_t returnCode = error::errnoToHresult(errno);
            WDC_REPORT_ERROR(returnCode);
            return returnCode;
        }

        // Set the memory size.
        int32_t const rcTruncate = ftruncate(fd_, size_);
        if (-1 == rcTruncate)
        {
            hresult_t returnCode = error::errnoToHresult(errno);
            WDC_REPORT_ERROR(returnCode);
            return returnCode;
        }

        shmAddress_ = mmap(NULL, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (MAP_FAILED == shmAddress_)
        {
            shmAddress_ = NULL;

            hresult_t returnCode = error::errnoToHresult(errno);
            WDC_REPORT_ERROR(returnCode);
            return returnCode;
        }

        return error::S_OK;
    }


    hresult_t SharedMemory::destroy()
    {
        if (NULL != shmAddress_)
        {
            // Remove the mapped memory segment from the address space of the process.
            int32_t const rcUnmap = munmap(shmAddress_, size_);
            if (-1 == rcUnmap)
            {
                hresult_t returnCode = error::errnoToHresult(errno);
                WDC_REPORT_ERROR(returnCode);
                return returnCode;
            }
        }

        int32_t const rcClose = close(fd_);
        if (-1 == rcClose)
        {
            hresult_t returnCode = error::errnoToHresult(errno);
            WDC_REPORT_ERROR(returnCode);
            return returnCode;
        }

        // Reset members.
        shmAddress_ = NULL;
        fd_ = -1;

        return error::S_OK;
    }


    void* SharedMemory::address()
    {
        return shmAddress_;
    }


    std::size_t SharedMemory::size() const
    {
        return size_;
    }
}
}
