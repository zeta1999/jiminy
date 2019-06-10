#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "exo_simu/core/io/AbstractSocket.h"

namespace exo_simu
{
namespace core
{
    AbstractSocket::AbstractSocket()
        : AbstractIODevice()
        , socketFd_(-1)
    {
    }


    AbstractSocket::~AbstractSocket()
    {
        close();
    }


    bool_t AbstractSocket::isSequential() const
    {
        return true;
    }


    int64_t AbstractSocket::bytesAvailable()
    {
        int64_t count;
        int32_t const rc = ::ioctl(socketFd_, FIONREAD, &count);
        if (rc < 0)
        {
            lastError_ = error::errnoToHresult(errno);
            return rc;
        }

        return count;
    }


    int64_t AbstractSocket::size()
    {
        return 0;
    }


    int64_t AbstractSocket::readData(void* data, int64_t dataSize)
    {
        ssize_t const rc = ::read(socketFd_, data, static_cast<size_t>(dataSize));
        if (rc < 0)
        {
            lastError_ = error::errnoToHresult(errno);
        }

        return rc;
    }


    int64_t AbstractSocket::writeData(void const* data, int64_t dataSize)
    {
        ssize_t const rc = ::write(socketFd_, data, static_cast<size_t>(dataSize));
        if (rc < 0)
        {
            lastError_ = error::errnoToHresult(errno);
        }

        return rc;
    }


    void AbstractSocket::doClose()
    {
        if (socketFd_ != -1)
        {
            int32_t const rc = ::close(socketFd_);
            if (rc < 0)
            {
                lastError_ = error::errnoToHresult(errno);
            }
            socketFd_ = -1;
        }
    }


    hresult_t AbstractSocket::setBlockingMode(bool_t shouldBlock)
    {
        int32_t settings = fcntl(socketFd_, F_GETFL, 0);
        if (shouldBlock)
        {
            modes_ &= ~OpenMode::NON_BLOCKING;
            settings &= ~O_NONBLOCK;
        }
        else
        {
            modes_ |= OpenMode::NON_BLOCKING;
            settings |= O_NONBLOCK;
        }

        int32_t rc = fcntl(socketFd_, F_SETFL, settings);
        if (rc < 0)
        {
            lastError_ = error::errnoToHresult(errno);
            return lastError_;
        }

        return error::S_OK;
    }


    AbstractListener::AbstractListener()
        : listenerFd_(-1)
    {
    }


    hresult_t AbstractListener::accept(bool_t isNewSocketBlocking, AbstractSocket& socket)
    {
        int32_t socketType = 0;
        if (not isNewSocketBlocking)
        {
            socketType |= SOCK_NONBLOCK;
        }

        int32_t const newSocketFd = accept4(listenerFd_, nullptr, nullptr, socketType);
        if (newSocketFd == -1)
        {
            return error::errnoToHresult(errno);
        }
        initClient(socket, newSocketFd, isNewSocketBlocking);
        return error::S_OK;
    }



    AbstractIODevice* AbstractListener::accept(bool_t isNewSocketBlocking)
    {
        AbstractSocket* client = createSocket();
        hresult_t rc = accept(isNewSocketBlocking, *client);
        if (failed(rc))
        {
            delete client;
            return nullptr;
        }
        return client;
    }


    void AbstractListener::initClient(AbstractSocket& socket, int32_t fd, bool_t shouldBlock)
    {
        socket.socketFd_ = fd;
        socket.modes_ = OpenMode::READ_WRITE;
        if (not shouldBlock)
        {
            socket.modes_ |= OpenMode::NON_BLOCKING;
        }
    }
}
}
