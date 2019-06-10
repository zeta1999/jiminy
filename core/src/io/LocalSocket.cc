#include <unistd.h>
#include <sys/un.h>

#include "exo_simu/core/io/LocalSocket.h"

namespace exo_simu
{
namespace core
{
    LocalListener::LocalListener(std::string const& localPath)
        : AbstractListener()
        , socketPath_(localPath)
    {
    }


    LocalListener::~LocalListener()
    {
        if (listenerFd_ != -1)
        {
            ::close(listenerFd_);
            ::unlink(socketPath_.c_str());
        }
    }


    hresult_t LocalListener::listen(bool_t isAcceptBlocking, int32_t backLog)
    {
        int32_t socketType = SOCK_STREAM;
        if (not isAcceptBlocking)
        {
            socketType |= SOCK_NONBLOCK;
        }

        listenerFd_ = ::socket(AF_UNIX, socketType, 0);
        if (listenerFd_ == -1)
        {
            return error::errnoToHresult(errno);
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(struct sockaddr_un));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path) - 1);

        (void)unlink(socketPath_.c_str()); // Destroy a potential socket with the same name.
        if (::bind(listenerFd_, (struct sockaddr*)&addr, sizeof(addr)) == -1)
        {
            ::close(listenerFd_);
            return error::errnoToHresult(errno);
        }


        if (::listen(listenerFd_, backLog) == -1)
        {
            ::close(listenerFd_);
            return error::errnoToHresult(errno);
        }
        return error::S_OK;
    }


    AbstractSocket* LocalListener::createSocket()
    {
        return new LocalSocket();
    }


    LocalSocket::LocalSocket(std::string const& address)
        : AbstractSocket()
        , socketPath_(address)
    {
        supportedModes_ = OpenMode::READ_WRITE | OpenMode::NON_BLOCKING;
    }


    hresult_t LocalSocket::doOpen(enum OpenMode mode)
    {
        int32_t socketType = SOCK_STREAM;
        if (mode & OpenMode::NON_BLOCKING)
        {
            socketType |= SOCK_NONBLOCK;
        }

        socketFd_ = ::socket(AF_UNIX, socketType, 0);
        if (socketFd_ == -1)
        {
            lastError_ = error::errnoToHresult(errno);
            return lastError_;
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(struct sockaddr_un));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path) - 1);

        if (::connect(socketFd_, (struct sockaddr*)&addr, sizeof(addr)) == -1)
        {
            lastError_ = error::errnoToHresult(errno);
            ::close(socketFd_);
            return lastError_;
        }

        return error::S_OK;
    }


    void LocalSocket::setAddress(std::string const& address)
    {
        socketPath_ = address;
    }
}
}
