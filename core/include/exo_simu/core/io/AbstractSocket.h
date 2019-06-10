///////////////////////////////////////////////////////////////////////////////
/// \copyright Wandercraft
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_NETWORK_ABSTRACT_SOCKET_H
#define WDC_NETWORK_ABSTRACT_SOCKET_H

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>

#include "exo_simu/core/io/AbstractIODevice.h"

namespace exo_simu
{
namespace core
{
    class AbstractListener;
    class AbstractFactory;

    class AbstractSocket : public AbstractIODevice
    {
        friend class AbstractListener;
    public:
        AbstractSocket();
        virtual ~AbstractSocket();

        bool_t isSequential() const override;
        int64_t bytesAvailable() override;
        int64_t size() override;

        int64_t readData(void* data, int64_t dataSize) override;
        int64_t writeData(void const* data, int64_t dataSize) override;

        hresult_t setBlockingMode(bool_t shouldBlock) override;

    protected:
        void doClose() override;
        int32_t socketFd_;
    };


    class AbstractListener
    {
    public:
        AbstractListener();
        virtual ~AbstractListener() = default;

        virtual hresult_t listen(bool_t isAcceptBlocking, int32_t backLog) = 0;
        virtual hresult_t accept(bool_t isNewSocketBlocking, AbstractSocket& socket);
        virtual AbstractIODevice* accept(bool_t isNewSocketBlocking);

    protected:
        void initClient(AbstractSocket& socket, int32_t fd, bool_t shouldBlock);
        virtual AbstractSocket* createSocket() = 0;

        int32_t listenerFd_;
    };
}
}

#endif
