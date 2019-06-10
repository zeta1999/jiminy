#ifndef WDC_NETWORK_LOCAL_SOCKET_H
#define WDC_NETWORK_LOCAL_SOCKET_H

#include "exo_simu/core/io/AbstractSocket.h"

namespace exo_simu
{
namespace core
{
    class LocalListener : public AbstractListener
    {
    public:
        LocalListener(std::string const& localPath);
        virtual ~LocalListener();

        hresult_t listen(bool_t isAcceptBlocking, int32_t backLog) override;

    protected:
        AbstractSocket* createSocket() override;

    private:
        std::string socketPath_;
    };


    class LocalSocket : public AbstractSocket
    {
    public:
        LocalSocket(std::string const& address="");
        virtual ~LocalSocket() = default;

        void setAddress(std::string const& address);

    protected:
        hresult_t doOpen(enum OpenMode mode) override;

    private:
        std::string socketPath_;
    };

}
}

#endif
