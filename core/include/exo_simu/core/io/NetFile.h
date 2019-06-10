#ifndef WDC_SYSTEM_NET_FILE_RAW_H
#define WDC_SYSTEM_NET_FILE_RAW_H

#include "exo_simu/core/io/AbstractIODevice.h"

namespace exo_simu
{
namespace core
{
    class NetFile : public AbstractIODevice
    {
        DISABLE_COPY(NetFile);

    public:
        NetFile(std::string const& filename="", AbstractIODevice* io=nullptr);
        virtual ~NetFile();

        /// Set the filename to work on.
        void setFilename(std::string const& filename);

        virtual int64_t readData(void* data, int64_t dataSize);
        virtual int64_t writeData(void const* data, int64_t dataSize);

        virtual int64_t bytesAvailable();
        virtual int64_t size();
        virtual int64_t pos();

        void setBackend(AbstractIODevice* io) override;

    protected:
        virtual hresult_t doOpen(enum OpenMode modes);
        virtual void doClose();

    private:
        std::string filename_;
    };
}
}

#endif
