///////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains the AbstractIODevice class methods implementations.
///
/// \copyright Wandercraft
///
///////////////////////////////////////////////////////////////////////////////

#include "exo_simu/core/io/AbstractIODevice.h"

namespace exo_simu
{
namespace core
{
    enum OpenMode operator | (enum OpenMode const& modeA, enum OpenMode const& modeB)
    {
        return static_cast<enum OpenMode>(static_cast<int32_t>(modeA) | static_cast<int32_t>(modeB));
    }


    enum OpenMode operator & (enum OpenMode const& modeA, enum OpenMode const& modeB)
    {
        return static_cast<enum OpenMode>(static_cast<int32_t>(modeA) & static_cast<int32_t>(modeB));
    }


    enum OpenMode operator |= (enum OpenMode& modeA, enum OpenMode const& modeB)
    {
        return modeA = modeA | modeB;
    }


    enum OpenMode operator &= (enum OpenMode& modeA, enum OpenMode const& modeB)
    {
        return modeA = modeA & modeB;
    }


    enum OpenMode operator ~(enum OpenMode mode)
    {
        int32_t tmpMode = ~static_cast<int32_t>(mode);
        return static_cast<enum OpenMode>(tmpMode);
    }


    AbstractIODevice::AbstractIODevice()
        : modes_(OpenMode::NOT_OPEN)
        , supportedModes_(OpenMode::NOT_OPEN)
        , lastError_(error::E_GENERIC_ERROR)
        , io_(nullptr)
    {
    }


    AbstractIODevice::~AbstractIODevice()
    {

    }


    hresult_t AbstractIODevice::open(OpenMode modes)
    {
        if (isOpen())
        {
            lastError_ = error::E_EALREADY;
            return lastError_;
        }

        if ((modes & supportedModes_) != modes)
        {
            lastError_ = error::E_EINVAL;
            return lastError_;
        }

        hresult_t returnCode = doOpen(modes);
        if (succeeded(returnCode))
        {
            modes_ = modes;
        }
        else
        {
            lastError_ = returnCode;
        }

        return returnCode;
    }


    void AbstractIODevice::close()
    {
        if (not isOpen())
        {
            lastError_ = error::E_EBADF;
            return;
        }

        doClose();
        modes_ = NOT_OPEN;
    }


    enum OpenMode AbstractIODevice::openModes() const
    {
        return modes_;
    }


    enum OpenMode AbstractIODevice::supportedModes() const
    {
        return supportedModes_;
    }


    bool_t AbstractIODevice::isWritable() const
    {
        return (modes_ & OpenMode::WRITE_ONLY) or (modes_ & OpenMode::READ_WRITE);
    }


    bool_t AbstractIODevice::isReadable() const
    {
        return (modes_ & OpenMode::READ_ONLY) or (modes_ & OpenMode::READ_WRITE);
    }


    bool_t AbstractIODevice::isOpen() const
    {
        return (modes_ != OpenMode::NOT_OPEN);
    }


    bool_t AbstractIODevice::isSequential() const
    {
        return false;
    }


    int64_t AbstractIODevice::size()
    {
        return bytesAvailable();
    }


    hresult_t AbstractIODevice::seek(int64_t pos)
    {
        (void) pos;
        return error::E_ENOSYS;
    }


    int64_t AbstractIODevice::pos()
    {
        return 0;
    }


    int64_t AbstractIODevice::bytesAvailable()
    {
        return 0;
    }

    hresult_t AbstractIODevice::getLastError() const
    {
        return lastError_;
    }


    hresult_t AbstractIODevice::write(void const* data, int64_t dataSize)
    {
        int64_t toWrite = dataSize;
        uint8_t const* bufferPos = static_cast<uint8_t const*>(data);

        while (toWrite > 0)
        {
            int64_t writtenBytes = writeData(bufferPos + (dataSize - toWrite), toWrite);
            if (writtenBytes < 0)
            {
                return lastError_;
            }

            if (writtenBytes == 0)
            {
                lastError_ = error::E_ENOMEM;
                return lastError_;
            }

            toWrite -= writtenBytes;
        }

        return error::S_OK;
    }


    hresult_t AbstractIODevice::read(void* data, int64_t dataSize)
    {
        int64_t toRead = dataSize;
        uint8_t* bufferPos = static_cast<uint8_t*>(data);

        while (toRead > 0)
        {
            int64_t readBytes = readData(bufferPos + (dataSize - toRead), toRead);
            if (readBytes < 0)
            {
                return lastError_;
            }

            if (readBytes == 0)
            {
                lastError_ = error::E_ENOMEM;
                return lastError_;
            }
            toRead -= readBytes;
        }

        return error::S_OK;
    }


    hresult_t AbstractIODevice::setBlockingMode(bool_t shouldBlock)
    {
        (void) shouldBlock;
        lastError_ = error::E_ENOSYS;
        return lastError_;
    }


    void AbstractIODevice::setBackend(AbstractIODevice* io)
    {
        io_ = io;
        supportedModes_ = io_->supportedModes();
    }


    void AbstractIODevice::removeBackend()
    {
        io_ = nullptr;
        supportedModes_ = OpenMode::NOT_OPEN;
    }

    // Specific implementation - std::vector<uint8_t>
    template<>
    hresult_t AbstractIODevice::read<std::vector<uint8_t> >(std::vector<uint8_t>& v)
    {
        int64_t toRead = static_cast<int64_t>(v.size() * sizeof(uint8_t));
        uint8_t* bufferPos = reinterpret_cast<uint8_t*>(v.data());

        return read(bufferPos, toRead);
    }

    // Specific implementation - std::vector<char_t>
    template<>
    hresult_t AbstractIODevice::read<std::vector<char_t> >(std::vector<char_t>& v)
    {
        int64_t toRead = static_cast<int64_t>(v.size() * sizeof(char_t));
        uint8_t* bufferPos = reinterpret_cast<uint8_t*>(v.data());

        return read(bufferPos, toRead);
    }


    // Specific implementation - std::string
    template<>
    hresult_t AbstractIODevice::write<std::string>(std::string const& str)
    {
        int64_t toWrite = static_cast<int64_t>(str.size());
        uint8_t const* bufferPos = reinterpret_cast<uint8_t const*>(str.c_str());
        return write(bufferPos, toWrite);
    }

    // Specific implementation - std::vector<uint8_t>
    template<>
    hresult_t AbstractIODevice::write<std::vector<uint8_t> >(std::vector<uint8_t> const& v)
    {
        int64_t toWrite = static_cast<int64_t>(v.size() * sizeof(uint8_t));
        uint8_t const* bufferPos = reinterpret_cast<uint8_t const*>(v.data());

        return write(bufferPos, toWrite);
    }

    // Specific implementation - std::vector<char_t>
    template<>
    hresult_t AbstractIODevice::write<std::vector<char_t> >(std::vector<char_t> const& v)
    {
        int64_t toWrite = static_cast<int64_t>(v.size() * sizeof(char_t));
        uint8_t const* bufferPos = reinterpret_cast<uint8_t const*>(v.data());

        return write(bufferPos, toWrite);
    }

    // Specific implementation - std::vector<uint64_t>
    template<>
    hresult_t AbstractIODevice::write<std::vector<uint64_t> >(std::vector<uint64_t> const& v)
    {
        int64_t toWrite = static_cast<int64_t>(v.size() * sizeof(uint64_t));
        uint8_t const* bufferPos = reinterpret_cast<uint8_t const*>(&v[0]);

        return write(bufferPos, toWrite);
    }
}
}
