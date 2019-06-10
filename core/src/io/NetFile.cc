///////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains the NetFileRaw class methods implementations.
///
/// \copyright Wandercraft
///
///////////////////////////////////////////////////////////////////////////////

#include <cstring>

#include "exo_simu/core/io/NetFile.h"
#include "exo_simu/core/io/FileManager.h"
#include "exo_simu/core/os/Time.h"

namespace exo_simu
{
namespace core
{
    NetFile::NetFile(std::string const& filename, AbstractIODevice* io)
    : filename_(filename)
    {
        supportedModes_ = OpenMode::READ_ONLY     | OpenMode::WRITE_ONLY | OpenMode::READ_WRITE |
                          OpenMode::NON_BLOCKING  | OpenMode::TRUNCATE   | OpenMode::NEW_ONLY   |
                          OpenMode::EXISTING_ONLY | OpenMode::APPEND     | OpenMode::SYNC;
    }


    NetFile::~NetFile()
    {
        close();
        delete io_;
    }


    void NetFile::setBackend(AbstractIODevice* io)
    {
        io_ = io;
    }


    void NetFile::setFilename(std::string const& filename)
    {
        filename_ = filename;
    }


    hresult_t NetFile::doOpen(enum OpenMode modes)
    {
        struct protocol::Handshake header;
        header.version = protocol::version::V1;
        strncpy(header.filename, filename_.c_str(), sizeof(header.filename) - 1);

        if (modes & OpenMode::NON_BLOCKING)
        {
            // blocking mode is handled at medium layer.
            header.modes = modes & ~OpenMode::NON_BLOCKING;
        }
        else
        {
            header.modes = modes;
        }

        hresult_t returnCode = io_->open(OpenMode::READ_WRITE);
        if (succeeded(returnCode))
        {
            returnCode = io_->write(header);
        }

        hresult_t rcRead;
        if (succeeded(returnCode))
        {
            returnCode = io_->read(rcRead);
        }

        if (succeeded(returnCode))
        {
            returnCode = rcRead;
        }

        if (succeeded(returnCode))
        {
            if (modes & OpenMode::NON_BLOCKING)
            {
                returnCode = io_->setBlockingMode(false);
            }
        }

        if (failed(returnCode))
        {
            io_->close();
        }

        return returnCode;
    }


    void NetFile::doClose()
    {
        struct protocol::FrameHeader frame;
        frame.cmd = protocol::command::CLOSE;

        hresult_t rc = io_->write(frame);
        if (failed(rc))
        {
            lastError_ = rc;
        }

        io_->close();
    }


    int64_t NetFile::readData(void* data, int64_t dataSize)
    {
        (void) data;
        (void) dataSize;

        lastError_ = error::E_ENOSYS;
        return -1;
    }


    int64_t NetFile::writeData(void const* data, int64_t dataSize)
    {
        struct protocol::FrameHeader frame;
        frame.cmd = protocol::command::WRITE;
        frame.dataSize = static_cast<uint32_t>(dataSize); // Net write limited to 4GB per call.
        frame.timestamp = getCurrentTimeUs();

        hresult_t returnCode = io_->write(frame);
        if (succeeded(returnCode))
        {
            int64_t written = io_->writeData(data, dataSize);
            lastError_ = io_->getLastError();
            return written;
        }

        // Something wrong happened.
        lastError_ = io_->getLastError();
        return -1;
    }


    int64_t NetFile::bytesAvailable()
    {
        lastError_ = error::E_ENOSYS;
        return -1;
    }


    int64_t NetFile::size()
    {
        lastError_ = error::E_ENOSYS;
        return -1;
    }


    int64_t NetFile::pos()
    {
        lastError_ = error::E_ENOSYS;
        return -1;
    }
}
}
