///////////////////////////////////////////////////////////////////////////////
/// \brief Logger class implementation
///
/// This file contains the implementation of the class Logger.
/// See Logger.h for more details.
///
/// \copyright Wandercraft
//////////////////////////////////////////////////////////////////////////////

// System includes
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

// Associated header
#include "exo_simu/core/Logger.h"

// Internal includes
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/io/NetFile.h"
#include "exo_simu/core/io/FileDevice.h"
#include "exo_simu/core/os/Lock.h"
#include "exo_simu/core/os/Time.h"


namespace exo_simu
{
// Hack to solve shared_tool / wanderbrain dependency
namespace path
{
        std::string const FILE_MANAGER_SOCKET  = "/tmp/file_manager";
}

namespace core
{
    // Initialization of instance_
    Logger Logger::instance_;

    hresult_t Logger::startLogging(
        logginglevel::Enum const loggingLevelIn,
        std::string const& filenameIn,
        bool_t const useFileManager)
    {
        if (instance_.IOdevice_ != nullptr)
        {
            WDC_REPORT_ERROR(error::E_EALREADY);
            return error::E_EALREADY;
        }

        if (useFileManager)
        {
            // Create and configure remote file writer
            NetFile* netFile = new NetFile(filenameIn);
            netFile->setBackend(new LocalSocket(path::FILE_MANAGER_SOCKET));
            instance_.IOdevice_ = netFile;
            // Connection to remote centralized writer.
            instance_.IOdevice_->open(OpenMode::WRITE_ONLY |
                                      OpenMode::APPEND | // allow a multiple process to use the same file output
                                      OpenMode::NON_BLOCKING);
        }
        else // Write directly on file device (Do not use in real time App)
        {
            instance_.IOdevice_ = new FileDevice(filenameIn);
            instance_.IOdevice_->open(OpenMode::WRITE_ONLY |
                                      OpenMode::TRUNCATE |
                                      OpenMode::NON_BLOCKING);
        }

        // Initialization other object members
        instance_.loggingLevel_ = loggingLevelIn;
        instance_.startTime_ = getStartTimeUs();

        if (instance_.IOdevice_->isOpen())
        {
            return error::S_OK;
        }
        else // trace outside the logger and return error
        {
            std::cerr << "        ********************************** WARNING ********************************"
                << std::endl;
            std::cerr << "        *                                                                         *"
                << std::endl;
            std::cerr << "        *          Cannot open text log file: logs will not be saved!             *"
                << std::endl;
            std::cerr << "        *                                                                         *"
                << std::endl;
            std::cerr << "        ***************************************************************************"
                << std::endl;
            delete instance_.IOdevice_;
            instance_.IOdevice_ = nullptr;
        }
        return error::E_EBADF;
    }


    hresult_t Logger::startLogging(
        logginglevel::Enum const loggingLevelIn,
        std::string const& filenameIn,
        uint64_t const startTime,
        bool_t const useFileManager,
        std::string const& processNameTrace)
    {
        hresult_t returnCode = startLogging(loggingLevelIn, filenameIn, useFileManager);
        if (not processNameTrace.empty())
        {
            instance_.processTrace_ = "[" + processNameTrace + "]";
        }
        instance_.startTime_ = startTime;
        return returnCode;
    }


    void Logger::stopLogging()
    {
        if (instance_.IOdevice_ != nullptr)
        {
            // Close the output file for writing.
            instance_.IOdevice_->close();
            delete instance_.IOdevice_;
            instance_.IOdevice_ = nullptr;
        }
    }


    Logger::Logger()
        : loggingLevel_(logginglevel::ALL)
        , IOdevice_(nullptr)
        , startTime_(getStartTimeUs())
        , processTrace_()
    {
        // Make stdout/stderr non blocking
        int32_t flags = fcntl(STDOUT_FILENO, F_GETFL);
        if (flags < 0)
        {
            std::cerr << "Can not get STDOUT current flags: " << std::strerror(errno) << std::endl;
            std::abort();
        }

        int32_t rc = fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK);
        if (rc < 0)
        {
            std::cerr << "Can not set STDOUT flags: " << std::strerror(errno) << std::endl;
            std::abort();
        }

        flags = fcntl(STDERR_FILENO, F_GETFL);
        if (flags < 0)
        {
            std::cerr << "Can not get STDERR current flags: " << std::strerror(errno) << std::endl;
            std::abort();
        }

        rc = fcntl(STDERR_FILENO, F_SETFL, flags | O_NONBLOCK);
        if (rc < 0)
        {
            std::cerr << "Can not set STDERR flags: " << std::strerror(errno) << std::endl;
            std::abort();
        }
    }


    logginglevel::Enum Logger::getLoggingLevel()
    {
        return instance_.loggingLevel_;
    }


    void Logger::setLoggingLevel(logginglevel::Enum const& loggingLevelIn)
    {
        instance_.loggingLevel_ = loggingLevelIn;
    }


    hresult_t Logger::buildMessage(
            logginglevel::Enum const logLevel,
            char_t const *format,
            std::va_list args)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);
        char_t message[DEFAULT_TEXTLOG_LINE_LENGTH];

        uint32_t position = 0;
        hresult_t const rcPrefix = addPrefixTimestampAndLogLevel(logLevel, message, position);
        if (succeeded(rcPrefix))
        {
            uint32_t const maxSize = DEFAULT_TEXTLOG_LINE_LENGTH - position;
            int32_t rcVsnprintf = std::vsnprintf(&message[position], maxSize, format, args);

            // vsnprintf returns the size of the message formatted. If the value is greater that maxSize,
            // it means the message was truncated. This is the desired behavior for the logger, to truncate
            // silently in a safe way. It returns a negative value in case of formatting error.
            if (0 < rcVsnprintf)
            {
                returnCode = write(message, logLevel);
            }
            else
            {
                returnCode = error::E_ECANCELED;
            }
        }
        else
        {
            returnCode = rcPrefix;
        }

        return returnCode;
    }

    hresult_t Logger::write(std::string const& message, logginglevel::Enum const& messageLoggingLevel)
    {
        static Lock continuousTextLock_;
        hresult_t returnCode(error::S_OK);

        // Check that the the message shall be logged.
        if (instance_.loggingLevel_ <= messageLoggingLevel)
        {
            // If this is an ERROR message, write it to stderr.
            if (messageLoggingLevel > logginglevel::WARN)
            {
                std::cerr << message << std::endl;
            }
            else // Else, write to stdout.
            {
                std::cout << message << std::endl;
            }

            if (instance_.IOdevice_ != nullptr)
            {
                LockGuard lock(continuousTextLock_);
                returnCode = instance_.IOdevice_->write(message+"\n");
            }
        }

        return returnCode;
    }


    void Logger::writeRaw(char_t const* message)
    {
        std::cerr << message << std::endl;

        if (instance_.IOdevice_ != nullptr)
        {
            instance_.IOdevice_->write(message);
        }
    }



    void Logger::info(char_t const *format, ...)
    {
        std::va_list args;
        va_start(args, format);
        (void) buildMessage(logginglevel::INFO, format, args);
        va_end(args);
    }


    void Logger::warn(char_t const *format, ...)
    {
        std::va_list args;
        va_start(args, format);
        (void) buildMessage(logginglevel::WARN, format, args);
        va_end(args);
    }


    void Logger::error(char_t const *format, ...)
    {
        std::va_list args;
        va_start(args, format);
        (void) buildMessage(logginglevel::ERROR, format, args);
        va_end(args);
    }


    hresult_t Logger::addPrefixTimestampAndLogLevel(
            logginglevel::Enum const logLevel,
            char_t (&message)[DEFAULT_TEXTLOG_LINE_LENGTH],
            uint32_t &pos)
    {
        hresult_t returnCode(error::E_GENERIC_ERROR);

        int32_t milliTimeRunning = getElapsedTimeMs(instance_.startTime_);
        char_t const logChar = logginglevel::getLogLevelStr(logLevel);

        // format the timestamp to a fixed size "s.ms" format
        int32_t const rcSize = snprintf(
                            &message[0],
                            LOGMSG_PREFIX_BUFFER_SIZE + PROCESS_NAME_SIZE,
                            "%8i.%03d [%c]%s",
                            milliTimeRunning / 1000,
                            milliTimeRunning % 1000,
                            logChar, instance_.processTrace_.c_str());
        // snprintf returns size of the formatted string even if it overflows the allowed size.
        if ( (0 < rcSize) && (LOGMSG_PREFIX_BUFFER_SIZE + PROCESS_NAME_SIZE >= static_cast<uint32_t>(rcSize)))
        {
            // return the size so that we can append the log message after this position
            pos = static_cast<uint32_t>(rcSize);
            returnCode = error::S_OK;
        }
        return returnCode;
    }
}
}
