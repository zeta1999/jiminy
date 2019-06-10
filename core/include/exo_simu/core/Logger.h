///////////////////////////////////////////////////////////////////////////////
///
/// \brief Logger class to log everything to a specified output.
///
/// Inspired by https://wiki.calculquebec.ca/w/C%2B%2B_:_classe_Logger/fr.
///
/// Logger class is used to log things in a logfile and/or on the standard
/// outputs (stdout / stderr, depending on the logging level), from all the
/// different parts of the program. It allows to aggregate all the logging
/// facilities of the program.
///
/// Most of the methods are static so that it can be used from any part of
/// the program.
///
/// One should call `startLogging` first, in the main thread of any executable,
/// to enable logging (and optionally provide it a logfile to write into).  When
/// finished logging, or to avoid logging somewhere in the program, one should
/// call `stopLogging`. Note that since Logger is supposed to work as a
/// singleton, and to avoid problems with multiple threads, `startLogging` and
/// `stopLogging` should *only* be called in the main thread.
///
/// This class exposes an API to log with different logging levels:
/// * `INFO` which is for basic debugging information.
/// * `WARN` which is for non-fatal errors.
/// * `ERROR` which is for fatal errors.
///
/// When starting the Logger, one can specify which level of details they want,
/// so that the Logger will only log messages above this threshold
/// (logging with a level `WARN` will filter out `INFO` messages).
///
/// \remark This Logger class is responsible for logging useful log messages,
/// and should not be confused with the DataLogger class which is responsible for
/// logging the Exoskeleton state in realtime. Logging can be done either directly
/// on a file or piped via socket to network or filemanager.
///
/// \copyright Wandercraft
///
//////////////////////////////////////////////////////////////////////////////

#ifndef WDC_WANDERBRAIN_LOGGER_H
#define WDC_WANDERBRAIN_LOGGER_H

#include <cstdarg> // needed for va_list definition

// Internal includes
#include "exo_simu/core/Constants.h"
#include "exo_simu/core/io/LocalSocket.h"

namespace exo_simu
{
namespace core
{
    namespace logginglevel
    {
        enum Enum
        {
            ALL,
            INFO,
            WARN,
            ERROR,
            OFF,
            LOGLEVEL_COUNT
        };

        char_t const logRepr[LOGLEVEL_COUNT] = {'A', 'I', 'W', 'E', 'O'};
        inline char_t getLogLevelStr(logginglevel::Enum const loglevel)
        {
            char_t result = ' ';
            if (LOGLEVEL_COUNT > loglevel)
            {
                result = logRepr[loglevel];
            }
            return result;
        }
    }  // namespace logginglevel

    ////////////////////////////////////////////////////////////////////////////
    /// \brief  Size of buffer for writing timestamp (seconds.ms) in string.
    ///
    /// \details Timestamp is the relative time since start of program.
    /// It is formatted using " %8d.%03d [L][pn]" that is a fixed size of 16 chars + '\0'.
    /// 10e8 seconds is 3 years of logs.
    /// [L] is the log level indicator
    /// [pn] char indicator for process name
    ////////////////////////////////////////////////////////////////////////////
    uint32_t const LOGMSG_PREFIX_BUFFER_SIZE = 17U;

    uint32_t const PROCESS_NAME_SIZE = 15U;

    class Logger
    {
    public:
        ///////////////////////////////////////////////////////////////////////
        /// \brief      Start logging.
        ///
        /// \param[in]  loggingLevelIn    the level of details of the resulting
        ///                               log.
        /// \param[in]  filenameIn        a filename to output the log to.
        /// \param[in]  outputToStdoutIn  whether to output also on stdout or
        ///                               not.
        //
        /// \remark     Note, this should be called in the main
        ///             thread before starting any additional threads.
        /// \retval     error::S_OK  on success
        ///////////////////////////////////////////////////////////////////////
        static hresult_t startLogging(
            logginglevel::Enum const loggingLevelIn,
            std::string const& filenameIn,
            bool_t const useFileManager = false);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Start logging with timestamped logging enabled.
        ///
        /// \param[in]  loggingLevelIn    the level of details of the resulting
        ///                               log.
        /// \param[in]  filenameIn        a filename to output the log to.
        /// \param[in]  outputToStdoutIn  whether to output also on stdout or
        /// \param[in]  startTime         the time origin for log messages
        //
        /// \remark     Note, this should be called in the main
        ///             thread before starting any additional threads.
        /// \retval     error::S_OK  on success
        ///////////////////////////////////////////////////////////////////////
        static hresult_t startLogging(
            logginglevel::Enum const loggingLevelIn,
            std::string const& filenameIn,
            uint64_t const startTime,
            bool_t const useFileManager = false,
            std::string const& processNameTrace = "");


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Stop logging.
        ///
        /// \details    Flushes the file buffer and then closes it. To ensure there
        ///             are no problems in multi-threaded usage, this function
        ///             must be called in the main thread, after all other
        ///             threads have been terminated.
        ///////////////////////////////////////////////////////////////////////
        static void stopLogging();


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Get the current logging level.
        ///
        /// \retval      The current logging level.
        ///////////////////////////////////////////////////////////////////////
        static logginglevel::Enum getLoggingLevel();


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Set the current logging level.
        ///
        /// \param[in]  loggingLevelIn the desired logging level.
        ///////////////////////////////////////////////////////////////////////
        static void setLoggingLevel(logginglevel::Enum const& loggingLevelIn);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Log a message.
        ///
        /// \param[in]  message  a string containing the message.
        /// \param[in]  messageLoggingLevel  a level associated to the error
        ///                                  message (warning, error, ...)
        /// \retval     error::E_EACCES if file is not opened / forward writing errors
        ///////////////////////////////////////////////////////////////////////
        static hresult_t write(
            std::string const& message,
            logginglevel::Enum const& messageLoggingLevel);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Write unformated buffer.
        ///
        /// \param[in]  message  a C string containing the message.
        ///////////////////////////////////////////////////////////////////////
        static void writeRaw(char_t const* message);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Log an info message with variable arguments like printf.
        ///
        /// \details    This is wrapper to the write() function, but with the logging
        ///             level set to logginglevel::INFO.
        /// \param[in]  format the format string as in printf
        /// \param[in]  ... : the variable arguments to print
        /// \see        write()
        ///////////////////////////////////////////////////////////////////////
        static void info(char_t const *format, ...);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Log a warning message with variable arguments like printf.
        ///
        /// \details    This is wrapper to the write() function, but with the logging
        ///             level set to logginglevel::WARN.
        /// \param[in]  format the format string as in printf
        /// \param[in] ... : the variable arguments to print
        ///////////////////////////////////////////////////////////////////////
        static void warn(char_t const *format, ...);


        ///////////////////////////////////////////////////////////////////////
        /// \brief      Log an error message with variable arguments like printf.
        ///
        /// \details    This is wrapper to the write() function, but with the logging
        ///             level set to logginglevel::ERROR.
        /// \param[in]  format the format string as in printf
        /// \param[in]  ... : the variable arguments to print

        ///////////////////////////////////////////////////////////////////////
        static void error(char_t const *format, ...);

    protected:
    private:
        ///////////////////////////////////////////////////////////////////////
        /// \brief      Construct a Logger object.
        ///
        /// \details    The object is made ready to be used to log errors
        ///             and warnings.  Private as this class is a
        ///             singleton.
        ///////////////////////////////////////////////////////////////////////
        explicit Logger(void);

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Copy constructor, private as this is a singleton.
        ///////////////////////////////////////////////////////////////////////
        Logger(Logger const&) = delete;

        ///////////////////////////////////////////////////////////////////////
        /// \brief      Assignment operator is private.
        ///////////////////////////////////////////////////////////////////////
        Logger& operator = (Logger const&);

        ///////////////////////////////////////////////////////////////////////
        /// \brief     wrapper for printf like logging functions.
        ///
        /// \param[in] logevel one of ERROR, WARNING, INFO severity level.
        /// \param[in] format the format string as in printf.
        /// \param[in] args variable list of parameters to log.
        /// \retval    error::S_OK on success
        /// \retval    error::E_STRING_BUFFER if the message was truncated
        ///////////////////////////////////////////////////////////////////////
        static hresult_t buildMessage(
                logginglevel::Enum const logLevel,
                char_t const *format,
                std::va_list args);

        ///////////////////////////////////////////////////////////////////////
        /// \brief     add a prefix timestamp and the loglevel to a log message
        ///
        /// \param[in] message, log buffer where the prefix will be added
        /// \param[in] logevel one of ERROR, WARNING, INFO severity level.
        /// \param[out] posOut: position after the prefix to start writing the message.
        /// \retval    error::S_OK on success
        /// \retval    error::E_STRING_BUFFER by default.
        ///////////////////////////////////////////////////////////////////////
        static hresult_t addPrefixTimestampAndLogLevel(
                logginglevel::Enum const logLevel,
                char_t (&message)[DEFAULT_TEXTLOG_LINE_LENGTH],
                uint32_t &posOut);

        ///////////////////////////////////////////////////////////////////////
        ///\brief   Logging level, to discard some messages if needed
        ///////////////////////////////////////////////////////////////////////
        logginglevel::Enum loggingLevel_;

        AbstractIODevice* IOdevice_; ///< main device output (console is treated separetely for now)
        uint64_t startTime_;                 ///< Time at which the logger was started.

        static Logger instance_;             ///< The sole logger instance (singleton).
        std::string processTrace_;
    };
}
}
#endif  // WDC_WANDERBRAIN_LOGGER_H_
