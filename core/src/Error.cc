///////////////////////////////////////////////////////////////////////////////
///
/// \file              Error.cc
/// \brief             Implement result handle functions.
/// \details           Implement functions used to check result handles.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

// System headers.
#include <errno.h>
#include <map>

// Local headers.
#include "exo_simu/core/Error.h"
#include "exo_simu/core/os/Time.h"
#include "exo_simu/core/Logger.h"


namespace exo_simu
{
namespace error
{
    static error::AbstractErrorHandler* errorHandler = NULL;
    static uint64_t filteringWindowMs = 1 * MILLISECONDS_IN_SECOND;

    bool_t processError(error::level::Enum levelIn, hresult_t errorIn, std::string& level, uint32_t& counter)
    {
        // Don't report when there is no error.
        if (succeeded(errorIn))
        {
            return false;
        }

        static error::level::Enum levelMax = error::level::WARNING;

        // Map to filter events frequency.
        static ::std::map<hresult_t, struct error::report> errors;

        // Get the last report (auto create it if this is the first time) and increment counter.
        struct error::report* rpt = &errors[errorIn];
        rpt->counter++;

        // Report the error if the last occurrence happened more than a second ago (default: Fc = 1Hz)
        // unless the severity level has increased in which case the reporting and action is done immediately
        if (filteringWindowMs < core::getElapsedTimeMs(rpt->timestampUs)
            or (levelIn > levelMax))
        {
            // Get the trace level.
            switch (levelIn)
            {
                case error::level::WARNING:   { level = "WARNING"; break; }
                case error::level::ERROR:     { level = "ERROR";   break; }
                case error::level::CRITICAL:  { level = "CRITICAL";  errorHandler->critical();  break; }
                case error::level::ALERT:     { level = "ALERT";     errorHandler->alert();     break; }
                case error::level::EMERGENCY: { level = "EMERGENCY"; errorHandler->emergency(); break; }
                default:                      { level = "UNKNOWN"; break; }
            }

            // save the current counter.
            counter = rpt->counter;

            // Update report time and reset counter.
            rpt->timestampUs = core::getCurrentTimeUs();
            rpt->counter = 0U;

            levelMax = levelIn;

            return true;
        }

        return false;
    }


    void printError(char_t errorMessage[ERROR_MESSAGE_LENGTH])
    {
        core::Logger::error("%s", errorMessage);
    }


    void setErrorHandler(error::AbstractErrorHandler* handlerIn)
    {
        errorHandler = handlerIn;
    }


    void setErrorFilteringWindow(uint64_t windowMsIn)
    {
        filteringWindowMs = windowMsIn;
    }


    hresult_t errnoToHresult(int32_t const& errnoIn)
    {
        hresult_t errorCode(S_OK);

        switch (errnoIn)
        {
            case 0:            { return S_OK;                       } // Special case: this code is not an error.
            case EACCES:       { errorCode = E_EACCES;       break; }
            case EAGAIN:       { errorCode = E_EAGAIN;       break; }
            case EBADF:        { errorCode = E_EBADF;        break; }
            case ECANCELED:    { errorCode = E_ECANCELED;    break; }
            case EDEADLK:      { errorCode = E_EDEADLK;      break; }
            case EINTR:        { errorCode = E_EINTR;        break; }
            case EINVAL:       { errorCode = E_EINVAL;       break; }
            case EMSGSIZE:     { errorCode = E_EMSGSIZE;     break; }
            case ENODEV:       { errorCode = E_ENODEV;       break; }
            case ENOENT:       { errorCode = E_ENOENT;       break; }
            case ENOMEM:       { errorCode = E_ENOMEM;       break; }
            case ENOMSG:       { errorCode = E_ENOMSG;       break; }
            case ENOSYS:       { errorCode = E_ENOSYS;       break; }
            case ENOTDIR:      { errorCode = E_ENOTDIR;      break; }
            case ENOTSOCK:     { errorCode = E_ENOTSOCK;     break; }
            case ENOTSUP:      { errorCode = E_ENOTSUP;      break; }
            case EOVERFLOW:    { errorCode = E_EOVERFLOW;    break; }
            case EPROTO:       { errorCode = E_EPROTO;       break; }
            case EIO:          { errorCode = E_EIO;          break; }
            case ETIME:        { errorCode = E_ETIME;        break; }
            case ETIMEDOUT:    { errorCode = E_ETIMEDOUT;    break; }
            case ELOOP:        { errorCode = E_ELOOP;        break; }
            case EFAULT:       { errorCode = E_EFAULT;       break; }
            case ENAMETOOLONG: { errorCode = E_ENAMETOOLONG; break; }
            case ESRCH:        { errorCode = E_ESRCH;        break; }
            case EPERM:        { errorCode = E_EPERM;        break; }
            case ENXIO:        { errorCode = E_ENXIO;        break; }
            case E2BIG:        { errorCode = E_E2BIG;        break; }
            case ECHILD:       { errorCode = E_ECHILD;       break; }
            case EEXIST:       { errorCode = E_EEXIST;       break; }
            case EXDEV:        { errorCode = E_EXDEV;        break; }
            case EISDIR:       { errorCode = E_EISDIR;       break; }
            case ENFILE:       { errorCode = E_ENFILE;       break; }
            case EMFILE:       { errorCode = E_EMFILE;       break; }
            case ENOTTY:       { errorCode = E_ENOTTY;       break; }
            case ETXTBSY:      { errorCode = E_ETXTBSY;      break; }
            case EFBIG:        { errorCode = E_EFBIG;        break; }
            case ENOSPC:       { errorCode = E_ENOSPC;       break; }
            case ESPIPE:       { errorCode = E_ESPIPE;       break; }
            case EROFS:        { errorCode = E_EROFS;        break; }
            case EMLINK:       { errorCode = E_EMLINK;       break; }
            case EDOM:         { errorCode = E_EDOM;         break; }
            case ERANGE:       { errorCode = E_ERANGE;       break; }
            case ENOTEMPTY:    { errorCode = E_ENOTEMPTY;    break; }
            case ENODATA:      { errorCode = E_ENODATA;      break; }
            case EBUSY:        { errorCode = E_EBUSY;        break; }
            case EALREADY:     { errorCode = E_EALREADY;     break; }
            case EBADR:        { errorCode = E_EBADR;        break; }
            case EBADE:        { errorCode = E_EBADE;        break; }
            case EADDRINUSE:   { errorCode = E_EADDRINUSE;   break; }
            case EIDRM:        { errorCode = E_EIDRM;        break; }
            case ENOSR:        { errorCode = E_ENOSR;        break; }
            case EBADMSG:      { errorCode = E_EBADMSG;        break; }
            case ENOSTR:       { errorCode = E_ENOSTR;        break; }

            default:
            {
                errorCode = E_UNKNOWN_ERRNO;
            }
        }

        return errorCode;
    }


    std::string const& getErrorString(hresult_t errorIn)
    {
        switch (errorIn)
        {
            // LCOV_EXCL_BR_START
    //i.e   case E_ERROR_NAME                        { static std::string const str = "max size of message is 48 bytes../.......here ->"; return str; }
            case E_ENOTEMPTY:                        { static std::string const str = "Directory not empty ";                             return str; }
            case E_EACCES:                           { static std::string const str = "Permission denied";                                return str; }
            case E_EAGAIN:                           { static std::string const str = "Operation would block";                            return str; }
            case E_EBADF:                            { static std::string const str = "Bad file descriptor";                              return str; }
            case E_EBUSY:                            { static std::string const str = "Bad file descriptor";                              return str; }
            case E_ECANCELED:                        { static std::string const str = "Operation canceled";                               return str; }
            case E_EDEADLK:                          { static std::string const str = "Deadlock avoided ";                                return str; }
            case E_EINTR:                            { static std::string const str = "Interrupted call";                                 return str; }
            case E_EINVAL:                           { static std::string const str = "Invalid argument";                                 return str; }
            case E_EMSGSIZE:                         { static std::string const str = "Message too long";                                 return str; }
            case E_ENODEV:                           { static std::string const str = "No such device";                                   return str; }
            case E_ENOENT:                           { static std::string const str = "No such file or directory";                        return str; }
            case E_ENOMEM:                           { static std::string const str = "Not enough space/memory";                          return str; }
            case E_ENOMSG:                           { static std::string const str = "No message of the desired type";                   return str; }
            case E_ENOSYS:                           { static std::string const str = "Not implemented";                                  return str; }
            case E_ENOTDIR:                          { static std::string const str = "Not a directory";                                  return str; }
            case E_ENOTSOCK:                         { static std::string const str = "Not a socket";                                     return str; }
            case E_ENOTSUP:                          { static std::string const str = "Operation not supported";                          return str; }
            case E_EOVERFLOW:                        { static std::string const str = "Overflow";                                         return str; }
            case E_EPROTO:                           { static std::string const str = "Protocol error";                                   return str; }
            case E_EIO:                              { static std::string const str = "Input/output error";                               return str; }
            case E_ETIME :                           { static std::string const str = "Timer expired";                                    return str; }
            case E_ETIMEDOUT:                        { static std::string const str = "Timeout";                                          return str; }
            case E_ELOOP:                            { static std::string const str = "Too many levels of symbolic links";                return str; }
            case E_EFAULT:                           { static std::string const str = "Bad address";                                      return str; }
            case E_ENAMETOOLONG:                     { static std::string const str = "Filename too long";                                return str; }
            case E_ESRCH:                            { static std::string const str = "No such process";                                  return str; }
            case E_EPERM:                            { static std::string const str = "Operation not permitted";                          return str; }
            case E_ENXIO:                            { static std::string const str = "No such device or address";                        return str; }
            case E_E2BIG:                            { static std::string const str = "Argument list too long";                           return str; }
            case E_ECHILD:                           { static std::string const str = "Child error";                                      return str; }
            case E_EEXIST:                           { static std::string const str = "File exists";                                      return str; }
            case E_EXDEV:                            { static std::string const str = "Improper link";                                    return str; }
            case E_EISDIR:                           { static std::string const str = "Is a directory";                                   return str; }
            case E_ENFILE:                           { static std::string const str = "Too many open files in system";                    return str; }
            case E_EMFILE:                           { static std::string const str = "Too many open files";                              return str; }
            case E_ENOTTY:                           { static std::string const str = "Inappropriate I/O control operation";              return str; }
            case E_ETXTBSY:                          { static std::string const str = "Text file busy";                                   return str; }
            case E_EFBIG:                            { static std::string const str = "File too large";                                   return str; }
            case E_ENOSPC:                           { static std::string const str = "No space left on device";                          return str; }
            case E_ESPIPE:                           { static std::string const str = "Invalid seek";                                     return str; }
            case E_EROFS:                            { static std::string const str = "Read-only filesystem";                             return str; }
            case E_EMLINK:                           { static std::string const str = "Too many links";                                   return str; }
            case E_EDOM:                             { static std::string const str = "Out of domain of function";                        return str; }
            case E_ERANGE:                           { static std::string const str = "Out of bounds";                                    return str; }
            case E_ENODATA:                          { static std::string const str = "No message is available";                          return str; }
            case E_EALREADY:                         { static std::string const str = "Already in progress";                              return str; }
            case E_EBADR:                            { static std::string const str = "Invalid request descriptor";                       return str; }
            case E_EBADE:                            { static std::string const str = "Invalid exchange";                                 return str; }
            case E_EADDRINUSE:                       { static std::string const str = "Address already in use";                           return str; }
            case E_EOPNOTSUPP:                       { static std::string const str = "Operation not supported on transport endpoint";    return str; }
            case E_EIDRM:                            { static std::string const str = "Identifier removed";                               return str; }
            case E_ENOSR:                            { static std::string const str = "No stream resource";                               return str; }
            case E_EBADMSG:                          { static std::string const str = "Bad message";                                      return str; }
            case E_ENOSTR:                           { static std::string const str = "Not a stream";                                     return str; }

            case E_GENERIC_ERROR:                    { static std::string const str = "Generic error";                                    return str; }
            case E_CRITICAL_BATTERY:                 { static std::string const str = "Critical battery level";                           return str; }
            case E_LOW_BATTERY:                      { static std::string const str = "Low battery level";                                return str; }
            case E_MOTOR_FAULT:                      { static std::string const str = "Motor in fault";                                   return str; }
            case E_UNDERVOLTAGE:                     { static std::string const str = "Undervoltage";                                     return str; }
            case E_PRECONDITION:                     { static std::string const str = "Precondition";                                     return str; }
            case E_POSTCONDITION:                    { static std::string const str = "Postcondition";                                    return str; }
            case E_INCONSISTENT_ESTIMATE:            { static std::string const str = "Inconsistent state estimate";                      return str; }
            case E_WRONG_BRAIN_4_DEV:                { static std::string const str = "Wrong brain4dev";                                  return str; }
            case E_MOTOR_WATCHDOG:                   { static std::string const str = "Motor watchdog";                                   return str; }
            case E_MOTOR_START_TIMEOUT:              { static std::string const str = "Motor start timeout";                              return str; }
            case E_VERTICAL:                         { static std::string const str = "Vertical";                                         return str; }
            case E_STATIONARY_POINT:                 { static std::string const str = "Stationary point detected in trajectory";          return str; }
            case E_NOT_INITIALIZED:                  { static std::string const str = "Trying to use an uninitialized object.";           return str; }
            case E_MOTOR_DISABLED:                   { static std::string const str = "Motor disabled";                                   return str; }
            case E_UNDEFINED_BEHAVIOR:               { static std::string const str = "This option is not available";                     return str; }
            case E_COMPRESS:                         { static std::string const str = "Compression";                                      return str; }

            case E_NO_ACTIVE_SESSION:                { static std::string const str = "No active session";                                return str; }
            case E_ALREADY_IN_SESSION:               { static std::string const str = "Already in session";                               return str; }
            case E_NOT_IN_INSTALLATION:              { static std::string const str = "Not in installation state";                         return str; }

            case E_FAILED_TO_SOLVE_TASK:             { static std::string const str = "Failed to solve task";                             return str; }
            case E_NO_SOLUTION:                      { static std::string const str = "No solution founded";                              return str; }

            case INVALID_NB_AXIS:                    { static std::string const str = "INVALID_NB_AXIS";                                  return str; }
            case UPDATE_PERIOD_NOT_CONSISTENT:       { static std::string const str = "UPDATE_PERIOD_NOT_CONSISTENT";                     return str; }
            case E_SET_FEEDFORWARD:                  { static std::string const str = "E_SET_FEEDFORWARD";                                return str; }
            case E_INVERSE_DYNAMICS:                 { static std::string const str = "E_INVERSE_DYNAMICS";                               return str; }
            case E_WANDERADAPTER_COMPUTECOP:         { static std::string const str = "E_WANDERADAPTER_COMPUTECOP";                       return str; }
            case E_AXIS_ORDER_MISMATCH:              { static std::string const str = "E_AXIS_ORDER_MISMATCH";                            return str; }

            case E_FRICTION:                         { static std::string const str = "E_FRICTION";                                       return str; }
            case E_UNKNOWN_FOOT_CONSTRAINTS:         { static std::string const str = "E_UNKNOWN_FOOT_CONSTRAINTS";                       return str; }
            case E_FREEFLYER_STATE_ESTIMATE:         { static std::string const str = "E_FREEFLYER_STATE_ESTIMATE";                       return str; }
            case E_UPDATE_RIGID_MODEL_STATE:         { static std::string const str = "E_UPDATE_RIGID_MODEL_STATE";                       return str; }
            case E_UPDATE_COP:                       { static std::string const str = "E_UPDATE_COP";                                     return str; }
            case E_TARGET_COMPUTATION:               { static std::string const str = "E_TARGET_COMPUTATION";                             return str; }

            case E_FILTER_VELOCITY:                  { static std::string const str = "E_FILTER_VELOCITY";                                return str; }
            case E_INITIAL_POSITION_IS_INVALID:      { static std::string const str = "E_INITIAL_POSITION_IS_INVALID";                    return str; }

            case E_EMERGENCY_3310:                   { static std::string const str = "elmo 3310";                                        return str; }
            case E_EMERGENCY_7100:                   { static std::string const str = "elmo 7100";                                        return str; }
            case E_EMERGENCY_7300:                   { static std::string const str = "elmo 7300 encoder";                                return str; }

            case E_CAT_OUTOFMEMORY:                  { static std::string const str = "CAT_OUTOFMEMORY";                                  return str; }
            case E_CAT_INVALIDARG:                   { static std::string const str = "CAT_INVALIDARG";                                   return str; }
            case E_CAT_FAIL:                         { static std::string const str = "CAT_FAIL";                                         return str; }
            case E_CAT_NOTIMPL:                      { static std::string const str = "CAT_NOTIMPL";                                      return str; }
            case E_CAT_XML_OPEN:                     { static std::string const str = "CAT_XML_OPEN";                                     return str; }
            case E_CAT_XML_PARSE:                    { static std::string const str = "CAT_XML_PARSE";                                    return str; }
            case E_CAT_MASTER_NOT_CONFIGURED:        { static std::string const str = "CAT_MASTER_NOT_CONFIGURED";                        return str; }
            case E_CAT_MASTER_NOT_CONNECTED:         { static std::string const str = "CAT_MASTER_NOT_CONNECTED";                         return str; }
            case E_CAT_MASTER_ALREADY_CONNECTED:     { static std::string const str = "CAT_MASTER_ALREADY_CONNECTED";                     return str; }
            case E_CAT_INVALID_SLAVE_INDEX:          { static std::string const str = "CAT_INVALID_SLAVE_INDEX";                          return str; }
            case E_CAT_INVALID_TRANSITION:           { static std::string const str = "CAT_INVALID_TRANSITION";                           return str; }
            case E_CAT_DRIVER_LOAD:                  { static std::string const str = "CAT_DRIVER_LOAD";                                  return str; }
            case E_CAT_DRIVER_INIT:                  { static std::string const str = "CAT_DRIVER_INIT";                                  return str; }
            case E_CAT_DRIVER_NOT_SUPPORTED:         { static std::string const str = "CAT_DRIVER_NOT_SUPPORTED ";                        return str; }
            case E_CAT_DRIVER_NOT_LOADED:            { static std::string const str = "CAT_DRIVER_NOT_LOADED";                            return str; }
            case E_CAT_SEND_FRAME_FAILED:            { static std::string const str = "CAT_SEND_FRAME_FAILED";                            return str; }
            case E_CAT_RECV_FRAME_FAILED:            { static std::string const str = "CAT_RECV_FRAME_FAILED";                            return str; }
            case E_CAT_TRACE_BUFFER_OVERFLOW:        { static std::string const str = "CAT_TRACE_BUFFER_OVERFLOW";                        return str; }
            case E_CAT_NOTAVIABLE_TRANSITION:        { static std::string const str = "CAT_NOTAVIABLE_TRANSITION";                        return str; }
            case E_CAT_TRANSITION_ERROR:             { static std::string const str = "CAT_TRANSITION_ERROR";                             return str; }
            case E_CAT_INVALID_POINTER:              { static std::string const str = "CAT_INVALID_POINTER";                              return str; }
            case E_CAT_INVALID_COMMAND_TYPE:         { static std::string const str = "CAT_INVALID_COMMAND_TYPE";                         return str; }
            case E_CAT_TRANSITION_FORBIDDEN:         { static std::string const str = "CAT_TRANSITION_FORBIDDEN";                         return str; }
            case E_CAT_SYNC_CALL_TIMEOUT:            { static std::string const str = "CAT_SYNC_CALL_TIMEOUT";                            return str; }
            case E_CAT_MB_COE_TRANSITION_ABORTED:    { static std::string const str = "CAT_MB_COE_TRANSITION_ABORTED";                    return str; }
            case E_CAT_INVALID_WC:                   { static std::string const str = "CAT_INVALID_WC";                                   return str; }
            case E_CAT_INVALID_SLAVE_STATE:          { static std::string const str = "CAT_INVALID_SLAVE_STATE";                          return str; }
            case E_CAT_SNAPSHOT_DATA_LOSE:           { static std::string const str = "CAT_SNAPSHOT_DATA_LOSE";                           return str; }
            case E_CAT_XML_LICENSE_OPEN:             { static std::string const str = "CAT_XML_LICENSE_OPEN";                             return str; }
            case E_CAT_XML_LICENSE_PARSE:            { static std::string const str = "CAT_XML_LICENSE_PARSE";                            return str; }
            case E_CAT_XML_LICENSE_PRODUCT_NAME:     { static std::string const str = "CAT_XML_LICENSE_PRODUCT_NAME";                     return str; }
            case E_CAT_XML_LICENSE_LICENSED_TO:      { static std::string const str = "CAT_XML_LICENSE_LICENSED_TO";                      return str; }
            case E_CAT_XML_LICENSE_EXP_DATE:         { static std::string const str = "CAT_XML_LICENSE_EXP_DATE";                         return str; }
            case E_CAT_XML_LICENSE_DEMO_MODE:        { static std::string const str = "CAT_XML_LICENSE_DEMO_MODE";                        return str; }
            case E_CAT_XML_LICENSE_HW_BINDING:       { static std::string const str = "CAT_XML_LICENSE_HW_BINDING";                       return str; }
            case E_CAT_XML_LICENSE_PRODUCT_KEY:      { static std::string const str = "CAT_XML_LICENSE_PRODUCT_KEY";                      return str; }
            case E_CAT_CMD_ACYCLIC_SET_DATA:         { static std::string const str = "CAT_CMD_ACYCLIC_SET_DATA";                         return str; }
            case E_CAT_CMD_CONTAINER_ADD_RES_CMD:    { static std::string const str = "CAT_CMD_CONTAINER_ADD_RES_CMD";                    return str; }
            case E_CAT_BUFFER_SIZE_LIMIT:            { static std::string const str = "CAT_BUFFER_SIZE_LIMIT";                            return str; }
            case E_CAT_TELEGRAM_BUILD:               { static std::string const str = "CAT_TELEGRAM_BUILD";                               return str; }
            case E_CAT_FRAME_BUILD:                  { static std::string const str = "CAT_FRAME_BUILD";                                  return str; }
            case E_CAT_LOAD_FORBIDDEN:               { static std::string const str = "CAT_LOAD_FORBIDDEN";                               return str; }
            case E_CAT_INVALID_CONFIGURATION:        { static std::string const str = "CAT_INVALID_CONFIGURATION";                        return str; }
            case E_CAT_READ_SLAVE_STATES:            { static std::string const str = "CAT_READ_SLAVE_STATES";                            return str; }
            case E_CAT_TRANSIT_SLAVES:               { static std::string const str = "CAT_TRANSIT_SLAVES";                               return str; }
            case E_CAT_SEND_MASTER_CMDS:             { static std::string const str = "CAT_SEND_MASTER_CMDS";                             return str; }
            case E_CAT_MASTER_ALREADY_STARTED:       { static std::string const str = "CAT_MASTER_ALREADY_STARTED";                       return str; }
            case E_CAT_THREAD_INITIALISE:            { static std::string const str = "CAT_THREAD_INITIALISE";                            return str; }
            case E_CAT_READ_EEPROM:                  { static std::string const str = "CAT_READ_EEPROM";                                  return str; }
            case E_CAT_MB_NOT_SUPPORTED:             { static std::string const str = "CAT_MB_NOT_SUPPORTED";                             return str; }
            case E_CAT_MB_COE_NOT_SUPPORTED:         { static std::string const str = "CAT_MB_COE_NOT_SUPPORTED";                         return str; }
            case E_CAT_PI_ALREADY_UNLOCKED:          { static std::string const str = "CAT_PI_ALREADY_UNLOCKED";                          return str; }
            case E_CAT_INVALID_FRAME_SIZE:           { static std::string const str = "CAT_INVALID_FRAME_SIZE";                           return str; }
            case E_CAT_INVALID_FRAME_TYPE:           { static std::string const str = "CAT_INVALID_FRAME_TYPE";                           return str; }
            case E_CAT_INVALID_COMMAND_SIZE:         { static std::string const str = "CAT_INVALID_COMMAND_SIZE";                         return str; }
            case E_CAT_INVALID_COMMAND_DATA:         { static std::string const str = "CAT_INVALID_COMMAND_DATA";                         return str; }
            case E_CAT_WRITE_EEPROM:                 { static std::string const str = "CAT_WRITE_EEPROM";                                 return str; }
            case E_CAT_MASTER_NOT_STARTED:           { static std::string const str = "CAT_MASTER_NOT_STARTED";                           return str; }
            case E_CAT_ARRAY_BOUNDS_EXCEEDED:        { static std::string const str = "CAT_ARRAY_BOUNDS_EXCEEDED";                        return str; }
            case E_CAT_INVALID_LICENSE:              { static std::string const str = "CAT_INVALID_LICENSE";                              return str; }
            case E_CAT_RECV_FRAME_TIMEOUT:           { static std::string const str = "CAT_RECV_FRAME_TIMEOUT";                           return str; }
            case E_CAT_EEPROM_ACCESS_DENIED:         { static std::string const str = "CAT_EEPROM_ACCESS_DENIED";                         return str; }
            case E_CAT_BUSY:                         { static std::string const str = "CAT_BUSY";                                         return str; }
            case E_CAT_TIME:                         { static std::string const str = "CAT_TIME";                                         return str; }
            case E_CAT_ABORTED:                      { static std::string const str = "CAT_ABORTED";                                      return str; }
            case E_CAT_SLAVE_PI_NOT_EXIST:           { static std::string const str = "CAT_SLAVE_PI_NOT_EXIST";                           return str; }
            case E_CAT_DC_SYNC:                      { static std::string const str = "CAT_DC_SYNC";                                      return str; }
            case E_CAT_OPEN_FILE:                    { static std::string const str = "CAT_OPEN_FILE";                                    return str; }
            case E_CAT_WRITE_FILE:                   { static std::string const str = "CAT_WRITE_FILE";                                   return str; }
            case E_CAT_READ_FILE:                    { static std::string const str = "CAT_READ_FILE";                                    return str; }
            case E_CAT_CANT_GET_HW_KEY:              { static std::string const str = "CAT_CANT_GET_HW_KEY";                              return str; }
            case E_CAT_CANT_ENCODE_DATA:             { static std::string const str = "CAT_CANT_ENCODE_DATA";                             return str; }
            case E_CAT_CANT_DECODE_DATA:             { static std::string const str = "CAT_CANT_DECODE_DATA";                             return str; }
            case E_CAT_MB_COE_UNSUPPORTED_DATA_TYPE: { static std::string const str = "CAT_MB_COE_UNSUPPORTED_DATA_TYP";                  return str; }
            case E_CAT_MB_COE_OBJECT_IS_READ_ONLY:   { static std::string const str = "CAT_MB_COE_OBJECT_IS_READ_ONLY";                   return str; }
            case E_CAT_MB_COE_OBJECT_IS_WRITE_ONLY:  { static std::string const str = "CAT_MB_COE_OBJECT_IS_WRITE_ONLY";                  return str; }
            case E_CAT_MB_COE_INCOMPLETE_ENTRY_DESC: { static std::string const str = "CAT_MB_COE_INCOMPLETE_ENTRY_DES";                  return str; }
            case E_CAT_MB_COE_SDOINFO_ERROR:         { static std::string const str = "CAT_MB_COE_SDOINFO_ERROR";                         return str; }
            case E_CAT_MB_VOE_NOT_SUPPORTED:         { static std::string const str = "CAT_MB_VOE_NOT_SUPPORTED";                         return str; }
            case E_CAT_MASTER_IS_LOCKED:             { static std::string const str = "CAT_MASTER_IS_LOCKED";                             return str; }
            case E_CAT_MASTER_NO_RIGHTS:             { static std::string const str = "CAT_MASTER_NO_RIGHTS";                             return str; }
            case E_CAT_MASTER_USER_NOT_FOUND:        { static std::string const str = "CAT_MASTER_USER_NOT_FOUND";                        return str; }
            case E_CAT_AOE_ERROR_CODE:               { static std::string const str = "CAT_AOE_ERROR_CODE";                               return str; }
            case E_CAT_AOE_CMD_ERROR:                { static std::string const str = "CAT_AOE_CMD_ERROR";                                return str; }
            case E_CAT_AOE_INVALID_HEADER:           { static std::string const str = "CAT_AOE_INVALID_HEADER";                           return str; }
            case E_CAT_MB_AOE_NOT_SUPPORTED:         { static std::string const str = "CAT_MB_AOE_NOT_SUPPORTED";                         return str; }
            case E_CAT_AOE_INVALID_ROUTE:            { static std::string const str = "CAT_AOE_INVALID_ROUTE";                            return str; }
            case E_CAT_MB_SOE_NOT_SUPPORTED:         { static std::string const str = "CAT_MB_SOE_NOT_SUPPORTED";                         return str; }
            case E_CAT_SOE_ERROR_CODE:               { static std::string const str = "CAT_SOE_ERROR_CODE";                               return str; }
            case E_CAT_SOE_INVALID_HEADER:           { static std::string const str = "CAT_SOE_INVALID_HEADER";                           return str; }
            case E_CAT_SOE_FRAGMENT_LEFT:            { static std::string const str = "CAT_SOE_FRAGMENT_LEFT";                            return str; }
            case E_CAT_OD_OBJECT_NOTFOUND:           { static std::string const str = "CAT_OD_OBJECT_NOTFOUND";                           return str; }
            case E_CAT_OD_OBJECT_ALREADY_EXISTS:     { static std::string const str = "CAT_OD_OBJECT_ALREADY_EXISTS";                     return str; }
            case E_CAT_OD_ENTRY_DSC_ALREADY_EXISTS:  { static std::string const str = "CAT_OD_ENTRY_DSC_ALREADY_EXISTS";                  return str; }
            case E_CAT_OD_ENTRY_DSC_FAILED:          { static std::string const str = "CAT_OD_ENTRY_DSC_FAILED";                          return str; }
            case E_CAT_OD_INVALID_OBJECT_TYPE:       { static std::string const str = "CAT_OD_INVALID_OBJECT_TYPE";                       return str; }
            case E_CAT_OD_INVALID_ACCESS_TYPE:       { static std::string const str = "CAT_OD_INVALID_ACCESS_TYPE";                       return str; }
            case E_CAT_OD_INVALID_DATA_TYPE:         { static std::string const str = "CAT_OD_INVALID_DATA_TYPE";                         return str; }
            case E_CAT_OD_ACCESS_DENIED:             { static std::string const str = "CAT_OD_ACCESS_DENIED";                             return str; }
            case E_CAT_OD_NOT_CREATED:               { static std::string const str = "CAT_OD_NOT_CREATED";                               return str; }
            case E_CAT_OD_SDO_SERVICE_NOT_SUPORTED:  { static std::string const str = "CAT_OD_SDO_SERVICE_NOT_SUPORTED";                  return str; }
            case E_CAT_OD_SDO_SIZE_INVALID_HEADER:   { static std::string const str = "CAT_OD_SDO_SIZE_INVALID_HEADER";                   return str; }
            case E_CAT_OD_SDO_SIZE_TOO_SHORT:        { static std::string const str = "CAT_OD_SDO_SIZE_TOO_SHORT";                        return str; }
            case E_CAT_OD_SDO_INVALID_SIZE:          { static std::string const str = "CAT_OD_SDO_INVALID_SIZE";                          return str; }
            case E_CAT_OD_SUB_OBJ_NOTFOUND:          { static std::string const str = "CAT_OD_SUB_OBJ_NOTFOUND";                          return str; }
            case E_CAT_OD_MORE_MAXIMUM_VALUE:        { static std::string const str = "CAT_OD_MORE_MAXIMUM_VALUE";                        return str; }
            case E_CAT_OD_LESS_MINIMUM_VALUE:        { static std::string const str = "CAT_OD_LESS_MINIMUM_VALUE";                        return str; }
            case E_CAT_LICENSE_INIT:                 { static std::string const str = "CAT_LICENSE_INIT";                                 return str; }
            case E_CAT_LICENSE_LOAD:                 { static std::string const str = "CAT_LICENSE_LOAD";                                 return str; }
            case E_CAT_LICENSE_INVALID_TARGET:       { static std::string const str = "CAT_LICENSE_INVALID_TARGET";                       return str; }
            case E_CAT_LICENSE_EXPIRED:              { static std::string const str = "CAT_LICENSE_EXPIRED";                              return str; }
            case E_CAT_LICENSE_INVALID_HW:           { static std::string const str = "CAT_LICENSE_INVALID_HW";                           return str; }
            case E_CAT_NO_CONFIG:                    { static std::string const str = "CAT_NO_CONFIG";                                    return str; }
            case E_CAT_INVAL_MASTER_CONF:            { static std::string const str = "CAT_INVAL_MASTER_CONF";                            return str; }
            case E_CAT_ALREADY_INITIALIZED:          { static std::string const str = "CAT_ALREADY_INITIALIZED";                          return str; }
            case E_CAT_DRIVER_INVALID_NIC:           { static std::string const str = "CAT_DRIVER_INVALID_NIC";                           return str; }

            case E_UNKNOWN_ERRNO:
            default:                 { static std::string const str = "Unknown error code"; return str; }
            // LCOV_EXCL_BR_STOP
        }
    }
}

    bool_t failed(hresult_t const& hresultIn)
    {
        return (error::S_OK != hresultIn);
    }


    bool_t succeeded(hresult_t const& hresultIn)
    {
        return (error::S_OK == hresultIn);
    }


    bool_t assertFail(
        char_t const* const conditionStringIn,
        char_t const* const fileNameIn,
        std::size_t const lineNumberIn,
        char_t const* const functionNameIn)
    {
        core::Logger::error("%s:%d: %s: Assertion %s failed.",
                            fileNameIn, lineNumberIn, functionNameIn, conditionStringIn);
        std::abort();
        return false;
    }
}
