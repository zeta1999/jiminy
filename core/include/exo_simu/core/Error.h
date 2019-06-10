///////////////////////////////////////////////////////////////////////////////
///
/// \file              definition/Error.h
/// \brief             Define result handles.
/// \details           Define main result handles which can (and must) be
///                    returned by all function. Result handles are
///                    largely inspired from Microsoft's HRESULT.
///
/// \copyright         Wandercraft
///
////////////////////////////////////////////////////////////////////////////////

#ifndef WDC_DEFINITION_ERROR_H
#define WDC_DEFINITION_ERROR_H

// System.
#include <string>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/AbstractErrorHandler.h"
#include "exo_simu/core/Constants.h"

namespace exo_simu
{
    namespace error
    {
        int32_t constexpr WHERE_LENGTH = 64;
        int32_t constexpr ERROR_STRING_LENGTH = 48;
        int32_t constexpr ERROR_MESSAGE_LENGTH = 256;

        namespace level
        {
            /// Enum to describe the error level. The categories are inspired from the Linux kernel.
            enum Enum
            {
                WARNING,   ///< A warning, meaning nothing serious by itself but might indicate problems.
                ERROR,     ///< An error condition. The reporter shall manage it by itself.
                CRITICAL,  ///< A critical condition occurred, but it still managed by the reporter.
                ALERT,     ///< Something bad happened and action must be taken immediately by the error manager.
                EMERGENCY  ///< Emergency! The system is about to crash or is unstable. Safety action must be taken.
            };
        } // End of namespace level.


        /// structure to manage reported error. For internal use only.
        struct report
        {
            uint64_t timestampUs; ///< Last time the error was processed.
            uint32_t counter;     ///< Number of time the error was reported between two processing.
        };

        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \var hresult_t const S_OK
        ///
        /// The main success code.
        ///
        ///////////////////////////////////////////////////////////////////////////////
        hresult_t const S_OK = 0x00000000U;

        /// Wanderbrain errno equivalent
        hresult_t const E_UNKNOWN_ERRNO    = 0x00000001U;
        hresult_t const E_EACCES           = 0x00000002U;
        hresult_t const E_EAGAIN           = 0x00000003U;
        hresult_t const E_EBADF            = 0x00000004U;
        hresult_t const E_EBUSY            = 0x00000005U;
        hresult_t const E_ECANCELED        = 0x00000006U;
        hresult_t const E_EDEADLK          = 0x00000007U;
        hresult_t const E_EINTR            = 0x00000008U;
        hresult_t const E_EINVAL           = 0x00000009U;
        hresult_t const E_EMSGSIZE         = 0x0000000aU;
        hresult_t const E_ENODEV           = 0x0000000bU;
        hresult_t const E_ENOENT           = 0x0000000cU;
        hresult_t const E_ENOMEM           = 0x0000000dU;
        hresult_t const E_ENOMSG           = 0x0000000eU;
        hresult_t const E_ENOSYS           = 0x0000000fU;  ///! Function not implemented
        hresult_t const E_ENOTDIR          = 0x00000010U;
        hresult_t const E_ENOTSOCK         = 0x00000011U;
        hresult_t const E_ENOTSUP          = 0x00000012U;
        hresult_t const E_EOVERFLOW        = 0x00000013U;
        hresult_t const E_EPROTO           = 0x00000014U;
        hresult_t const E_EIO              = 0x00000015U;
        hresult_t const E_ETIME            = 0x00000016U;
        hresult_t const E_ETIMEDOUT        = 0x00000017U;
        hresult_t const E_ELOOP            = 0x00000019U;
        hresult_t const E_EFAULT           = 0x0000001aU;
        hresult_t const E_ENAMETOOLONG     = 0x0000001bU;
        hresult_t const E_ESRCH            = 0x0000001cU;
        hresult_t const E_EPERM            = 0x0000001dU;
        hresult_t const E_ENXIO            = 0x0000001eU;
        hresult_t const E_E2BIG            = 0x0000001fU;
        hresult_t const E_ECHILD           = 0x00000020U;
        hresult_t const E_EEXIST           = 0x00000021U;
        hresult_t const E_EXDEV            = 0x00000022U;
        hresult_t const E_EISDIR           = 0x00000023U;
        hresult_t const E_ENFILE           = 0x00000024U;
        hresult_t const E_EMFILE           = 0x00000025U;
        hresult_t const E_ENOTTY           = 0x00000026U;
        hresult_t const E_ETXTBSY          = 0x00000027U;
        hresult_t const E_EFBIG            = 0x00000028U;
        hresult_t const E_ENOSPC           = 0x00000029U;
        hresult_t const E_ESPIPE           = 0x0000002aU;
        hresult_t const E_EROFS            = 0x0000002bU;
        hresult_t const E_EMLINK           = 0x0000002cU;
        hresult_t const E_EDOM             = 0x0000002dU;
        hresult_t const E_ERANGE           = 0x0000002eU;
        hresult_t const E_ENOTEMPTY        = 0x0000002fU;
        hresult_t const E_ENODATA          = 0x00000030U;
        hresult_t const E_EALREADY         = 0x00000031U;
        hresult_t const E_EBADR            = 0x00000032U;
        hresult_t const E_EBADE            = 0x00000033U;
        hresult_t const E_EADDRINUSE       = 0x00000034U;
        hresult_t const E_EOPNOTSUPP       = 0x00000035U;

        hresult_t const E_EIDRM            = 0x00000051U;
        hresult_t const E_ENOSR            = 0x00000052U;
        hresult_t const E_EBADMSG          = 0x00000054U;
        hresult_t const E_ENOSTR           = 0x00000057U;

        /// Wandercraft error code
        hresult_t const E_GENERIC_ERROR    = 0x00000100U;
        hresult_t const E_CRITICAL_BATTERY = 0x00000101U;
        hresult_t const E_LOW_BATTERY      = 0x00000102U;
        hresult_t const E_MOTOR_FAULT      = 0x00000103U;
        hresult_t const E_UNDERVOLTAGE     = 0x00000104U;
        hresult_t const E_PRECONDITION     = 0x00000105U;
        hresult_t const E_POSTCONDITION    = 0x00000106U;
        hresult_t const E_INCONSISTENT_ESTIMATE = 0x00000107U;
        hresult_t const E_WRONG_BRAIN_4_DEV     = 0x00000108U;
        hresult_t const E_MOTOR_WATCHDOG        = 0x00000109U;
        hresult_t const E_FAILED_TO_SOLVE_TASK  = 0x0000010aU;
        hresult_t const E_NO_SOLUTION           = 0x0000010bU;
        hresult_t const E_MOTOR_START_TIMEOUT   = 0x0000010cU;
        hresult_t const E_VERTICAL              = 0x0000010dU;
        hresult_t const E_COMPUTE               = 0x0000010eU;
        hresult_t const E_MOTOR_DISABLED        = 0x0000010fU;
        hresult_t const E_UNDEFINED_BEHAVIOR    = 0x00000110U;
        hresult_t const E_NO_ACTIVE_SESSION     = 0x00000111U;
        hresult_t const E_ALREADY_IN_SESSION    = 0x00000112U;
        hresult_t const E_NOT_IN_INSTALLATION   = 0x00000113U;
        hresult_t const E_COMPRESS              = 0x00000114U;

        /// HighController error codes.
        hresult_t const INVALID_NB_AXIS               = 0x00001000U;
        hresult_t const UPDATE_PERIOD_NOT_CONSISTENT  = 0x00001001U;
        hresult_t const E_SET_FEEDFORWARD             = 0x00001002U;
        hresult_t const E_INVERSE_DYNAMICS            = 0x00001003U;
        hresult_t const E_WANDERADAPTER_COMPUTECOP    = 0x00001004U;
        hresult_t const E_AXIS_ORDER_MISMATCH         = 0x00001005U;

        hresult_t const E_FRICTION                    = 0x00001006U;
        hresult_t const E_UNKNOWN_FOOT_CONSTRAINTS    = 0x00001007U;
        hresult_t const E_FREEFLYER_STATE_ESTIMATE    = 0x00001008U;
        hresult_t const E_UPDATE_RIGID_MODEL_STATE    = 0x00001009U;
        hresult_t const E_UPDATE_COP                  = 0x0000100aU;
        hresult_t const E_TARGET_COMPUTATION          = 0x0000100bU;

        hresult_t const E_FILTER_VELOCITY             = 0x0000100cU;
        hresult_t const E_INITIAL_POSITION_IS_INVALID = 0x0000100dU;
        hresult_t const E_STATIONARY_POINT            = 0x0000100eU;
        hresult_t const E_NOT_INITIALIZED             = 0x0000100fU;

        /// Ethercat emergency error code.
        hresult_t const E_EMERGENCY_3310   = 0x00003310U;
        hresult_t const E_EMERGENCY_7100   = 0x00007100U;
        hresult_t const E_EMERGENCY_7300   = 0x00007300U;


        /// KPA Ethercat error codes.
        hresult_t const E_CAT_OUTOFMEMORY                  = 0x8001;   /*!< Memory limit(usually can not allocate new memory). */
        hresult_t const E_CAT_INVALIDARG                   = 0x8002;   /*!< Function calls with invalid arguments. */
        hresult_t const E_CAT_FAIL                         = 0x8003;   /*!< General error. */
        hresult_t const E_CAT_NOTIMPL                      = 0x8004;   /*!< Function not implemented. */
        hresult_t const E_CAT_XML_OPEN                     = 0x8005;   /*!< Can't open xml file. */
        hresult_t const E_CAT_XML_PARSE                    = 0x8006;   /*!< Xml configuration file contains errors. */
        hresult_t const E_CAT_MASTER_NOT_CONFIGURED        = 0x8007;   /*!< Master not configured. You must configure master before call. */
        hresult_t const E_CAT_MASTER_NOT_CONNECTED         = 0x8008;   /*!< Master not connected. You must connect master before call. */
        hresult_t const E_CAT_MASTER_ALREADY_CONNECTED     = 0x8009;   /*!< Master already connected. */
        hresult_t const E_CAT_INVALID_SLAVE_INDEX          = 0x800A;   /*!< Invalid slave index. */
        hresult_t const E_CAT_INVALID_TRANSITION           = 0x800B;   /*!< Invalid transition. Can't transit master to request state. */
        hresult_t const E_CAT_DRIVER_LOAD                  = 0x800C;   /*!< Can't load driver. */
        hresult_t const E_CAT_DRIVER_INIT                  = 0x800D;   /*!< Can't initialize driver. */
        hresult_t const E_CAT_DRIVER_NOT_SUPPORTED         = 0x800E;   /*!< Driver not supported. */
        hresult_t const E_CAT_DRIVER_NOT_LOADED            = 0x800F;   /*!< Driver not loaded. */
        hresult_t const E_CAT_SEND_FRAME_FAILED            = 0x8010;   /*!< Can't send frame. */
        hresult_t const E_CAT_RECV_FRAME_FAILED            = 0x8011;   /*!< Can't receive frame. */
        hresult_t const E_CAT_TRACE_BUFFER_OVERFLOW        = 0x8012;   /*!< Trace buffer overflow. */
        hresult_t const E_CAT_NOTAVIABLE_TRANSITION        = 0x8013;   /*!< Incorrect transition for current master state. */
        hresult_t const E_CAT_TRANSITION_ERROR             = 0x8014;   /*!< Transition error. */
        hresult_t const E_CAT_INVALID_POINTER              = 0x8015;   /*!< Invalid pointer. */
        hresult_t const E_CAT_INVALID_COMMAND_TYPE         = 0x8016;   /*!< Invalid command type. */
        hresult_t const E_CAT_TRANSITION_FORBIDDEN         = 0x8017;   /*!< Transition forbidden (can't send frames to the network). */
        hresult_t const E_CAT_SYNC_CALL_TIMEOUT            = 0x8018;   /*!< Call timeout. */
        hresult_t const E_CAT_MB_COE_TRANSITION_ABORTED    = 0x8019;   /*!< Mailbox CanOpen transition aborted. */
        hresult_t const E_CAT_INVALID_WC                   = 0x801A;   /*!< Invalid working counter received. */
        hresult_t const E_CAT_INVALID_SLAVE_STATE          = 0x801B;   /*!< Invalid slave state. */
        hresult_t const E_CAT_SNAPSHOT_DATA_LOSE           = 0x801C;   /*!< Data lose at snapshot call. */
        hresult_t const E_CAT_XML_LICENSE_OPEN             = 0x801D;   /*!< Can't open license xml file. */
        hresult_t const E_CAT_XML_LICENSE_PARSE            = 0x801E;   /*!< License Xml configuration file contains errors. */
        hresult_t const E_CAT_XML_LICENSE_PRODUCT_NAME     = 0x801F;   /*!< License Xml configuration file doesn't contain Product tag. */
        hresult_t const E_CAT_XML_LICENSE_LICENSED_TO      = 0x8020;   /*!< License Xml configuration file doesn't contain LicensedTo tag. */
        hresult_t const E_CAT_XML_LICENSE_EXP_DATE         = 0x8021;   /*!< License Xml configuration file doesn't contain ExpirationDate tag. */
        hresult_t const E_CAT_XML_LICENSE_DEMO_MODE        = 0x8022;   /*!< License Xml configuration file doesn't contain DemoMode tag. */
        hresult_t const E_CAT_XML_LICENSE_HW_BINDING       = 0x8023;   /*!< License Xml configuration file doesn't contain HardwareBinding tag. */
        hresult_t const E_CAT_XML_LICENSE_PRODUCT_KEY      = 0x8024;   /*!< License Xml configuration file doesn't contain ProductKey tag. */
        hresult_t const E_CAT_CMD_ACYCLIC_SET_DATA         = 0x8025;   /*!< Acyclic cmd data is not set. */
        hresult_t const E_CAT_CMD_CONTAINER_ADD_RES_CMD    = 0x8026;   /*!< Can't add reserved cmd to command container. */
        hresult_t const E_CAT_BUFFER_SIZE_LIMIT            = 0x8027;   /*!< Buffer size limit. */
        hresult_t const E_CAT_TELEGRAM_BUILD               = 0x8028;   /*!< Can't build ecat telegram. */
        hresult_t const E_CAT_FRAME_BUILD                  = 0x8029;   /*!< Can't build ecat frame. */
        hresult_t const E_CAT_LOAD_FORBIDDEN               = 0x802A;   /*!< Can't load configuration at current master state. */
        hresult_t const E_CAT_INVALID_CONFIGURATION        = 0x802B;   /*!< Can't load configuration because verification errors. */
        hresult_t const E_CAT_READ_SLAVE_STATES            = 0x802C;   /*!< Can't read slave states from network. */
        hresult_t const E_CAT_TRANSIT_SLAVES               = 0x802D;   /*!< Can't transit slave to request state. */
        hresult_t const E_CAT_SEND_MASTER_CMDS             = 0x802E;   /*!< Can't send master init commands. */
        hresult_t const E_CAT_MASTER_ALREADY_STARTED       = 0x802F;   /*!< Master already started. */
        hresult_t const E_CAT_THREAD_INITIALISE            = 0x8030;   /*!< Can't initialize thread. */
        hresult_t const E_CAT_READ_EEPROM                  = 0x8031;   /*!< Error occurred while reading EEPROM. */
        hresult_t const E_CAT_MB_NOT_SUPPORTED             = 0x8032;   /*!< Mailbox not supported. */
        hresult_t const E_CAT_MB_COE_NOT_SUPPORTED         = 0x8033;   /*!< Mailbox CoE not supported. */
        hresult_t const E_CAT_PI_ALREADY_UNLOCKED          = 0x8034;   /*!< Process Image already unlocked. */
        hresult_t const E_CAT_INVALID_FRAME_SIZE           = 0x8035;   /*!< Process frame error: invalid frame size. */
        hresult_t const E_CAT_INVALID_FRAME_TYPE           = 0x8036;   /*!< Process frame error: unsupported EtherCAT&reg; frame type. */
        hresult_t const E_CAT_INVALID_COMMAND_SIZE         = 0x8037;   /*!< Process frame error: invalid command length. */
        hresult_t const E_CAT_INVALID_COMMAND_DATA         = 0x8038;   /*!< Process frame error: invalid internal data. */
        hresult_t const E_CAT_WRITE_EEPROM                 = 0x8039;   /*!< Error occurred while writing EEPROM. */
        hresult_t const E_CAT_MASTER_NOT_STARTED           = 0x803A;   /*!< Master not started. You must start master before call. */
        hresult_t const E_CAT_ARRAY_BOUNDS_EXCEEDED        = 0x803B;   /*!< Array bounds error exceeded. */
        hresult_t const E_CAT_INVALID_LICENSE              = 0x803C;   /*!< Invalid license. */
        hresult_t const E_CAT_RECV_FRAME_TIMEOUT           = 0x803D;   /*!< Frame receive time-out. */
        hresult_t const E_CAT_EEPROM_ACCESS_DENIED         = 0x803E;   /*!< Access to EEPROM temporarily denied. */
        hresult_t const E_CAT_BUSY                         = 0x803F;   /*!< Busy. */
        hresult_t const E_CAT_TIME                         = 0x8040;   /*!< Invalid time. */
        hresult_t const E_CAT_ABORTED                      = 0x8041;   /*!< Call has been aborted. */
        hresult_t const E_CAT_SLAVE_PI_NOT_EXIST           = 0x8042;   /*!< No slave PI region found. */
        hresult_t const E_CAT_DC_SYNC                      = 0x8043;   /*!< EtherCAT DC synchronization error. */
        hresult_t const E_CAT_OPEN_FILE                    = 0x8050;   /*!< Can't open file. */
        hresult_t const E_CAT_WRITE_FILE                   = 0x8051;   /*!< Can't write to file. */
        hresult_t const E_CAT_READ_FILE                    = 0x8052;   /*!< Can't read from file. */
        hresult_t const E_CAT_CANT_GET_HW_KEY              = 0x8053;   /*!< Can't obtain hardware key. */
        hresult_t const E_CAT_CANT_ENCODE_DATA             = 0x8054;   /*!< Can't encode data. */
        hresult_t const E_CAT_CANT_DECODE_DATA             = 0x8055;   /*!< Can't decode data. */
        hresult_t const E_CAT_MB_COE_UNSUPPORTED_DATA_TYPE = 0x8100;   /*!< CoE: Unsupported CANOpen data type. */
        hresult_t const E_CAT_MB_COE_OBJECT_IS_READ_ONLY   = 0x8101;   /*!< CoE: Access to read only object. */
        hresult_t const E_CAT_MB_COE_OBJECT_IS_WRITE_ONLY  = 0x8102;   /*!< CoE: Access to write only object. */
        hresult_t const E_CAT_MB_COE_INCOMPLETE_ENTRY_DESC = 0x8103;   /*!< CoE: Incomplete object entry description. */
        hresult_t const E_CAT_MB_COE_SDOINFO_ERROR         = 0x8104;   /*!< CoE: SDO information error response. */
        hresult_t const E_CAT_MB_VOE_NOT_SUPPORTED         = 0x8150;   /*!< Mailbox VoE not supported. */
        hresult_t const E_CAT_MASTER_IS_LOCKED             = 0x8200;   /*!< Master is already locked. */
        hresult_t const E_CAT_MASTER_NO_RIGHTS             = 0x8201;   /*!< No rights to complete this operation. */
        hresult_t const E_CAT_MASTER_USER_NOT_FOUND        = 0x8202;   /*!< User not found in the configuration. */
        hresult_t const E_CAT_AOE_ERROR_CODE               = 0x8300;   /*!< Error code in AoE header is received. */
        hresult_t const E_CAT_AOE_CMD_ERROR                = 0x8301;   /*!< Result in AoE command response not 0. */
        hresult_t const E_CAT_AOE_INVALID_HEADER           = 0x8302;   /*!< Failed to build/parse AoE header. */
        hresult_t const E_CAT_MB_AOE_NOT_SUPPORTED         = 0x8303;   /*!< Mailbox AoE not supported. */
        hresult_t const E_CAT_AOE_INVALID_ROUTE            = 0x8304;   /*!< Route not found. */
        hresult_t const E_CAT_MB_SOE_NOT_SUPPORTED         = 0x8310;   /*!< Mailbox SoE not supported. */
        hresult_t const E_CAT_SOE_ERROR_CODE               = 0x8311;   /*!< Error bit in SoE response is true. See error code for more information. */
        hresult_t const E_CAT_SOE_INVALID_HEADER           = 0x8312;   /*!< Failed to build/parse SoE header. */
        hresult_t const E_CAT_SOE_FRAGMENT_LEFT            = 0x8313;   /*!< SoE Fragment is left. */
        hresult_t const E_CAT_OD_OBJECT_NOTFOUND           = 0x8400;   /*!< No object in OD. */
        hresult_t const E_CAT_OD_OBJECT_ALREADY_EXISTS     = 0x8401;   /*!< Object is already in OD. */
        hresult_t const E_CAT_OD_ENTRY_DSC_ALREADY_EXISTS  = 0x8402;   /*!< OD entry description already exists. */
        hresult_t const E_CAT_OD_ENTRY_DSC_FAILED          = 0x8403;   /*!< Failed to create OD entry description. */
        hresult_t const E_CAT_OD_INVALID_OBJECT_TYPE       = 0x8404;   /*!< OD object type is invalid. */
        hresult_t const E_CAT_OD_INVALID_ACCESS_TYPE       = 0x8405;   /*!< OD access type is invalid.  */
        hresult_t const E_CAT_OD_INVALID_DATA_TYPE         = 0x8406;   /*!< Invalid OD data type.  */
        hresult_t const E_CAT_OD_ACCESS_DENIED             = 0x8407;   /*!< Access to OD denied. */
        hresult_t const E_CAT_OD_NOT_CREATED               = 0x8408;   /*!< OD has not been created. */
        hresult_t const E_CAT_OD_SDO_SERVICE_NOT_SUPORTED  = 0x8409;   /*!< Mailbox error: not supported CoE service in CoE header. */
        hresult_t const E_CAT_OD_SDO_SIZE_INVALID_HEADER   = 0x840A;   /*!< Mailbox error: invalid CoE SDO header. */
        hresult_t const E_CAT_OD_SDO_SIZE_TOO_SHORT        = 0x840B;   /*!< Mailbox error: CoE SDO service with Len < 10. */
        hresult_t const E_CAT_OD_SDO_INVALID_SIZE          = 0x840C;   /*!< Mailbox error: invalid size. */
        hresult_t const E_CAT_OD_SUB_OBJ_NOTFOUND          = 0x840D;   /*!< Sub-object not found in object. */
        hresult_t const E_CAT_OD_MORE_MAXIMUM_VALUE        = 0x840E;   /*!< Entry value is more than maximum. */
        hresult_t const E_CAT_OD_LESS_MINIMUM_VALUE        = 0x840F;   /*!< Entry value is less than minimum. */
        hresult_t const E_CAT_LICENSE_INIT                 = 0x8502;   /*!< Can't initialize licensing. Check licensing configuration */
        hresult_t const E_CAT_LICENSE_LOAD                 = 0x8503;   /*!< License load error (invalid(not existing) license file, inconsistent content etc. ). */
        hresult_t const E_CAT_LICENSE_INVALID_TARGET       = 0x8504;   /*!< Invalid licensing target (product, OS, class etc.)*/
        hresult_t const E_CAT_LICENSE_EXPIRED              = 0x8505;   /*!< License time expired. */
        hresult_t const E_CAT_LICENSE_INVALID_HW           = 0x8506;   /*!< Valid hardware is not detected. */
        hresult_t const E_CAT_NO_CONFIG                    = 0x8507;   /*!< LicenseFile parameter cannot be found. */
        hresult_t const E_CAT_INVAL_MASTER_CONF            = 0x8508;   /*!< Path to the license file isn't specified. */
        hresult_t const E_CAT_ALREADY_INITIALIZED          = 0x8509;   /*!< Master library already initialized. */
        hresult_t const E_CAT_DRIVER_INVALID_NIC           = 0x8060;   /*!< Ethernet adapter cannot be found. */

        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \brief Return a string that represent the error code.
        ///
        /// \param[in] errorIn  The error code.
        ///
        /// \return The string that represent the error code.
        ///
        ///////////////////////////////////////////////////////////////////////////////
        std::string const& getErrorString(hresult_t errorIn);

        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \brief Convert an errno error code to a Wanderbrain error code
        ///
        /// \param[in] errnoIn  the errno error code to convert
        ///
        /// \retval the corresponding wanderbrain error
        /// \retval E_UNKNOWN_ERRNO if the errno code is unknown
        /// \retval S_OK if successful
        ///
        ///////////////////////////////////////////////////////////////////////////////
        hresult_t errnoToHresult(int32_t const& errnoIn);

        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \brief Configure the process error handler.
        ///
        /// \param[in] handlerIn A pointer on the global error handler.
        ///
        ///////////////////////////////////////////////////////////////////////////////
        void setErrorHandler(AbstractErrorHandler* handlerIn);

        ///////////////////////////////////////////////////////////////////////////////
        ///
        /// \brief Configure the error filtering window.
        ///
        /// \param[in] windowMsIn  New value of the window (in milliseconds).
        ///
        ///////////////////////////////////////////////////////////////////////////////
        void setErrorFilteringWindow(uint64_t windowMsIn = MILLISECONDS_IN_SECOND);

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Clear the currently configured error handler.
        ///////////////////////////////////////////////////////////////////////////////
        void clearErrorHandler();

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Enable the process death handler.
        ///////////////////////////////////////////////////////////////////////////////
        void enableDeathHandler();

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Process an error code.
        ///
        /// \details    Filter the reported code following the previous calls and the
        ///             current level.
        ///
        /// \param[in]  levelIn     Level of the reported error.
        /// \param[in]  errorIn     Reported error code.
        /// \param[out] levelOut    Level to display.
        /// \param[out] counterOut  Counter value to display.
        ///
        /// \return true if the error shall be displayed, false otherwise (not an error or filtered).
        ///////////////////////////////////////////////////////////////////////////////
        bool_t processError(error::level::Enum levelIn, hresult_t errorIn, std::string& levelOut, uint32_t& counterOut);

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Print the error (this function is spearated to avoid the templated
        ///        part to include the logger).
        ///////////////////////////////////////////////////////////////////////////////
        void printError(char_t errorMessage[ERROR_MESSAGE_LENGTH]);
    }  // end of namespace error

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Check if result handle corresponds to a failure.
    ///
    /// \param[in] hresultIn The result handle.
    ///
    /// \return true in case of failure, false otherwise.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    bool_t failed(hresult_t const& hresultIn);

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Check if result handle corresponds to a success.
    ///
    /// \param[in] hresultIn The result handle.
    ///
    /// \return true in case of success, false otherwise.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    bool_t succeeded(hresult_t const& hresultIn);

    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Report an error to the error manager.
    ///
    /// \param[in] levelIn     the level of the error (cf. level::Enum).
    /// \param[in] facilityIn  the value corresponding to a module. Should be in bits 16 and more.
    /// \param[in] errorIn     the reported error code.
    /// \param[in] lineIn      the line of the file where the error was reported (should be __LINE__)
    /// \param[in] legend1     legend for the first optional value.
    /// \param[in] val1        optional value that the reporter may add to understand the error.
    /// \param[in] legend2     legend for the second optional value.
    /// \param[in] val2        optional value that the reporter may add to understand the error.
    /// \param[in] legend3     legend for the third optional value.
    /// \param[in] val3        optional value that the reporter may add to understand the error.
    /// \param[in] legend4     legend for the fourth optional value.
    /// \param[in] val4        optional value that the reporter may add to understand the error.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    template<typename T1, typename T2, typename T3, typename T4>
    void reportError(error::level::Enum levelIn, std::string const& where, hresult_t errorIn, int32_t lineIn,
                     std::string const& legend1, T1 val1,
                     std::string const& legend2, T2 val2,
                     std::string const& legend3, T3 val3,
                     std::string const& legend4, T4 val4);

    //////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Handle an assert failure.
    ///
    /// \details Output the failed test, file name, line number and
    ///          function name causing the assertion to fail.
    ///
    /// \param[in] conditionStringIn The stringified boolean test.
    /// \param[in] fileNameIn The file name where the failure occured.
    /// \param[in] lineNumberIn The line number where the failure occured.
    /// \param[in] functionNameIn The function name where the failure occured.
    ///
    //////////////////////////////////////////////////////////////////////////////
    bool_t assertFail(char_t const* const conditionStringIn,
                      char_t const* const fileNameIn,
                      std::size_t const   lineNumberIn,
                      char_t const* const functionNameIn);

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Theses macros wrap reportError.
    /// \param[in] errorIn     the reported error code.
    ////////////////////////////////////////////////////////////////////////////////
    #define WDC_REPORT_WARNING(errorIn) \
        reportError(error::level::WARNING,   __FILE__, errorIn, __LINE__, "n/a", 0, "n/a", 0, "n/a", 0, "n/a", 0)
    #define WDC_REPORT_ERROR(errorIn) \
        reportError(error::level::ERROR,     __FILE__, errorIn, __LINE__, "n/a", 0, "n/a", 0, "n/a", 0, "n/a", 0)
    #define WDC_REPORT_CRITICAL(errorIn) \
        reportError(error::level::CRITICAL,  __FILE__, errorIn, __LINE__, "n/a", 0, "n/a", 0, "n/a", 0, "n/a", 0)
    #define WDC_REPORT_ALERT(errorIn) \
        reportError(error::level::ALERT,     __FILE__, errorIn, __LINE__, "n/a", 0, "n/a", 0, "n/a", 0, "n/a", 0)
    #define WDC_REPORT_EMERGENCY(errorIn) \
        reportError(error::level::EMERGENCY, __FILE__, errorIn, __LINE__, "n/a", 0, "n/a", 0, "n/a", 0, "n/a", 0)


    ///////////////////////////////////////////////////////////////////////////////
    ///
    /// \def #WDC_ASSERT Assertion macro.
    ///
    /// \details The input boolean condition is tested; if it is false, an
    ///          assert is raised and the process is terminated.
    ///
    /// \note Note that contrary to native assert macro, the assert is
    ///       raised even in Release builds (more specifically when
    ///       NDEDBUG variable is defined)
    ///
    /// \param[in] EXPR_IN [bool_t] The boolean expression being tested.
    ///
    ///////////////////////////////////////////////////////////////////////////////
    #define WDC_ASSERT(EXPR_IN) (void)((EXPR_IN) || (assertFail(#EXPR_IN, __FILE__, __LINE__, __PRETTY_FUNCTION__)))
}

#include "Error.tpp"

#endif  // WDC_DEFINITION_ERROR_H
