#ifndef _VL53L0X_BASE_H_
#define _VL53L0X_BASE_H_

#include "vl53l0x_types.h"
#include "vl53l0x_def.h"


#ifdef _MSC_VER
#   ifdef VL53L0X_API_EXPORTS
#       define VL53L0X_API  __declspec(dllexport)
#   else
#       define VL53L0X_API
#   endif
#else
#   define VL53L0X_API
#endif


class VL53L0xBase
{
public :
    VL53L0xBase();
    virtual ~VL53L0xBase();

    /**
     * @struct  VL53L0X_Dev_t
     * @brief    Generic PAL device type that does link between API and platform abstraction layer
     *
     */
    typedef struct {
        VL53L0X_DevData_t Data;               /*!< embed ST Ewok Dev  data as "Data"*/

        /*!< user specific field */
        uint8_t   I2cDevAddr;                /*!< i2c device address user specific field */
        uint8_t   comms_type;                /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
        uint16_t  comms_speed_khz;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */

    } VL53L0X_Dev_t;

    /**
     * @brief   Declare the device Handle as a pointer of the structure @a VL53L0X_Dev_t.
     *
     */
    typedef VL53L0X_Dev_t* VL53L0X_DEV;


    VL53L0X_Error init(uint8_t i2c_address=VL53L0X_I2C_DEFAULT_ADDR, uint32_t time_budget=33000);
    uint16_t get_last_distance();
    VL53L0X_Error read_distance(uint16_t *distance);
    VL53L0X_Error change_adress(uint8_t new_adress);
    VL53L0X_DEV get_device();

    uint16_t m_last_distance;
    uint32_t m_read_error_count;
    bool m_init_ok;

protected :
    VL53L0X_Dev_t m_vl53_dev;

public :
    // ____________________________________________________________________
    // Méthodes virtuelles pures à ré-implémenter par la classe utilisatrice
    virtual bool i2c_write_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout=1000/*msec*/) = 0;
    virtual bool i2c_read_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout=1000/*msec*/) = 0;
    virtual void delay_ms(int msec) = 0;


    // ====================================================================
    //                            PLATFORM
    // ====================================================================
    /**
     * @defgroup VL53L0X_platform_group VL53L0X Platform Functions
     * @brief    VL53L0X Platform Functions
     *  @{
     */
    /**
     * @def PALDevDataGet
     * @brief Get ST private structure @a VL53L0X_DevData_t data access
     *
     * @param Dev       Device Handle
     * @param field     ST structure field name
     * It maybe used and as real data "ref" not just as "get" for sub-structure item
     * like PALDevDataGet(FilterData.field)[i] or PALDevDataGet(FilterData.MeasurementIndex)++
     */
    #define PALDevDataGet(Dev, field) (Dev->Data.field)

    /**
     * @def PALDevDataSet(Dev, field, data)
     * @brief  Set ST private structure @a VL53L0X_DevData_t data field
     * @param Dev       Device Handle
     * @param field     ST structure field name
     * @param data      Data to be set
     */
    #define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)


    /**
     * @defgroup VL53L0X_registerAccess_group PAL Register Access Functions
     * @brief    PAL Register Access Functions
     *  @{
     */

    /**
     * Lock comms interface to serialize all commands to a shared I2C interface for a specific device
     * @param   Dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev);

    /**
     * Unlock comms interface to serialize all commands to a shared I2C interface for a specific device
     * @param   Dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev);


    /**
     * Writes the supplied byte buffer to the device
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   pdata     Pointer to uint8_t buffer containing the data to be written
     * @param   count     Number of bytes in the supplied byte buffer
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

    /**
     * Reads the requested number of bytes from the device
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   pdata     Pointer to the uint8_t buffer to store read data
     * @param   count     Number of uint8_t's to read
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

    /**
     * Write single byte register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      8 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);

    /**
     * Write word register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      16 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data);

    /**
     * Write double word (4 byte) register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      32 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data);

    /**
     * Read single byte register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 8 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data);

    /**
     * Read word (2byte) register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 16 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data);

    /**
     * Read dword (4byte) register
     * @param   Dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 32 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data);

    /**
     * Threat safe Update (read/modify/write) single byte register
     *
     * Final_reg = (Initial_reg & and_data) |or_data
     *
     * @param   Dev        Device Handle
     * @param   index      The register index
     * @param   AndData    8 bit and data
     * @param   OrData     8 bit or data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);

    /** @} end of VL53L0X_registerAccess_group */


    /**
     * @brief execute delay in all polling API call
     *
     * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
     * if nothing specific is need you can define it as an empty/void macro
     * @code
     * #define VL53L0X_PollingDelay(...) (void)0
     * @endcode
     * @param Dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev); /* usually best implemented as a real function */



    // ====================================================================
    //                             API
    // ====================================================================
    /** @defgroup VL53L0X_cut11_group VL53L0X cut1.1 Function Definition
     *  @brief    VL53L0X cut1.1 Function Definition
     *  @{
     */

    /** @defgroup VL53L0X_general_group VL53L0X General Functions
     *  @brief    General functions and definitions
     *  @{
     */

    /**
     * @brief Return the VL53L0X PAL Implementation Version
     *
     * @note This function doesn't access to the device
     *
     * @param   pVersion              Pointer to current PAL Implementation Version
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetVersion(VL53L0X_Version_t *pVersion);

    /**
     * @brief Return the PAL Specification Version used for the current
     * implementation.
     *
     * @note This function doesn't access to the device
     *
     * @param   pPalSpecVersion       Pointer to current PAL Specification Version
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetPalSpecVersion(
        VL53L0X_Version_t *pPalSpecVersion);

    /**
     * @brief Reads the Product Revision for a for given Device
     * This function can be used to distinguish cut1.0 from cut1.1.
     *
     * @note This function Access to the device
     *
     * @param   Dev                 Device Handle
     * @param   pProductRevisionMajor  Pointer to Product Revision Major
     * for a given Device
     * @param   pProductRevisionMinor  Pointer to Product Revision Minor
     * for a given Device
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetProductRevision(VL53L0X_DEV Dev,
        uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor);

    /**
     * @brief Reads the Device information for given Device
     *
     * @note This function Access to the device
     *
     * @param   Dev                 Device Handle
     * @param   pVL53L0X_DeviceInfo  Pointer to current device info for a given
     *  Device
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetDeviceInfo(VL53L0X_DEV Dev,
        VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);

    /**
     * @brief Read current status of the error register for the selected device
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pDeviceErrorStatus    Pointer to current error code of the device
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetDeviceErrorStatus(VL53L0X_DEV Dev,
        VL53L0X_DeviceError * pDeviceErrorStatus);

    /**
     * @brief Human readable Range Status string for a given RangeStatus
     *
     * @note This function doesn't access to the device
     *
     * @param   RangeStatus         The RangeStatus code as stored on
     * @a VL53L0X_RangingMeasurementData_t
     * @param   pRangeStatusString  The returned RangeStatus string.
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetRangeStatusString(uint8_t RangeStatus,
        char *pRangeStatusString);

    /**
     * @brief Human readable error string for a given Error Code
     *
     * @note This function doesn't access to the device
     *
     * @param  ErrorCode           The error code as stored on ::VL53L0X_DeviceError
     * @param  pDeviceErrorString  The error string corresponding to the ErrorCode
     * @return VL53L0X_ERROR_NONE   Success
     * @return "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetDeviceErrorString(
        VL53L0X_DeviceError ErrorCode, char *pDeviceErrorString);

    /**
     * @brief Human readable error string for current PAL error status
     *
     * @note This function doesn't access to the device
     *
     * @param   PalErrorCode       The error code as stored on @a VL53L0X_Error
     * @param   pPalErrorString    The error string corresponding to the
     * PalErrorCode
     * @return  VL53L0X_ERROR_NONE  Success
     * @return  "Other error code" See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetPalErrorString(VL53L0X_Error PalErrorCode,
        char *pPalErrorString);

    /**
     * @brief Human readable PAL State string
     *
     * @note This function doesn't access to the device
     *
     * @param   PalStateCode          The State code as stored on @a VL53L0X_State
     * @param   pPalStateString       The State string corresponding to the
     * PalStateCode
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetPalStateString(VL53L0X_State PalStateCode,
        char *pPalStateString);

    /**
     * @brief Reads the internal state of the PAL for a given Device
     *
     * @note This function doesn't access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pPalState             Pointer to current state of the PAL for a
     * given Device
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetPalState(VL53L0X_DEV Dev,
        VL53L0X_State * pPalState);

    /**
     * @brief Set the power mode for a given Device
     * The power mode can be Standby or Idle. Different level of both Standby and
     * Idle can exists.
     * This function should not be used when device is in Ranging state.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   PowerMode             The value of the power mode to set.
     * see ::VL53L0X_PowerModes
     *                                Valid values are:
     *                                VL53L0X_POWERMODE_STANDBY_LEVEL1,
     *                                VL53L0X_POWERMODE_IDLE_LEVEL1
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED    This error occurs when PowerMode
     * is not in the supported list
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetPowerMode(VL53L0X_DEV Dev,
        VL53L0X_PowerModes PowerMode);

    /**
     * @brief Get the power mode for a given Device
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pPowerMode            Pointer to the current value of the power
     * mode. see ::VL53L0X_PowerModes
     *                                Valid values are:
     *                                VL53L0X_POWERMODE_STANDBY_LEVEL1,
     *                                VL53L0X_POWERMODE_IDLE_LEVEL1
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetPowerMode(VL53L0X_DEV Dev,
        VL53L0X_PowerModes * pPowerMode);

    /**
     * Set or over-hide part to part calibration offset
     * \sa VL53L0X_DataInit()   VL53L0X_GetOffsetCalibrationDataMicroMeter()
     *
     * @note This function Access to the device
     *
     * @param   Dev                                Device Handle
     * @param   OffsetCalibrationDataMicroMeter    Offset (microns)
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetOffsetCalibrationDataMicroMeter(
        VL53L0X_DEV Dev, int32_t OffsetCalibrationDataMicroMeter);

    /**
     * @brief Get part to part calibration offset
     *
     * @par Function Description
     * Should only be used after a successful call to @a VL53L0X_DataInit to backup
     * device NVM value
     *
     * @note This function Access to the device
     *
     * @param   Dev                                Device Handle
     * @param   pOffsetCalibrationDataMicroMeter   Return part to part
     * calibration offset from device (microns)
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetOffsetCalibrationDataMicroMeter(
        VL53L0X_DEV Dev, int32_t *pOffsetCalibrationDataMicroMeter);

    /**
     * Set the linearity corrective gain
     *
     * @note This function Access to the device
     *
     * @param   Dev                                Device Handle
     * @param   LinearityCorrectiveGain            Linearity corrective
     * gain in x1000
     * if value is 1000 then no modification is applied.
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetLinearityCorrectiveGain(VL53L0X_DEV Dev,
        int16_t LinearityCorrectiveGain);

    /**
     * @brief Get the linearity corrective gain
     *
     * @par Function Description
     * Should only be used after a successful call to @a VL53L0X_DataInit to backup
     * device NVM value
     *
     * @note This function Access to the device
     *
     * @param   Dev                                Device Handle
     * @param   pLinearityCorrectiveGain           Pointer to the linearity
     * corrective gain in x1000
     * if value is 1000 then no modification is applied.
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLinearityCorrectiveGain(VL53L0X_DEV Dev,
        uint16_t *pLinearityCorrectiveGain);

    /**
     * Set Group parameter Hold state
     *
     * @par Function Description
     * Set or remove device internal group parameter hold
     *
     * @note This function is not Implemented
     *
     * @param   Dev      Device Handle
     * @param   GroupParamHold   Group parameter Hold state to be set (on/off)
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED        Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetGroupParamHold(VL53L0X_DEV Dev,
        uint8_t GroupParamHold);

    /**
     * @brief Get the maximal distance for actual setup
     * @par Function Description
     * Device must be initialized through @a VL53L0X_SetParameters() prior calling
     * this function.
     *
     * Any range value more than the value returned is to be considered as
     * "no target detected" or
     * "no target in detectable range"\n
     * @warning The maximal distance depends on the setup
     *
     * @note This function is not Implemented
     *
     * @param   Dev      Device Handle
     * @param   pUpperLimitMilliMeter   The maximal range limit for actual setup
     * (in millimeter)
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED        Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetUpperLimitMilliMeter(VL53L0X_DEV Dev,
        uint16_t *pUpperLimitMilliMeter);


    /**
     * @brief Get the Total Signal Rate
     * @par Function Description
     * This function will return the Total Signal Rate after a good ranging is done.
     *
     * @note This function access to Device
     *
     * @param   Dev      Device Handle
     * @param   pTotalSignalRate   Total Signal Rate value in Mega count per second
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_GetTotalSignalRate(VL53L0X_DEV Dev,
        FixPoint1616_t *pTotalSignalRate);

    /** @} VL53L0X_general_group */

    /** @defgroup VL53L0X_init_group VL53L0X Init Functions
     *  @brief    VL53L0X Init Functions
     *  @{
     */

    /**
     * @brief Set new device address
     *
     * After completion the device will answer to the new address programmed.
     * This function should be called when several devices are used in parallel
     * before start programming the sensor.
     * When a single device us used, there is no need to call this function.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   DeviceAddress         The new Device address
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetDeviceAddress(VL53L0X_DEV Dev,
        uint8_t DeviceAddress);

    /**
     *
     * @brief One time device initialization
     *
     * To be called once and only once after device is brought out of reset
     * (Chip enable) and booted see @a VL53L0X_WaitDeviceBooted()
     *
     * @par Function Description
     * When not used after a fresh device "power up" or reset, it may return
     * @a #VL53L0X_ERROR_CALIBRATION_WARNING meaning wrong calibration data
     * may have been fetched from device that can result in ranging offset error\n
     * If application cannot execute device reset or need to run VL53L0X_DataInit
     * multiple time then it  must ensure proper offset calibration saving and
     * restore on its own by using @a VL53L0X_GetOffsetCalibrationData() on first
     * power up and then @a VL53L0X_SetOffsetCalibrationData() in all subsequent
     * init.
     * This function will change the VL53L0X_State from VL53L0X_STATE_POWERDOWN to
     * VL53L0X_STATE_WAIT_STATICINIT.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_DataInit(VL53L0X_DEV Dev);

    /**
     * @brief Set the tuning settings pointer
     *
     * This function is used to specify the Tuning settings buffer to be used
     * for a given device. The buffer contains all the necessary data to permit
     * the API to write tuning settings.
     * This function permit to force the usage of either external or internal
     * tuning settings.
     *
     * @note This function Access to the device
     *
     * @param   Dev                             Device Handle
     * @param   pTuningSettingBuffer            Pointer to tuning settings buffer.
     * @param   UseInternalTuningSettings       Use internal tuning settings value.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetTuningSettingBuffer(VL53L0X_DEV Dev,
        uint8_t *pTuningSettingBuffer, uint8_t UseInternalTuningSettings);

    /**
     * @brief Get the tuning settings pointer and the internal external switch
     * value.
     *
     * This function is used to get the Tuning settings buffer pointer and the
     * value.
     * of the switch to select either external or internal tuning settings.
     *
     * @note This function Access to the device
     *
     * @param   Dev                        Device Handle
     * @param   ppTuningSettingBuffer      Pointer to tuning settings buffer.
     * @param   pUseInternalTuningSettings Pointer to store Use internal tuning
     *                                     settings value.
     * @return  VL53L0X_ERROR_NONE          Success
     * @return  "Other error code"         See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetTuningSettingBuffer(VL53L0X_DEV Dev,
        uint8_t **ppTuningSettingBuffer, uint8_t *pUseInternalTuningSettings);

    /**
     * @brief Do basic device init (and eventually patch loading)
     * This function will change the VL53L0X_State from
     * VL53L0X_STATE_WAIT_STATICINIT to VL53L0X_STATE_IDLE.
     * In this stage all default setting will be applied.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_StaticInit(VL53L0X_DEV Dev);

    /**
     * @brief Wait for device booted after chip enable (hardware standby)
     * This function can be run only when VL53L0X_State is VL53L0X_STATE_POWERDOWN.
     *
     * @note This function is not Implemented
     *
     * @param   Dev      Device Handle
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED Not implemented
     *
     */
    VL53L0X_API VL53L0X_Error VL53L0X_WaitDeviceBooted(VL53L0X_DEV Dev);

    /**
     * @brief Do an hard reset or soft reset (depending on implementation) of the
     * device \nAfter call of this function, device must be in same state as right
     * after a power-up sequence.This function will change the VL53L0X_State to
     * VL53L0X_STATE_POWERDOWN.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_ResetDevice(VL53L0X_DEV Dev);

    /** @} VL53L0X_init_group */

    /** @defgroup VL53L0X_parameters_group VL53L0X Parameters Functions
     *  @brief    Functions used to prepare and setup the device
     *  @{
     */

    /**
     * @brief  Prepare device for operation
     * @par Function Description
     * Update device with provided parameters
     * @li Then start ranging operation.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pDeviceParameters     Pointer to store current device parameters.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetDeviceParameters(VL53L0X_DEV Dev,
        const VL53L0X_DeviceParameters_t *pDeviceParameters);

    /**
     * @brief  Retrieve current device parameters
     * @par Function Description
     * Get actual parameters of the device
     * @li Then start ranging operation.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pDeviceParameters     Pointer to store current device parameters.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetDeviceParameters(VL53L0X_DEV Dev,
        VL53L0X_DeviceParameters_t *pDeviceParameters);

    /**
     * @brief  Set a new device mode
     * @par Function Description
     * Set device to a new mode (ranging, histogram ...)
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   DeviceMode            New device mode to apply
     *                                Valid values are:
     *                                VL53L0X_DEVICEMODE_SINGLE_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     *
     * @return  VL53L0X_ERROR_NONE               Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED This error occurs when DeviceMode
     *                                           is not in the supported list
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetDeviceMode(VL53L0X_DEV Dev,
        VL53L0X_DeviceModes DeviceMode);

    /**
     * @brief  Get current new device mode
     * @par Function Description
     * Get actual mode of the device(ranging, histogram ...)
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pDeviceMode           Pointer to current apply mode value
     *                                Valid values are:
     *                                VL53L0X_DEVICEMODE_SINGLE_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED     This error occurs when
     * DeviceMode is not in the supported list
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetDeviceMode(VL53L0X_DEV Dev,
        VL53L0X_DeviceModes * pDeviceMode);

    /**
     * @brief  Sets the resolution of range measurements.
     * @par Function Description
     * Set resolution of range measurements to either 0.25mm if
     * fraction enabled or 1mm if not enabled.
     *
     * @note This function Accesses the device
     *
     * @param   Dev               Device Handle
     * @param   Enable            Enable high resolution
     *
     * @return  VL53L0X_ERROR_NONE               Success
     * @return  "Other error code"              See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetRangeFractionEnable(VL53L0X_DEV Dev,
        uint8_t Enable);

    /**
     * @brief  Gets the fraction enable parameter indicating the resolution of
     * range measurements.
     *
     * @par Function Description
     * Gets the fraction enable state, which translates to the resolution of
     * range measurements as follows :Enabled:=0.25mm resolution,
     * Not Enabled:=1mm resolution.
     *
     * @note This function Accesses the device
     *
     * @param   Dev        Device Handle
     * @param   pEnable    Output Parameter reporting the fraction enable state.
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  "Other error code"                  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetFractionEnable(VL53L0X_DEV Dev,
        uint8_t *pEnable);

    /**
     * @brief  Set a new Histogram mode
     * @par Function Description
     * Set device to a new Histogram mode
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   HistogramMode         New device mode to apply
     *                                Valid values are:
     *                                VL53L0X_HISTOGRAMMODE_DISABLED
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED     This error occurs when
     * HistogramMode is not in the supported list
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetHistogramMode(VL53L0X_DEV Dev,
        VL53L0X_HistogramModes HistogramMode);

    /**
     * @brief  Get current new device mode
     * @par Function Description
     * Get current Histogram mode of a Device
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pHistogramMode        Pointer to current Histogram Mode value
     *                                Valid values are:
     *                                VL53L0X_HISTOGRAMMODE_DISABLED
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetHistogramMode(VL53L0X_DEV Dev,
        VL53L0X_HistogramModes * pHistogramMode);

    /**
     * @brief Set Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Defines the maximum time allowed by the user to the device to run a
     * full ranging sequence for the current mode (ranging, histogram, ASL ...)
     *
     * @note This function Access to the device
     *
     * @param   Dev                                Device Handle
     * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
     * microseconds.
     *                                   Valid values are:
     *                                   >= 17000 microsecs when wraparound enabled
     *                                   >= 12000 microsecs when wraparound disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned if
     MeasurementTimingBudgetMicroSeconds out of range
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        VL53L0X_DEV Dev, uint32_t MeasurementTimingBudgetMicroSeconds);

    /**
     * @brief Get Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Returns the programmed the maximum time allowed by the user to the
     * device to run a full ranging sequence for the current mode
     * (ranging, histogram, ASL ...)
     *
     * @note This function Access to the device
     *
     * @param   Dev                                    Device Handle
     * @param   pMeasurementTimingBudgetMicroSeconds   Max measurement time in
     * microseconds.
     *                                   Valid values are:
     *                                   >= 17000 microsecs when wraparound enabled
     *                                   >= 12000 microsecs when wraparound disabled
     * @return  VL53L0X_ERROR_NONE                      Success
     * @return  "Other error code"                     See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetMeasurementTimingBudgetMicroSeconds(
        VL53L0X_DEV Dev, uint32_t *pMeasurementTimingBudgetMicroSeconds);

    /**
     * @brief Gets the VCSEL pulse period.
     *
     * @par Function Description
     * This function retrieves the VCSEL pulse period for the given period type.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                      Device Handle
     * @param   VcselPeriodType          VCSEL period identifier (pre-range|final).
     * @param   pVCSELPulsePeriod        Pointer to VCSEL period value.
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetVcselPulsePeriod(VL53L0X_DEV Dev,
        VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriod);

    /**
     * @brief Sets the VCSEL pulse period.
     *
     * @par Function Description
     * This function retrieves the VCSEL pulse period for the given period type.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                       Device Handle
     * @param   VcselPeriodType	      VCSEL period identifier (pre-range|final).
     * @param   VCSELPulsePeriod          VCSEL period value
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetVcselPulsePeriod(VL53L0X_DEV Dev,
        VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

    /**
     * @brief Sets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function enables/disables a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   SequenceStepId	         Sequence step identifier.
     * @param   SequenceStepEnabled          Demanded state {0=Off,1=On}
     *                                       is enabled.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetSequenceStepEnable(VL53L0X_DEV Dev,
        VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);

    /**
     * @brief Gets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function retrieves the state of a requested sequence step, i.e. on/off.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                    Device Handle
     * @param   SequenceStepId         Sequence step identifier.
     * @param   pSequenceStepEnabled   Out parameter reporting if the sequence step
     *                                 is enabled {0=Off,1=On}.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSequenceStepEnable(VL53L0X_DEV Dev,
        VL53L0X_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled);

    /**
     * @brief Gets the (on/off) state of all sequence steps.
     *
     * @par Function Description
     * This function retrieves the state of all sequence step in the scheduler.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   pSchedulerSequenceSteps      Pointer to struct containing result.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSequenceStepEnables(VL53L0X_DEV Dev,
        VL53L0X_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);

    /**
     * @brief Sets the timeout of a requested sequence step.
     *
     * @par Function Description
     * This function sets the timeout of a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   SequenceStepId               Sequence step identifier.
     * @param   TimeOutMilliSecs             Demanded timeout
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetSequenceStepTimeout(VL53L0X_DEV Dev,
        VL53L0X_SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs);

    /**
     * @brief Gets the timeout of a requested sequence step.
     *
     * @par Function Description
     * This function retrieves the timeout of a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   SequenceStepId               Sequence step identifier.
     * @param   pTimeOutMilliSecs            Timeout value.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSequenceStepTimeout(VL53L0X_DEV Dev,
        VL53L0X_SequenceStepId SequenceStepId,
        FixPoint1616_t *pTimeOutMilliSecs);

    /**
     * @brief Gets number of sequence steps managed by the API.
     *
     * @par Function Description
     * This function retrieves the number of sequence steps currently managed
     * by the API
     *
     * @note This function Accesses the device
     *
     * @param   Dev                          Device Handle
     * @param   pNumberOfSequenceSteps       Out parameter reporting the number of
     *                                       sequence steps.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetNumberOfSequenceSteps(VL53L0X_DEV Dev,
        uint8_t *pNumberOfSequenceSteps);

    /**
     * @brief Gets the name of a given sequence step.
     *
     * @par Function Description
     * This function retrieves the name of sequence steps corresponding to
     * SequenceStepId.
     *
     * @note This function doesn't Accesses the device
     *
     * @param   SequenceStepId               Sequence step identifier.
     * @param   pSequenceStepsString         Pointer to Info string
     *
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSequenceStepsInfo(
        VL53L0X_SequenceStepId SequenceStepId, char *pSequenceStepsString);

    /**
     * Program continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     * When trying to set too short time return  INVALID_PARAMS minimal value
     *
     * @note This function Access to the device
     *
     * @param   Dev                                  Device Handle
     * @param   InterMeasurementPeriodMilliSeconds   Inter-Measurement Period in ms.
     * @return  VL53L0X_ERROR_NONE                    Success
     * @return  "Other error code"                   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetInterMeasurementPeriodMilliSeconds(
        VL53L0X_DEV Dev, uint32_t InterMeasurementPeriodMilliSeconds);

    /**
     * Get continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     * When trying to set too short time return  INVALID_PARAMS minimal value
     *
     * @note This function Access to the device
     *
     * @param   Dev                                  Device Handle
     * @param   pInterMeasurementPeriodMilliSeconds  Pointer to programmed
     *  Inter-Measurement Period in milliseconds.
     * @return  VL53L0X_ERROR_NONE                    Success
     * @return  "Other error code"                   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetInterMeasurementPeriodMilliSeconds(
        VL53L0X_DEV Dev, uint32_t *pInterMeasurementPeriodMilliSeconds);

    /**
     * @brief Enable/Disable Cross talk compensation feature
     *
     * @note This function is not Implemented.
     * Enable/Disable Cross Talk by set to zero the Cross Talk value
     * by using @a VL53L0X_SetXTalkCompensationRateMegaCps().
     *
     * @param   Dev                       Device Handle
     * @param   XTalkCompensationEnable   Cross talk compensation
     *  to be set 0=disabled else = enabled
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetXTalkCompensationEnable(VL53L0X_DEV Dev,
        uint8_t XTalkCompensationEnable);

    /**
     * @brief Get Cross talk compensation rate
     *
     * @note This function is not Implemented.
     * Enable/Disable Cross Talk by set to zero the Cross Talk value by
     * using @a VL53L0X_SetXTalkCompensationRateMegaCps().
     *
     * @param   Dev                        Device Handle
     * @param   pXTalkCompensationEnable   Pointer to the Cross talk compensation
     *  state 0=disabled or 1 = enabled
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
        uint8_t *pXTalkCompensationEnable);

    /**
     * @brief Set Cross talk compensation rate
     *
     * @par Function Description
     * Set Cross talk compensation rate.
     *
     * @note This function Access to the device
     *
     * @param   Dev                            Device Handle
     * @param   XTalkCompensationRateMegaCps   Compensation rate in
     *  Mega counts per second (16.16 fix point) see datasheet for details
     * @return  VL53L0X_ERROR_NONE              Success
     * @return  "Other error code"             See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetXTalkCompensationRateMegaCps(
        VL53L0X_DEV Dev,
        FixPoint1616_t XTalkCompensationRateMegaCps);

    /**
     * @brief Get Cross talk compensation rate
     *
     * @par Function Description
     * Get Cross talk compensation rate.
     *
     * @note This function Access to the device
     *
     * @param   Dev                            Device Handle
     * @param   pXTalkCompensationRateMegaCps  Pointer to Compensation rate
     in Mega counts per second (16.16 fix point) see datasheet for details
     * @return  VL53L0X_ERROR_NONE              Success
     * @return  "Other error code"             See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetXTalkCompensationRateMegaCps(
        VL53L0X_DEV Dev,
        FixPoint1616_t *pXTalkCompensationRateMegaCps);

    /**
     * @brief Set Reference Calibration Parameters
     *
     * @par Function Description
     * Set Reference Calibration Parameters.
     *
     * @note This function Access to the device
     *
     * @param   Dev                            Device Handle
     * @param   VhvSettings                    Parameter for VHV
     * @param   PhaseCal                       Parameter for PhaseCal
     * @return  VL53L0X_ERROR_NONE              Success
     * @return  "Other error code"             See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetRefCalibration(VL53L0X_DEV Dev,
        uint8_t VhvSettings, uint8_t PhaseCal);

    /**
     * @brief Get Reference Calibration Parameters
     *
     * @par Function Description
     * Get Reference Calibration Parameters.
     *
     * @note This function Access to the device
     *
     * @param   Dev                            Device Handle
     * @param   pVhvSettings                   Pointer to VHV parameter
     * @param   pPhaseCal                      Pointer to PhaseCal Parameter
     * @return  VL53L0X_ERROR_NONE              Success
     * @return  "Other error code"             See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetRefCalibration(VL53L0X_DEV Dev,
        uint8_t *pVhvSettings, uint8_t *pPhaseCal);

    /**
     * @brief  Get the number of the check limit managed by a given Device
     *
     * @par Function Description
     * This function give the number of the check limit managed by the Device
     *
     * @note This function doesn't Access to the device
     *
     * @param   pNumberOfLimitCheck           Pointer to the number of check limit.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetNumberOfLimitCheck(
        uint16_t *pNumberOfLimitCheck);

    /**
     * @brief  Return a description string for a given limit check number
     *
     * @par Function Description
     * This function returns a description string for a given limit check number.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckString             Pointer to the
     description string of the given check limit.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is
     returned when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLimitCheckInfo(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, char *pLimitCheckString);

    /**
     * @brief  Return a the Status of the specified check limit
     *
     * @par Function Description
     * This function returns the Status of the specified check limit.
     * The value indicate if the check is fail or not.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckStatus             Pointer to the
     Limit Check Status of the given check limit.
     * LimitCheckStatus :
     * 0 the check is not fail
     * 1 the check if fail or not enabled
     *
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is
     returned when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLimitCheckStatus(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, uint8_t *pLimitCheckStatus);

    /**
     * @brief  Enable/Disable a specific limit check
     *
     * @par Function Description
     * This function Enable/Disable a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   LimitCheckEnable              if 1 the check limit
     *  corresponding to LimitCheckId is Enabled
     *                                        if 0 the check limit
     *  corresponding to LimitCheckId is disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetLimitCheckEnable(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, uint8_t LimitCheckEnable);

    /**
     * @brief  Get specific limit check enable state
     *
     * @par Function Description
     * This function get the enable state of a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckEnable             Pointer to the check limit enable
     * value.
     *  if 1 the check limit
     *        corresponding to LimitCheckId is Enabled
     *  if 0 the check limit
     *        corresponding to LimitCheckId is disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);

    /**
     * @brief  Set a specific limit check value
     *
     * @par Function Description
     * This function set a specific limit check value.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   LimitCheckValue               Limit check Value for a given
     * LimitCheckId
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned when either
     *  LimitCheckId or LimitCheckValue value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetLimitCheckValue(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);

    /**
     * @brief  Get a specific limit check value
     *
     * @par Function Description
     * This function get a specific limit check value from device then it updates
     * internal values and check enables.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckValue              Pointer to Limit
     *  check Value for a given LimitCheckId.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);

    /**
     * @brief  Get the current value of the signal used for the limit check
     *
     * @par Function Description
     * This function get a the current value of the signal used for the limit check.
     * To obtain the latest value you should run a ranging before.
     * The value reported is linked to the limit check identified with the
     * LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   LimitCheckId                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckCurrent            Pointer to current Value for a
     * given LimitCheckId.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned when
     * LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetLimitCheckCurrent(VL53L0X_DEV Dev,
        uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);

    /**
     * @brief  Enable (or disable) Wrap around Check
     *
     * @note This function Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   WrapAroundCheckEnable  Wrap around Check to be set
     *                                 0=disabled, other = enabled
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetWrapAroundCheckEnable(VL53L0X_DEV Dev,
            uint8_t WrapAroundCheckEnable);

    /**
     * @brief  Get setup of Wrap around Check
     *
     * @par Function Description
     * This function get the wrapAround check enable parameters
     *
     * @note This function Access to the device
     *
     * @param   Dev                     Device Handle
     * @param   pWrapAroundCheckEnable  Pointer to the Wrap around Check state
     *                                  0=disabled or 1 = enabled
     * @return  VL53L0X_ERROR_NONE       Success
     * @return  "Other error code"      See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetWrapAroundCheckEnable(VL53L0X_DEV Dev,
            uint8_t *pWrapAroundCheckEnable);

    /** @} VL53L0X_parameters_group */

    /** @defgroup VL53L0X_measurement_group VL53L0X Measurement Functions
     *  @brief    Functions used for the measurements
     *  @{
     */

    /**
     * @brief Single shot measurement.
     *
     * @par Function Description
     * Perform simple measurement sequence (Start measure, Wait measure to end,
     * and returns when measurement is done).
     * Once function returns, user can get valid data by calling
     * VL53L0X_GetRangingMeasurement or VL53L0X_GetHistogramMeasurement
     * depending on defined measurement mode
     * User should Clear the interrupt in case this are enabled by using the
     * function VL53L0X_ClearInterruptMask().
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev);

    /**
     * @brief Perform Reference Calibration
     *
     * @details Perform a reference calibration of the Device.
     * This function should be run from time to time before doing
     * a ranging measurement.
     * This function will launch a special ranging measurement, so
     * if interrupt are enable an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @param   pVhvSettings         Pointer to vhv settings parameter.
     * @param   pPhaseCal            Pointer to PhaseCal parameter.
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformRefCalibration(VL53L0X_DEV Dev,
        uint8_t *pVhvSettings, uint8_t *pPhaseCal);

    /**
     * @brief Perform XTalk Measurement
     *
     * @details Measures the current cross talk from glass in front
     * of the sensor.
     * This functions performs a histogram measurement and uses the results
     * to measure the crosstalk. For the function to be successful, there
     * must be no target in front of the sensor.
     *
     * @warning This function is a blocking function
     *
     * @warning This function is not supported when the final range
     * vcsel clock period is set below 10 PCLKS.
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @param   TimeoutMs            Histogram measurement duration.
     * @param   pXtalkPerSpad        Output parameter containing the crosstalk
     * measurement result, in MCPS/Spad. Format fixpoint 16:16.
     * @param   pAmbientTooHigh      Output parameter which indicate that
     * pXtalkPerSpad is not good if the Ambient is too high.
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS vcsel clock period not supported
     * for this operation. Must not be less than 10PCLKS.
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformXTalkMeasurement(VL53L0X_DEV Dev,
        uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad,
        uint8_t *pAmbientTooHigh);

    /**
     * @brief Perform XTalk Calibration
     *
     * @details Perform a XTalk calibration of the Device.
     * This function will launch a ranging measurement, if interrupts
     * are enabled an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     * This function will program a new value for the XTalk compensation
     * and it will enable the cross talk before exit.
     * This function will disable the VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @note This function change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING
     *
     * @param   Dev                  Device Handle
     * @param   XTalkCalDistance     XTalkCalDistance value used for the XTalk
     * computation.
     * @param   pXTalkCompensationRateMegaCps  Pointer to new
     * XTalkCompensation value.
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformXTalkCalibration(VL53L0X_DEV Dev,
        FixPoint1616_t XTalkCalDistance,
        FixPoint1616_t *pXTalkCompensationRateMegaCps);

    /**
     * @brief Perform Offset Calibration
     *
     * @details Perform a Offset calibration of the Device.
     * This function will launch a ranging measurement, if interrupts are
     * enabled an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     * This function will program a new value for the Offset calibration value
     * This function will disable the VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @note This function does not change the device mode.
     *
     * @param   Dev                  Device Handle
     * @param   CalDistanceMilliMeter     Calibration distance value used for the
     * offset compensation.
     * @param   pOffsetMicroMeter  Pointer to new Offset value computed by the
     * function.
     *
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformOffsetCalibration(VL53L0X_DEV Dev,
        FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter);

    /**
     * @brief Start device measurement
     *
     * @details Started measurement will depend on device parameters set through
     * @a VL53L0X_SetParameters()
     * This is a non-blocking function.
     * This function will change the VL53L0X_State from VL53L0X_STATE_IDLE to
     * VL53L0X_STATE_RUNNING.
     *
     * @note This function Access to the device
     *

     * @param   Dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED    This error occurs when
     * DeviceMode programmed with @a VL53L0X_SetDeviceMode is not in the supported
     * list:
     *                                   Supported mode are:
     *                                   VL53L0X_DEVICEMODE_SINGLE_RANGING,
     *                                   VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
     *                                   VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     * @return  VL53L0X_ERROR_TIME_OUT    Time out on start measurement
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_StartMeasurement(VL53L0X_DEV Dev);

    /**
     * @brief Stop device measurement
     *
     * @details Will set the device in standby mode at end of current measurement\n
     *          Not necessary in single mode as device shall return automatically
     *          in standby mode at end of measurement.
     *          This function will change the VL53L0X_State from
     *          VL53L0X_STATE_RUNNING to VL53L0X_STATE_IDLE.
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_StopMeasurement(VL53L0X_DEV Dev);

    /**
     * @brief Return Measurement Data Ready
     *
     * @par Function Description
     * This function indicate that a measurement data is ready.
     * This function check if interrupt mode is used then check is done accordingly.
     * If perform function clear the interrupt, this function will not work,
     * like in case of @a VL53L0X_PerformSingleRangingMeasurement().
     * The previous function is blocking function, VL53L0X_GetMeasurementDataReady
     * is used for non-blocking capture.
     *
     * @note This function Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   pMeasurementDataReady  Pointer to Measurement Data Ready.
     *  0=data not ready, 1 = data ready
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetMeasurementDataReady(VL53L0X_DEV Dev,
        uint8_t *pMeasurementDataReady);

    /**
     * @brief Wait for device ready for a new measurement command.
     * Blocking function.
     *
     * @note This function is not Implemented
     *
     * @param   Dev      Device Handle
     * @param   MaxLoop    Max Number of polling loop (timeout).
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_WaitDeviceReadyForNewMeasurement(
        VL53L0X_DEV Dev,
        uint32_t MaxLoop);

    /**
     * @brief Retrieve the Reference Signal after a measurements
     *
     * @par Function Description
     * Get Reference Signal from last successful Ranging measurement
     * This function return a valid value after that you call the
     * @a VL53L0X_GetRangingMeasurementData().
     *
     * @note This function Access to the device
     *
     * @param   Dev                      Device Handle
     * @param   pMeasurementRefSignal    Pointer to the Ref Signal to fill up.
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetMeasurementRefSignal(VL53L0X_DEV Dev,
        FixPoint1616_t *pMeasurementRefSignal);

    /**
     * @brief Retrieve the measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Ranging measurement
     * @warning USER should take care about  @a VL53L0X_GetNumberOfROIZones()
     * before get data.
     * PAL will fill a NumberOfROIZones times the corresponding data
     * structure used in the measurement function.
     *
     * @note This function Access to the device
     *
     * @param   Dev                      Device Handle
     * @param   pRangingMeasurementData  Pointer to the data structure to fill up.
     * @param   lightProcessing    		 true : compute light processing and limit i2c bus commmunication to strict necessary / false: compute fullprocessing
     *
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetRangingMeasurementData(VL53L0X_DEV Dev,
        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
		bool lightProcessing = false);

    /**
     * @brief Retrieve the measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Histogram measurement
     * @warning USER should take care about  @a VL53L0X_GetNumberOfROIZones()
     * before get data.
     * PAL will fill a NumberOfROIZones times the corresponding data structure
     * used in the measurement function.
     *
     * @note This function is not Implemented
     *
     * @param   Dev                         Device Handle
     * @param   pHistogramMeasurementData   Pointer to the histogram data structure.
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetHistogramMeasurementData(VL53L0X_DEV Dev,
        VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData);

    /**
     * @brief Performs a single ranging measurement and retrieve the ranging
     * measurement data
     *
     * @par Function Description
     * This function will change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING with @a VL53L0X_SetDeviceMode(),
     * It performs measurement with @a VL53L0X_PerformSingleMeasurement()
     * It get data from last successful Ranging measurement with
     * @a VL53L0X_GetRangingMeasurementData.
     * Finally it clear the interrupt with @a VL53L0X_ClearInterruptMask().
     *
     * @note This function Access to the device
     *
     * @note This function change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING
     *
     * @param   Dev                       Device Handle
     * @param   pRangingMeasurementData   Pointer to the data structure to fill up.
     * @return  VL53L0X_ERROR_NONE         Success
     * @return  "Other error code"        See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformSingleRangingMeasurement(
        VL53L0X_DEV Dev,
        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);

    /**
     * @brief Performs a single histogram measurement and retrieve the histogram
     * measurement data
     *   Is equivalent to VL53L0X_PerformSingleMeasurement +
     *   VL53L0X_GetHistogramMeasurementData
     *
     * @par Function Description
     * Get data from last successful Ranging measurement.
     * This function will clear the interrupt in case of these are enabled.
     *
     * @note This function is not Implemented
     *
     * @param   Dev                        Device Handle
     * @param   pHistogramMeasurementData  Pointer to the data structure to fill up.
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformSingleHistogramMeasurement(
        VL53L0X_DEV Dev,
        VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData);

    /**
     * @brief Set the number of ROI Zones to be used for a specific Device
     *
     * @par Function Description
     * Set the number of ROI Zones to be used for a specific Device.
     * The programmed value should be less than the max number of ROI Zones given
     * with @a VL53L0X_GetMaxNumberOfROIZones().
     * This version of API manage only one zone.
     *
     * @param   Dev                           Device Handle
     * @param   NumberOfROIZones              Number of ROI Zones to be used for a
     *  specific Device.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned if
     * NumberOfROIZones != 1
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetNumberOfROIZones(VL53L0X_DEV Dev,
        uint8_t NumberOfROIZones);

    /**
     * @brief Get the number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get number of ROI Zones managed by the Device
     * USER should take care about  @a VL53L0X_GetNumberOfROIZones()
     * before get data after a perform measurement.
     * PAL will fill a NumberOfROIZones times the corresponding data
     * structure used in the measurement function.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pNumberOfROIZones     Pointer to the Number of ROI Zones value.
     * @return  VL53L0X_ERROR_NONE     Success
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetNumberOfROIZones(VL53L0X_DEV Dev,
        uint8_t *pNumberOfROIZones);

    /**
     * @brief Get the Maximum number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get Maximum number of ROI Zones managed by the Device.
     *
     * @note This function doesn't Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   pMaxNumberOfROIZones   Pointer to the Maximum Number
     *  of ROI Zones value.
     * @return  VL53L0X_ERROR_NONE      Success
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetMaxNumberOfROIZones(VL53L0X_DEV Dev,
        uint8_t *pMaxNumberOfROIZones);

    /** @} VL53L0X_measurement_group */

    /** @defgroup VL53L0X_interrupt_group VL53L0X Interrupt Functions
     *  @brief    Functions used for interrupt managements
     *  @{
     */

    /**
     * @brief Set the configuration of GPIO pin for a given device
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   Pin                   ID of the GPIO Pin
     * @param   Functionality         Select Pin functionality.
     *  Refer to ::VL53L0X_GpioFunctionality
     * @param   DeviceMode            Device Mode associated to the Gpio.
     * @param   Polarity              Set interrupt polarity. Active high
     *   or active low see ::VL53L0X_InterruptPolarity
     * @return  VL53L0X_ERROR_NONE                            Success
     * @return  VL53L0X_ERROR_GPIO_NOT_EXISTING               Only Pin=0 is accepted
     * @return  VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED    This error occurs
     * when Functionality programmed is not in the supported list:
     *                             Supported value are:
     *                             VL53L0X_GPIOFUNCTIONALITY_OFF,
     *                             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
     *                             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
     *                               VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
        VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
        VL53L0X_InterruptPolarity Polarity);

    /**
     * @brief Get current configuration for GPIO pin for a given device
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   Pin                   ID of the GPIO Pin
     * @param   pDeviceMode           Pointer to Device Mode associated to the Gpio.
     * @param   pFunctionality        Pointer to Pin functionality.
     *  Refer to ::VL53L0X_GpioFunctionality
     * @param   pPolarity             Pointer to interrupt polarity.
     *  Active high or active low see ::VL53L0X_InterruptPolarity
     * @return  VL53L0X_ERROR_NONE                            Success
     * @return  VL53L0X_ERROR_GPIO_NOT_EXISTING               Only Pin=0 is accepted
     * @return  VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED   This error occurs
     * when Functionality programmed is not in the supported list:
     *                      Supported value are:
     *                      VL53L0X_GPIOFUNCTIONALITY_OFF,
     *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
     *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
     *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
     *                      VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
        VL53L0X_DeviceModes * pDeviceMode,
        VL53L0X_GpioFunctionality * pFunctionality,
        VL53L0X_InterruptPolarity * pPolarity);

    /**
     * @brief Set low and high Interrupt thresholds for a given mode
     * (ranging, ALS, ...) for a given device
     *
     * @par Function Description
     * Set low and high Interrupt thresholds for a given mode (ranging, ALS, ...)
     * for a given device
     *
     * @note This function Access to the device
     *
     * @note DeviceMode is ignored for the current device
     *
     * @param   Dev              Device Handle
     * @param   DeviceMode       Device Mode for which change thresholds
     * @param   ThresholdLow     Low threshold (mm, lux ..., depending on the mode)
     * @param   ThresholdHigh    High threshold (mm, lux ..., depending on the mode)
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetInterruptThresholds(VL53L0X_DEV Dev,
        VL53L0X_DeviceModes DeviceMode, FixPoint1616_t ThresholdLow,
        FixPoint1616_t ThresholdHigh);

    /**
     * @brief  Get high and low Interrupt thresholds for a given mode
     *  (ranging, ALS, ...) for a given device
     *
     * @par Function Description
     * Get high and low Interrupt thresholds for a given mode (ranging, ALS, ...)
     * for a given device
     *
     * @note This function Access to the device
     *
     * @note DeviceMode is ignored for the current device
     *
     * @param   Dev              Device Handle
     * @param   DeviceMode       Device Mode from which read thresholds
     * @param   pThresholdLow    Low threshold (mm, lux ..., depending on the mode)
     * @param   pThresholdHigh   High threshold (mm, lux ..., depending on the mode)
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetInterruptThresholds(VL53L0X_DEV Dev,
        VL53L0X_DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow,
        FixPoint1616_t *pThresholdHigh);

    /**
     * @brief Return device stop completion status
     *
     * @par Function Description
     * Returns stop completiob status.
     * User shall call this function after a stop command
     *
     * @note This function Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   pStopStatus            Pointer to status variable to update
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetStopCompletedStatus(VL53L0X_DEV Dev,
        uint32_t *pStopStatus);


    /**
     * @brief Clear given system interrupt condition
     *
     * @par Function Description
     * Clear given interrupt(s).
     *
     * @note This function Access to the device
     *
     * @param   Dev                  Device Handle
     * @param   InterruptMask        Mask of interrupts to clear
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  VL53L0X_ERROR_INTERRUPT_NOT_CLEARED    Cannot clear interrupts
     *
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_ClearInterruptMask(VL53L0X_DEV Dev,
        uint32_t InterruptMask);

    /**
     * @brief Return device interrupt status
     *
     * @par Function Description
     * Returns currently raised interrupts by the device.
     * User shall be able to activate/deactivate interrupts through
     * @a VL53L0X_SetGpioConfig()
     *
     * @note This function Access to the device
     *
     * @param   Dev                    Device Handle
     * @param   pInterruptMaskStatus   Pointer to status variable to update
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetInterruptMaskStatus(VL53L0X_DEV Dev,
        uint32_t *pInterruptMaskStatus);

    /**
     * @brief Configure ranging interrupt reported to system
     *
     * @note This function is not Implemented
     *
     * @param   Dev                  Device Handle
     * @param   InterruptMask         Mask of interrupt to Enable/disable
     *  (0:interrupt disabled or 1: interrupt enabled)
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_API VL53L0X_Error VL53L0X_EnableInterruptMask(VL53L0X_DEV Dev,
        uint32_t InterruptMask);

    /** @} VL53L0X_interrupt_group */

    /** @defgroup VL53L0X_SPADfunctions_group VL53L0X SPAD Functions
     *  @brief    Functions used for SPAD managements
     *  @{
     */

    /**
     * @brief  Set the SPAD Ambient Damper Threshold value
     *
     * @par Function Description
     * This function set the SPAD Ambient Damper Threshold value
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   SpadAmbientDamperThreshold    SPAD Ambient Damper Threshold value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetSpadAmbientDamperThreshold(VL53L0X_DEV Dev,
        uint16_t SpadAmbientDamperThreshold);

    /**
     * @brief  Get the current SPAD Ambient Damper Threshold value
     *
     * @par Function Description
     * This function get the SPAD Ambient Damper Threshold value
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   pSpadAmbientDamperThreshold   Pointer to programmed
     *                                        SPAD Ambient Damper Threshold value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSpadAmbientDamperThreshold(VL53L0X_DEV Dev,
        uint16_t *pSpadAmbientDamperThreshold);

    /**
     * @brief  Set the SPAD Ambient Damper Factor value
     *
     * @par Function Description
     * This function set the SPAD Ambient Damper Factor value
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   SpadAmbientDamperFactor       SPAD Ambient Damper Factor value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetSpadAmbientDamperFactor(VL53L0X_DEV Dev,
        uint16_t SpadAmbientDamperFactor);

    /**
     * @brief  Get the current SPAD Ambient Damper Factor value
     *
     * @par Function Description
     * This function get the SPAD Ambient Damper Factor value
     *
     * @note This function Access to the device
     *
     * @param   Dev                           Device Handle
     * @param   pSpadAmbientDamperFactor      Pointer to programmed SPAD Ambient
     * Damper Factor value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetSpadAmbientDamperFactor(VL53L0X_DEV Dev,
        uint16_t *pSpadAmbientDamperFactor);

    /**
     * @brief Performs Reference Spad Management
     *
     * @par Function Description
     * The reference SPAD initialization procedure determines the minimum amount
     * of reference spads to be enables to achieve a target reference signal rate
     * and should be performed once during initialization.
     *
     * @note This function Access to the device
     *
     * @note This function change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING
     *
     * @param   Dev                          Device Handle
     * @param   refSpadCount                 Reports ref Spad Count
     * @param   isApertureSpads              Reports if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT   Error in the Ref Spad procedure.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_PerformRefSpadManagement(VL53L0X_DEV Dev,
        uint32_t *refSpadCount, uint8_t *isApertureSpads);

    /**
     * @brief Applies Reference SPAD configuration
     *
     * @par Function Description
     * This function applies a given number of reference spads, identified as
     * either Aperture or Non-Aperture.
     * The requested spad count and type are stored within the device specific
     * parameters data for access by the host.
     *
     * @note This function Access to the device
     *
     * @param   Dev                          Device Handle
     * @param   refSpadCount                 Number of ref spads.
     * @param   isApertureSpads              Defines if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT   Error in the in the reference
     *                                       spad configuration.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_SetReferenceSpads(VL53L0X_DEV Dev,
        uint32_t refSpadCount, uint8_t isApertureSpads);

    /**
     * @brief Retrieves SPAD configuration
     *
     * @par Function Description
     * This function retrieves the current number of applied reference spads
     * and also their type : Aperture or Non-Aperture.
     *
     * @note This function Access to the device
     *
     * @param   Dev                          Device Handle
     * @param   refSpadCount                 Number ref Spad Count
     * @param   isApertureSpads              Reports if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT   Error in the in the reference
     *                                       spad configuration.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_API VL53L0X_Error VL53L0X_GetReferenceSpads(VL53L0X_DEV Dev,
        uint32_t *refSpadCount, uint8_t *isApertureSpads);

    /** @} VL53L0X_SPADfunctions_group */

    /** @} VL53L0X_cut11_group */

    // ====================================================================
    //                          API CORE
    // ====================================================================
    VL53L0X_Error VL53L0X_reverse_bytes(uint8_t *data, uint32_t size);
    VL53L0X_Error VL53L0X_measurement_poll_for_completion(VL53L0X_DEV Dev);
    uint8_t VL53L0X_encode_vcsel_period(uint8_t vcsel_period_pclks);
    uint8_t VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg);
    uint32_t VL53L0X_isqrt(uint32_t num);
    uint32_t VL53L0X_quadrature_sum(uint32_t a, uint32_t b);
    VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8_t option);
    VL53L0X_Error VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV Dev, VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);
    VL53L0X_Error VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV Dev, VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK);
    uint32_t VL53L0X_decode_timeout(uint16_t encoded_timeout);
    VL53L0X_Error get_sequence_step_timeout(VL53L0X_DEV Dev, VL53L0X_SequenceStepId SequenceStepId, uint32_t *pTimeOutMicroSecs);
    VL53L0X_Error set_sequence_step_timeout(VL53L0X_DEV Dev, VL53L0X_SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs);
    VL53L0X_Error VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev, uint32_t MeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev, uint32_t *pMeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error VL53L0X_load_tuning_settings(VL53L0X_DEV Dev, uint8_t *pTuningSettingBuffer);
    VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *pSigmaEstimate);
    VL53L0X_Error VL53L0X_calc_dmax(VL53L0X_DEV Dev, FixPoint1616_t ambRateMeas, uint32_t *pdmax_mm);
    VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev,VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_xtalk_rate_mcps);
    VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t *ptotal_signal_rate_mcps);
    VL53L0X_Error VL53L0X_get_pal_range_status(VL53L0X_DEV Dev,
                                               uint8_t DeviceRangeStatus,
                                               FixPoint1616_t SignalRate,
                                               uint16_t EffectiveSpadRtnCount,
                                               VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
                                               uint8_t *pPalRangeStatus);
    uint32_t VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev, uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    uint16_t VL53L0X_encode_timeout(uint32_t timeout_macro_clks);

private :
    VL53L0X_Error VL53L0X_device_read_strobe(VL53L0X_DEV Dev);
    uint32_t VL53L0X_calc_macro_period_ps(VL53L0X_DEV Dev, uint8_t vcsel_period_pclks);
    uint32_t VL53L0X_calc_timeout_us(VL53L0X_DEV Dev, uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    VL53L0X_Error get_dmax_lut_points(VL53L0X_DMaxLUT_t data, uint32_t lut_size, FixPoint1616_t input, int32_t *index0, int32_t *index1);
    VL53L0X_Error sequence_step_enabled(VL53L0X_DEV Dev, VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceConfig, uint8_t *pSequenceStepEnabled);
    VL53L0X_Error VL53L0X_CheckAndLoadInterruptSettings(VL53L0X_DEV Dev, uint8_t StartNotStopFlag);

    // ====================================================================
    //                        CALIBRATION
    // ====================================================================
public :
#define REF_ARRAY_SPAD_0  0
#define REF_ARRAY_SPAD_5  5
#define REF_ARRAY_SPAD_10 10

    uint32_t refArrayQuadrants[4] = {REF_ARRAY_SPAD_10, REF_ARRAY_SPAD_5,
                                     REF_ARRAY_SPAD_0, REF_ARRAY_SPAD_5 };

    VL53L0X_Error VL53L0X_perform_xtalk_calibration(VL53L0X_DEV Dev, FixPoint1616_t XTalkCalDistance, FixPoint1616_t *pXTalkCompensationRateMegaCps);
    VL53L0X_Error VL53L0X_perform_offset_calibration(VL53L0X_DEV Dev, FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter);
    VL53L0X_Error VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV Dev, int32_t OffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV Dev, int32_t *pOffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_apply_offset_adjustment(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_perform_ref_spad_management(VL53L0X_DEV Dev, uint32_t *refSpadCount, uint8_t *isApertureSpads);
    VL53L0X_Error VL53L0X_set_reference_spads(VL53L0X_DEV Dev, uint32_t count, uint8_t isApertureSpads);
    VL53L0X_Error VL53L0X_get_reference_spads(VL53L0X_DEV Dev, uint32_t *pSpadCount, uint8_t *pIsApertureSpads);
    VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev, uint8_t *pPhaseCal, const uint8_t get_data_enable, const uint8_t restore_config);
    VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable);
    VL53L0X_Error VL53L0X_set_ref_calibration(VL53L0X_DEV Dev, uint8_t VhvSettings, uint8_t PhaseCal);
    VL53L0X_Error VL53L0X_get_ref_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, uint8_t *pPhaseCal);

private :
    void get_next_good_spad(uint8_t goodSpadArray[], uint32_t size, uint32_t curr, int32_t *next);
    uint8_t is_aperture(uint32_t spadIndex);
    VL53L0X_Error enable_spad_bit(uint8_t spadArray[], uint32_t size, uint32_t spadIndex);
    VL53L0X_Error count_enabled_spads(uint8_t spadArray[], uint32_t byteCount, uint32_t maxSpads, uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture);
    VL53L0X_Error set_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);
    VL53L0X_Error get_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);
    VL53L0X_Error enable_ref_spads(VL53L0X_DEV Dev, uint8_t apertureSpads, uint8_t goodSpadArray[], uint8_t spadArray[], uint32_t size, uint32_t start, uint32_t offset, uint32_t spadCount, uint32_t *lastSpad);
    VL53L0X_Error perform_ref_signal_measurement(VL53L0X_DEV Dev, uint16_t *refSignalRate);
    VL53L0X_Error VL53L0X_perform_single_ref_calibration(VL53L0X_DEV Dev, uint8_t vhv_init_byte);
    VL53L0X_Error VL53L0X_ref_calibration_io(VL53L0X_DEV Dev, uint8_t read_not_write, uint8_t VhvSettings, uint8_t PhaseCal, uint8_t *pVhvSettings, uint8_t *pPhaseCal, const uint8_t vhv_enable, const uint8_t phase_enable);
    VL53L0X_Error VL53L0X_perform_vhv_calibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings, const uint8_t get_data_enable, const uint8_t restore_config);


    // ====================================================================
    //                            API STRINGS
    // ====================================================================
public :
    VL53L0X_Error VL53L0X_get_device_info(VL53L0X_DEV Dev, VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);
    VL53L0X_Error VL53L0X_get_device_error_string(VL53L0X_DeviceError ErrorCode, char *pDeviceErrorString);
    VL53L0X_Error VL53L0X_get_range_status_string(uint8_t RangeStatus, char *pRangeStatusString);
    VL53L0X_Error VL53L0X_get_pal_error_string(VL53L0X_Error PalErrorCode, char *pPalErrorString);
    VL53L0X_Error VL53L0X_get_pal_state_string(VL53L0X_State PalStateCode, char *pPalStateString);
    VL53L0X_Error VL53L0X_get_sequence_steps_info(VL53L0X_SequenceStepId SequenceStepId, char *pSequenceStepsString);
    VL53L0X_Error VL53L0X_get_limit_check_info(VL53L0X_DEV Dev, uint16_t LimitCheckId, char *pLimitCheckString);

private :
    VL53L0X_Error VL53L0X_check_part_used(VL53L0X_DEV Dev, uint8_t *Revision, VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);

    #ifdef USE_EMPTY_STRING
        #define  VL53L0X_STRING_DEVICE_INFO_NAME                             ""
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0                         ""
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1                         ""
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2                         ""
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1                         ""
        #define  VL53L0X_STRING_DEVICE_INFO_TYPE                             ""

        /* PAL ERROR strings */
        #define  VL53L0X_STRING_ERROR_NONE                                   ""
        #define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING                    ""
        #define  VL53L0X_STRING_ERROR_MIN_CLIPPED                            ""
        #define  VL53L0X_STRING_ERROR_UNDEFINED                              ""
        #define  VL53L0X_STRING_ERROR_INVALID_PARAMS                         ""
        #define  VL53L0X_STRING_ERROR_NOT_SUPPORTED                          ""
        #define  VL53L0X_STRING_ERROR_RANGE_ERROR                            ""
        #define  VL53L0X_STRING_ERROR_TIME_OUT                               ""
        #define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED                     ""
        #define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL                       ""
        #define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING                      ""
        #define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED       ""
        #define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE                      ""
        #define  VL53L0X_STRING_ERROR_INVALID_COMMAND                        ""
        #define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO                       ""
        #define  VL53L0X_STRING_ERROR_REF_SPAD_INIT                          ""
        #define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED                        ""

        #define  VL53L0X_STRING_UNKNOW_ERROR_CODE                            ""



        /* Range Status */
        #define  VL53L0X_STRING_RANGESTATUS_NONE                             ""
        #define  VL53L0X_STRING_RANGESTATUS_RANGEVALID                       ""
        #define  VL53L0X_STRING_RANGESTATUS_SIGMA                            ""
        #define  VL53L0X_STRING_RANGESTATUS_SIGNAL                           ""
        #define  VL53L0X_STRING_RANGESTATUS_MINRANGE                         ""
        #define  VL53L0X_STRING_RANGESTATUS_PHASE                            ""
        #define  VL53L0X_STRING_RANGESTATUS_HW                               ""


        /* Range Status */
        #define  VL53L0X_STRING_STATE_POWERDOWN                              ""
        #define  VL53L0X_STRING_STATE_WAIT_STATICINIT                        ""
        #define  VL53L0X_STRING_STATE_STANDBY                                ""
        #define  VL53L0X_STRING_STATE_IDLE                                   ""
        #define  VL53L0X_STRING_STATE_RUNNING                                ""
        #define  VL53L0X_STRING_STATE_UNKNOWN                                ""
        #define  VL53L0X_STRING_STATE_ERROR                                  ""


        /* Device Specific */
        #define  VL53L0X_STRING_DEVICEERROR_NONE                             ""
        #define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE       ""
        #define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE         ""
        #define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND                  ""
        #define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET                     ""
        #define  VL53L0X_STRING_DEVICEERROR_SNRCHECK                         ""
        #define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK                  ""
        #define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK              ""
        #define  VL53L0X_STRING_DEVICEERROR_TCC                              ""
        #define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY                 ""
        #define  VL53L0X_STRING_DEVICEERROR_MINCLIP                          ""
        #define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE                    ""
        #define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW                    ""
        #define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW                     ""
        #define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD             ""
        #define  VL53L0X_STRING_DEVICEERROR_UNKNOWN                          ""

        /* Check Enable */
        #define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE                ""
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE          ""
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP                  ""
        #define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD           ""

        /* Sequence Step */
        #define  VL53L0X_STRING_SEQUENCESTEP_TCC                             ""
        #define  VL53L0X_STRING_SEQUENCESTEP_DSS                             ""
        #define  VL53L0X_STRING_SEQUENCESTEP_MSRC                            ""
        #define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE                       ""
        #define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE                     ""
    #else
        #define  VL53L0X_STRING_DEVICE_INFO_NAME          "VL53L0X cut1.0"
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0      "VL53L0X TS0"
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1      "VL53L0X TS1"
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2      "VL53L0X TS2"
        #define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1      "VL53L0X ES1 or later"
        #define  VL53L0X_STRING_DEVICE_INFO_TYPE          "VL53L0X"

        /* PAL ERROR strings */
        #define  VL53L0X_STRING_ERROR_NONE \
                "No Error"
        #define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING \
                "Calibration Warning Error"
        #define  VL53L0X_STRING_ERROR_MIN_CLIPPED \
                "Min clipped error"
        #define  VL53L0X_STRING_ERROR_UNDEFINED \
                "Undefined error"
        #define  VL53L0X_STRING_ERROR_INVALID_PARAMS \
                "Invalid parameters error"
        #define  VL53L0X_STRING_ERROR_NOT_SUPPORTED \
                "Not supported error"
        #define  VL53L0X_STRING_ERROR_RANGE_ERROR \
                "Range error"
        #define  VL53L0X_STRING_ERROR_TIME_OUT \
                "Time out error"
        #define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED \
                "Mode not supported error"
        #define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL \
                "Buffer too small"
        #define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING \
                "GPIO not existing"
        #define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED \
                "GPIO funct not supported"
        #define  VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED \
                "Interrupt not Cleared"
        #define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE \
                "Control Interface Error"
        #define  VL53L0X_STRING_ERROR_INVALID_COMMAND \
                "Invalid Command Error"
        #define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO \
                "Division by zero Error"
        #define  VL53L0X_STRING_ERROR_REF_SPAD_INIT \
                "Reference Spad Init Error"
        #define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED \
                "Not implemented error"

        #define  VL53L0X_STRING_UNKNOW_ERROR_CODE \
                "Unknown Error Code"



        /* Range Status */
        #define  VL53L0X_STRING_RANGESTATUS_NONE                 "No Update"
        #define  VL53L0X_STRING_RANGESTATUS_RANGEVALID           "Range Valid"
        #define  VL53L0X_STRING_RANGESTATUS_SIGMA                "Sigma Fail"
        #define  VL53L0X_STRING_RANGESTATUS_SIGNAL               "Signal Fail"
        #define  VL53L0X_STRING_RANGESTATUS_MINRANGE             "Min Range Fail"
        #define  VL53L0X_STRING_RANGESTATUS_PHASE                "Phase Fail"
        #define  VL53L0X_STRING_RANGESTATUS_HW                   "Hardware Fail"


        /* Range Status */
        #define  VL53L0X_STRING_STATE_POWERDOWN               "POWERDOWN State"
        #define  VL53L0X_STRING_STATE_WAIT_STATICINIT \
                "Wait for staticinit State"
        #define  VL53L0X_STRING_STATE_STANDBY                 "STANDBY State"
        #define  VL53L0X_STRING_STATE_IDLE                    "IDLE State"
        #define  VL53L0X_STRING_STATE_RUNNING                 "RUNNING State"
        #define  VL53L0X_STRING_STATE_UNKNOWN                 "UNKNOWN State"
        #define  VL53L0X_STRING_STATE_ERROR                   "ERROR State"


        /* Device Specific */
        #define  VL53L0X_STRING_DEVICEERROR_NONE                   "No Update"
        #define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE \
                "VCSEL Continuity Test Failure"
        #define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE \
                "VCSEL Watchdog Test Failure"
        #define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND \
                "No VHV Value found"
        #define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET \
                "MSRC No Target Error"
        #define  VL53L0X_STRING_DEVICEERROR_SNRCHECK \
                "SNR Check Exit"
        #define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK \
                "Range Phase Check Error"
        #define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK \
                "Sigma Threshold Check Error"
        #define  VL53L0X_STRING_DEVICEERROR_TCC \
                "TCC Error"
        #define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY \
                "Phase Consistency Error"
        #define  VL53L0X_STRING_DEVICEERROR_MINCLIP \
                "Min Clip Error"
        #define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE \
                "Range Complete"
        #define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW \
                "Range Algo Underflow Error"
        #define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW \
                "Range Algo Overlow Error"
        #define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD \
                "Range Ignore Threshold Error"
        #define  VL53L0X_STRING_DEVICEERROR_UNKNOWN \
                "Unknown error code"

        /* Check Enable */
        #define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE \
                "SIGMA FINAL RANGE"
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE \
                "SIGNAL RATE FINAL RANGE"
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP \
                "SIGNAL REF CLIP"
        #define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD \
                "RANGE IGNORE THRESHOLD"
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC \
                "SIGNAL RATE MSRC"
        #define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE \
                "SIGNAL RATE PRE RANGE"

        /* Sequence Step */
        #define  VL53L0X_STRING_SEQUENCESTEP_TCC                   "TCC"
        #define  VL53L0X_STRING_SEQUENCESTEP_DSS                   "DSS"
        #define  VL53L0X_STRING_SEQUENCESTEP_MSRC                  "MSRC"
        #define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE             "PRE RANGE"
        #define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE           "FINAL RANGE"
    #endif /* USE_EMPTY_STRING */




};

#endif // _VL53L0X_BASE_H_
