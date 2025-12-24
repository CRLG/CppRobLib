#include "vl53l0x_base.h"
#include "vl53l0x_device.h"
#include "vl53l0x_interrupt_threshold_settings.h"
#include "vl53l0x_tuning.h"
#include "vl53l0x_platform_log.h"

// _____________________________________________
// Directives & Macros
#ifndef __KERNEL__
#include <stdlib.h>
#endif

#define LOG_FUNCTION_START(fmt, ... )           _LOG_FUNCTION_START(TRACE_MODULE_PLATFORM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ... )          _LOG_FUNCTION_END(TRACE_MODULE_PLATFORM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ... ) _LOG_FUNCTION_END_FMT(TRACE_MODULE_PLATFORM, status, fmt, ##__VA_ARGS__)

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_API, \
    level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#endif


// _____________________________________________
VL53L0xBase::VL53L0xBase()
	: m_last_distance(0xFFFF),
	  m_read_error_count(0),
	  m_init_ok(false)
{
}

// _____________________________________________
VL53L0xBase::~VL53L0xBase()
{
}

// ====================================================================
//                            MAIN API
// ====================================================================


// _____________________________________________
VL53L0X_Error VL53L0xBase::init(uint8_t i2c_address, uint32_t time_budget)
{
    VL53L0X_Error status;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    VL53L0X_DEV Dev = get_device();

    m_vl53_dev.I2cDevAddr = VL53L0X_I2C_DEFAULT_ADDR; // device starts always with default adress
    m_vl53_dev.comms_type = 1; // I2C
    m_vl53_dev.comms_speed_khz = 400; // kHz

    status = VL53L0X_ResetDevice(Dev);
    if (status != VL53L0X_ERROR_NONE)   return status;

    // change I2C address if needed
    if(i2c_address != VL53L0X_I2C_DEFAULT_ADDR) {
    	status = change_adress(i2c_address);
        if (status != VL53L0X_ERROR_NONE)   return status;
    }

    status = VL53L0X_DataInit(Dev);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_StaticInit(Dev); // Device Initialization
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads); // Device Initialization
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, time_budget);
    if (status != VL53L0X_ERROR_NONE)   return status;

    status = VL53L0X_StartMeasurement(Dev);
    if (status != VL53L0X_ERROR_NONE)   return status;

    m_init_ok = true;

    return VL53L0X_ERROR_NONE;
}

// _____________________________________________
VL53L0X_Error VL53L0xBase::read_distance(uint16_t *distance)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	const bool light_processing = true;

	Status = VL53L0X_GetRangingMeasurementData(&m_vl53_dev, &RangingMeasurementData, light_processing);
	if (Status == VL53L0X_ERROR_NONE) {
		m_last_distance = RangingMeasurementData.RangeMilliMeter;
		m_read_error_count = 0;
	}
	else {
		m_last_distance = 0xFFFF;
		m_read_error_count++;
	}
	if (distance) *distance= m_last_distance;

	return Status;
}


// _____________________________________________
uint16_t VL53L0xBase::get_last_distance()
{
	return m_last_distance;
}


// _____________________________________________
VL53L0X_Error VL53L0xBase::change_adress(uint8_t new_adress)
{
	VL53L0X_Error status = VL53L0X_SetDeviceAddress(&m_vl53_dev, new_adress);
	m_vl53_dev.I2cDevAddr = new_adress;
    if (status != VL53L0X_ERROR_NONE) return status;
    return VL53L0X_ERROR_NONE;
}


// _____________________________________________
VL53L0xBase::VL53L0X_DEV VL53L0xBase::get_device()
{
    return &m_vl53_dev;
}



// ====================================================================
//                            PLATFORM
// ====================================================================
/**
 * @def I2C_BUFFER_CONFIG
 *
 * @brief Configure Device register I2C access
 *
 * @li 0 : one GLOBAL buffer \n
 *   Use one global buffer of MAX_I2C_XFER_SIZE byte in data space \n
 *   This solution is not multi-Device compliant nor multi-thread cpu safe \n
 *   It can be the best option for small 8/16 bit MCU without stack and limited ram  (STM8s, 80C51 ...)
 *
 * @li 1 : ON_STACK/local \n
 *   Use local variable (on stack) buffer \n
 *   This solution is multi-thread with use of i2c resource lock or mutex see VL6180x_GetI2CAccess() \n
 *
 * @li 2 : User defined \n
 *    Per Device potentially dynamic allocated. Requires VL6180x_GetI2cBuffer() to be implemented.
 * @ingroup Configuration
 */
#define I2C_BUFFER_CONFIG 1
/** Maximum buffer size to be used in i2c */
#define VL53L0X_MAX_I2C_XFER_SIZE   64 /* Maximum buffer size to be used in i2c */

#if I2C_BUFFER_CONFIG == 0
    /* GLOBAL config buffer */
    uint8_t i2c_global_buffer[VL53L0X_MAX_I2C_XFER_SIZE];

    #define DECL_I2C_BUFFER
    #define VL53L0X_GetLocalBuffer(Dev, n_byte)  i2c_global_buffer

#elif I2C_BUFFER_CONFIG == 1
    /* ON STACK */
    #define DECL_I2C_BUFFER  uint8_t LocBuffer[VL53L0X_MAX_I2C_XFER_SIZE];
    #define VL53L0X_GetLocalBuffer(Dev, n_byte)  LocBuffer
#elif I2C_BUFFER_CONFIG == 2
    /* user define buffer type declare DECL_I2C_BUFFER  as access  via VL53L0X_GetLocalBuffer */
    #define DECL_I2C_BUFFER
#else
#error "invalid I2C_BUFFER_CONFIG "
#endif


#define VL53L0X_I2C_USER_VAR         /* none but could be for a flag var to get/pass to mutex interruptible  return flags and try again */
#define VL53L0X_GetI2CAccess(Dev)    /* todo mutex acquire */
#define VL53L0X_DoneI2CAcces(Dev)    /* todo mutex release */


VL53L0X_Error VL53L0xBase::VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0xBase::VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int = 0;
    uint8_t deviceAddress;

    if (count>=VL53L0X_MAX_I2C_XFER_SIZE){
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_write_register(deviceAddress, index, pdata, count);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0xBase::VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){
    VL53L0X_I2C_USER_VAR
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    if (count>=VL53L0X_MAX_I2C_XFER_SIZE){
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_read_register(deviceAddress, index, pdata, count);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_write_register(deviceAddress, index, &data, 1);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t  buffer[2];

    deviceAddress = Dev->I2cDevAddr;
    //decoupage 16-bit en MSB-LSB
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    status_int = i2c_write_register(deviceAddress, index, buffer, 2);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t buffer[4];

    deviceAddress = Dev->I2cDevAddr;

    //decoupage 32-bit en MSB-LSB
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status_int = i2c_write_register(deviceAddress, index, buffer, 4);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t data;

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_read_register(deviceAddress, index, &data, 1);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    if (Status == VL53L0X_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status_int = i2c_write_register(deviceAddress, index, &data, 1);

        if (status_int != true)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_read_register(deviceAddress, index, data, 1);

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t buffer[2];

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_read_register(deviceAddress, index, buffer, 2);
    *data = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error  VL53L0xBase::VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t buffer[4];

    deviceAddress = Dev->I2cDevAddr;

    status_int = i2c_read_register(deviceAddress, index, buffer, 4);
    *data = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    if (status_int != true)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

#define VL53L0X_POLLINGDELAY_LOOPNB  250
VL53L0X_Error VL53L0xBase::VL53L0X_PollingDelay(VL53L0X_DEV Dev){
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    delay_ms(1);

    LOG_FUNCTION_END(status);
    return status;
}

// ====================================================================
//                             API
// ====================================================================
/* Group PAL General Functions */

VL53L0X_Error VL53L0xBase::VL53L0X_GetVersion(VL53L0X_Version_t *pVersion)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    pVersion->major = VL53L0X_IMPLEMENTATION_VER_MAJOR;
    pVersion->minor = VL53L0X_IMPLEMENTATION_VER_MINOR;
    pVersion->build = VL53L0X_IMPLEMENTATION_VER_SUB;

    pVersion->revision = VL53L0X_IMPLEMENTATION_VER_REVISION;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetPalSpecVersion(VL53L0X_Version_t *pPalSpecVersion)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    pPalSpecVersion->major = VL53L0X_SPECIFICATION_VER_MAJOR;
    pPalSpecVersion->minor = VL53L0X_SPECIFICATION_VER_MINOR;
    pPalSpecVersion->build = VL53L0X_SPECIFICATION_VER_SUB;

    pPalSpecVersion->revision = VL53L0X_SPECIFICATION_VER_REVISION;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetProductRevision(VL53L0X_DEV Dev,
    uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t revision_id;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_IDENTIFICATION_REVISION_ID,
        &revision_id);
    *pProductRevisionMajor = 1;
    *pProductRevisionMinor = (revision_id & 0xF0) >> 4;

    LOG_FUNCTION_END(Status);
    return Status;

}

VL53L0X_Error VL53L0xBase::VL53L0X_GetDeviceInfo(VL53L0X_DEV Dev,
    VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_device_info(Dev, pVL53L0X_DeviceInfo);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetDeviceErrorStatus(VL53L0X_DEV Dev,
    VL53L0X_DeviceError *pDeviceErrorStatus)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t RangeStatus;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_RESULT_RANGE_STATUS,
        &RangeStatus);

    *pDeviceErrorStatus = (VL53L0X_DeviceError)((RangeStatus & 0x78) >> 3);

    LOG_FUNCTION_END(Status);
    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_GetDeviceErrorString(VL53L0X_DeviceError ErrorCode,
    char *pDeviceErrorString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_device_error_string(ErrorCode, pDeviceErrorString);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetRangeStatusString(uint8_t RangeStatus,
    char *pRangeStatusString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_range_status_string(RangeStatus,
        pRangeStatusString);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetPalErrorString(VL53L0X_Error PalErrorCode,
    char *pPalErrorString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_pal_error_string(PalErrorCode, pPalErrorString);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetPalStateString(VL53L0X_State PalStateCode,
    char *pPalStateString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_pal_state_string(PalStateCode, pPalStateString);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetPalState(VL53L0X_DEV Dev, VL53L0X_State *pPalState)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pPalState = PALDevDataGet(Dev, PalState);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetPowerMode(VL53L0X_DEV Dev,
                   VL53L0X_PowerModes PowerMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    /* Only level1 of Power mode exists */
    if ((PowerMode != VL53L0X_POWERMODE_STANDBY_LEVEL1)
        && (PowerMode != VL53L0X_POWERMODE_IDLE_LEVEL1)) {
        Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
    } else if (PowerMode == VL53L0X_POWERMODE_STANDBY_LEVEL1) {
        /* set the standby level1 of power mode */
        Status = VL53L0X_WrByte(Dev, 0x80, 0x00);
        if (Status == VL53L0X_ERROR_NONE) {
            /* Set PAL State to standby */
            PALDevDataSet(Dev, PalState, VL53L0X_STATE_STANDBY);
            PALDevDataSet(Dev, PowerMode,
                VL53L0X_POWERMODE_STANDBY_LEVEL1);
        }

    } else {
        /* VL53L0X_POWERMODE_IDLE_LEVEL1 */
        Status = VL53L0X_WrByte(Dev, 0x80, 0x00);
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_StaticInit(Dev);

        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, PowerMode,
                VL53L0X_POWERMODE_IDLE_LEVEL1);

    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetPowerMode(VL53L0X_DEV Dev,
                   VL53L0X_PowerModes *pPowerMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    /* Only level1 of Power mode exists */
    Status = VL53L0X_RdByte(Dev, 0x80, &Byte);

    if (Status == VL53L0X_ERROR_NONE) {
        if (Byte == 1) {
            PALDevDataSet(Dev, PowerMode,
                VL53L0X_POWERMODE_IDLE_LEVEL1);
        } else {
            PALDevDataSet(Dev, PowerMode,
                VL53L0X_POWERMODE_STANDBY_LEVEL1);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
    int32_t OffsetCalibrationDataMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_set_offset_calibration_data_micro_meter(Dev,
        OffsetCalibrationDataMicroMeter);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
    int32_t *pOffsetCalibrationDataMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_offset_calibration_data_micro_meter(Dev,
        pOffsetCalibrationDataMicroMeter);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetLinearityCorrectiveGain(VL53L0X_DEV Dev,
    int16_t LinearityCorrectiveGain)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    if ((LinearityCorrectiveGain < 0) || (LinearityCorrectiveGain > 1000))
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    else {
        PALDevDataSet(Dev, LinearityCorrectiveGain,
            LinearityCorrectiveGain);

        if (LinearityCorrectiveGain != 1000) {
            /* Disable FW Xtalk */
            Status = VL53L0X_WrWord(Dev,
            VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 0);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLinearityCorrectiveGain(VL53L0X_DEV Dev,
    uint16_t *pLinearityCorrectiveGain)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pLinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetGroupParamHold(VL53L0X_DEV Dev, uint8_t GroupParamHold)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetUpperLimitMilliMeter(VL53L0X_DEV Dev,
    uint16_t *pUpperLimitMilliMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetTotalSignalRate(VL53L0X_DEV Dev,
    FixPoint1616_t *pTotalSignalRate)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

    LOG_FUNCTION_START("");

    LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

    Status = VL53L0X_get_total_signal_rate(
        Dev, &LastRangeDataBuffer, pTotalSignalRate);

    LOG_FUNCTION_END(Status);
    return Status;
}

/* End Group PAL General Functions */

/* Group PAL Init Functions */
VL53L0X_Error VL53L0xBase::VL53L0X_SetDeviceAddress(VL53L0X_DEV Dev, uint8_t DeviceAddress)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
        DeviceAddress / 2);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_DataInit(VL53L0X_DEV Dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_DeviceParameters_t CurrentParameters;
	int i;
	uint8_t StopVariable;

	LOG_FUNCTION_START("");

	/* by default the I2C is running at 1V8 if you want to change it you
	 * need to include this define at compilation level. */
#ifdef USE_I2C_2V8
	Status = VL53L0X_UpdateByte(Dev,
		VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
		0xFE,
		0x01);
#endif

	/* Set I2C standard mode */
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_WrByte(Dev, 0x88, 0x00);

	VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone, 0);

#ifdef USE_IQC_STATION
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_apply_offset_adjustment(Dev);
#endif

	/* Default value is 1000 for Linearity Corrective Gain */
	PALDevDataSet(Dev, LinearityCorrectiveGain, 1000);

	/* Dmax default Parameter */
	PALDevDataSet(Dev, DmaxCalRangeMilliMeter, 400);
	PALDevDataSet(Dev, DmaxCalSignalRateRtnMegaCps,
		(FixPoint1616_t)((0x00016B85))); /* 1.42 No Cover Glass*/

	/* Set Default static parameters
	 *set first temporary values 9.44MHz * 65536 = 618660 */
	VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz, 618660);

	/* Set Default XTalkCompensationRateMegaCps to 0  */
	VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, 0);

	/* Get default parameters */
	Status = VL53L0X_GetDeviceParameters(Dev, &CurrentParameters);
	if (Status == VL53L0X_ERROR_NONE) {
		/* initialize PAL values */
		CurrentParameters.DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
		CurrentParameters.HistogramMode = VL53L0X_HISTOGRAMMODE_DISABLED;
		PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
	}

	/* Sigma estimator variable */
	PALDevDataSet(Dev, SigmaEstRefArray, 100);
	PALDevDataSet(Dev, SigmaEstEffPulseWidth, 900);
	PALDevDataSet(Dev, SigmaEstEffAmbWidth, 500);
	PALDevDataSet(Dev, targetRefRate, 0x0A00); /* 20 MCPS in 9:7 format */

	/* Use internal default settings */
	PALDevDataSet(Dev, UseInternalTuningSettings, 1);

	Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);
	Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
	Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);
	Status |= VL53L0X_RdByte(Dev, 0x91, &StopVariable);
	PALDevDataSet(Dev, StopVariable, StopVariable);
	Status |= VL53L0X_WrByte(Dev, 0x00, 0x01);
	Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
	Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);

	/* Enable all check */
	for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
		if (Status == VL53L0X_ERROR_NONE)
			Status |= VL53L0X_SetLimitCheckEnable(Dev, i, 1);
		else
			break;

	}

	/* Disable the following checks */
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, 0);

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, 0);

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetLimitCheckEnable(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, 0);

	/* Limit default values */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(Dev,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				(FixPoint1616_t)(18 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				(FixPoint1616_t)(25 * 65536 / 100));
				/* 0.25 * 65536 */
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(Dev,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
				(FixPoint1616_t)(35 * 65536));
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(Dev,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
				(FixPoint1616_t)(0 * 65536));
	}

	if (Status == VL53L0X_ERROR_NONE) {

		PALDevDataSet(Dev, SequenceConfig, 0xFF);
		Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
			0xFF);

		/* Set PAL state to tell that we are waiting for call to
		 * VL53L0X_StaticInit */
		PALDevDataSet(Dev, PalState, VL53L0X_STATE_WAIT_STATICINIT);
	}

	if (Status == VL53L0X_ERROR_NONE)
		VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 0);


	LOG_FUNCTION_END(Status);
	return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetTuningSettingBuffer(VL53L0X_DEV Dev,
	uint8_t *pTuningSettingBuffer, uint8_t UseInternalTuningSettings)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (UseInternalTuningSettings == 1) {
		/* Force use internal settings */
		PALDevDataSet(Dev, UseInternalTuningSettings, 1);
	} else {

		/* check that the first byte is not 0 */
		if (*pTuningSettingBuffer != 0) {
			PALDevDataSet(Dev, pTuningSettingsPointer,
				pTuningSettingBuffer);
			PALDevDataSet(Dev, UseInternalTuningSettings, 0);

		} else {
			Status = VL53L0X_ERROR_INVALID_PARAMS;
		}
	}

	LOG_FUNCTION_END(Status);
	return Status;
}



VL53L0X_Error VL53L0xBase::VL53L0X_GetTuningSettingBuffer(VL53L0X_DEV Dev,
    uint8_t **ppTuningSettingBuffer, uint8_t *pUseInternalTuningSettings)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *ppTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
    *pUseInternalTuningSettings = PALDevDataGet(Dev,
        UseInternalTuningSettings);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_StaticInit(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceParameters_t CurrentParameters = {0};
    uint8_t *pTuningSettingBuffer;
    uint16_t tempword = 0;
    uint8_t tempbyte = 0;
    uint8_t UseInternalTuningSettings = 0;
    uint32_t count = 0;
    uint8_t isApertureSpads = 0;
    uint32_t refSpadCount = 0;
    uint8_t ApertureSpads = 0;
    uint8_t vcselPulsePeriodPCLK;
    uint32_t seqTimeoutMicroSecs;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_info_from_device(Dev, 1);

    /* set the ref spad from NVM */
    count	= (uint32_t)VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
        ReferenceSpadCount);
    ApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
        ReferenceSpadType);

    /* NVM value invalid */
    if ((ApertureSpads > 1) ||
        ((ApertureSpads == 1) && (count > 32)) ||
        ((ApertureSpads == 0) && (count > 12)))
        Status = VL53L0X_perform_ref_spad_management(Dev, &refSpadCount,
            &isApertureSpads);
    else
        Status = VL53L0X_set_reference_spads(Dev, count, ApertureSpads);


    /* Initialize tuning settings buffer to prevent compiler warning. */
    pTuningSettingBuffer = DefaultTuningSettings;

    if (Status == VL53L0X_ERROR_NONE) {
        UseInternalTuningSettings = PALDevDataGet(Dev,
            UseInternalTuningSettings);

        if (UseInternalTuningSettings == 0)
            pTuningSettingBuffer = PALDevDataGet(Dev,
                pTuningSettingsPointer);
        else
            pTuningSettingBuffer = DefaultTuningSettings;

    }

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_load_tuning_settings(Dev,
                              pTuningSettingBuffer);


    /* Set interrupt config to new sample ready */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetGpioConfig(Dev, 0, 0,
        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
        VL53L0X_INTERRUPTPOLARITY_LOW);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0X_RdWord(Dev, 0x84, &tempword);
        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz,
            VL53L0X_FIXPOINT412TOFIXPOINT1616(tempword));
    }

    /* After static init, some device parameters may be changed,
     * so update them
     */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetDeviceParameters(Dev, &CurrentParameters);


    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetFractionEnable(Dev, &tempbyte);
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, RangeFractionalEnable, tempbyte);

    }

    if (Status == VL53L0X_ERROR_NONE)
        PALDevDataSet(Dev, CurrentParameters, CurrentParameters);


    /* read the sequence config and save it */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdByte(Dev,
        VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, tempbyte);

    }

    /* Disable MSRC and TCC by default */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetSequenceStepEnable(Dev,
                    VL53L0X_SEQUENCESTEP_TCC, 0);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetSequenceStepEnable(Dev,
        VL53L0X_SEQUENCESTEP_MSRC, 0);


    /* Set PAL State to standby */
    if (Status == VL53L0X_ERROR_NONE)
        PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);



    /* Store pre-range vcsel period */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetVcselPulsePeriod(
            Dev,
            VL53L0X_VCSEL_PERIOD_PRE_RANGE,
            &vcselPulsePeriodPCLK);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            Dev,
            PreRangeVcselPulsePeriod,
            vcselPulsePeriodPCLK);
    }

    /* Store final-range vcsel period */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetVcselPulsePeriod(
            Dev,
            VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
            &vcselPulsePeriodPCLK);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            Dev,
            FinalRangeVcselPulsePeriod,
            vcselPulsePeriodPCLK);
    }

    /* Store pre-range timeout */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = get_sequence_step_timeout(
            Dev,
            VL53L0X_SEQUENCESTEP_PRE_RANGE,
            &seqTimeoutMicroSecs);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            Dev,
            PreRangeTimeoutMicroSecs,
            seqTimeoutMicroSecs);
    }

    /* Store final-range timeout */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = get_sequence_step_timeout(
            Dev,
            VL53L0X_SEQUENCESTEP_FINAL_RANGE,
            &seqTimeoutMicroSecs);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            Dev,
            FinalRangeTimeoutMicroSecs,
            seqTimeoutMicroSecs);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_WaitDeviceBooted(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_ResetDevice(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    /* Set reset bit */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N,
        0x00);

    /* Wait for some time */
    if (Status == VL53L0X_ERROR_NONE) {
        do {
            Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Byte);
        } while (Byte != 0x00);
    }

    VL53L0X_PollingDelay(Dev);

    /* Release reset */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N,
        0x01);

    /* Wait until correct boot-up of the device */
    if (Status == VL53L0X_ERROR_NONE) {
        do {
            Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Byte);
        } while (Byte == 0x00);
    }

    VL53L0X_PollingDelay(Dev);

    /* Set PAL State to VL53L0X_STATE_POWERDOWN */
    if (Status == VL53L0X_ERROR_NONE)
        PALDevDataSet(Dev, PalState, VL53L0X_STATE_POWERDOWN);


    LOG_FUNCTION_END(Status);
    return Status;
}
/* End Group PAL Init Functions */

/* Group PAL Parameters Functions */
VL53L0X_Error VL53L0xBase::VL53L0X_SetDeviceParameters(VL53L0X_DEV Dev,
    const VL53L0X_DeviceParameters_t *pDeviceParameters)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int i;

    LOG_FUNCTION_START("");
    Status = VL53L0X_SetDeviceMode(Dev, pDeviceParameters->DeviceMode);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(Dev,
            pDeviceParameters->InterMeasurementPeriodMilliSeconds);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetXTalkCompensationRateMegaCps(Dev,
            pDeviceParameters->XTalkCompensationRateMegaCps);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev,
            pDeviceParameters->RangeOffsetMicroMeters);


    for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
        if (Status == VL53L0X_ERROR_NONE)
            Status |= VL53L0X_SetLimitCheckEnable(Dev, i,
                pDeviceParameters->LimitChecksEnable[i]);
        else
            break;

        if (Status == VL53L0X_ERROR_NONE)
            Status |= VL53L0X_SetLimitCheckValue(Dev, i,
                pDeviceParameters->LimitChecksValue[i]);
        else
            break;

    }

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetWrapAroundCheckEnable(Dev,
            pDeviceParameters->WrapAroundCheckEnable);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,
            pDeviceParameters->MeasurementTimingBudgetMicroSeconds);


    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetDeviceParameters(VL53L0X_DEV Dev,
    VL53L0X_DeviceParameters_t *pDeviceParameters)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int i;

    LOG_FUNCTION_START("");

    Status = VL53L0X_GetDeviceMode(Dev, &(pDeviceParameters->DeviceMode));

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetInterMeasurementPeriodMilliSeconds(Dev,
        &(pDeviceParameters->InterMeasurementPeriodMilliSeconds));


    if (Status == VL53L0X_ERROR_NONE)
        pDeviceParameters->XTalkCompensationEnable = 0;

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetXTalkCompensationRateMegaCps(Dev,
            &(pDeviceParameters->XTalkCompensationRateMegaCps));


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetOffsetCalibrationDataMicroMeter(Dev,
            &(pDeviceParameters->RangeOffsetMicroMeters));


    if (Status == VL53L0X_ERROR_NONE) {
        for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
            /* get first the values, then the enables.
             * VL53L0X_GetLimitCheckValue will modify the enable
             * flags
             */
            if (Status == VL53L0X_ERROR_NONE) {
                Status |= VL53L0X_GetLimitCheckValue(Dev, i,
                &(pDeviceParameters->LimitChecksValue[i]));
            } else {
                break;
            }
            if (Status == VL53L0X_ERROR_NONE) {
                Status |= VL53L0X_GetLimitCheckEnable(Dev, i,
                &(pDeviceParameters->LimitChecksEnable[i]));
            } else {
                break;
            }
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetWrapAroundCheckEnable(Dev,
            &(pDeviceParameters->WrapAroundCheckEnable));
    }

    /* Need to be done at the end as it uses VCSELPulsePeriod */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(Dev,
        &(pDeviceParameters->MeasurementTimingBudgetMicroSeconds));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        for (i = 0; i < VL53L0X_DMAX_LUT_SIZE; i++) {
            pDeviceParameters->dmax_lut.ambRate_mcps[i] =
               Dev->Data.CurrentParameters.dmax_lut.ambRate_mcps[i];
            pDeviceParameters->dmax_lut.dmax_mm[i] =
               Dev->Data.CurrentParameters.dmax_lut.dmax_mm[i];
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetDeviceMode(VL53L0X_DEV Dev,
                    VL53L0X_DeviceModes DeviceMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("%d", (int)DeviceMode);

    switch (DeviceMode) {
    case VL53L0X_DEVICEMODE_SINGLE_RANGING:
    case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
    case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
    case VL53L0X_DEVICEMODE_GPIO_DRIVE:
    case VL53L0X_DEVICEMODE_GPIO_OSC:
        /* Supported modes */
        VL53L0X_SETPARAMETERFIELD(Dev, DeviceMode, DeviceMode);
        break;
    default:
        /* Unsupported mode */
        Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetDeviceMode(VL53L0X_DEV Dev,
    VL53L0X_DeviceModes *pDeviceMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(Dev, DeviceMode, *pDeviceMode);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetRangeFractionEnable(VL53L0X_DEV Dev,	uint8_t Enable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("%d", (int)Enable);

    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_RANGE_CONFIG, Enable);

    if (Status == VL53L0X_ERROR_NONE)
        PALDevDataSet(Dev, RangeFractionalEnable, Enable);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetFractionEnable(VL53L0X_DEV Dev, uint8_t *pEnabled)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_RANGE_CONFIG, pEnabled);

    if (Status == VL53L0X_ERROR_NONE)
        *pEnabled = (*pEnabled & 1);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetHistogramMode(VL53L0X_DEV Dev,
    VL53L0X_HistogramModes HistogramMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetHistogramMode(VL53L0X_DEV Dev,
    VL53L0X_HistogramModes *pHistogramMode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
    uint32_t MeasurementTimingBudgetMicroSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_set_measurement_timing_budget_micro_seconds(Dev,
        MeasurementTimingBudgetMicroSeconds);

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
    uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_measurement_timing_budget_micro_seconds(Dev,
        pMeasurementTimingBudgetMicroSeconds);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetVcselPulsePeriod(VL53L0X_DEV Dev,
    VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_set_vcsel_pulse_period(Dev, VcselPeriodType,
        VCSELPulsePeriodPCLK);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetVcselPulsePeriod(VL53L0X_DEV Dev,
    VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_vcsel_pulse_period(Dev, VcselPeriodType,
        pVCSELPulsePeriodPCLK);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetSequenceStepEnable(VL53L0X_DEV Dev,
    VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;
    uint8_t SequenceConfigNew = 0;
    uint32_t MeasurementTimingBudgetMicroSeconds;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
        &SequenceConfig);

    SequenceConfigNew = SequenceConfig;

    if (Status == VL53L0X_ERROR_NONE) {
        if (SequenceStepEnabled == 1) {

            /* Enable requested sequence step
             */
            switch (SequenceStepId) {
            case VL53L0X_SEQUENCESTEP_TCC:
                SequenceConfigNew |= 0x10;
                break;
            case VL53L0X_SEQUENCESTEP_DSS:
                SequenceConfigNew |= 0x28;
                break;
            case VL53L0X_SEQUENCESTEP_MSRC:
                SequenceConfigNew |= 0x04;
                break;
            case VL53L0X_SEQUENCESTEP_PRE_RANGE:
                SequenceConfigNew |= 0x40;
                break;
            case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
                SequenceConfigNew |= 0x80;
                break;
            default:
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        } else {
            /* Disable requested sequence step
             */
            switch (SequenceStepId) {
            case VL53L0X_SEQUENCESTEP_TCC:
                SequenceConfigNew &= 0xef;
                break;
            case VL53L0X_SEQUENCESTEP_DSS:
                SequenceConfigNew &= 0xd7;
                break;
            case VL53L0X_SEQUENCESTEP_MSRC:
                SequenceConfigNew &= 0xfb;
                break;
            case VL53L0X_SEQUENCESTEP_PRE_RANGE:
                SequenceConfigNew &= 0xbf;
                break;
            case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
                SequenceConfigNew &= 0x7f;
                break;
            default:
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }
    }

    if (SequenceConfigNew != SequenceConfig) {
        /* Apply New Setting */
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
        }
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, SequenceConfigNew);


        /* Recalculate timing budget */
        if (Status == VL53L0X_ERROR_NONE) {
            VL53L0X_GETPARAMETERFIELD(Dev,
                MeasurementTimingBudgetMicroSeconds,
                MeasurementTimingBudgetMicroSeconds);

            VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,
                MeasurementTimingBudgetMicroSeconds);
        }
    }

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::sequence_step_enabled(VL53L0X_DEV Dev,
    VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceConfig,
    uint8_t *pSequenceStepEnabled)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    *pSequenceStepEnabled = 0;
    LOG_FUNCTION_START("");

    switch (SequenceStepId) {
    case VL53L0X_SEQUENCESTEP_TCC:
        *pSequenceStepEnabled = (SequenceConfig & 0x10) >> 4;
        break;
    case VL53L0X_SEQUENCESTEP_DSS:
        *pSequenceStepEnabled = (SequenceConfig & 0x08) >> 3;
        break;
    case VL53L0X_SEQUENCESTEP_MSRC:
        *pSequenceStepEnabled = (SequenceConfig & 0x04) >> 2;
        break;
    case VL53L0X_SEQUENCESTEP_PRE_RANGE:
        *pSequenceStepEnabled = (SequenceConfig & 0x40) >> 6;
        break;
    case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
        *pSequenceStepEnabled = (SequenceConfig & 0x80) >> 7;
        break;
    default:
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSequenceStepEnable(VL53L0X_DEV Dev,
    VL53L0X_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
        &SequenceConfig);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev, SequenceStepId,
            SequenceConfig, pSequenceStepEnabled);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSequenceStepEnables(VL53L0X_DEV Dev,
    VL53L0X_SchedulerSequenceSteps_t *pSchedulerSequenceSteps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
        &SequenceConfig);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
        VL53L0X_SEQUENCESTEP_TCC, SequenceConfig,
            &pSchedulerSequenceSteps->TccOn);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
        VL53L0X_SEQUENCESTEP_DSS, SequenceConfig,
            &pSchedulerSequenceSteps->DssOn);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
        VL53L0X_SEQUENCESTEP_MSRC, SequenceConfig,
            &pSchedulerSequenceSteps->MsrcOn);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
        VL53L0X_SEQUENCESTEP_PRE_RANGE, SequenceConfig,
            &pSchedulerSequenceSteps->PreRangeOn);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
        VL53L0X_SEQUENCESTEP_FINAL_RANGE, SequenceConfig,
            &pSchedulerSequenceSteps->FinalRangeOn);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetNumberOfSequenceSteps(VL53L0X_DEV Dev,
    uint8_t *pNumberOfSequenceSteps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pNumberOfSequenceSteps = VL53L0X_SEQUENCESTEP_NUMBER_OF_CHECKS;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSequenceStepsInfo(
    VL53L0X_SequenceStepId SequenceStepId,
    char *pSequenceStepsString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_sequence_steps_info(
            SequenceStepId,
            pSequenceStepsString);

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetSequenceStepTimeout(VL53L0X_DEV Dev,
    VL53L0X_SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Error Status1 = VL53L0X_ERROR_NONE;
    uint32_t TimeoutMicroSeconds = ((TimeOutMilliSecs * 1000) + 0x8000)
        >> 16;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    FixPoint1616_t OldTimeOutMicroSeconds;

    LOG_FUNCTION_START("");

    /* Read back the current value in case we need to revert back to this.
     */
    Status = get_sequence_step_timeout(Dev, SequenceStepId,
        &OldTimeOutMicroSeconds);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = set_sequence_step_timeout(Dev, SequenceStepId,
            TimeoutMicroSeconds);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_GETPARAMETERFIELD(Dev,
            MeasurementTimingBudgetMicroSeconds,
            MeasurementTimingBudgetMicroSeconds);

        /* At this point we don't know if the requested value is valid,
         * therefore proceed to update the entire timing budget and
         * if this fails, revert back to the previous value.
         */
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,
            MeasurementTimingBudgetMicroSeconds);

        if (Status != VL53L0X_ERROR_NONE) {
            Status1 = set_sequence_step_timeout(Dev, SequenceStepId,
                OldTimeOutMicroSeconds);

            if (Status1 == VL53L0X_ERROR_NONE) {
                Status1 =
                VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
                    Dev,
                    MeasurementTimingBudgetMicroSeconds);
            }

            Status = Status1;
        }
    }

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSequenceStepTimeout(VL53L0X_DEV Dev,
    VL53L0X_SequenceStepId SequenceStepId,
    FixPoint1616_t *pTimeOutMilliSecs)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t TimeoutMicroSeconds;

    LOG_FUNCTION_START("");

    Status = get_sequence_step_timeout(Dev, SequenceStepId,
        &TimeoutMicroSeconds);
    if (Status == VL53L0X_ERROR_NONE) {
        TimeoutMicroSeconds <<= 8;
        *pTimeOutMilliSecs = (TimeoutMicroSeconds + 500)/1000;
        *pTimeOutMilliSecs <<= 8;
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
    uint32_t InterMeasurementPeriodMilliSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdWord(Dev, VL53L0X_REG_OSC_CALIBRATE_VAL,
        &osc_calibrate_val);

    if (Status == VL53L0X_ERROR_NONE) {
        if (osc_calibrate_val != 0) {
            IMPeriodMilliSeconds =
                InterMeasurementPeriodMilliSeconds
                    * osc_calibrate_val;
        } else {
            IMPeriodMilliSeconds =
                InterMeasurementPeriodMilliSeconds;
        }
        Status = VL53L0X_WrDWord(Dev,
        VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
            IMPeriodMilliSeconds);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(Dev,
            InterMeasurementPeriodMilliSeconds,
            InterMeasurementPeriodMilliSeconds);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
    uint32_t *pInterMeasurementPeriodMilliSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdWord(Dev, VL53L0X_REG_OSC_CALIBRATE_VAL,
        &osc_calibrate_val);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdDWord(Dev,
        VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
            &IMPeriodMilliSeconds);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        if (osc_calibrate_val != 0) {
            *pInterMeasurementPeriodMilliSeconds =
                IMPeriodMilliSeconds / osc_calibrate_val;
        }
        VL53L0X_SETPARAMETERFIELD(Dev,
            InterMeasurementPeriodMilliSeconds,
            *pInterMeasurementPeriodMilliSeconds);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetXTalkCompensationEnable(VL53L0X_DEV Dev,
    uint8_t XTalkCompensationEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t TempFix1616;
    uint16_t LinearityCorrectiveGain;

    LOG_FUNCTION_START("");

    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    if ((XTalkCompensationEnable == 0)
        || (LinearityCorrectiveGain != 1000)) {
        TempFix1616 = 0;
    } else {
        VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            TempFix1616);
    }

    /* the following register has a format 3.13 */
    Status = VL53L0X_WrWord(Dev,
    VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
        VL53L0X_FIXPOINT1616TOFIXPOINT313(TempFix1616));

    if (Status == VL53L0X_ERROR_NONE) {
        if (XTalkCompensationEnable == 0) {
            VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                0);
        } else {
            VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                1);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
    uint8_t *pXTalkCompensationEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Temp8;

    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
    *pXTalkCompensationEnable = Temp8;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
    FixPoint1616_t XTalkCompensationRateMegaCps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Temp8;
    uint16_t LinearityCorrectiveGain;
    uint16_t data;

    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    if (Temp8 == 0) { /* disabled write only internal value */
        VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            XTalkCompensationRateMegaCps);
    } else {
        /* the following register has a format 3.13 */
        if (LinearityCorrectiveGain == 1000) {
            data = VL53L0X_FIXPOINT1616TOFIXPOINT313(
                XTalkCompensationRateMegaCps);
        } else {
            data = 0;
        }

        Status = VL53L0X_WrWord(Dev,
        VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, data);

        if (Status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETPARAMETERFIELD(Dev,
                XTalkCompensationRateMegaCps,
                XTalkCompensationRateMegaCps);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
    FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t Value;
    FixPoint1616_t TempFix1616;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdWord(Dev,
    VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, (uint16_t *)&Value);
    if (Status == VL53L0X_ERROR_NONE) {
        if (Value == 0) {
            /* the Xtalk is disabled return value from memory */
            VL53L0X_GETPARAMETERFIELD(Dev,
                XTalkCompensationRateMegaCps, TempFix1616);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                0);
        } else {
            TempFix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(Value);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0X_SETPARAMETERFIELD(Dev,
                XTalkCompensationRateMegaCps, TempFix1616);
            VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                1);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetRefCalibration(VL53L0X_DEV Dev, uint8_t VhvSettings,
    uint8_t PhaseCal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_set_ref_calibration(Dev, VhvSettings, PhaseCal);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetRefCalibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings,
    uint8_t *pPhaseCal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_ref_calibration(Dev, pVhvSettings, pPhaseCal);

    LOG_FUNCTION_END(Status);
    return Status;
}

/*
 * CHECK LIMIT FUNCTIONS
 */

VL53L0X_Error VL53L0xBase::VL53L0X_GetNumberOfLimitCheck(uint16_t *pNumberOfLimitCheck)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pNumberOfLimitCheck = VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLimitCheckInfo(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    char *pLimitCheckString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_limit_check_info(Dev, LimitCheckId,
        pLimitCheckString);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLimitCheckStatus(VL53L0X_DEV Dev,
    uint16_t LimitCheckId,
    uint8_t *pLimitCheckStatus)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Temp8;

    LOG_FUNCTION_START("");

    if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    } else {

        VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
            LimitCheckId, Temp8);

        *pLimitCheckStatus = Temp8;

    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetLimitCheckEnable(VL53L0X_DEV Dev,
    uint16_t LimitCheckId,
    uint8_t LimitCheckEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t TempFix1616 = 0;
    uint8_t LimitCheckEnableInt = 0;
    uint8_t LimitCheckDisable = 0;
    uint8_t Temp8;

    LOG_FUNCTION_START("");

    if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    } else {
        if (LimitCheckEnable == 0) {
            TempFix1616 = 0;
            LimitCheckEnableInt = 0;
            LimitCheckDisable = 1;

        } else {
            VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                LimitCheckId, TempFix1616);
            LimitCheckDisable = 0;
            /* this to be sure to have either 0 or 1 */
            LimitCheckEnableInt = 1;
        }

        switch (LimitCheckId) {

        case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                LimitCheckEnableInt);

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

            Status = VL53L0X_WrWord(Dev,
            VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                VL53L0X_FIXPOINT1616TOFIXPOINT97(TempFix1616));

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                LimitCheckEnableInt);

            break;

        case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                LimitCheckEnableInt);

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:

            Temp8 = (uint8_t)(LimitCheckDisable << 1);
            Status = VL53L0X_UpdateByte(Dev,
                VL53L0X_REG_MSRC_CONFIG_CONTROL,
                0xFE, Temp8);

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:

            Temp8 = (uint8_t)(LimitCheckDisable << 4);
            Status = VL53L0X_UpdateByte(Dev,
                VL53L0X_REG_MSRC_CONFIG_CONTROL,
                0xEF, Temp8);

            break;


        default:
            Status = VL53L0X_ERROR_INVALID_PARAMS;

        }

    }

    if (Status == VL53L0X_ERROR_NONE) {
        if (LimitCheckEnable == 0) {
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                LimitCheckId, 0);
        } else {
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                LimitCheckId, 1);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev,
    uint16_t LimitCheckId,
    uint8_t *pLimitCheckEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Temp8;

    LOG_FUNCTION_START("");

    if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0X_ERROR_INVALID_PARAMS;
        *pLimitCheckEnable = 0;
    } else {
        VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
            LimitCheckId, Temp8);
        *pLimitCheckEnable = Temp8;
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    FixPoint1616_t LimitCheckValue)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Temp8;

    LOG_FUNCTION_START("");

    VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId,
        Temp8);

    if (Temp8 == 0) { /* disabled write only internal value */
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            LimitCheckId, LimitCheckValue);
    } else {

        switch (LimitCheckId) {

        case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                LimitCheckValue);
            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

            Status = VL53L0X_WrWord(Dev,
            VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                VL53L0X_FIXPOINT1616TOFIXPOINT97(
                    LimitCheckValue));

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                LimitCheckValue);

            break;

        case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

            /* internal computation: */
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                LimitCheckValue);

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
        case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:

            Status = VL53L0X_WrWord(Dev,
                VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
                VL53L0X_FIXPOINT1616TOFIXPOINT97(
                    LimitCheckValue));

            break;

        default:
            Status = VL53L0X_ERROR_INVALID_PARAMS;

        }

        if (Status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                LimitCheckId, LimitCheckValue);
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    FixPoint1616_t *pLimitCheckValue)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t EnableZeroValue = 0;
    uint16_t Temp16;
    FixPoint1616_t TempFix1616;

    LOG_FUNCTION_START("");

    switch (LimitCheckId) {

    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
        /* internal computation: */
        VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
        EnableZeroValue = 0;
        break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        Status = VL53L0X_RdWord(Dev,
        VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
            &Temp16);
        if (Status == VL53L0X_ERROR_NONE)
            TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);


        EnableZeroValue = 1;
        break;

    case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
        /* internal computation: */
        VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
        EnableZeroValue = 0;
        break;

    case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        /* internal computation: */
        VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
            TempFix1616);
        EnableZeroValue = 0;
        break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
    case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        Status = VL53L0X_RdWord(Dev,
            VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
            &Temp16);
        if (Status == VL53L0X_ERROR_NONE)
            TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);


        EnableZeroValue = 0;
        break;

    default:
        Status = VL53L0X_ERROR_INVALID_PARAMS;

    }

    if (Status == VL53L0X_ERROR_NONE) {

        if (EnableZeroValue == 1) {

            if (TempFix1616 == 0) {
                /* disabled: return value from memory */
                VL53L0X_GETARRAYPARAMETERFIELD(Dev,
                    LimitChecksValue, LimitCheckId,
                    TempFix1616);
                *pLimitCheckValue = TempFix1616;
                VL53L0X_SETARRAYPARAMETERFIELD(Dev,
                    LimitChecksEnable, LimitCheckId, 0);
            } else {
                *pLimitCheckValue = TempFix1616;
                VL53L0X_SETARRAYPARAMETERFIELD(Dev,
                    LimitChecksValue, LimitCheckId,
                    TempFix1616);
                VL53L0X_SETARRAYPARAMETERFIELD(Dev,
                    LimitChecksEnable, LimitCheckId, 1);
            }
        } else {
            *pLimitCheckValue = TempFix1616;
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;

}

VL53L0X_Error VL53L0xBase::VL53L0X_GetLimitCheckCurrent(VL53L0X_DEV Dev,
    uint16_t LimitCheckId,
    FixPoint1616_t *pLimitCheckCurrent)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

    LOG_FUNCTION_START("");

    if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    } else {
        switch (LimitCheckId) {
        case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
            /* Need to run a ranging to have the latest values */
            *pLimitCheckCurrent = PALDevDataGet(Dev, SigmaEstimate);

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
            /* Need to run a ranging to have the latest values */
            LastRangeDataBuffer = PALDevDataGet(Dev,
                LastRangeMeasure);
            *pLimitCheckCurrent =
                LastRangeDataBuffer.SignalRateRtnMegaCps;

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
            /* Need to run a ranging to have the latest values */
            *pLimitCheckCurrent = PALDevDataGet(Dev,
                LastSignalRefMcps);

            break;

        case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
            /* Need to run a ranging to have the latest values */
            LastRangeDataBuffer = PALDevDataGet(Dev,
                LastRangeMeasure);
            *pLimitCheckCurrent =
                LastRangeDataBuffer.SignalRateRtnMegaCps;

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
            /* Need to run a ranging to have the latest values */
            LastRangeDataBuffer = PALDevDataGet(Dev,
                LastRangeMeasure);
            *pLimitCheckCurrent =
                LastRangeDataBuffer.SignalRateRtnMegaCps;

            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
            /* Need to run a ranging to have the latest values */
            LastRangeDataBuffer = PALDevDataGet(Dev,
                LastRangeMeasure);
            *pLimitCheckCurrent =
                LastRangeDataBuffer.SignalRateRtnMegaCps;

            break;

        default:
            Status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;

}

/*
 * WRAPAROUND Check
 */
VL53L0X_Error VL53L0xBase::VL53L0X_SetWrapAroundCheckEnable(VL53L0X_DEV Dev,
    uint8_t WrapAroundCheckEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;
    uint8_t WrapAroundCheckEnableInt;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &Byte);
    if (WrapAroundCheckEnable == 0) {
        /* Disable wraparound */
        Byte = Byte & 0x7F;
        WrapAroundCheckEnableInt = 0;
    } else {
        /*Enable wraparound */
        Byte = Byte | 0x80;
        WrapAroundCheckEnableInt = 1;
    }

    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, Byte);

    if (Status == VL53L0X_ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, Byte);
        VL53L0X_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable,
            WrapAroundCheckEnableInt);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetWrapAroundCheckEnable(VL53L0X_DEV Dev,
    uint8_t *pWrapAroundCheckEnable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &data);
    if (Status == VL53L0X_ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, data);
        if (data & (0x01 << 7))
            *pWrapAroundCheckEnable = 0x01;
        else
            *pWrapAroundCheckEnable = 0x00;
    }
    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable,
            *pWrapAroundCheckEnable);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

/* End Group PAL Parameters Functions */

/* Group PAL Measurement Functions */
VL53L0X_Error VL53L0xBase::VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceModes DeviceMode;

    LOG_FUNCTION_START("");

    /* Get Current DeviceMode */
    Status = VL53L0X_GetDeviceMode(Dev, &DeviceMode);

    /* Start immediately to run a single ranging measurement in case of
     * single ranging or single histogram
     */
    if (Status == VL53L0X_ERROR_NONE
        && DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
        Status = VL53L0X_StartMeasurement(Dev);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_measurement_poll_for_completion(Dev);


    /* Change PAL State in case of single ranging or single histogram */
    if (Status == VL53L0X_ERROR_NONE
        && DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
        PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);


    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformSingleHistogramMeasurement(VL53L0X_DEV Dev,
    VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformRefCalibration(VL53L0X_DEV Dev,
    uint8_t *pVhvSettings,
    uint8_t *pPhaseCal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_perform_ref_calibration(Dev, pVhvSettings,
        pPhaseCal, 1);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformXTalkMeasurement(VL53L0X_DEV Dev,
    uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad,
    uint8_t *pAmbientTooHigh)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformXTalkCalibration(VL53L0X_DEV Dev,
    FixPoint1616_t XTalkCalDistance,
    FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_perform_xtalk_calibration(Dev, XTalkCalDistance,
        pXTalkCompensationRateMegaCps);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformOffsetCalibration(VL53L0X_DEV Dev,
    FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_perform_offset_calibration(Dev, CalDistanceMilliMeter,
        pOffsetMicroMeter);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_CheckAndLoadInterruptSettings(VL53L0X_DEV Dev,
    uint8_t StartNotStopFlag)
{
    uint8_t InterruptConfig;
    FixPoint1616_t ThresholdLow;
    FixPoint1616_t ThresholdHigh;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
        Pin0GpioFunctionality);

    switch (InterruptConfig) {
    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
        Status = VL53L0X_GetInterruptThresholds(Dev,
                    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    &ThresholdLow, &ThresholdHigh);

        if ((ThresholdLow > 255*65536) &&
            (Status == VL53L0X_ERROR_NONE)) {

            if (StartNotStopFlag != 0) {
                Status = VL53L0X_load_tuning_settings(Dev,
                        InterruptThresholdSettings);
            } else {
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x04);
                Status |= VL53L0X_WrByte(Dev, 0x70, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);
            }
        }
        break;
    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
        Status = VL53L0X_GetInterruptThresholds(Dev,
                    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    &ThresholdLow, &ThresholdHigh);

        if ((ThresholdHigh > 0) &&
            (Status == VL53L0X_ERROR_NONE)) {

            if (StartNotStopFlag != 0) {
                Status = VL53L0X_load_tuning_settings(Dev,
                        InterruptThresholdSettings);
            } else {
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x04);
                Status |= VL53L0X_WrByte(Dev, 0x70, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);
            }
        }
        break;
    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
        Status = VL53L0X_GetInterruptThresholds(Dev,
                    VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    &ThresholdLow, &ThresholdHigh);

        if (Status == VL53L0X_ERROR_NONE) {
            if (StartNotStopFlag != 0) {
                Status = VL53L0X_load_tuning_settings(Dev,
                        InterruptThresholdSettings);
            } else {
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x04);
                Status |= VL53L0X_WrByte(Dev, 0x70, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
                Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);
            }
        }
        break;
    }

    LOG_FUNCTION_END(Status);
    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_StartMeasurement(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceModes DeviceMode;
    uint8_t Byte;
    uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
    uint32_t LoopNb;

    LOG_FUNCTION_START("");

    /* Get Current DeviceMode */
    VL53L0X_GetDeviceMode(Dev, &DeviceMode);

    Status = VL53L0X_WrByte(Dev, 0x80, 0x01);
    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
    Status = VL53L0X_WrByte(Dev, 0x91, PALDevDataGet(Dev, StopVariable));
    Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
    Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);
    Status = VL53L0X_WrByte(Dev, 0x80, 0x00);

    switch (DeviceMode) {
    case VL53L0X_DEVICEMODE_SINGLE_RANGING:
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START, 0x01);

        Byte = StartStopByte;
        if (Status == VL53L0X_ERROR_NONE) {
            /* Wait until start bit has been cleared */
            LoopNb = 0;
            do {
                if (LoopNb > 0)
                    Status = VL53L0X_RdByte(Dev,
                    VL53L0X_REG_SYSRANGE_START, &Byte);
                LoopNb = LoopNb + 1;
            } while (((Byte & StartStopByte) == StartStopByte)
                && (Status == VL53L0X_ERROR_NONE)
                && (LoopNb < VL53L0X_DEFAULT_MAX_LOOP));

            if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
                Status = VL53L0X_ERROR_TIME_OUT;

        }

        break;
    case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
        /* Back-to-back mode */

        /* Check if need to apply interrupt settings */
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);

        Status = VL53L0X_WrByte(Dev,
        VL53L0X_REG_SYSRANGE_START,
        VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
        if (Status == VL53L0X_ERROR_NONE) {
            /* Set PAL State to Running */
            PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
        }
        break;
    case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
        /* Continuous mode */
        /* Check if need to apply interrupt settings */
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);

        Status = VL53L0X_WrByte(Dev,
        VL53L0X_REG_SYSRANGE_START,
        VL53L0X_REG_SYSRANGE_MODE_TIMED);

        if (Status == VL53L0X_ERROR_NONE) {
            /* Set PAL State to Running */
            PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
        }
        break;
    default:
        /* Selected mode not supported */
        Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
    }


    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_StopMeasurement(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START,
    VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
    Status = VL53L0X_WrByte(Dev, 0x91, 0x00);
    Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
    Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0X_ERROR_NONE) {
        /* Set PAL State to Idle */
        PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);
    }

    /* Check if need to apply interrupt settings */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 0);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetMeasurementDataReady(VL53L0X_DEV Dev,
    uint8_t *pMeasurementDataReady)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SysRangeStatusRegister;
    uint8_t InterruptConfig;
    uint32_t InterruptMask;

    LOG_FUNCTION_START("");

    InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
        Pin0GpioFunctionality);

    if (InterruptConfig ==
        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
        Status = VL53L0X_GetInterruptMaskStatus(Dev, &InterruptMask);
        if (InterruptMask ==
        VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY)
            *pMeasurementDataReady = 1;
        else
            *pMeasurementDataReady = 0;
    } else {
        Status = VL53L0X_RdByte(Dev, VL53L0X_REG_RESULT_RANGE_STATUS,
            &SysRangeStatusRegister);
        if (Status == VL53L0X_ERROR_NONE) {
            if (SysRangeStatusRegister & 0x01)
                *pMeasurementDataReady = 1;
            else
                *pMeasurementDataReady = 0;
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_WaitDeviceReadyForNewMeasurement(VL53L0X_DEV Dev,
    uint32_t MaxLoop)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented for VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_GetRangingMeasurementData(VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData, bool lightProcessing)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t DeviceRangeStatus;
    uint8_t RangeFractionalEnable;
    uint8_t PalRangeStatus;
    uint8_t XTalkCompensationEnable;
    uint16_t AmbientRate;
    FixPoint1616_t SignalRate;
    uint16_t XTalkCompensationRateMegaCps;
    uint16_t EffectiveSpadRtnCount;
    uint16_t tmpuint16;
    uint16_t XtalkRangeMilliMeter;
    uint16_t LinearityCorrectiveGain;
    uint8_t localBuffer[12];
    VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;

    LOG_FUNCTION_START("");

    /*
     * use multi read even if some registers are not useful, result will
     * be more efficient
     * start reading at 0x14 dec20
     * end reading at 0x21 dec33 total 14 bytes to read
     */
    Status = VL53L0X_ReadMulti(Dev, 0x14, localBuffer, 12);

    if (Status == VL53L0X_ERROR_NONE) {

        pRangingMeasurementData->ZoneId = 0; /* Only one zone */
        pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */

        tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11],
                           localBuffer[10]);
        /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
         *(format 11.2) else no fractional
         */

        pRangingMeasurementData->MeasurementTimeUsec = 0;


        SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(
            VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6]));
        /* peak_signal_count_rate_rtn_mcps */
        pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

        AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9],
                         localBuffer[8]);
        pRangingMeasurementData->AmbientRateRtnMegaCps =
            VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);

        EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3],
            localBuffer[2]);
        /* EffectiveSpadRtnCount is 8.8 format */
        pRangingMeasurementData->EffectiveSpadRtnCount =
            EffectiveSpadRtnCount;

        DeviceRangeStatus = localBuffer[0];

        /* Get Linearity Corrective Gain */
        LinearityCorrectiveGain = PALDevDataGet(Dev,
            LinearityCorrectiveGain);

        /* Get ranging configuration */
        RangeFractionalEnable = PALDevDataGet(Dev,
            RangeFractionalEnable);

        if (LinearityCorrectiveGain != 1000) {

            tmpuint16 = (uint16_t)((LinearityCorrectiveGain
                * tmpuint16 + 500) / 1000);

            /* Implement Xtalk */
            VL53L0X_GETPARAMETERFIELD(Dev,
                XTalkCompensationRateMegaCps,
                XTalkCompensationRateMegaCps);
            VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable,
                XTalkCompensationEnable);

            if (XTalkCompensationEnable) {

                if ((SignalRate
                    - ((XTalkCompensationRateMegaCps
                    * EffectiveSpadRtnCount) >> 8))
                    <= 0) {
                    if (RangeFractionalEnable)
                        XtalkRangeMilliMeter = 8888;
                    else
                        XtalkRangeMilliMeter = 8888
                            << 2;
                } else {
                    XtalkRangeMilliMeter =
                    (tmpuint16 * SignalRate)
                        / (SignalRate
                        - ((XTalkCompensationRateMegaCps
                        * EffectiveSpadRtnCount)
                        >> 8));
                }

                tmpuint16 = XtalkRangeMilliMeter;
            }

        }

        if (RangeFractionalEnable) {
            pRangingMeasurementData->RangeMilliMeter =
                (uint16_t)((tmpuint16) >> 2);
            pRangingMeasurementData->RangeFractionalPart =
                (uint8_t)((tmpuint16 & 0x03) << 6);
        } else {
            pRangingMeasurementData->RangeMilliMeter = tmpuint16;
            pRangingMeasurementData->RangeFractionalPart = 0;
        }

        if(!lightProcessing) {
			/*
			 * For a standard definition of RangeStatus, this should
			 * return 0 in case of good result after a ranging
			 * The range status depends on the device so call a device
			 * specific function to obtain the right Status.
			 */
			Status |= VL53L0X_get_pal_range_status(Dev, DeviceRangeStatus,
				SignalRate, EffectiveSpadRtnCount,
				pRangingMeasurementData, &PalRangeStatus);

			if (Status == VL53L0X_ERROR_NONE)
				pRangingMeasurementData->RangeStatus = PalRangeStatus;

		}

		if (Status == VL53L0X_ERROR_NONE) {
			/* Copy last read data into Dev buffer */
			LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

			LastRangeDataBuffer.RangeMilliMeter =
				pRangingMeasurementData->RangeMilliMeter;
			LastRangeDataBuffer.RangeFractionalPart =
				pRangingMeasurementData->RangeFractionalPart;
			LastRangeDataBuffer.RangeDMaxMilliMeter =
				pRangingMeasurementData->RangeDMaxMilliMeter;
			LastRangeDataBuffer.MeasurementTimeUsec =
				pRangingMeasurementData->MeasurementTimeUsec;
			LastRangeDataBuffer.SignalRateRtnMegaCps =
				pRangingMeasurementData->SignalRateRtnMegaCps;
			LastRangeDataBuffer.AmbientRateRtnMegaCps =
				pRangingMeasurementData->AmbientRateRtnMegaCps;
			LastRangeDataBuffer.EffectiveSpadRtnCount =
				pRangingMeasurementData->EffectiveSpadRtnCount;
			LastRangeDataBuffer.RangeStatus =
				pRangingMeasurementData->RangeStatus;

			PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
    	} // if (full _processing)
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetMeasurementRefSignal(VL53L0X_DEV Dev,
    FixPoint1616_t *pMeasurementRefSignal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SignalRefClipLimitCheckEnable = 0;

    LOG_FUNCTION_START("");

    Status = VL53L0X_GetLimitCheckEnable(Dev,
            VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
            &SignalRefClipLimitCheckEnable);
    if (SignalRefClipLimitCheckEnable != 0)
        *pMeasurementRefSignal = PALDevDataGet(Dev, LastSignalRefMcps);
    else
        Status = VL53L0X_ERROR_INVALID_COMMAND;

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetHistogramMeasurementData(VL53L0X_DEV Dev,
    VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformSingleRangingMeasurement(VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    /* This function will do a complete single ranging
     * Here we fix the mode!
     */
    Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_PerformSingleMeasurement(Dev);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetRangingMeasurementData(Dev,
            pRangingMeasurementData);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_ClearInterruptMask(Dev, 0);


    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetNumberOfROIZones(VL53L0X_DEV Dev,
    uint8_t NumberOfROIZones)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    if (NumberOfROIZones != 1)
        Status = VL53L0X_ERROR_INVALID_PARAMS;


    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetNumberOfROIZones(VL53L0X_DEV Dev,
    uint8_t *pNumberOfROIZones)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pNumberOfROIZones = 1;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetMaxNumberOfROIZones(VL53L0X_DEV Dev,
    uint8_t *pMaxNumberOfROIZones)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    *pMaxNumberOfROIZones = 1;

    LOG_FUNCTION_END(Status);
    return Status;
}

/* End Group PAL Measurement Functions */

VL53L0X_Error VL53L0xBase::VL53L0X_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
    VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
    VL53L0X_InterruptPolarity Polarity)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    LOG_FUNCTION_START("");

    if (Pin != 0) {
        Status = VL53L0X_ERROR_GPIO_NOT_EXISTING;
    } else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_DRIVE) {
        if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW)
            data = 0x10;
        else
            data = 1;

        Status = VL53L0X_WrByte(Dev,
        VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

    } else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_OSC) {

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0x85, 0x02);

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x04);
        Status |= VL53L0X_WrByte(Dev, 0xcd, 0x00);
        Status |= VL53L0X_WrByte(Dev, 0xcc, 0x11);

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x07);
        Status |= VL53L0X_WrByte(Dev, 0xbe, 0x00);

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x06);
        Status |= VL53L0X_WrByte(Dev, 0xcc, 0x09);

        Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);

    } else {

        if (Status == VL53L0X_ERROR_NONE) {
            switch (Functionality) {
            case VL53L0X_GPIOFUNCTIONALITY_OFF:
                data = 0x00;
                break;
            case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
                data = 0x01;
                break;
            case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
                data = 0x02;
                break;
            case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
                data = 0x03;
                break;
            case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
                data = 0x04;
                break;
            default:
                Status =
                VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
            }
        }

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);

        if (Status == VL53L0X_ERROR_NONE) {
            if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW)
                data = 0;
            else
                data = (uint8_t)(1 << 4);

            Status = VL53L0X_UpdateByte(Dev,
            VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
        }

        if (Status == VL53L0X_ERROR_NONE)
            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                Pin0GpioFunctionality, Functionality);

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_ClearInterruptMask(Dev, 0);

    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
    VL53L0X_DeviceModes *pDeviceMode,
    VL53L0X_GpioFunctionality *pFunctionality,
    VL53L0X_InterruptPolarity *pPolarity)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_GpioFunctionality GpioFunctionality;
    uint8_t data;

    LOG_FUNCTION_START("");

    /* pDeviceMode not managed by Ewok it return the current mode */

    Status = VL53L0X_GetDeviceMode(Dev, pDeviceMode);

    if (Status == VL53L0X_ERROR_NONE) {
        if (Pin != 0) {
            Status = VL53L0X_ERROR_GPIO_NOT_EXISTING;
        } else {
            Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, &data);
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        switch (data & 0x07) {
        case 0x00:
            GpioFunctionality = VL53L0X_GPIOFUNCTIONALITY_OFF;
            break;
        case 0x01:
            GpioFunctionality =
            VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
            break;
        case 0x02:
            GpioFunctionality =
            VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
            break;
        case 0x03:
            GpioFunctionality =
            VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
            break;
        case 0x04:
            GpioFunctionality =
            VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
            break;
        default:
            Status = VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
        }
    }

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
            &data);

    if (Status == VL53L0X_ERROR_NONE) {
        if ((data & (uint8_t)(1 << 4)) == 0)
            *pPolarity = VL53L0X_INTERRUPTPOLARITY_LOW;
        else
            *pPolarity = VL53L0X_INTERRUPTPOLARITY_HIGH;
    }

    if (Status == VL53L0X_ERROR_NONE) {
        *pFunctionality = GpioFunctionality;
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality,
            GpioFunctionality);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetInterruptThresholds(VL53L0X_DEV Dev,
    VL53L0X_DeviceModes DeviceMode, FixPoint1616_t ThresholdLow,
    FixPoint1616_t ThresholdHigh)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t Threshold16;

    LOG_FUNCTION_START("");

    /* no dependency on DeviceMode for Ewok */
    /* Need to divide by 2 because the FW will apply a x2 */
    Threshold16 = (uint16_t)((ThresholdLow >> 17) & 0x00fff);
    Status = VL53L0X_WrWord(Dev, VL53L0X_REG_SYSTEM_THRESH_LOW,
                Threshold16);

    if (Status == VL53L0X_ERROR_NONE) {
        /* Need to divide by 2 because the FW will apply a x2 */
        Threshold16 = (uint16_t)((ThresholdHigh >> 17) & 0x00fff);
        Status = VL53L0X_WrWord(Dev, VL53L0X_REG_SYSTEM_THRESH_HIGH,
            Threshold16);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetInterruptThresholds(VL53L0X_DEV Dev,
    VL53L0X_DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow,
    FixPoint1616_t *pThresholdHigh)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t Threshold16;

    LOG_FUNCTION_START("");

    /* no dependency on DeviceMode for Ewok */

    Status = VL53L0X_RdWord(Dev, VL53L0X_REG_SYSTEM_THRESH_LOW,
                &Threshold16);
    /* Need to multiply by 2 because the FW will apply a x2 */
    *pThresholdLow = (FixPoint1616_t)((0x00fff & Threshold16) << 17);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdWord(Dev, VL53L0X_REG_SYSTEM_THRESH_HIGH,
            &Threshold16);
        /* Need to multiply by 2 because the FW will apply a x2 */
        *pThresholdHigh =
            (FixPoint1616_t)((0x00fff & Threshold16) << 17);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetStopCompletedStatus(VL53L0X_DEV Dev,
    uint32_t *pStopStatus)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte = 0;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_RdByte(Dev, 0x04, &Byte);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x0);

    *pStopStatus = Byte;

    if (Byte == 0) {
        Status = VL53L0X_WrByte(Dev, 0x80, 0x01);
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status = VL53L0X_WrByte(Dev, 0x00, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x91,
            PALDevDataGet(Dev, StopVariable));
        Status = VL53L0X_WrByte(Dev, 0x00, 0x01);
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);
        Status = VL53L0X_WrByte(Dev, 0x80, 0x00);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

/* Group PAL Interrupt Functions */
VL53L0X_Error VL53L0xBase::VL53L0X_ClearInterruptMask(VL53L0X_DEV Dev,
                     uint32_t InterruptMask)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t LoopCount;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    /* clear bit 0 range interrupt, bit 1 error interrupt */
    LoopCount = 0;
    do {
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
        Status |= VL53L0X_WrByte(Dev,
            VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
        Status |= VL53L0X_RdByte(Dev,
            VL53L0X_REG_RESULT_INTERRUPT_STATUS, &Byte);
        LoopCount++;
    } while (((Byte & 0x07) != 0x00)
            && (LoopCount < 3)
            && (Status == VL53L0X_ERROR_NONE));


    if (LoopCount >= 3)
        Status = VL53L0X_ERROR_INTERRUPT_NOT_CLEARED;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetInterruptMaskStatus(VL53L0X_DEV Dev,
    uint32_t *pInterruptMaskStatus)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    Status = VL53L0X_RdByte(Dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS,
                &Byte);
    *pInterruptMaskStatus = Byte & 0x07;

    if (Byte & 0x18)
        Status = VL53L0X_ERROR_RANGE_ERROR;

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_EnableInterruptMask(VL53L0X_DEV Dev,
                      uint32_t InterruptMask)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;

    LOG_FUNCTION_START("");

    /* not implemented for VL53L0X */

    LOG_FUNCTION_END(Status);
    return Status;
}

/* End Group PAL Interrupt Functions */

/* Group SPAD functions */

VL53L0X_Error VL53L0xBase::VL53L0X_SetSpadAmbientDamperThreshold(VL53L0X_DEV Dev,
    uint16_t SpadAmbientDamperThreshold)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrWord(Dev, 0x40, SpadAmbientDamperThreshold);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSpadAmbientDamperThreshold(VL53L0X_DEV Dev,
    uint16_t *pSpadAmbientDamperThreshold)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_RdWord(Dev, 0x40, pSpadAmbientDamperThreshold);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_SetSpadAmbientDamperFactor(VL53L0X_DEV Dev,
    uint16_t SpadAmbientDamperFactor)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    Byte = (uint8_t)(SpadAmbientDamperFactor & 0x00FF);

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0x42, Byte);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetSpadAmbientDamperFactor(VL53L0X_DEV Dev,
    uint16_t *pSpadAmbientDamperFactor)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t Byte;

    LOG_FUNCTION_START("");

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_RdByte(Dev, 0x42, &Byte);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    *pSpadAmbientDamperFactor = (uint16_t)Byte;

    LOG_FUNCTION_END(Status);
    return Status;
}

/* END Group SPAD functions */

/*****************************************************************************
 * Internal functions
 *****************************************************************************/

VL53L0X_Error VL53L0xBase::VL53L0X_SetReferenceSpads(VL53L0X_DEV Dev, uint32_t count,
    uint8_t isApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_set_reference_spads(Dev, count, isApertureSpads);

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_GetReferenceSpads(VL53L0X_DEV Dev, uint32_t *pSpadCount,
    uint8_t *pIsApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_reference_spads(Dev, pSpadCount, pIsApertureSpads);

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_PerformRefSpadManagement(VL53L0X_DEV Dev,
    uint32_t *refSpadCount, uint8_t *isApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    Status = VL53L0X_perform_ref_spad_management(Dev, refSpadCount,
        isApertureSpads);

    LOG_FUNCTION_END(Status);

    return Status;
}



// ====================================================================
//                          API CORE
// ====================================================================
VL53L0X_Error VL53L0xBase::VL53L0X_reverse_bytes(uint8_t *data, uint32_t size)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t tempData;
    uint32_t mirrorIndex;
    uint32_t middle = size/2;
    uint32_t index;

    for (index = 0; index < middle; index++) {
        mirrorIndex		 = size - index - 1;
        tempData		 = data[index];
        data[index]		 = data[mirrorIndex];
        data[mirrorIndex] = tempData;
    }
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_measurement_poll_for_completion(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDataReady = 0;
    uint32_t LoopNb;

    LOG_FUNCTION_START("");

    LoopNb = 0;

    do {
        Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDataReady);
        if (Status != 0)
            break; /* the error is set */

        if (NewDataReady == 1)
            break; /* done note that status == 0 */

        LoopNb++;
        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
            break;
        }

        VL53L0X_PollingDelay(Dev);
    } while (1);

    LOG_FUNCTION_END(Status);

    return Status;
}


uint8_t VL53L0xBase::VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg)
{
    /*!
     * Converts the encoded VCSEL period register value into the real
     * period in PLL clocks
     */

    uint8_t vcsel_period_pclks = 0;

    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

    return vcsel_period_pclks;
}

uint8_t VL53L0xBase::VL53L0X_encode_vcsel_period(uint8_t vcsel_period_pclks)
{
    /*!
     * Converts the encoded VCSEL period register value into the real period
     * in PLL clocks
     */

    uint8_t vcsel_period_reg = 0;

    vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

    return vcsel_period_reg;
}


uint32_t VL53L0xBase::VL53L0X_isqrt(uint32_t num)
{
    /*
     * Implements an integer square root
     *
     * From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
     */

    uint32_t  res = 0;
    uint32_t  bit = 1 << 30;
    /* The second-to-top bit is set:
     *	1 << 14 for 16-bits, 1 << 30 for 32 bits
     */

     /* "bit" starts at the highest power of four <= the argument. */
    while (bit > num)
        bit >>= 2;


    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else
            res >>= 1;

        bit >>= 2;
    }

    return res;
}


uint32_t VL53L0xBase::VL53L0X_quadrature_sum(uint32_t a, uint32_t b)
{
    /*
     * Implements a quadrature sum
     *
     * rea = sqrt(a^2 + b^2)
     *
     * Trap overflow case max input value is 65535 (16-bit value)
     * as internal calc are 32-bit wide
     *
     * If overflow then seta output to maximum
     */
    uint32_t  res = 0;

    if (a > 65535 || b > 65535)
        res = 65535;
    else
        res = VL53L0X_isqrt(a * a + b * b);

    return res;
}


VL53L0X_Error VL53L0xBase::VL53L0X_device_read_strobe(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t strobe;
    uint32_t LoopNb;

    LOG_FUNCTION_START("");

    Status |= VL53L0X_WrByte(Dev, 0x83, 0x00);

    /* polling
     * use timeout to avoid deadlock
     */
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_RdByte(Dev, 0x83, &strobe);
            if ((strobe != 0x00) || Status != VL53L0X_ERROR_NONE)
                break;

            LoopNb = LoopNb + 1;
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
            Status = VL53L0X_ERROR_TIME_OUT;

    }

    Status |= VL53L0X_WrByte(Dev, 0x83, 0x01);

    LOG_FUNCTION_END(Status);
    return Status;

}

VL53L0X_Error VL53L0xBase::VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8_t option)
{

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t byte;
    uint32_t TmpDWord;
    uint8_t ModuleId;
    uint8_t Revision;
    uint8_t ReferenceSpadCount = 0;
    uint8_t ReferenceSpadType = 0;
    uint32_t PartUIDUpper = 0;
    uint32_t PartUIDLower = 0;
    uint32_t OffsetFixed1104_mm = 0;
    int16_t OffsetMicroMeters = 0;
    uint32_t DistMeasTgtFixed1104_mm = 400 << 4;
    uint32_t DistMeasFixed1104_400_mm = 0;
    uint32_t SignalRateMeasFixed1104_400_mm = 0;
    char ProductId[19];
    char *ProductId_tmp;
    uint8_t ReadDataFromDeviceDone;
    FixPoint1616_t SignalRateMeasFixed400mmFix = 0;
    uint8_t NvmRefGoodSpadMap[VL53L0X_REF_SPAD_BUFFER_SIZE];
    int i;


    LOG_FUNCTION_START("");

    ReadDataFromDeviceDone = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
            ReadDataFromDeviceDone);

    /* This access is done only once after that a GetDeviceInfo or
     * datainit is done
     */
    if (ReadDataFromDeviceDone != 7) {

        Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);

        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0X_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0X_WrByte(Dev, 0x83, byte|4);
        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x07);
        Status |= VL53L0X_WrByte(Dev, 0x81, 0x01);

        Status |= VL53L0X_PollingDelay(Dev);

        Status |= VL53L0X_WrByte(Dev, 0x80, 0x01);

        if (((option & 1) == 1) &&
            ((ReadDataFromDeviceDone & 1) == 0)) {
            Status |= VL53L0X_WrByte(Dev, 0x94, 0x6b);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            ReferenceSpadCount = (uint8_t)((TmpDWord >> 8) & 0x07f);
            ReferenceSpadType  = (uint8_t)((TmpDWord >> 15) & 0x01);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x24);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);


            NvmRefGoodSpadMap[0] = (uint8_t)((TmpDWord >> 24)
                & 0xff);
            NvmRefGoodSpadMap[1] = (uint8_t)((TmpDWord >> 16)
                & 0xff);
            NvmRefGoodSpadMap[2] = (uint8_t)((TmpDWord >> 8)
                & 0xff);
            NvmRefGoodSpadMap[3] = (uint8_t)(TmpDWord & 0xff);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x25);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            NvmRefGoodSpadMap[4] = (uint8_t)((TmpDWord >> 24)
                & 0xff);
            NvmRefGoodSpadMap[5] = (uint8_t)((TmpDWord >> 16)
                & 0xff);
        }

        if (((option & 2) == 2) &&
            ((ReadDataFromDeviceDone & 2) == 0)) {

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x02);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdByte(Dev, 0x90, &ModuleId);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x7B);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdByte(Dev, 0x90, &Revision);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x77);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[0] = (char)((TmpDWord >> 25) & 0x07f);
            ProductId[1] = (char)((TmpDWord >> 18) & 0x07f);
            ProductId[2] = (char)((TmpDWord >> 11) & 0x07f);
            ProductId[3] = (char)((TmpDWord >> 4) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x00f) << 3);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x78);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[4] = (char)(byte +
                    ((TmpDWord >> 29) & 0x07f));
            ProductId[5] = (char)((TmpDWord >> 22) & 0x07f);
            ProductId[6] = (char)((TmpDWord >> 15) & 0x07f);
            ProductId[7] = (char)((TmpDWord >> 8) & 0x07f);
            ProductId[8] = (char)((TmpDWord >> 1) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x001) << 6);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x79);

            Status |= VL53L0X_device_read_strobe(Dev);

            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[9] = (char)(byte +
                    ((TmpDWord >> 26) & 0x07f));
            ProductId[10] = (char)((TmpDWord >> 19) & 0x07f);
            ProductId[11] = (char)((TmpDWord >> 12) & 0x07f);
            ProductId[12] = (char)((TmpDWord >> 5) & 0x07f);

            byte = (uint8_t)((TmpDWord & 0x01f) << 2);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x7A);

            Status |= VL53L0X_device_read_strobe(Dev);

            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            ProductId[13] = (char)(byte +
                    ((TmpDWord >> 30) & 0x07f));
            ProductId[14] = (char)((TmpDWord >> 23) & 0x07f);
            ProductId[15] = (char)((TmpDWord >> 16) & 0x07f);
            ProductId[16] = (char)((TmpDWord >> 9) & 0x07f);
            ProductId[17] = (char)((TmpDWord >> 2) & 0x07f);
            ProductId[18] = '\0';

        }

        if (((option & 4) == 4) &&
            ((ReadDataFromDeviceDone & 4) == 0)) {

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x7B);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &PartUIDUpper);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x7C);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &PartUIDLower);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x73);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            SignalRateMeasFixed1104_400_mm = (TmpDWord &
                0x0000000ff) << 8;

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x74);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            SignalRateMeasFixed1104_400_mm |= ((TmpDWord &
                0xff000000) >> 24);

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x75);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            DistMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff)
                            << 8;

            Status |= VL53L0X_WrByte(Dev, 0x94, 0x76);
            Status |= VL53L0X_device_read_strobe(Dev);
            Status |= VL53L0X_RdDWord(Dev, 0x90, &TmpDWord);

            DistMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000)
                            >> 24);
        }

        Status |= VL53L0X_WrByte(Dev, 0x81, 0x00);
        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0X_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0X_WrByte(Dev, 0x83, byte&0xfb);
        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0X_WrByte(Dev, 0x00, 0x01);

        Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
        Status |= VL53L0X_WrByte(Dev, 0x80, 0x00);
    }

    if ((Status == VL53L0X_ERROR_NONE) &&
        (ReadDataFromDeviceDone != 7)) {
        /* Assign to variable if status is ok */
        if (((option & 1) == 1) &&
            ((ReadDataFromDeviceDone & 1) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                ReferenceSpadCount, ReferenceSpadCount);

            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                ReferenceSpadType, ReferenceSpadType);

            for (i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE; i++) {
                Dev->Data.SpadData.RefGoodSpadMap[i] =
                    NvmRefGoodSpadMap[i];
            }
        }

        if (((option & 2) == 2) &&
            ((ReadDataFromDeviceDone & 2) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    ModuleId, ModuleId);

            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    Revision, Revision);

            ProductId_tmp = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
                    ProductId);
            VL53L0X_COPYSTRING(ProductId_tmp, ProductId);

        }

        if (((option & 4) == 4) &&
            ((ReadDataFromDeviceDone & 4) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                        PartUIDUpper, PartUIDUpper);

            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                        PartUIDLower, PartUIDLower);

            SignalRateMeasFixed400mmFix =
                VL53L0X_FIXPOINT97TOFIXPOINT1616(
                    SignalRateMeasFixed1104_400_mm);

            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                SignalRateMeasFixed400mm,
                SignalRateMeasFixed400mmFix);

            OffsetMicroMeters = 0;
            if (DistMeasFixed1104_400_mm != 0) {
                OffsetFixed1104_mm =
                    DistMeasFixed1104_400_mm -
                    DistMeasTgtFixed1104_mm;
                OffsetMicroMeters = (OffsetFixed1104_mm
                        * 1000) >> 4;
                OffsetMicroMeters *= -1;
            }

            PALDevDataSet(Dev,
                Part2PartOffsetAdjustmentNVMMicroMeter,
                OffsetMicroMeters);
        }
        byte = (uint8_t)(ReadDataFromDeviceDone|option);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone,
                byte);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}


uint32_t VL53L0xBase::VL53L0X_calc_macro_period_ps(VL53L0X_DEV Dev,
                      uint8_t vcsel_period_pclks)
{
    uint64_t PLL_period_ps;
    uint32_t macro_period_vclks;
    uint32_t macro_period_ps;

    LOG_FUNCTION_START("");

    /* The above calculation will produce rounding errors,
     *  therefore set fixed value
     */
    PLL_period_ps = 1655;

    macro_period_vclks = 2304;
    macro_period_ps = (uint32_t)(macro_period_vclks
            * vcsel_period_pclks * PLL_period_ps);

    LOG_FUNCTION_END("");
    return macro_period_ps;
}

uint16_t VL53L0xBase::VL53L0X_encode_timeout(uint32_t timeout_macro_clks)
{
    /*!
     * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
     */

    uint16_t encoded_timeout = 0;
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_macro_clks > 0) {
        ls_byte = timeout_macro_clks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte = ls_byte >> 1;
            ms_byte++;
        }

        encoded_timeout = (ms_byte << 8)
                + (uint16_t) (ls_byte & 0x000000FF);
    }

    return encoded_timeout;

}

uint32_t VL53L0xBase::VL53L0X_decode_timeout(uint16_t encoded_timeout)
{
    /*!
     * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
     */

    uint32_t timeout_macro_clks = 0;

    timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
            << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

    return timeout_macro_clks;
}


/* To convert ms into register value */
uint32_t VL53L0xBase::VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev,
        uint32_t timeout_period_us,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t timeout_period_mclks = 0;

    macro_period_ps = VL53L0X_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    timeout_period_mclks =
        (uint32_t) (((timeout_period_us * 1000)
        + (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

/* To convert register value into us */
uint32_t VL53L0xBase::VL53L0X_calc_timeout_us(VL53L0X_DEV Dev,
        uint16_t timeout_period_mclks,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t actual_timeout_period_us = 0;

    macro_period_ps = VL53L0X_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    actual_timeout_period_us =
        ((timeout_period_mclks * macro_period_ns) + 500) / 1000;

    return actual_timeout_period_us;
}


VL53L0X_Error VL53L0xBase::get_sequence_step_timeout(VL53L0X_DEV Dev,
                VL53L0X_SequenceStepId SequenceStepId,
                uint32_t *pTimeOutMicroSecs)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t EncodedTimeOutByte = 0;
    uint32_t TimeoutMicroSeconds = 0;
    uint16_t PreRangeEncodedTimeOut = 0;
    uint16_t MsrcTimeOutMClks;
    uint16_t PreRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks = 0;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;

    if ((SequenceStepId == VL53L0X_SEQUENCESTEP_TCC)	 ||
        (SequenceStepId == VL53L0X_SEQUENCESTEP_DSS)	 ||
        (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)) {

        Status = VL53L0X_GetVcselPulsePeriod(Dev,
                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_RdByte(Dev,
                    VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                    &EncodedTimeOutByte);
        }
        MsrcTimeOutMClks = VL53L0X_decode_timeout(EncodedTimeOutByte);

        TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev,
                        MsrcTimeOutMClks,
                        CurrentVCSELPulsePeriodPClk);
    } else if (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE) {
        /* Retrieve PRE-RANGE VCSEL Period */
        Status = VL53L0X_GetVcselPulsePeriod(Dev,
                        VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                        &CurrentVCSELPulsePeriodPClk);

        /* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
        if (Status == VL53L0X_ERROR_NONE) {

            /* Retrieve PRE-RANGE VCSEL Period */
            Status = VL53L0X_GetVcselPulsePeriod(Dev,
                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);

            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_RdWord(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                &PreRangeEncodedTimeOut);
            }

            PreRangeTimeOutMClks = VL53L0X_decode_timeout(
                    PreRangeEncodedTimeOut);

            TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev,
                    PreRangeTimeOutMClks,
                    CurrentVCSELPulsePeriodPClk);
        }
    } else if (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

        VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
        PreRangeTimeOutMClks = 0;

        if (SchedulerSequenceSteps.PreRangeOn) {
            /* Retrieve PRE-RANGE VCSEL Period */
            Status = VL53L0X_GetVcselPulsePeriod(Dev,
                VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                &CurrentVCSELPulsePeriodPClk);

            /* Retrieve PRE-RANGE Timeout in Macro periods
             * (MCLKS)
             */
            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_RdWord(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                &PreRangeEncodedTimeOut);
                PreRangeTimeOutMClks = VL53L0X_decode_timeout(
                        PreRangeEncodedTimeOut);
            }
        }

        if (Status == VL53L0X_ERROR_NONE) {
            /* Retrieve FINAL-RANGE VCSEL Period */
            Status = VL53L0X_GetVcselPulsePeriod(Dev,
                    VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                    &CurrentVCSELPulsePeriodPClk);
        }

        /* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_RdWord(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                &FinalRangeEncodedTimeOut);
            FinalRangeTimeOutMClks = VL53L0X_decode_timeout(
                    FinalRangeEncodedTimeOut);
        }

        FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;
        TimeoutMicroSeconds = VL53L0X_calc_timeout_us(Dev,
                        FinalRangeTimeOutMClks,
                        CurrentVCSELPulsePeriodPClk);
    }

    *pTimeOutMicroSecs = TimeoutMicroSeconds;

    return Status;
}


VL53L0X_Error VL53L0xBase::set_sequence_step_timeout(VL53L0X_DEV Dev,
                    VL53L0X_SequenceStepId SequenceStepId,
                    uint32_t TimeOutMicroSecs)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t MsrcEncodedTimeOut;
    uint16_t PreRangeEncodedTimeOut;
    uint16_t PreRangeTimeOutMClks;
    uint16_t MsrcRangeTimeOutMClks;
    uint32_t FinalRangeTimeOutMClks;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;

    if ((SequenceStepId == VL53L0X_SEQUENCESTEP_TCC)	 ||
        (SequenceStepId == VL53L0X_SEQUENCESTEP_DSS)	 ||
        (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)) {

        Status = VL53L0X_GetVcselPulsePeriod(Dev,
                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);

        if (Status == VL53L0X_ERROR_NONE) {
            MsrcRangeTimeOutMClks = VL53L0X_calc_timeout_mclks(Dev,
                    TimeOutMicroSecs,
                    (uint8_t)CurrentVCSELPulsePeriodPClk);

            if (MsrcRangeTimeOutMClks > 256)
                MsrcEncodedTimeOut = 255;
            else
                MsrcEncodedTimeOut =
                    (uint8_t)MsrcRangeTimeOutMClks - 1;

            VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                LastEncodedTimeout,
                MsrcEncodedTimeOut);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                MsrcEncodedTimeOut);
        }
    } else {

        if (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE) {

            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_GetVcselPulsePeriod(Dev,
                        VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                        &CurrentVCSELPulsePeriodPClk);
                PreRangeTimeOutMClks =
                    VL53L0X_calc_timeout_mclks(Dev,
                    TimeOutMicroSecs,
                    (uint8_t)CurrentVCSELPulsePeriodPClk);
                PreRangeEncodedTimeOut = VL53L0X_encode_timeout(
                    PreRangeTimeOutMClks);

                VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    LastEncodedTimeout,
                    PreRangeEncodedTimeOut);
            }

            if (Status == VL53L0X_ERROR_NONE) {
                Status = VL53L0X_WrWord(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                PreRangeEncodedTimeOut);
            }

            if (Status == VL53L0X_ERROR_NONE) {
                VL53L0X_SETDEVICESPECIFICPARAMETER(
                    Dev,
                    PreRangeTimeoutMicroSecs,
                    TimeOutMicroSecs);
            }
        } else if (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

            /* For the final range timeout, the pre-range timeout
             * must be added. To do this both final and pre-range
             * timeouts must be expressed in macro periods MClks
             * because they have different vcsel periods.
             */

            VL53L0X_GetSequenceStepEnables(Dev,
                    &SchedulerSequenceSteps);
            PreRangeTimeOutMClks = 0;
            if (SchedulerSequenceSteps.PreRangeOn) {

                /* Retrieve PRE-RANGE VCSEL Period */
                Status = VL53L0X_GetVcselPulsePeriod(Dev,
                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                    &CurrentVCSELPulsePeriodPClk);

                /* Retrieve PRE-RANGE Timeout in Macro periods
                 * (MCLKS)
                 */
                if (Status == VL53L0X_ERROR_NONE) {
                    Status = VL53L0X_RdWord(Dev, 0x51,
                        &PreRangeEncodedTimeOut);
                    PreRangeTimeOutMClks =
                        VL53L0X_decode_timeout(
                            PreRangeEncodedTimeOut);
                }
            }

            /* Calculate FINAL RANGE Timeout in Macro Periods
             * (MCLKS) and add PRE-RANGE value
             */
            if (Status == VL53L0X_ERROR_NONE) {

                Status = VL53L0X_GetVcselPulsePeriod(Dev,
                        VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                        &CurrentVCSELPulsePeriodPClk);
            }
            if (Status == VL53L0X_ERROR_NONE) {

                FinalRangeTimeOutMClks =
                    VL53L0X_calc_timeout_mclks(Dev,
                    TimeOutMicroSecs,
                    (uint8_t) CurrentVCSELPulsePeriodPClk);

                FinalRangeTimeOutMClks += PreRangeTimeOutMClks;

                FinalRangeEncodedTimeOut =
                VL53L0X_encode_timeout(FinalRangeTimeOutMClks);

                if (Status == VL53L0X_ERROR_NONE) {
                    Status = VL53L0X_WrWord(Dev, 0x71,
                    FinalRangeEncodedTimeOut);
                }

                if (Status == VL53L0X_ERROR_NONE) {
                    VL53L0X_SETDEVICESPECIFICPARAMETER(
                        Dev,
                        FinalRangeTimeoutMicroSecs,
                        TimeOutMicroSecs);
                }
            }
        } else
            Status = VL53L0X_ERROR_INVALID_PARAMS;

    }
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV Dev,
    VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t vcsel_period_reg;
    uint8_t MinPreVcselPeriodPCLK = 12;
    uint8_t MaxPreVcselPeriodPCLK = 18;
    uint8_t MinFinalVcselPeriodPCLK = 8;
    uint8_t MaxFinalVcselPeriodPCLK = 14;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t PreRangeTimeoutMicroSeconds;
    uint32_t MsrcTimeoutMicroSeconds;
    uint8_t PhaseCalInt = 0;

    /* Check if valid clock period requested */

    if ((VCSELPulsePeriodPCLK % 2) != 0) {
        /* Value must be an even number */
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    } else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_PRE_RANGE &&
        (VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK ||
        VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    } else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_FINAL_RANGE &&
        (VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK ||
         VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {

        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    /* Apply specific settings for the requested clock period */

    if (Status != VL53L0X_ERROR_NONE)
        return Status;


    if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {

        /* Set phase check limits */
        if (VCSELPulsePeriodPCLK == 12) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x18);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);
        } else if (VCSELPulsePeriodPCLK == 14) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x30);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);
        } else if (VCSELPulsePeriodPCLK == 16) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x40);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);
        } else if (VCSELPulsePeriodPCLK == 18) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x50);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);
        }
    } else if (VcselPeriodType == VL53L0X_VCSEL_PERIOD_FINAL_RANGE) {

        if (VCSELPulsePeriodPCLK == 8) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x10);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);

            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);

            Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_LIM,
                0x30);
            Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 10) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x28);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);

            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);

            Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_LIM,
                0x20);
            Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 12) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x38);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);

            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);

            Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_LIM,
                0x20);
            Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        } else if (VCSELPulsePeriodPCLK == 14) {

            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                0x048);
            Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                0x08);

            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);

            Status |= VL53L0X_WrByte(Dev, 0xff, 0x01);
            Status |= VL53L0X_WrByte(Dev,
                VL53L0X_REG_ALGO_PHASECAL_LIM,
                0x20);
            Status |= VL53L0X_WrByte(Dev, 0xff, 0x00);
        }
    }


    /* Re-calculate and apply timeouts, in macro periods */

    if (Status == VL53L0X_ERROR_NONE) {
        vcsel_period_reg = VL53L0X_encode_vcsel_period((uint8_t)
            VCSELPulsePeriodPCLK);

        /* When the VCSEL period for the pre or final range is changed,
        * the corresponding timeout must be read from the device using
        * the current VCSEL period, then the new VCSEL period can be
        * applied. The timeout then must be written back to the device
        * using the new VCSEL period.
        *
        * For the MSRC timeout, the same applies - this timeout being
        * dependant on the pre-range vcsel period.
        */
        switch (VcselPeriodType) {
        case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
            Status = get_sequence_step_timeout(Dev,
                VL53L0X_SEQUENCESTEP_PRE_RANGE,
                &PreRangeTimeoutMicroSeconds);

            if (Status == VL53L0X_ERROR_NONE)
                Status = get_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_MSRC,
                    &MsrcTimeoutMicroSeconds);

            if (Status == VL53L0X_ERROR_NONE)
                Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                    vcsel_period_reg);


            if (Status == VL53L0X_ERROR_NONE)
                Status = set_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_PRE_RANGE,
                    PreRangeTimeoutMicroSeconds);


            if (Status == VL53L0X_ERROR_NONE)
                Status = set_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_MSRC,
                    MsrcTimeoutMicroSeconds);

            VL53L0X_SETDEVICESPECIFICPARAMETER(
                Dev,
                PreRangeVcselPulsePeriod,
                VCSELPulsePeriodPCLK);
            break;
        case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
            Status = get_sequence_step_timeout(Dev,
                VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                &FinalRangeTimeoutMicroSeconds);

            if (Status == VL53L0X_ERROR_NONE)
                Status = VL53L0X_WrByte(Dev,
                VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                    vcsel_period_reg);


            if (Status == VL53L0X_ERROR_NONE)
                Status = set_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                    FinalRangeTimeoutMicroSeconds);

            VL53L0X_SETDEVICESPECIFICPARAMETER(
                Dev,
                FinalRangeVcselPulsePeriod,
                VCSELPulsePeriodPCLK);
            break;
        default:
            Status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }

    /* Finally, the timing budget must be re-applied */
    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_GETPARAMETERFIELD(Dev,
            MeasurementTimingBudgetMicroSeconds,
            MeasurementTimingBudgetMicroSeconds);

        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,
                MeasurementTimingBudgetMicroSeconds);
    }

    /* Perform the phase calibration. This is needed after changing on
     * vcsel period.
     * get_data_enable = 0, restore_config = 1
     */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_perform_phase_calibration(
            Dev, &PhaseCalInt, 0, 1);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV Dev,
    VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t vcsel_period_reg;

    switch (VcselPeriodType) {
    case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
        Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
            &vcsel_period_reg);
    break;
    case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
        Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
            &vcsel_period_reg);
    break;
    default:
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0X_ERROR_NONE)
        *pVCSELPulsePeriodPCLK =
            VL53L0X_decode_vcsel_period(vcsel_period_reg);

    return Status;
}



VL53L0X_Error VL53L0xBase::VL53L0X_set_measurement_timing_budget_micro_seconds(
        VL53L0X_DEV Dev,
        uint32_t MeasurementTimingBudgetMicroSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t FinalRangeTimingBudgetMicroSeconds;
    VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t MsrcDccTccTimeoutMicroSeconds	= 2000;
    uint32_t StartOverheadMicroSeconds		= 1910;
    uint32_t EndOverheadMicroSeconds		= 960;
    uint32_t MsrcOverheadMicroSeconds		= 660;
    uint32_t TccOverheadMicroSeconds		= 590;
    uint32_t DssOverheadMicroSeconds		= 690;
    uint32_t PreRangeOverheadMicroSeconds	= 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds	= 0;
    uint32_t SubTimeout = 0;

    LOG_FUNCTION_START("");

    FinalRangeTimingBudgetMicroSeconds =
        MeasurementTimingBudgetMicroSeconds -
        (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

    Status = VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

    if (Status == VL53L0X_ERROR_NONE &&
        (SchedulerSequenceSteps.TccOn  ||
        SchedulerSequenceSteps.MsrcOn ||
        SchedulerSequenceSteps.DssOn)) {

        /* TCC, MSRC and DSS all share the same timeout */
        Status = get_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_MSRC,
                    &MsrcDccTccTimeoutMicroSeconds);

        /* Subtract the TCC, MSRC and DSS timeouts if they are
         * enabled.
         */

        if (Status != VL53L0X_ERROR_NONE)
            return Status;

        /* TCC */
        if (SchedulerSequenceSteps.TccOn) {

            SubTimeout = MsrcDccTccTimeoutMicroSeconds
                + TccOverheadMicroSeconds;

            if (SubTimeout <
                FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds -=
                            SubTimeout;
            } else {
                /* Requested timeout too big. */
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }

        if (Status != VL53L0X_ERROR_NONE) {
            LOG_FUNCTION_END(Status);
            return Status;
        }

        /* DSS */
        if (SchedulerSequenceSteps.DssOn) {

            SubTimeout = 2 * (MsrcDccTccTimeoutMicroSeconds +
                DssOverheadMicroSeconds);

            if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds
                            -= SubTimeout;
            } else {
                /* Requested timeout too big. */
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        } else if (SchedulerSequenceSteps.MsrcOn) {
            /* MSRC */
            SubTimeout = MsrcDccTccTimeoutMicroSeconds +
                        MsrcOverheadMicroSeconds;

            if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds
                            -= SubTimeout;
            } else {
                /* Requested timeout too big. */
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }

    }

    if (Status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }

    if (SchedulerSequenceSteps.PreRangeOn) {

        /* Subtract the Pre-range timeout if enabled. */

        Status = get_sequence_step_timeout(Dev,
                VL53L0X_SEQUENCESTEP_PRE_RANGE,
                &PreRangeTimeoutMicroSeconds);

        SubTimeout = PreRangeTimeoutMicroSeconds +
                PreRangeOverheadMicroSeconds;

        if (SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
            FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
        } else {
            /* Requested timeout too big. */
            Status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }


    if (Status == VL53L0X_ERROR_NONE &&
        SchedulerSequenceSteps.FinalRangeOn) {

        FinalRangeTimingBudgetMicroSeconds -=
                FinalRangeOverheadMicroSeconds;

        /* Final Range Timeout
         * Note that the final range timeout is determined by the timing
         * budget and the sum of all other timeouts within the sequence.
         * If there is no room for the final range timeout, then an
         * error will be set. Otherwise the remaining time will be
         * applied to the final range.
         */
        Status = set_sequence_step_timeout(Dev,
            VL53L0X_SEQUENCESTEP_FINAL_RANGE,
            FinalRangeTimingBudgetMicroSeconds);

        VL53L0X_SETPARAMETERFIELD(Dev,
            MeasurementTimingBudgetMicroSeconds,
            MeasurementTimingBudgetMicroSeconds);
    }

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_measurement_timing_budget_micro_seconds(
        VL53L0X_DEV Dev,
        uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t MsrcDccTccTimeoutMicroSeconds	= 2000;
    uint32_t StartOverheadMicroSeconds		= 1910;
    uint32_t EndOverheadMicroSeconds		= 960;
    uint32_t MsrcOverheadMicroSeconds		= 660;
    uint32_t TccOverheadMicroSeconds		= 590;
    uint32_t DssOverheadMicroSeconds		= 690;
    uint32_t PreRangeOverheadMicroSeconds	= 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds	= 0;

    LOG_FUNCTION_START("");

    /* Start and end overhead times always present */
    *pMeasurementTimingBudgetMicroSeconds
        = StartOverheadMicroSeconds + EndOverheadMicroSeconds;

    Status = VL53L0X_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

    if (Status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }


    if (SchedulerSequenceSteps.TccOn  ||
        SchedulerSequenceSteps.MsrcOn ||
        SchedulerSequenceSteps.DssOn) {

        Status = get_sequence_step_timeout(Dev,
                VL53L0X_SEQUENCESTEP_MSRC,
                &MsrcDccTccTimeoutMicroSeconds);

        if (Status == VL53L0X_ERROR_NONE) {
            if (SchedulerSequenceSteps.TccOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                    MsrcDccTccTimeoutMicroSeconds +
                    TccOverheadMicroSeconds;
            }

            if (SchedulerSequenceSteps.DssOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                2 * (MsrcDccTccTimeoutMicroSeconds +
                    DssOverheadMicroSeconds);
            } else if (SchedulerSequenceSteps.MsrcOn) {
                *pMeasurementTimingBudgetMicroSeconds +=
                    MsrcDccTccTimeoutMicroSeconds +
                    MsrcOverheadMicroSeconds;
            }
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        if (SchedulerSequenceSteps.PreRangeOn) {
            Status = get_sequence_step_timeout(Dev,
                VL53L0X_SEQUENCESTEP_PRE_RANGE,
                &PreRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                PreRangeTimeoutMicroSeconds +
                PreRangeOverheadMicroSeconds;
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        if (SchedulerSequenceSteps.FinalRangeOn) {
            Status = get_sequence_step_timeout(Dev,
                    VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                    &FinalRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                (FinalRangeTimeoutMicroSeconds +
                FinalRangeOverheadMicroSeconds);
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(Dev,
            MeasurementTimingBudgetMicroSeconds,
            *pMeasurementTimingBudgetMicroSeconds);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}



VL53L0X_Error VL53L0xBase::VL53L0X_load_tuning_settings(VL53L0X_DEV Dev,
        uint8_t *pTuningSettingBuffer)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int i;
    int Index;
    uint8_t msb;
    uint8_t lsb;
    uint8_t SelectParam;
    uint8_t NumberOfWrites;
    uint8_t Address;
    uint8_t localBuffer[4]; /* max */
    uint16_t Temp16;

    LOG_FUNCTION_START("");

    Index = 0;

    while ((*(pTuningSettingBuffer + Index) != 0) &&
            (Status == VL53L0X_ERROR_NONE)) {
        NumberOfWrites = *(pTuningSettingBuffer + Index);
        Index++;
        if (NumberOfWrites == 0xFF) {
            /* internal parameters */
            SelectParam = *(pTuningSettingBuffer + Index);
            Index++;
            switch (SelectParam) {
            case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
                msb = *(pTuningSettingBuffer + Index);
                Index++;
                lsb = *(pTuningSettingBuffer + Index);
                Index++;
                Temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                PALDevDataSet(Dev, SigmaEstRefArray, Temp16);
                break;
            case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
                msb = *(pTuningSettingBuffer + Index);
                Index++;
                lsb = *(pTuningSettingBuffer + Index);
                Index++;
                Temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                PALDevDataSet(Dev, SigmaEstEffPulseWidth,
                    Temp16);
                break;
            case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
                msb = *(pTuningSettingBuffer + Index);
                Index++;
                lsb = *(pTuningSettingBuffer + Index);
                Index++;
                Temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                PALDevDataSet(Dev, SigmaEstEffAmbWidth, Temp16);
                break;
            case 3: /* uint16_t targetRefRate -> 2 bytes */
                msb = *(pTuningSettingBuffer + Index);
                Index++;
                lsb = *(pTuningSettingBuffer + Index);
                Index++;
                Temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                PALDevDataSet(Dev, targetRefRate, Temp16);
                break;
            default: /* invalid parameter */
                Status = VL53L0X_ERROR_INVALID_PARAMS;
            }

        } else if (NumberOfWrites <= 4) {
            Address = *(pTuningSettingBuffer + Index);
            Index++;

            for (i = 0; i < NumberOfWrites; i++) {
                localBuffer[i] = *(pTuningSettingBuffer +
                            Index);
                Index++;
            }

            Status = VL53L0X_WriteMulti(Dev, Address, localBuffer,
                    NumberOfWrites);

        } else {
            Status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    FixPoint1616_t *ptotal_xtalk_rate_mcps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    uint8_t xtalkCompEnable;
    FixPoint1616_t totalXtalkMegaCps;
    FixPoint1616_t xtalkPerSpadMegaCps;

    *ptotal_xtalk_rate_mcps = 0;

    Status = VL53L0X_GetXTalkCompensationEnable(Dev, &xtalkCompEnable);
    if (Status == VL53L0X_ERROR_NONE) {

        if (xtalkCompEnable) {

            VL53L0X_GETPARAMETERFIELD(
                Dev,
                XTalkCompensationRateMegaCps,
                xtalkPerSpadMegaCps);

            /* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
            totalXtalkMegaCps =
                pRangingMeasurementData->EffectiveSpadRtnCount *
                xtalkPerSpadMegaCps;

            /* FixPoint0824 >> 8 = FixPoint1616 */
            *ptotal_xtalk_rate_mcps =
                (totalXtalkMegaCps + 0x80) >> 8;
        }
    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    FixPoint1616_t *ptotal_signal_rate_mcps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t totalXtalkMegaCps;

    LOG_FUNCTION_START("");

    *ptotal_signal_rate_mcps =
        pRangingMeasurementData->SignalRateRtnMegaCps;

    Status = VL53L0X_get_total_xtalk_rate(
        Dev, pRangingMeasurementData, &totalXtalkMegaCps);

    if (Status == VL53L0X_ERROR_NONE)
        *ptotal_signal_rate_mcps += totalXtalkMegaCps;

    return Status;
}

VL53L0X_Error VL53L0xBase::get_dmax_lut_points(VL53L0X_DMaxLUT_t data, uint32_t lut_size,
    FixPoint1616_t input, int32_t *index0,	int32_t *index1){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t index0_tmp = 0;
    FixPoint1616_t index1_tmp = 0;
    int index = 0;

    for (index = 0; index < lut_size; index++) {
        if (input <= data.ambRate_mcps[index]) {
            index1_tmp = index;
            break;
        }
    }

    if (index == lut_size) {
        /* input is higher than last x point */
        index0_tmp = index1_tmp = lut_size - 1;
    } else if (index1_tmp == 0) {
        /* input is lower than first x point */
        index0_tmp = 0;
    } else{
        /* input is in between 2 points */
        index0_tmp = index1_tmp - 1;
    }

    *index0 = index0_tmp;
    *index1 = index1_tmp;

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_calc_dmax(
    VL53L0X_DEV Dev, FixPoint1616_t ambRateMeas, uint32_t *pdmax_mm){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceParameters_t CurrentParameters;
    int32_t index0 = 0;
    int32_t index1 = 0;
    FixPoint1616_t amb0, amb1, dmax0, dmax1;
    FixPoint1616_t dmax_mm;
    FixPoint1616_t linearSlope;

    LOG_FUNCTION_START("");

    Status = VL53L0X_GetDeviceParameters(Dev, &CurrentParameters);

    if (ambRateMeas <= CurrentParameters.dmax_lut.ambRate_mcps[0]) {
        dmax_mm = CurrentParameters.dmax_lut.dmax_mm[0];
    } else if (ambRateMeas >=
           CurrentParameters.dmax_lut.
           ambRate_mcps[VL53L0X_DMAX_LUT_SIZE - 1]) {
        dmax_mm =
            CurrentParameters.dmax_lut.dmax_mm[VL53L0X_DMAX_LUT_SIZE -
                               1];
    } else{
        get_dmax_lut_points(CurrentParameters.dmax_lut,
            VL53L0X_DMAX_LUT_SIZE, ambRateMeas, &index0, &index1);

        if (index0 == index1) {
            dmax_mm = CurrentParameters.dmax_lut.dmax_mm[index0];
        } else {
            amb0 = CurrentParameters.dmax_lut.ambRate_mcps[index0];
            amb1 = CurrentParameters.dmax_lut.ambRate_mcps[index1];
            dmax0 = CurrentParameters.dmax_lut.dmax_mm[index0];
            dmax1 = CurrentParameters.dmax_lut.dmax_mm[index1];
            if ((amb1 - amb0) != 0) {
                /* Fix16:16/Fix16:8 => Fix16:8 */
                linearSlope = (dmax0-dmax1)/((amb1-amb0) >> 8);

                /* Fix16:8 * Fix16:8 => Fix16:16 */
                dmax_mm =
                    (((amb1 -
                       ambRateMeas) >> 8) * linearSlope) +
                    dmax1;
            } else{
                dmax_mm = dmax0;
            }
        }
    }
    *pdmax_mm = (uint32_t)(dmax_mm >> 16);

    LOG_FUNCTION_END(Status);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    FixPoint1616_t *pSigmaEstimate)
{
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
    const FixPoint1616_t cDfltFinalRangeIntegrationTimeMilliSecs =
                        0x00190000; /* 25ms */
    const uint32_t cVcselPulseWidth_ps	= 4700; /* pico secs */
    const FixPoint1616_t cSigmaEstMax	= 0x028F87AE;
    const FixPoint1616_t cSigmaEstRtnMax	= 0xF000;
    const FixPoint1616_t cAmbToSignalRatioMax = 0xF0000000/
        cAmbientEffectiveWidth_centi_ns;
    /* Time Of Flight per mm (6.6 pico secs) */
    const FixPoint1616_t cTOF_per_mm_ps		= 0x0006999A;
    const uint32_t c16BitRoundingParam		= 0x00008000;
    const FixPoint1616_t cMaxXTalk_kcps		= 0x00320000;
    const uint32_t cPllPeriod_ps			= 1655;

    uint32_t vcselTotalEventsRtn;
    uint32_t finalRangeTimeoutMicroSecs;
    uint32_t preRangeTimeoutMicroSecs;
    uint32_t finalRangeIntegrationTimeMilliSecs;
    FixPoint1616_t sigmaEstimateP1;
    FixPoint1616_t sigmaEstimateP2;
    FixPoint1616_t sigmaEstimateP3;
    FixPoint1616_t deltaT_ps;
    FixPoint1616_t pwMult;
    FixPoint1616_t sigmaEstRtn;
    FixPoint1616_t sigmaEstimate;
    FixPoint1616_t xTalkCorrection;
    FixPoint1616_t ambientRate_kcps;
    FixPoint1616_t peakSignalRate_kcps;
    FixPoint1616_t xTalkCompRate_mcps;
    uint32_t xTalkCompRate_kcps;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t diff1_mcps;
    FixPoint1616_t diff2_mcps;
    FixPoint1616_t sqr1;
    FixPoint1616_t sqr2;
    FixPoint1616_t sqrSum;
    FixPoint1616_t sqrtResult_centi_ns;
    FixPoint1616_t sqrtResult;
    FixPoint1616_t totalSignalRate_mcps;
    FixPoint1616_t sigmaEstRef;
    uint32_t vcselWidth;
    uint32_t finalRangeMacroPCLKS;
    uint32_t preRangeMacroPCLKS;
    uint32_t peakVcselDuration_us;
    uint8_t finalRangeVcselPCLKS;
    uint8_t preRangeVcselPCLKS;
    /*! \addtogroup calc_sigma_estimate
     * @{
     *
     * Estimates the range sigma
     */

    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            xTalkCompRate_mcps);

    /*
     * We work in kcps rather than mcps as this helps keep within the
     * confines of the 32 Fix1616 type.
     */

    ambientRate_kcps =
        (pRangingMeasurementData->AmbientRateRtnMegaCps * 1000) >> 16;

    Status = VL53L0X_get_total_signal_rate(
        Dev, pRangingMeasurementData, &totalSignalRate_mcps);
    Status = VL53L0X_get_total_xtalk_rate(
        Dev, pRangingMeasurementData, &xTalkCompRate_mcps);


    /* Signal rate measurement provided by device is the
     * peak signal rate, not average.
     */
    peakSignalRate_kcps = (totalSignalRate_mcps * 1000);
    peakSignalRate_kcps = (peakSignalRate_kcps + 0x8000) >> 16;

    xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;

    if (xTalkCompRate_kcps > cMaxXTalk_kcps)
        xTalkCompRate_kcps = cMaxXTalk_kcps;

    if (Status == VL53L0X_ERROR_NONE) {

        /* Calculate final range macro periods */
        finalRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(
            Dev, FinalRangeTimeoutMicroSecs);

        finalRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(
            Dev, FinalRangeVcselPulsePeriod);

        finalRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(
            Dev, finalRangeTimeoutMicroSecs, finalRangeVcselPCLKS);

        /* Calculate pre-range macro periods */
        preRangeTimeoutMicroSecs = VL53L0X_GETDEVICESPECIFICPARAMETER(
            Dev, PreRangeTimeoutMicroSecs);

        preRangeVcselPCLKS = VL53L0X_GETDEVICESPECIFICPARAMETER(
            Dev, PreRangeVcselPulsePeriod);

        preRangeMacroPCLKS = VL53L0X_calc_timeout_mclks(
            Dev, preRangeTimeoutMicroSecs, preRangeVcselPCLKS);

        vcselWidth = 3;
        if (finalRangeVcselPCLKS == 8)
            vcselWidth = 2;


        peakVcselDuration_us = vcselWidth * 2048 *
            (preRangeMacroPCLKS + finalRangeMacroPCLKS);
        peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;
        peakVcselDuration_us *= cPllPeriod_ps;
        peakVcselDuration_us = (peakVcselDuration_us + 500)/1000;

        /* Fix1616 >> 8 = Fix2408 */
        totalSignalRate_mcps = (totalSignalRate_mcps + 0x80) >> 8;

        /* Fix2408 * uint32 = Fix2408 */
        vcselTotalEventsRtn = totalSignalRate_mcps *
            peakVcselDuration_us;

        /* Fix2408 >> 8 = uint32 */
        vcselTotalEventsRtn = (vcselTotalEventsRtn + 0x80) >> 8;

        /* Fix2408 << 8 = Fix1616 = */
        totalSignalRate_mcps <<= 8;
    }

    if (Status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(Status);
        return Status;
    }

    if (peakSignalRate_kcps == 0) {
        *pSigmaEstimate = cSigmaEstMax;
        PALDevDataSet(Dev, SigmaEstimate, cSigmaEstMax);
    } else {
        if (vcselTotalEventsRtn < 1)
            vcselTotalEventsRtn = 1;

        sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

        /* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
        sigmaEstimateP2 = (ambientRate_kcps << 16)/peakSignalRate_kcps;
        if (sigmaEstimateP2 > cAmbToSignalRatioMax) {
            /* Clip to prevent overflow. Will ensure safe
             * max result.
             */
            sigmaEstimateP2 = cAmbToSignalRatioMax;
        }
        sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

        sigmaEstimateP3 = 2 * VL53L0X_isqrt(vcselTotalEventsRtn * 12);

        /* uint32 * FixPoint1616 = FixPoint1616 */
        deltaT_ps = pRangingMeasurementData->RangeMilliMeter *
                    cTOF_per_mm_ps;

        /*
         * vcselRate - xtalkCompRate
         * (uint32 << 16) - FixPoint1616 = FixPoint1616.
         * Divide result by 1000 to convert to mcps.
         * 500 is added to ensure rounding when integer division
         * truncates.
         */
        diff1_mcps = (((peakSignalRate_kcps << 16) -
            2 * xTalkCompRate_kcps) + 500)/1000;

        /* vcselRate + xtalkCompRate */
        diff2_mcps = ((peakSignalRate_kcps << 16) + 500)/1000;

        /* Shift by 8 bits to increase resolution prior to the
         * division
         */
        diff1_mcps <<= 8;

        /* FixPoint0824/FixPoint1616 = FixPoint2408 */
        //xTalkCorrection	 = abs(diff1_mcps/diff2_mcps);
        // abs is causing compiler overloading isue in C++, but unsigned types. So, redundant call anyway!
        xTalkCorrection	 = diff1_mcps/diff2_mcps;

        /* FixPoint2408 << 8 = FixPoint1616 */
        xTalkCorrection <<= 8;

        if (pRangingMeasurementData->RangeStatus != 0) {
            pwMult = 1 << 16;
        } else {
            /* FixPoint1616/uint32 = FixPoint1616 */
            /* smaller than 1.0f */
            pwMult = deltaT_ps/cVcselPulseWidth_ps;

            /*
             * FixPoint1616 * FixPoint1616 = FixPoint3232, however
             * both values are small enough such that32 bits will
             * not be exceeded.
             */
            pwMult *= ((1 << 16) - xTalkCorrection);

            /* (FixPoint3232 >> 16) = FixPoint1616 */
            pwMult =  (pwMult + c16BitRoundingParam) >> 16;

            /* FixPoint1616 + FixPoint1616 = FixPoint1616 */
            pwMult += (1 << 16);

            /*
             * At this point the value will be 1.xx, therefore if we
             * square the value this will exceed 32 bits. To address
             * this perform a single shift to the right before the
             * multiplication.
             */
            pwMult >>= 1;
            /* FixPoint1715 * FixPoint1715 = FixPoint3430 */
            pwMult = pwMult * pwMult;

            /* (FixPoint3430 >> 14) = Fix1616 */
            pwMult >>= 14;
        }

        /* FixPoint1616 * uint32 = FixPoint1616 */
        sqr1 = pwMult * sigmaEstimateP1;

        /* (FixPoint1616 >> 16) = FixPoint3200 */
        sqr1 = (sqr1 + 0x8000) >> 16;

        /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
        sqr1 *= sqr1;

        sqr2 = sigmaEstimateP2;

        /* (FixPoint1616 >> 16) = FixPoint3200 */
        sqr2 = (sqr2 + 0x8000) >> 16;

        /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
        sqr2 *= sqr2;

        /* FixPoint64000 + FixPoint6400 = FixPoint6400 */
        sqrSum = sqr1 + sqr2;

        /* SQRT(FixPoin6400) = FixPoint3200 */
        sqrtResult_centi_ns = VL53L0X_isqrt(sqrSum);

        /* (FixPoint3200 << 16) = FixPoint1616 */
        sqrtResult_centi_ns <<= 16;

        /*
         * Note that the Speed Of Light is expressed in um per 1E-10
         * seconds (2997) Therefore to get mm/ns we have to divide by
         * 10000
         */
        sigmaEstRtn = (((sqrtResult_centi_ns+50)/100) /
                sigmaEstimateP3);
        sigmaEstRtn		 *= VL53L0X_SPEED_OF_LIGHT_IN_AIR;

        /* Add 5000 before dividing by 10000 to ensure rounding. */
        sigmaEstRtn		 += 5000;
        sigmaEstRtn		 /= 10000;

        if (sigmaEstRtn > cSigmaEstRtnMax) {
            /* Clip to prevent overflow. Will ensure safe
             * max result.
             */
            sigmaEstRtn = cSigmaEstRtnMax;
        }
        finalRangeIntegrationTimeMilliSecs =
            (finalRangeTimeoutMicroSecs + preRangeTimeoutMicroSecs +
             500) / 1000;

        /* sigmaEstRef = 1mm * 25ms/final range integration time
         * (inc pre-range)
         * sqrt(FixPoint1616/int) = FixPoint2408)
         */
        sigmaEstRef =
            VL53L0X_isqrt((cDfltFinalRangeIntegrationTimeMilliSecs +
                finalRangeIntegrationTimeMilliSecs/2)/
                finalRangeIntegrationTimeMilliSecs);

        /* FixPoint2408 << 8 = FixPoint1616 */
        sigmaEstRef <<= 8;
        sigmaEstRef = (sigmaEstRef + 500)/1000;

        /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
        sqr1 = sigmaEstRtn * sigmaEstRtn;
        /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
        sqr2 = sigmaEstRef * sigmaEstRef;

        /* sqrt(FixPoint3232) = FixPoint1616 */
        sqrtResult = VL53L0X_isqrt((sqr1 + sqr2));
        /*
         * Note that the Shift by 4 bits increases resolution prior to
         * the sqrt, therefore the result must be shifted by 2 bits to
         * the right to revert back to the FixPoint1616 format.
         */

        sigmaEstimate	 = 1000 * sqrtResult;

        if ((peakSignalRate_kcps < 1) || (vcselTotalEventsRtn < 1) ||
                (sigmaEstimate > cSigmaEstMax)) {
            sigmaEstimate = cSigmaEstMax;
        }

        *pSigmaEstimate = (uint32_t)(sigmaEstimate);
        PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_pal_range_status(VL53L0X_DEV Dev,
        uint8_t DeviceRangeStatus,
        FixPoint1616_t SignalRate,
        uint16_t EffectiveSpadRtnCount,
        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
        uint8_t *pPalRangeStatus)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NoneFlag;
    uint8_t SigmaLimitflag = 0;
    uint8_t SignalRefClipflag = 0;
    uint8_t RangeIgnoreThresholdflag = 0;
    uint8_t SigmaLimitCheckEnable = 0;
    uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
    uint8_t SignalRefClipLimitCheckEnable = 0;
    uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
    FixPoint1616_t SigmaEstimate;
    FixPoint1616_t SigmaLimitValue;
    FixPoint1616_t SignalRefClipValue;
    FixPoint1616_t RangeIgnoreThresholdValue;
    FixPoint1616_t SignalRatePerSpad;
    uint8_t DeviceRangeStatusInternal = 0;
    uint16_t tmpWord = 0;
    uint8_t Temp8;
    uint32_t Dmax_mm = 0;
    FixPoint1616_t LastSignalRefMcps;

    LOG_FUNCTION_START("");


    /*
     * VL53L0X has a good ranging when the value of the
     * DeviceRangeStatus = 11. This function will replace the value 0 with
     * the value 11 in the DeviceRangeStatus.
     * In addition, the SigmaEstimator is not included in the VL53L0X
     * DeviceRangeStatus, this will be added in the PalRangeStatus.
     */

    DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);

    if (DeviceRangeStatusInternal == 0 ||
        DeviceRangeStatusInternal == 5 ||
        DeviceRangeStatusInternal == 7 ||
        DeviceRangeStatusInternal == 12 ||
        DeviceRangeStatusInternal == 13 ||
        DeviceRangeStatusInternal == 14 ||
        DeviceRangeStatusInternal == 15
            ) {
        NoneFlag = 1;
    } else {
        NoneFlag = 0;
    }

    /*
     * Check if Sigma limit is enabled, if yes then do comparison with limit
     * value and put the result back into pPalRangeStatus.
     */
    if (Status == VL53L0X_ERROR_NONE)
        Status =  VL53L0X_GetLimitCheckEnable(Dev,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            &SigmaLimitCheckEnable);

    if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0X_ERROR_NONE)) {
        /*
        * compute the Sigma and check with limit
        */
        Status = VL53L0X_calc_sigma_estimate(
            Dev,
            pRangingMeasurementData,
            &SigmaEstimate);
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_calc_dmax(
                Dev,
                pRangingMeasurementData->AmbientRateRtnMegaCps,
                &Dmax_mm);
        if (Status == VL53L0X_ERROR_NONE)
            pRangingMeasurementData->RangeDMaxMilliMeter = Dmax_mm;

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_GetLimitCheckValue(Dev,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                &SigmaLimitValue);

            if ((SigmaLimitValue > 0) &&
                (SigmaEstimate > SigmaLimitValue))
                    /* Limit Fail */
                SigmaLimitflag = 1;
        }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     */
    if (Status == VL53L0X_ERROR_NONE)
        Status =  VL53L0X_GetLimitCheckEnable(Dev,
                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                &SignalRefClipLimitCheckEnable);

    if ((SignalRefClipLimitCheckEnable != 0) &&
            (Status == VL53L0X_ERROR_NONE)) {

        Status = VL53L0X_GetLimitCheckValue(Dev,
                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                &SignalRefClipValue);

        /* Read LastSignalRefMcps from device */
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_RdWord(Dev,
                VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF,
                &tmpWord);

        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

        LastSignalRefMcps = VL53L0X_FIXPOINT97TOFIXPOINT1616(tmpWord);
        PALDevDataSet(Dev, LastSignalRefMcps, LastSignalRefMcps);

        if ((SignalRefClipValue > 0) &&
                (LastSignalRefMcps > SignalRefClipValue)) {
            /* Limit Fail */
            SignalRefClipflag = 1;
        }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     * EffectiveSpadRtnCount has a format 8.8
     * If (Return signal rate < (1.5 x Xtalk x number of Spads)) : FAIL
     */
    if (Status == VL53L0X_ERROR_NONE)
        Status =  VL53L0X_GetLimitCheckEnable(Dev,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                &RangeIgnoreThresholdLimitCheckEnable);

    if ((RangeIgnoreThresholdLimitCheckEnable != 0) &&
            (Status == VL53L0X_ERROR_NONE)) {

        /* Compute the signal rate per spad */
        if (EffectiveSpadRtnCount == 0) {
            SignalRatePerSpad = 0;
        } else {
            SignalRatePerSpad = (FixPoint1616_t)((256 * SignalRate)
                / EffectiveSpadRtnCount);
        }

        Status = VL53L0X_GetLimitCheckValue(Dev,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                &RangeIgnoreThresholdValue);

        if ((RangeIgnoreThresholdValue > 0) &&
            (SignalRatePerSpad < RangeIgnoreThresholdValue)) {
            /* Limit Fail add 2^6 to range status */
            RangeIgnoreThresholdflag = 1;
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        if (NoneFlag == 1) {
            *pPalRangeStatus = 255;	 /* NONE */
        } else if (DeviceRangeStatusInternal == 1 ||
                    DeviceRangeStatusInternal == 2 ||
                    DeviceRangeStatusInternal == 3) {
            *pPalRangeStatus = 5; /* HW fail */
        } else if (DeviceRangeStatusInternal == 6 ||
                    DeviceRangeStatusInternal == 9) {
            *pPalRangeStatus = 4;  /* Phase fail */
        } else if (DeviceRangeStatusInternal == 8 ||
                    DeviceRangeStatusInternal == 10 ||
                    SignalRefClipflag == 1) {
            *pPalRangeStatus = 3;  /* Min range */
        } else if (DeviceRangeStatusInternal == 4 ||
                    RangeIgnoreThresholdflag == 1) {
            *pPalRangeStatus = 2;  /* Signal Fail */
        } else if (SigmaLimitflag == 1) {
            *pPalRangeStatus = 1;  /* Sigma	 Fail */
        } else {
            *pPalRangeStatus = 0; /* Range Valid */
        }
    }

    /* fill the Limit Check Status */

    Status =  VL53L0X_GetLimitCheckEnable(Dev,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            &SignalRateFinalRangeLimitCheckEnable);

    if (Status == VL53L0X_ERROR_NONE) {
        if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);

        if ((DeviceRangeStatusInternal == 4) ||
                (SignalRateFinalRangeLimitCheckEnable == 0))
            Temp8 = 1;
        else
            Temp8 = 0;
        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                Temp8);

        if ((SignalRefClipLimitCheckEnable == 0) ||
                    (SignalRefClipflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;

        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, Temp8);

        if ((RangeIgnoreThresholdLimitCheckEnable == 0) ||
                (RangeIgnoreThresholdflag == 1))
            Temp8 = 1;
        else
            Temp8 = 0;

        VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                Temp8);
    }

    LOG_FUNCTION_END(Status);
    return Status;

}


// ====================================================================
//                        CALIBRATION
// ====================================================================
VL53L0X_Error VL53L0xBase::VL53L0X_perform_xtalk_calibration(VL53L0X_DEV Dev,
            FixPoint1616_t XTalkCalDistance,
            FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t sum_ranging = 0;
    uint16_t sum_spads = 0;
    FixPoint1616_t sum_signalRate = 0;
    FixPoint1616_t total_count = 0;
    uint8_t xtalk_meas = 0;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t xTalkStoredMeanSignalRate;
    FixPoint1616_t xTalkStoredMeanRange;
    FixPoint1616_t xTalkStoredMeanRtnSpads;
    uint32_t signalXTalkTotalPerSpad;
    uint32_t xTalkStoredMeanRtnSpadsAsInt;
    uint32_t xTalkCalDistanceAsInt;
    FixPoint1616_t XTalkCompensationRateMegaCps;

    if (XTalkCalDistance <= 0)
        Status = VL53L0X_ERROR_INVALID_PARAMS;

    /* Disable the XTalk compensation */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetXTalkCompensationEnable(Dev, 0);

    /* Disable the RIT */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(Dev,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    /* Perform 50 measurements and compute the averages */
    if (Status == VL53L0X_ERROR_NONE) {
        sum_ranging = 0;
        sum_spads = 0;
        sum_signalRate = 0;
        total_count = 0;
        for (xtalk_meas = 0; xtalk_meas < 50; xtalk_meas++) {
            Status = VL53L0X_PerformSingleRangingMeasurement(Dev,
                &RangingMeasurementData);

            if (Status != VL53L0X_ERROR_NONE)
                break;

            /* The range is valid when RangeStatus = 0 */
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging +
                    RangingMeasurementData.RangeMilliMeter;
                sum_signalRate = sum_signalRate +
                RangingMeasurementData.SignalRateRtnMegaCps;
                sum_spads = sum_spads +
                RangingMeasurementData.EffectiveSpadRtnCount
                    / 256;
                total_count = total_count + 1;
            }
        }

        /* no valid values found */
        if (total_count == 0)
            Status = VL53L0X_ERROR_RANGE_ERROR;

    }


    if (Status == VL53L0X_ERROR_NONE) {
        /* FixPoint1616_t / uint16_t = FixPoint1616_t */
        xTalkStoredMeanSignalRate = sum_signalRate / total_count;
        xTalkStoredMeanRange = (FixPoint1616_t)((uint32_t)(
            sum_ranging << 16) / total_count);
        xTalkStoredMeanRtnSpads = (FixPoint1616_t)((uint32_t)(
            sum_spads << 16) / total_count);

        /* Round Mean Spads to Whole Number.
         * Typically the calculated mean SPAD count is a whole number
         * or very close to a whole
         * number, therefore any truncation will not result in a
         * significant loss in accuracy.
         * Also, for a grey target at a typical distance of around
         * 400mm, around 220 SPADs will
         * be enabled, therefore, any truncation will result in a loss
         * of accuracy of less than
         * 0.5%.
         */
        xTalkStoredMeanRtnSpadsAsInt = (xTalkStoredMeanRtnSpads +
            0x8000) >> 16;

        /* Round Cal Distance to Whole Number.
         * Note that the cal distance is in mm, therefore no resolution
         * is lost.
         */
         xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

        if (xTalkStoredMeanRtnSpadsAsInt == 0 ||
           xTalkCalDistanceAsInt == 0 ||
           xTalkStoredMeanRange >= XTalkCalDistance) {
            XTalkCompensationRateMegaCps = 0;
        } else {
            /* Round Cal Distance to Whole Number.
             * Note that the cal distance is in mm, therefore no
             * resolution is lost.
             */
            xTalkCalDistanceAsInt = (XTalkCalDistance +
                0x8000) >> 16;

            /* Apply division by mean spad count early in the
             * calculation to keep the numbers small.
             * This ensures we can maintain a 32bit calculation.
             * Fixed1616 / int := Fixed1616
             */
            signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate) /
                xTalkStoredMeanRtnSpadsAsInt;

            /* Complete the calculation for total Signal XTalk per
             * SPAD
             * Fixed1616 * (Fixed1616 - Fixed1616/int) :=
             * (2^16 * Fixed1616)
             */
            signalXTalkTotalPerSpad *= ((1 << 16) -
                (xTalkStoredMeanRange / xTalkCalDistanceAsInt));

            /* Round from 2^16 * Fixed1616, to Fixed1616. */
            XTalkCompensationRateMegaCps = (signalXTalkTotalPerSpad
                + 0x8000) >> 16;
        }

        *pXTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;

        /* Enable the XTalk compensation */
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_SetXTalkCompensationEnable(Dev, 1);

        /* Enable the XTalk compensation */
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_SetXTalkCompensationRateMegaCps(Dev,
                    XTalkCompensationRateMegaCps);

    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_perform_offset_calibration(VL53L0X_DEV Dev,
            FixPoint1616_t CalDistanceMilliMeter,
            int32_t *pOffsetMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t sum_ranging = 0;
    FixPoint1616_t total_count = 0;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t StoredMeanRange;
    uint32_t StoredMeanRangeAsInt;
    uint32_t CalDistanceAsInt_mm;
    uint8_t SequenceStepEnabled;
    int meas = 0;

    if (CalDistanceMilliMeter <= 0)
        Status = VL53L0X_ERROR_INVALID_PARAMS;

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev, 0);


    /* Get the value of the TCC */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetSequenceStepEnable(Dev,
                VL53L0X_SEQUENCESTEP_TCC, &SequenceStepEnabled);


    /* Disable the TCC */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetSequenceStepEnable(Dev,
                VL53L0X_SEQUENCESTEP_TCC, 0);


    /* Disable the RIT */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_SetLimitCheckEnable(Dev,
                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

    /* Perform 50 measurements and compute the averages */
    if (Status == VL53L0X_ERROR_NONE) {
        sum_ranging = 0;
        total_count = 0;
        for (meas = 0; meas < 50; meas++) {
            Status = VL53L0X_PerformSingleRangingMeasurement(Dev,
                    &RangingMeasurementData);

            if (Status != VL53L0X_ERROR_NONE)
                break;

            /* The range is valid when RangeStatus = 0 */
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging +
                    RangingMeasurementData.RangeMilliMeter;
                total_count = total_count + 1;
            }
        }

        /* no valid values found */
        if (total_count == 0)
            Status = VL53L0X_ERROR_RANGE_ERROR;
    }


    if (Status == VL53L0X_ERROR_NONE) {
        /* FixPoint1616_t / uint16_t = FixPoint1616_t */
        StoredMeanRange = (FixPoint1616_t)((uint32_t)(sum_ranging << 16)
            / total_count);

        StoredMeanRangeAsInt = (StoredMeanRange + 0x8000) >> 16;

        /* Round Cal Distance to Whole Number.
         * Note that the cal distance is in mm, therefore no resolution
         * is lost.
         */
         CalDistanceAsInt_mm = (CalDistanceMilliMeter + 0x8000) >> 16;

         *pOffsetMicroMeter = (CalDistanceAsInt_mm -
                 StoredMeanRangeAsInt) * 1000;

        /* Apply the calculated offset */
        if (Status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters,
                    *pOffsetMicroMeter);
            Status = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev,
                    *pOffsetMicroMeter);
        }

    }

    /* Restore the TCC */
    if (Status == VL53L0X_ERROR_NONE) {
        if (SequenceStepEnabled != 0)
            Status = VL53L0X_SetSequenceStepEnable(Dev,
                    VL53L0X_SEQUENCESTEP_TCC, 1);
    }

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
        int32_t OffsetCalibrationDataMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t cMaxOffsetMicroMeter = 511000;
    int32_t cMinOffsetMicroMeter = -512000;
    int16_t cOffsetRange = 4096;
    uint32_t encodedOffsetVal;

    LOG_FUNCTION_START("");

    if (OffsetCalibrationDataMicroMeter > cMaxOffsetMicroMeter)
        OffsetCalibrationDataMicroMeter = cMaxOffsetMicroMeter;
    else if (OffsetCalibrationDataMicroMeter < cMinOffsetMicroMeter)
        OffsetCalibrationDataMicroMeter = cMinOffsetMicroMeter;

    /* The offset register is 10.2 format and units are mm
     * therefore conversion is applied by a division of
     * 250.
     */
    if (OffsetCalibrationDataMicroMeter >= 0) {
        encodedOffsetVal =
            OffsetCalibrationDataMicroMeter/250;
    } else {
        encodedOffsetVal =
            cOffsetRange +
            OffsetCalibrationDataMicroMeter/250;
    }

    Status = VL53L0X_WrWord(Dev,
        VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
        encodedOffsetVal);

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
        int32_t *pOffsetCalibrationDataMicroMeter)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint16_t RangeOffsetRegister;
    int16_t cMaxOffset = 2047;
    int16_t cOffsetRange = 4096;

    /* Note that offset has 10.2 format */

    Status = VL53L0X_RdWord(Dev,
                VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                &RangeOffsetRegister);

    if (Status == VL53L0X_ERROR_NONE) {
        RangeOffsetRegister = (RangeOffsetRegister & 0x0fff);

        /* Apply 12 bit 2's compliment conversion */
        if (RangeOffsetRegister > cMaxOffset)
            *pOffsetCalibrationDataMicroMeter =
                (int16_t)(RangeOffsetRegister - cOffsetRange)
                    * 250;
        else
            *pOffsetCalibrationDataMicroMeter =
                (int16_t)RangeOffsetRegister * 250;

    }

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_apply_offset_adjustment(VL53L0X_DEV Dev)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t CorrectedOffsetMicroMeters;
    int32_t CurrentOffsetMicroMeters;

    /* if we run on this function we can read all the NVM info
     * used by the API
     */
    Status = VL53L0X_get_info_from_device(Dev, 7);

    /* Read back current device offset */
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetOffsetCalibrationDataMicroMeter(Dev,
                    &CurrentOffsetMicroMeters);
    }

    /* Apply Offset Adjustment derived from 400mm measurements */
    if (Status == VL53L0X_ERROR_NONE) {

        /* Store initial device offset */
        PALDevDataSet(Dev, Part2PartOffsetNVMMicroMeter,
            CurrentOffsetMicroMeters);

        CorrectedOffsetMicroMeters = CurrentOffsetMicroMeters +
            (int32_t)PALDevDataGet(Dev,
                Part2PartOffsetAdjustmentNVMMicroMeter);

        Status = VL53L0X_SetOffsetCalibrationDataMicroMeter(Dev,
                    CorrectedOffsetMicroMeters);

        /* store current, adjusted offset */
        if (Status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters,
                    CorrectedOffsetMicroMeters);
        }
    }

    return Status;
}

void VL53L0xBase::get_next_good_spad(uint8_t goodSpadArray[], uint32_t size,
            uint32_t curr, int32_t *next)
{
    uint32_t startIndex;
    uint32_t fineOffset;
    uint32_t cSpadsPerByte = 8;
    uint32_t coarseIndex;
    uint32_t fineIndex;
    uint8_t dataByte;
    uint8_t success = 0;

    /*
     * Starting with the current good spad, loop through the array to find
     * the next. i.e. the next bit set in the sequence.
     *
     * The coarse index is the byte index of the array and the fine index is
     * the index of the bit within each byte.
     */

    *next = -1;

    startIndex = curr / cSpadsPerByte;
    fineOffset = curr % cSpadsPerByte;

    for (coarseIndex = startIndex; ((coarseIndex < size) && !success);
                coarseIndex++) {
        fineIndex = 0;
        dataByte = goodSpadArray[coarseIndex];

        if (coarseIndex == startIndex) {
            /* locate the bit position of the provided current
             * spad bit before iterating
             */
            dataByte >>= fineOffset;
            fineIndex = fineOffset;
        }

        while (fineIndex < cSpadsPerByte) {
            if ((dataByte & 0x1) == 1) {
                success = 1;
                *next = coarseIndex * cSpadsPerByte + fineIndex;
                break;
            }
            dataByte >>= 1;
            fineIndex++;
        }
    }
}


uint8_t VL53L0xBase::is_aperture(uint32_t spadIndex)
{
    /*
     * This function reports if a given spad index is an aperture SPAD by
     * deriving the quadrant.
     */
    uint32_t quadrant;
    uint8_t isAperture = 1;

    quadrant = spadIndex >> 6;
    if (refArrayQuadrants[quadrant] == REF_ARRAY_SPAD_0)
        isAperture = 0;

    return isAperture;
}


VL53L0X_Error VL53L0xBase::enable_spad_bit(uint8_t spadArray[], uint32_t size,
    uint32_t spadIndex)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t cSpadsPerByte = 8;
    uint32_t coarseIndex;
    uint32_t fineIndex;

    coarseIndex = spadIndex / cSpadsPerByte;
    fineIndex = spadIndex % cSpadsPerByte;
    if (coarseIndex >= size)
        status = VL53L0X_ERROR_REF_SPAD_INIT;
    else
        spadArray[coarseIndex] |= (1 << fineIndex);

    return status;
}

VL53L0X_Error VL53L0xBase::count_enabled_spads(uint8_t spadArray[],
        uint32_t byteCount, uint32_t maxSpads,
        uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t cSpadsPerByte = 8;
    uint32_t lastByte;
    uint32_t lastBit;
    uint32_t byteIndex = 0;
    uint32_t bitIndex = 0;
    uint8_t tempByte;
    uint8_t spadTypeIdentified = 0;

    /* The entire array will not be used for spads, therefore the last
     * byte and last bit is determined from the max spads value.
     */

    lastByte = maxSpads / cSpadsPerByte;
    lastBit = maxSpads % cSpadsPerByte;

    /* Check that the max spads value does not exceed the array bounds. */
    if (lastByte >= byteCount)
        status = VL53L0X_ERROR_REF_SPAD_INIT;

    *pTotalSpadsEnabled = 0;

    /* Count the bits enabled in the whole bytes */
    for (byteIndex = 0; byteIndex <= (lastByte - 1); byteIndex++) {
        tempByte = spadArray[byteIndex];

        for (bitIndex = 0; bitIndex <= cSpadsPerByte; bitIndex++) {
            if ((tempByte & 0x01) == 1) {
                (*pTotalSpadsEnabled)++;

                if (!spadTypeIdentified) {
                    *pIsAperture = 1;
                    if ((byteIndex < 2) && (bitIndex < 4))
                        *pIsAperture = 0;
                    spadTypeIdentified = 1;
                }
            }
            tempByte >>= 1;
        }
    }

    /* Count the number of bits enabled in the last byte accounting
     * for the fact that not all bits in the byte may be used.
     */
    tempByte = spadArray[lastByte];

    for (bitIndex = 0; bitIndex <= lastBit; bitIndex++) {
        if ((tempByte & 0x01) == 1)
            (*pTotalSpadsEnabled)++;
    }

    return status;
}

VL53L0X_Error VL53L0xBase::set_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray)
{
    VL53L0X_Error status = VL53L0X_WriteMulti(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                refSpadArray, 6);
    return status;
}

VL53L0X_Error VL53L0xBase::get_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray)
{
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev,
                VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                refSpadArray,
                6);
    return status;
}

VL53L0X_Error VL53L0xBase::enable_ref_spads(VL53L0X_DEV Dev,
                uint8_t apertureSpads,
                uint8_t goodSpadArray[],
                uint8_t spadArray[],
                uint32_t size,
                uint32_t start,
                uint32_t offset,
                uint32_t spadCount,
                uint32_t *lastSpad)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t index;
    uint32_t i;
    int32_t nextGoodSpad = offset;
    uint32_t currentSpad;
    uint8_t checkSpadArray[6];

    /*
     * This function takes in a spad array which may or may not have SPADS
     * already enabled and appends from a given offset a requested number
     * of new SPAD enables. The 'good spad map' is applied to
     * determine the next SPADs to enable.
     *
     * This function applies to only aperture or only non-aperture spads.
     * Checks are performed to ensure this.
     */

    currentSpad = offset;
    for (index = 0; index < spadCount; index++) {
        get_next_good_spad(goodSpadArray, size, currentSpad,
            &nextGoodSpad);

        if (nextGoodSpad == -1) {
            status = VL53L0X_ERROR_REF_SPAD_INIT;
            break;
        }

        /* Confirm that the next good SPAD is non-aperture */
        if (is_aperture(start + nextGoodSpad) != apertureSpads) {
            /* if we can't get the required number of good aperture
             * spads from the current quadrant then this is an error
             */
            status = VL53L0X_ERROR_REF_SPAD_INIT;
            break;
        }
        currentSpad = (uint32_t)nextGoodSpad;
        enable_spad_bit(spadArray, size, currentSpad);
        currentSpad++;
    }
    *lastSpad = currentSpad;

    if (status == VL53L0X_ERROR_NONE)
        status = set_ref_spad_map(Dev, spadArray);


    if (status == VL53L0X_ERROR_NONE) {
        status = get_ref_spad_map(Dev, checkSpadArray);

        i = 0;

        /* Compare spad maps. If not equal report error. */
        while (i < size) {
            if (spadArray[i] != checkSpadArray[i]) {
                status = VL53L0X_ERROR_REF_SPAD_INIT;
                break;
            }
            i++;
        }
    }
    return status;
}


VL53L0X_Error VL53L0xBase::perform_ref_signal_measurement(VL53L0X_DEV Dev,
        uint16_t *refSignalRate)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t rangingMeasurementData;

    uint8_t SequenceConfig = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    /*
     * This function performs a reference signal rate measurement.
     */
    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_PerformSingleRangingMeasurement(Dev,
                &rangingMeasurementData);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_RdWord(Dev,
            VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF,
            refSignalRate);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

    if (status == VL53L0X_ERROR_NONE) {
        /* restore the previous Sequence Config */
        status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                SequenceConfig);
        if (status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
    }

    return status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_perform_ref_spad_management(VL53L0X_DEV Dev,
                uint32_t *refSpadCount,
                uint8_t *isApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t lastSpadArray[6];
    uint8_t startSelect = 0xB4;
    uint32_t minimumSpadCount = 3;
    uint32_t maxSpadCount = 44;
    uint32_t currentSpadIndex = 0;
    uint32_t lastSpadIndex = 0;
    int32_t nextGoodSpad = 0;
    uint16_t targetRefRate = 0x0A00; /* 20 MCPS in 9:7 format */
    uint16_t peakSignalRateRef;
    uint32_t needAptSpads = 0;
    uint32_t index = 0;
    uint32_t spadArraySize = 6;
    uint32_t signalRateDiff = 0;
    uint32_t lastSignalRateDiff = 0;
    uint8_t complete = 0;
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;
    uint32_t refSpadCount_int = 0;
    uint8_t	 isApertureSpads_int = 0;

    /*
     * The reference SPAD initialization procedure determines the minimum
     * amount of reference spads to be enables to achieve a target reference
     * signal rate and should be performed once during initialization.
     *
     * Either aperture or non-aperture spads are applied but never both.
     * Firstly non-aperture spads are set, begining with 5 spads, and
     * increased one spad at a time until the closest measurement to the
     * target rate is achieved.
     *
     * If the target rate is exceeded when 5 non-aperture spads are enabled,
     * initialization is performed instead with aperture spads.
     *
     * When setting spads, a 'Good Spad Map' is applied.
     *
     * This procedure operates within a SPAD window of interest of a maximum
     * 44 spads.
     * The start point is currently fixed to 180, which lies towards the end
     * of the non-aperture quadrant and runs in to the adjacent aperture
     * quadrant.
     */


    targetRefRate = PALDevDataGet(Dev, targetRefRate);

    /*
     * Initialize Spad arrays.
     * Currently the good spad map is initialised to 'All good'.
     * This is a short term implementation. The good spad map will be
     * provided as an input.
     * Note that there are 6 bytes. Only the first 44 bits will be used to
     * represent spads.
     */
    for (index = 0; index < spadArraySize; index++)
        Dev->Data.SpadData.RefSpadEnables[index] = 0;


    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT,
            startSelect);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);

    /* Perform ref calibration */
    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_perform_ref_calibration(Dev, &VhvSettings,
            &PhaseCal, 0);

    if (Status == VL53L0X_ERROR_NONE) {
        /* Enable Minimum NON-APERTURE Spads */
        currentSpadIndex = 0;
        lastSpadIndex = currentSpadIndex;
        needAptSpads = 0;
        Status = enable_ref_spads(Dev,
                    needAptSpads,
                    Dev->Data.SpadData.RefGoodSpadMap,
                    Dev->Data.SpadData.RefSpadEnables,
                    spadArraySize,
                    startSelect,
                    currentSpadIndex,
                    minimumSpadCount,
                    &lastSpadIndex);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        currentSpadIndex = lastSpadIndex;

        Status = perform_ref_signal_measurement(Dev,
            &peakSignalRateRef);
        if ((Status == VL53L0X_ERROR_NONE) &&
            (peakSignalRateRef > targetRefRate)) {
            /* Signal rate measurement too high,
             * switch to APERTURE SPADs
             */

            for (index = 0; index < spadArraySize; index++)
                Dev->Data.SpadData.RefSpadEnables[index] = 0;


            /* Increment to the first APERTURE spad */
            while ((is_aperture(startSelect + currentSpadIndex)
                == 0) && (currentSpadIndex < maxSpadCount)) {
                currentSpadIndex++;
            }

            needAptSpads = 1;

            Status = enable_ref_spads(Dev,
                    needAptSpads,
                    Dev->Data.SpadData.RefGoodSpadMap,
                    Dev->Data.SpadData.RefSpadEnables,
                    spadArraySize,
                    startSelect,
                    currentSpadIndex,
                    minimumSpadCount,
                    &lastSpadIndex);

            if (Status == VL53L0X_ERROR_NONE) {
                currentSpadIndex = lastSpadIndex;
                Status = perform_ref_signal_measurement(Dev,
                        &peakSignalRateRef);

                if ((Status == VL53L0X_ERROR_NONE) &&
                    (peakSignalRateRef > targetRefRate)) {
                    /* Signal rate still too high after
                     * setting the minimum number of
                     * APERTURE spads. Can do no more
                     * therefore set the min number of
                     * aperture spads as the result.
                     */
                    isApertureSpads_int = 1;
                    refSpadCount_int = minimumSpadCount;
                }
            }
        } else {
            needAptSpads = 0;
        }
    }

    if ((Status == VL53L0X_ERROR_NONE) &&
        (peakSignalRateRef < targetRefRate)) {
        /* At this point, the minimum number of either aperture
         * or non-aperture spads have been set. Proceed to add
         * spads and perform measurements until the target
         * reference is reached.
         */
        isApertureSpads_int = needAptSpads;
        refSpadCount_int	= minimumSpadCount;

        memcpy(lastSpadArray, Dev->Data.SpadData.RefSpadEnables,
                spadArraySize);
        lastSignalRateDiff = abs(peakSignalRateRef -
            targetRefRate);
        complete = 0;

        while (!complete) {
            get_next_good_spad(
                Dev->Data.SpadData.RefGoodSpadMap,
                spadArraySize, currentSpadIndex,
                &nextGoodSpad);

            if (nextGoodSpad == -1) {
                Status = VL53L0X_ERROR_REF_SPAD_INIT;
                break;
            }

            /* Cannot combine Aperture and Non-Aperture spads, so
             * ensure the current spad is of the correct type.
             */
            if (is_aperture((uint32_t)startSelect + nextGoodSpad) !=
                    needAptSpads) {
                /* At this point we have enabled the maximum
                 * number of Aperture spads.
                 */
                complete = 1;
                break;
            }

            (refSpadCount_int)++;

            currentSpadIndex = nextGoodSpad;
            Status = enable_spad_bit(
                    Dev->Data.SpadData.RefSpadEnables,
                    spadArraySize, currentSpadIndex);

            if (Status == VL53L0X_ERROR_NONE) {
                currentSpadIndex++;
                /* Proceed to apply the additional spad and
                 * perform measurement.
                 */
                Status = set_ref_spad_map(Dev,
                    Dev->Data.SpadData.RefSpadEnables);
            }

            if (Status != VL53L0X_ERROR_NONE)
                break;

            Status = perform_ref_signal_measurement(Dev,
                    &peakSignalRateRef);

            if (Status != VL53L0X_ERROR_NONE)
                break;

            signalRateDiff = abs(peakSignalRateRef - targetRefRate);

            if (peakSignalRateRef > targetRefRate) {
                /* Select the spad map that provides the
                 * measurement closest to the target rate,
                 * either above or below it.
                 */
                if (signalRateDiff > lastSignalRateDiff) {
                    /* Previous spad map produced a closer
                     * measurement, so choose this.
                     */
                    Status = set_ref_spad_map(Dev,
                            lastSpadArray);
                    memcpy(
                    Dev->Data.SpadData.RefSpadEnables,
                    lastSpadArray, spadArraySize);

                    (refSpadCount_int)--;
                }
                complete = 1;
            } else {
                /* Continue to add spads */
                lastSignalRateDiff = signalRateDiff;
                memcpy(lastSpadArray,
                    Dev->Data.SpadData.RefSpadEnables,
                    spadArraySize);
            }

        } /* while */
    }

    if (Status == VL53L0X_ERROR_NONE) {
        *refSpadCount = refSpadCount_int;
        *isApertureSpads = isApertureSpads_int;

        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadCount, (uint8_t)(*refSpadCount));
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadType, *isApertureSpads);
    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_set_reference_spads(VL53L0X_DEV Dev,
                 uint32_t count, uint8_t isApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t currentSpadIndex = 0;
    uint8_t startSelect = 0xB4;
    uint32_t spadArraySize = 6;
    uint32_t maxSpadCount = 44;
    uint32_t lastSpadIndex;
    uint32_t index;

    /*
     * This function applies a requested number of reference spads, either
     * aperture or
     * non-aperture, as requested.
     * The good spad map will be applied.
     */

    Status = VL53L0X_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev,
            VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT,
            startSelect);

    for (index = 0; index < spadArraySize; index++)
        Dev->Data.SpadData.RefSpadEnables[index] = 0;

    if (isApertureSpads) {
        /* Increment to the first APERTURE spad */
        while ((is_aperture(startSelect + currentSpadIndex) == 0) &&
              (currentSpadIndex < maxSpadCount)) {
            currentSpadIndex++;
        }
    }
    Status = enable_ref_spads(Dev,
                isApertureSpads,
                Dev->Data.SpadData.RefGoodSpadMap,
                Dev->Data.SpadData.RefSpadEnables,
                spadArraySize,
                startSelect,
                currentSpadIndex,
                count,
                &lastSpadIndex);

    if (Status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadCount, (uint8_t)(count));
        VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadType, isApertureSpads);
    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_reference_spads(VL53L0X_DEV Dev,
            uint32_t *pSpadCount, uint8_t *pIsApertureSpads)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t refSpadsInitialised;
    uint8_t refSpadArray[6];
    uint32_t cMaxSpadCount = 44;
    uint32_t cSpadArraySize = 6;
    uint32_t spadsEnabled;
    uint8_t isApertureSpads = 0;

    refSpadsInitialised = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
                    RefSpadsInitialised);

    if (refSpadsInitialised == 1) {

        *pSpadCount = (uint32_t)VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadCount);
        *pIsApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
            ReferenceSpadType);
    } else {

        /* obtain spad info from device.*/
        Status = get_ref_spad_map(Dev, refSpadArray);

        if (Status == VL53L0X_ERROR_NONE) {
            /* count enabled spads within spad map array and
             * determine if Aperture or Non-Aperture.
             */
            Status = count_enabled_spads(refSpadArray,
                            cSpadArraySize,
                            cMaxSpadCount,
                            &spadsEnabled,
                            &isApertureSpads);

            if (Status == VL53L0X_ERROR_NONE) {

                *pSpadCount = spadsEnabled;
                *pIsApertureSpads = isApertureSpads;

                VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    RefSpadsInitialised, 1);
                VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    ReferenceSpadCount,
                    (uint8_t)spadsEnabled);
                VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
                    ReferenceSpadType, isApertureSpads);
            }
        }
    }

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_perform_single_ref_calibration(VL53L0X_DEV Dev,
        uint8_t vhv_init_byte)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START,
                VL53L0X_REG_SYSRANGE_MODE_START_STOP |
                vhv_init_byte);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_measurement_poll_for_completion(Dev);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_ClearInterruptMask(Dev, 0);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSRANGE_START, 0x00);

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_ref_calibration_io(VL53L0X_DEV Dev,
    uint8_t read_not_write,
    uint8_t VhvSettings, uint8_t PhaseCal,
    uint8_t *pVhvSettings, uint8_t *pPhaseCal,
    const uint8_t vhv_enable, const uint8_t phase_enable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t PhaseCalint = 0;

    /* Read VHV from device */
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0x00, 0x00);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

    if (read_not_write) {
        if (vhv_enable)
            Status |= VL53L0X_RdByte(Dev, 0xCB, pVhvSettings);
        if (phase_enable)
            Status |= VL53L0X_RdByte(Dev, 0xEE, &PhaseCalint);
    } else {
        if (vhv_enable)
            Status |= VL53L0X_WrByte(Dev, 0xCB, VhvSettings);
        if (phase_enable)
            Status |= VL53L0X_UpdateByte(Dev, 0xEE, 0x80, PhaseCal);
    }

    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0x00, 0x01);
    Status |= VL53L0X_WrByte(Dev, 0xFF, 0x00);

    *pPhaseCal = (uint8_t)(PhaseCalint&0xEF);

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_perform_vhv_calibration(VL53L0X_DEV Dev,
    uint8_t *pVhvSettings, const uint8_t get_data_enable,
    const uint8_t restore_config)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;
    uint8_t PhaseCalInt = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    if (restore_config)
        SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    /* Run VHV */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_perform_single_ref_calibration(Dev, 0x40);

    /* Read VHV from device */
    if ((Status == VL53L0X_ERROR_NONE) && (get_data_enable == 1)) {
        Status = VL53L0X_ref_calibration_io(Dev, 1,
            VhvSettings, PhaseCal, /* Not used here */
            pVhvSettings, &PhaseCalInt,
            1, 0);
    } else
        *pVhvSettings = 0;


    if ((Status == VL53L0X_ERROR_NONE) && restore_config) {
        /* restore the previous Sequence Config */
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                SequenceConfig);
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, SequenceConfig);

    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev,
    uint8_t *pPhaseCal, const uint8_t get_data_enable,
    const uint8_t restore_config)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;
    uint8_t VhvSettingsint;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    if (restore_config)
        SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    /* Run PhaseCal */
    Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_perform_single_ref_calibration(Dev, 0x0);

    /* Read PhaseCal from device */
    if ((Status == VL53L0X_ERROR_NONE) && (get_data_enable == 1)) {
        Status = VL53L0X_ref_calibration_io(Dev, 1,
            VhvSettings, PhaseCal, /* Not used here */
            &VhvSettingsint, pPhaseCal,
            0, 1);
    } else
        *pPhaseCal = 0;


    if ((Status == VL53L0X_ERROR_NONE) && restore_config) {
        /* restore the previous Sequence Config */
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                SequenceConfig);
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, SequenceConfig);

    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev,
    uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t SequenceConfig = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    /* In the following function we don't save the config to optimize
     * writes on device. Config is saved and restored only once.
     */
    Status = VL53L0X_perform_vhv_calibration(
            Dev, pVhvSettings, get_data_enable, 0);


    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_perform_phase_calibration(
            Dev, pPhaseCal, get_data_enable, 0);


    if (Status == VL53L0X_ERROR_NONE) {
        /* restore the previous Sequence Config */
        Status = VL53L0X_WrByte(Dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                SequenceConfig);
        if (Status == VL53L0X_ERROR_NONE)
            PALDevDataSet(Dev, SequenceConfig, SequenceConfig);

    }

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_set_ref_calibration(VL53L0X_DEV Dev,
        uint8_t VhvSettings, uint8_t PhaseCal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t pVhvSettings;
    uint8_t pPhaseCal;

    Status = VL53L0X_ref_calibration_io(Dev, 0,
        VhvSettings, PhaseCal,
        &pVhvSettings, &pPhaseCal,
        1, 1);

    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_ref_calibration(VL53L0X_DEV Dev,
        uint8_t *pVhvSettings, uint8_t *pPhaseCal)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;

    Status = VL53L0X_ref_calibration_io(Dev, 1,
        VhvSettings, PhaseCal,
        pVhvSettings, pPhaseCal,
        1, 1);

    return Status;
}


// ====================================================================
//                            API STRINGS
// ====================================================================
VL53L0X_Error VL53L0xBase::VL53L0X_check_part_used(VL53L0X_DEV Dev,
        uint8_t *Revision,
        VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t ModuleIdInt;
    char *ProductId_tmp;

    LOG_FUNCTION_START("");

    Status = VL53L0X_get_info_from_device(Dev, 2);

    if (Status == VL53L0X_ERROR_NONE) {
        ModuleIdInt = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, ModuleId);

    if (ModuleIdInt == 0) {
        *Revision = 0;
        VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->ProductId, "");
    } else {
        *Revision = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, Revision);
        ProductId_tmp = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
            ProductId);
        VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->ProductId,
                   ProductId_tmp);
    }
    }

    LOG_FUNCTION_END(Status);
    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_get_device_info(VL53L0X_DEV Dev,
                VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t revision_id;
    uint8_t Revision;

    Status = VL53L0X_check_part_used(Dev, &Revision, pVL53L0X_DeviceInfo);

    if (Status == VL53L0X_ERROR_NONE) {
        if (Revision == 0) {
            VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->Name,
                    VL53L0X_STRING_DEVICE_INFO_NAME_TS0);
        } else if ((Revision <= 34) && (Revision != 32)) {
            VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->Name,
                    VL53L0X_STRING_DEVICE_INFO_NAME_TS1);
        } else if (Revision < 39) {
            VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->Name,
                    VL53L0X_STRING_DEVICE_INFO_NAME_TS2);
        } else {
            VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->Name,
                    VL53L0X_STRING_DEVICE_INFO_NAME_ES1);
        }

        VL53L0X_COPYSTRING(pVL53L0X_DeviceInfo->Type,
                VL53L0X_STRING_DEVICE_INFO_TYPE);

    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdByte(Dev,
                VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                &pVL53L0X_DeviceInfo->ProductType);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_RdByte(Dev,
            VL53L0X_REG_IDENTIFICATION_REVISION_ID,
                &revision_id);
        pVL53L0X_DeviceInfo->ProductRevisionMajor = 1;
        pVL53L0X_DeviceInfo->ProductRevisionMinor =
                    (revision_id & 0xF0) >> 4;
    }

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_get_device_error_string(VL53L0X_DeviceError ErrorCode,
        char *pDeviceErrorString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (ErrorCode) {
    case VL53L0X_DEVICEERROR_NONE:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_NONE);
    break;
    case VL53L0X_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE);
    break;
    case VL53L0X_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE);
    break;
    case VL53L0X_DEVICEERROR_NOVHVVALUEFOUND:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND);
    break;
    case VL53L0X_DEVICEERROR_MSRCNOTARGET:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET);
    break;
    case VL53L0X_DEVICEERROR_SNRCHECK:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_SNRCHECK);
    break;
    case VL53L0X_DEVICEERROR_RANGEPHASECHECK:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK);
    break;
    case VL53L0X_DEVICEERROR_SIGMATHRESHOLDCHECK:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK);
    break;
    case VL53L0X_DEVICEERROR_TCC:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_TCC);
    break;
    case VL53L0X_DEVICEERROR_PHASECONSISTENCY:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY);
    break;
    case VL53L0X_DEVICEERROR_MINCLIP:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_MINCLIP);
    break;
    case VL53L0X_DEVICEERROR_RANGECOMPLETE:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE);
    break;
    case VL53L0X_DEVICEERROR_ALGOUNDERFLOW:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW);
    break;
    case VL53L0X_DEVICEERROR_ALGOOVERFLOW:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW);
    break;
    case VL53L0X_DEVICEERROR_RANGEIGNORETHRESHOLD:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD);
    break;

    default:
        VL53L0X_COPYSTRING(pDeviceErrorString,
            VL53L0X_STRING_UNKNOW_ERROR_CODE);

    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_range_status_string(uint8_t RangeStatus,
        char *pRangeStatusString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (RangeStatus) {
    case 0:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_RANGEVALID);
    break;
    case 1:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_SIGMA);
    break;
    case 2:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_SIGNAL);
    break;
    case 3:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_MINRANGE);
    break;
    case 4:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_PHASE);
    break;
    case 5:
        VL53L0X_COPYSTRING(pRangeStatusString,
            VL53L0X_STRING_RANGESTATUS_HW);
    break;

    default: /**/
        VL53L0X_COPYSTRING(pRangeStatusString,
                VL53L0X_STRING_RANGESTATUS_NONE);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_pal_error_string(VL53L0X_Error PalErrorCode,
        char *pPalErrorString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (PalErrorCode) {
    case VL53L0X_ERROR_NONE:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_NONE);
    break;
    case VL53L0X_ERROR_CALIBRATION_WARNING:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_CALIBRATION_WARNING);
    break;
    case VL53L0X_ERROR_MIN_CLIPPED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_MIN_CLIPPED);
    break;
    case VL53L0X_ERROR_UNDEFINED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_UNDEFINED);
    break;
    case VL53L0X_ERROR_INVALID_PARAMS:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_INVALID_PARAMS);
    break;
    case VL53L0X_ERROR_NOT_SUPPORTED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_NOT_SUPPORTED);
    break;
    case VL53L0X_ERROR_INTERRUPT_NOT_CLEARED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED);
    break;
    case VL53L0X_ERROR_RANGE_ERROR:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_RANGE_ERROR);
    break;
    case VL53L0X_ERROR_TIME_OUT:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_TIME_OUT);
    break;
    case VL53L0X_ERROR_MODE_NOT_SUPPORTED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED);
    break;
    case VL53L0X_ERROR_BUFFER_TOO_SMALL:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL);
    break;
    case VL53L0X_ERROR_GPIO_NOT_EXISTING:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING);
    break;
    case VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
    break;
    case VL53L0X_ERROR_CONTROL_INTERFACE:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_CONTROL_INTERFACE);
    break;
    case VL53L0X_ERROR_INVALID_COMMAND:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_INVALID_COMMAND);
    break;
    case VL53L0X_ERROR_DIVISION_BY_ZERO:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_DIVISION_BY_ZERO);
    break;
    case VL53L0X_ERROR_REF_SPAD_INIT:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_REF_SPAD_INIT);
    break;
    case VL53L0X_ERROR_NOT_IMPLEMENTED:
        VL53L0X_COPYSTRING(pPalErrorString,
            VL53L0X_STRING_ERROR_NOT_IMPLEMENTED);
    break;

    default:
        VL53L0X_COPYSTRING(pPalErrorString,
                VL53L0X_STRING_UNKNOW_ERROR_CODE);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_pal_state_string(VL53L0X_State PalStateCode,
        char *pPalStateString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (PalStateCode) {
    case VL53L0X_STATE_POWERDOWN:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_POWERDOWN);
    break;
    case VL53L0X_STATE_WAIT_STATICINIT:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_WAIT_STATICINIT);
    break;
    case VL53L0X_STATE_STANDBY:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_STANDBY);
    break;
    case VL53L0X_STATE_IDLE:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_IDLE);
    break;
    case VL53L0X_STATE_RUNNING:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_RUNNING);
    break;
    case VL53L0X_STATE_UNKNOWN:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_UNKNOWN);
    break;
    case VL53L0X_STATE_ERROR:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_ERROR);
    break;

    default:
        VL53L0X_COPYSTRING(pPalStateString,
            VL53L0X_STRING_STATE_UNKNOWN);
    }

    LOG_FUNCTION_END(Status);
    return Status;
}

VL53L0X_Error VL53L0xBase::VL53L0X_get_sequence_steps_info(
        VL53L0X_SequenceStepId SequenceStepId,
        char *pSequenceStepsString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (SequenceStepId) {
    case VL53L0X_SEQUENCESTEP_TCC:
        VL53L0X_COPYSTRING(pSequenceStepsString,
            VL53L0X_STRING_SEQUENCESTEP_TCC);
    break;
    case VL53L0X_SEQUENCESTEP_DSS:
        VL53L0X_COPYSTRING(pSequenceStepsString,
            VL53L0X_STRING_SEQUENCESTEP_DSS);
    break;
    case VL53L0X_SEQUENCESTEP_MSRC:
        VL53L0X_COPYSTRING(pSequenceStepsString,
            VL53L0X_STRING_SEQUENCESTEP_MSRC);
    break;
    case VL53L0X_SEQUENCESTEP_PRE_RANGE:
        VL53L0X_COPYSTRING(pSequenceStepsString,
            VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE);
    break;
    case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
        VL53L0X_COPYSTRING(pSequenceStepsString,
            VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE);
    break;

    default:
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    LOG_FUNCTION_END(Status);

    return Status;
}


VL53L0X_Error VL53L0xBase::VL53L0X_get_limit_check_info(VL53L0X_DEV Dev,
    uint16_t LimitCheckId,
    char *pLimitCheckString)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    switch (LimitCheckId) {
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE);
    break;
    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);
    break;
    case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP);
    break;
    case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD);
    break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC);
    break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE);
    break;

    default:
        VL53L0X_COPYSTRING(pLimitCheckString,
            VL53L0X_STRING_UNKNOW_ERROR_CODE);

    }

    LOG_FUNCTION_END(Status);
    return Status;
}
