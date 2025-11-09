/**
 * @file OtosXYTetaBase.cpp
 * @brief Implementation of the SparkFun Qwiic OTOS driver class
 * @details This file contains the implementation of methods for the OtosXYTetaBase class,
 *          which provides a C++ interface for the SparkFun Qwiic Optical Tracking
 *          Odometry Sensor (OTOS).
 *
 * Implementation Features:
 * - Platform-agnostic I2C communication using SparkFun Toolkit
 * - Unit conversion between metric/imperial and radians/degrees
 * - Register-level device interaction
 * - Data conversion between raw register values and floating point numbers
 * - Error handling and validation
 *
 * @author SparkFun Electronics
 * @date February 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_Toolkit
 */

/*******************************************************************************
    OtosXYTetaBase.h - C++ driver implementation for the SparkFun Qwiic Optical
    Tracking Odometry Sensor (OTOS).
    Modified for generic usage -> OTOS_XYTeta_Base.h / .cpp
*******************************************************************************/

#include "OTOS_XYTeta_Base.h"

OtosXYTetaBase::OtosXYTetaBase()
    : _linearUnit{kSfeOtosLinearUnitInches}, _angularUnit{kSfeOtosAngularUnitDegrees},
      _meterToUnit{kMeterToInch}, _radToUnit{kRadianToDegree}
{
    // Nothing to do here!
}


otos_error_t OtosXYTetaBase::isConnected()
{
    // Read the product ID
    uint8_t prodId;
    otos_error_t err = readRegister(kRegProductId, prodId);
    if (err != Otos_OK)
        return err;

    // Check if the product ID is correct
    if (prodId != kProductId)
        return Otos_ErrFail;

    // Everything checks out, we must be connected!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::getVersionInfo(otos_version_t &hwVersion, otos_version_t &fwVersion)
{
    // Read hardware and firmware version registers
    uint8_t rawData[2];
    uint16_t readBytes;
    otos_error_t err = readRegister(kRegHwVersion, rawData, sizeof(rawData), readBytes);
    if (err != Otos_OK)
        return err;

    // Check if we read the correct number of bytes
    if (readBytes != 2)
        return Otos_ErrFail;

    // Store the version info
    hwVersion.value = rawData[0];
    fwVersion.value = rawData[1];

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::selfTest()
{
    // Write the self-test register to start the test
    otos_self_test_config_t selfTest;
    selfTest.start = 1;
    otos_error_t err = writeRegister(kRegSelfTest, selfTest.value);
    if (err != Otos_OK)
        return err;

    // Loop until self-test is done, should only take ~20ms as of firmware v1.0
    for (int i = 0; i < 10; i++)
    {
        // Give a short delay between reads
        delayMs(5);

        // Read the self-test register
        err = readRegister(kRegSelfTest, selfTest.value);
        if (err != Otos_OK)
            return err;

        // Check if the self-test is done
        if (selfTest.inProgress == 0)
        {
            break;
        }
    }

    // Check if the self-test passed
    return (selfTest.pass == 1) ? Otos_OK : Otos_ErrFail;
}

otos_error_t OtosXYTetaBase::calibrateImu(uint8_t numSamples, bool waitUntilDone)
{
    // Write the number of samples to the device
    otos_error_t err = writeRegister(kRegImuCalib, numSamples);
    if (err != Otos_OK)
        return err;

    // Wait 1 sample period (2.4ms) to ensure the register updates
    delayMs(3);

    // Do we need to wait until the calibration finishes?
    if (!waitUntilDone)
        return Otos_OK;

    // Wait for the calibration to finish, which is indicated by the IMU
    // calibration register reading zero, or until we reach the maximum number
    // of read attempts
    for (uint8_t numAttempts = numSamples; numAttempts > 0; numAttempts--)
    {
        // Read the gryo calibration register value
        uint8_t calibrationValue;
        err = readRegister(kRegImuCalib, calibrationValue);
        if (err != Otos_OK)
            return err;

        // Check if calibration is done
        if (calibrationValue == 0)
            return Otos_OK;

        // Give a short delay between reads. As of firmware v1.0, samples take
        // 2.4ms each, so 3ms should guarantee the next sample is done. This
        // also ensures the max attempts is not exceeded in normal operation
        delayMs(3);
    }

    // Max number of attempts reached, calibration failed
    return Otos_ErrFail;
}

otos_error_t OtosXYTetaBase::getImuCalibrationProgress(uint8_t &numSamples)
{
    // Read the IMU calibration register
    return readRegister(kRegImuCalib, numSamples);
}

otos_linear_unit_t OtosXYTetaBase::getLinearUnit()
{
    return _linearUnit;
}

void OtosXYTetaBase::setLinearUnit(otos_linear_unit_t unit)
{
    // Check if this unit is already set
    if (unit == _linearUnit)
        return;

    // Store new unit
    _linearUnit = unit;

    // Compute conversion factor to new units
    _meterToUnit = (unit == kSfeOtosLinearUnitMeters) ? 1.0f : kMeterToInch;
}

otos_angular_unit_t OtosXYTetaBase::getAngularUnit()
{
    return _angularUnit;
}

void OtosXYTetaBase::setAngularUnit(otos_angular_unit_t unit)
{
    // Check if this unit is already set
    if (unit == _angularUnit)
        return;

    // Store new unit
    _angularUnit = unit;

    // Compute conversion factor to new units
    _radToUnit = (unit == kSfeOtosAngularUnitRadians) ? 1.0f : kRadianToDegree;
}

otos_error_t OtosXYTetaBase::getLinearScalar(float &scalar)
{
    // Read the linear scalar from the device
    uint8_t rawScalar;
    otos_error_t err = readRegister(kRegScalarLinear, rawScalar);
    if (err != Otos_OK)
        return Otos_ErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::setLinearScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if (scalar < kMinScalar || scalar > kMaxScalar)
        return Otos_ErrFail;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return writeRegister(kRegScalarLinear, rawScalar);  // TODO : revoir le code d'erreur de retour
}

otos_error_t OtosXYTetaBase::getAngularScalar(float &scalar)
{
    // Read the angular scalar from the device
    uint8_t rawScalar;
    otos_error_t err = readRegister(kRegScalarAngular, rawScalar);
    if (err != Otos_OK)
        return Otos_ErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::setAngularScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if (scalar < kMinScalar || scalar > kMaxScalar)
        return Otos_ErrFail;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return writeRegister(kRegScalarAngular, rawScalar);
}

otos_error_t OtosXYTetaBase::resetTracking()
{
    // Set tracking reset bit
    return writeRegister(kRegReset, (uint8_t)0x01);
}

otos_error_t OtosXYTetaBase::getSignalProcessConfig(otos_signal_process_config_t &config)
{
    // Read the signal process register
    return readRegister(kRegSignalProcess, config.value);
}

otos_error_t OtosXYTetaBase::setSignalProcessConfig(otos_signal_process_config_t &config)
{
    // Write the signal process register
    return writeRegister(kRegSignalProcess, config.value);
}

otos_error_t OtosXYTetaBase::getStatus(otos_status_t &status)
{
    return readRegister(kRegStatus, status.value);
}

otos_error_t OtosXYTetaBase::getOffset(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegOffXL, pose, kInt16ToMeter, kInt16ToRad);
}

otos_error_t OtosXYTetaBase::setOffset(otos_pose2d_t &pose)
{
    return writePoseRegs(kRegOffXL, pose, kMeterToInt16, kRadToInt16);
}

otos_error_t OtosXYTetaBase::getPosition(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosXL, pose, kInt16ToMeter, kInt16ToRad);
}

otos_error_t OtosXYTetaBase::setPosition(otos_pose2d_t &pose)
{
    return writePoseRegs(kRegPosXL, pose, kMeterToInt16, kRadToInt16);
}

otos_error_t OtosXYTetaBase::getVelocity(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelXL, pose, kInt16ToMps, kInt16ToRps);
}

otos_error_t OtosXYTetaBase::getAcceleration(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccXL, pose, kInt16ToMpss, kInt16ToRpss);
}

otos_error_t OtosXYTetaBase::getPositionStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosStdXL, pose, kInt16ToMeter, kInt16ToRad);
}

otos_error_t OtosXYTetaBase::getVelocityStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelStdXL, pose, kInt16ToMps, kInt16ToRps);
}

otos_error_t OtosXYTetaBase::getAccelerationStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccStdXL, pose, kInt16ToMpss, kInt16ToRpss);
}

otos_error_t OtosXYTetaBase::getPosVelAcc(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    uint16_t bytesRead;
    otos_error_t err = readRegister(kRegPosXL, rawData, 18, bytesRead);
    if (err != Otos_OK)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 18)
        return Otos_ErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::getPosVelAccStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    uint16_t bytesRead;
    otos_error_t err = readRegister(kRegPosStdXL, rawData, 18, bytesRead);
    if (err != Otos_OK)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 18)
        return Otos_ErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::getPosVelAccAndStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc,
                                             otos_pose2d_t &posStdDev, otos_pose2d_t &velStdDev,
                                             otos_pose2d_t &accStdDev)
{
    // Read all pose registers
    uint8_t rawData[36];
    uint16_t bytesRead;
    otos_error_t err = readRegister(kRegPosXL, rawData, 36, bytesRead);
    if (err != Otos_OK)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 36)
        return Otos_ErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);
    regsToPose(rawData + 18, posStdDev, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 24, velStdDev, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 30, accStdDev, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::readPoseRegs(uint8_t reg, otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    uint16_t bytesRead;
    uint8_t rawData[6];

    // Attempt to read the raw pose data
    otos_error_t err = readRegister(reg, rawData, 6, bytesRead);
    if (err != Otos_OK)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 6)
        return Otos_ErrFail;

    regsToPose(rawData, pose, rawToXY, rawToH);

    // Done!
    return Otos_OK;
}

otos_error_t OtosXYTetaBase::writePoseRegs(uint8_t reg, otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Store raw data in a temporary buffer
    uint8_t rawData[6];
    poseToRegs(rawData, pose, xyToRaw, hToRaw);

    // Write the raw data to the device
    return writeRegister(reg, rawData, 6);
}

void OtosXYTetaBase::regsToPose(uint8_t *rawData, otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    // Store raw data
    int16_t rawX = (rawData[1] << 8) | rawData[0];
    int16_t rawY = (rawData[3] << 8) | rawData[2];
    int16_t rawH = (rawData[5] << 8) | rawData[4];

    // Store in pose and convert to units
    pose.x = rawX * rawToXY * _meterToUnit;
    pose.y = rawY * rawToXY * _meterToUnit;
    pose.h = rawH * rawToH * _radToUnit;
}

void OtosXYTetaBase::poseToRegs(uint8_t *rawData, otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Convert pose units to raw data
    int16_t rawX = pose.x * xyToRaw / _meterToUnit;
    int16_t rawY = pose.y * xyToRaw / _meterToUnit;
    int16_t rawH = pose.h * hToRaw / _radToUnit;

    // Store raw data in buffer
    rawData[0] = rawX & 0xFF;
    rawData[1] = (rawX >> 8) & 0xFF;
    rawData[2] = rawY & 0xFF;
    rawData[3] = (rawY >> 8) & 0xFF;
    rawData[4] = rawH & 0xFF;
    rawData[5] = (rawH >> 8) & 0xFF;
}

otos_error_t OtosXYTetaBase::writeRegister(uint8_t reg, uint8_t value)
{
	return writeRegister(reg, &value , 1);
}

otos_error_t OtosXYTetaBase::readRegister(uint8_t reg, uint8_t &value)
{
	uint16_t byte_read;
	otos_error_t err = readRegister(reg, &value, 1, byte_read);
	if (byte_read != 1) return Otos_ErrFail;
	return err;
}


