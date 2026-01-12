/*
 * ina219.c
 *
 *  Created on: Jan 2, 2026
 *      Author: khanh
 */
#include "ina219.h"
#include "stdio.h"
// --- Hàm hỗ trợ ghi/đọc I2C (Private) ---
static void INA219_WriteRegister(INA219_t *ina, uint8_t reg, uint16_t value) {
	uint8_t data[2];
	data[0] = (value >> 8) & 0xFF; // MSB
	data[1] = value & 0xFF;        // LSB
	HAL_I2C_Mem_Write(ina->hi2c, ina->address << 1, reg, I2C_MEMADD_SIZE_8BIT,
			data, 2, 100);

	HAL_Delay(1);
}

static int16_t INA219_ReadRegister(INA219_t *ina, uint8_t reg) {
	uint8_t data[2];
	if (HAL_I2C_Mem_Read(ina->hi2c, ina->address << 1, reg,
			I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK) {
		return 0;
	}
	return (int16_t) ((data[0] << 8) | data[1]);
}

// --- Implementation ---

void INA219_Init(INA219_t *ina, I2C_HandleTypeDef *hi2c, uint8_t addr) {
	ina->hi2c = hi2c;
	ina->address = addr;
	ina->ina219_currentDivider_mA = 0;
	ina->ina219_powerMultiplier_mW = 0.0f;
}

bool INA219_Begin(INA219_t *ina) {
	if (HAL_I2C_IsDeviceReady(ina->hi2c, ina->address << 1, 2, 100) != HAL_OK) {
		return false;
	}
	INA219_setCalibration_32V_2A(ina);
	return true;
}

void INA219_setCalibration_32V_2A(INA219_t *ina) {
	ina->ina219_calValue = 4096;
	ina->ina219_currentDivider_mA = 10;
	ina->ina219_powerMultiplier_mW = 2.0f;

	INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->ina219_calValue);

	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	INA219_WriteRegister(ina, INA219_REG_CONFIG, config);
}

void INA219_setCalibration_32V_1A(INA219_t *ina) {
	ina->ina219_calValue = 10240;
	ina->ina219_currentDivider_mA = 25;
	ina->ina219_powerMultiplier_mW = 0.8f;

	INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->ina219_calValue);

	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	INA219_WriteRegister(ina, INA219_REG_CONFIG, config);
}

void INA219_setCalibration_16V_400mA(INA219_t *ina) {
	ina->ina219_calValue = 8192;
	ina->ina219_currentDivider_mA = 20;
	ina->ina219_powerMultiplier_mW = 1.0f;

	INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->ina219_calValue);

	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	INA219_WriteRegister(ina, INA219_REG_CONFIG, config);
}

int16_t INA219_getBusVoltage_raw(INA219_t *ina) {
	uint16_t value = INA219_ReadRegister(ina, INA219_REG_BUSVOLTAGE);
	// Shift right 3 to drop CNVR and OVF and multiply by LSB (4mV)
	return (int16_t) ((value >> 3) * 4);
}

int16_t INA219_getShuntVoltage_raw(INA219_t *ina) {
	return INA219_ReadRegister(ina, INA219_REG_SHUNTVOLTAGE);
}

int16_t INA219_getCurrent_raw(INA219_t *ina) {
	// Write cal value explicitly before reading (as per original lib)
	//INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->ina219_calValue);
	return INA219_ReadRegister(ina, INA219_REG_CURRENT);
}

int16_t INA219_getPower_raw(INA219_t *ina) {
	// Write cal value explicitly before reading
	//INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->ina219_calValue);
	return INA219_ReadRegister(ina, INA219_REG_POWER);
}

float INA219_getBusVoltage_V(INA219_t *ina) {
	int16_t value = INA219_getBusVoltage_raw(ina);
	return value * 0.001f;
}

float INA219_getShuntVoltage_mV(INA219_t *ina) {
	int16_t value = INA219_getShuntVoltage_raw(ina);
	return value * 0.01f;
}

float INA219_getCurrent_mA(INA219_t *ina) {
	float valueDec = INA219_getCurrent_raw(ina);
	valueDec /= ina->ina219_currentDivider_mA;
	return valueDec;
}

float INA219_getPower_mW(INA219_t *ina) {
	float valueDec = INA219_getPower_raw(ina);
	valueDec *= ina->ina219_powerMultiplier_mW;
	return valueDec;
}

void INA219_powerSave(INA219_t *ina, bool on) {
	// Read current config
	uint16_t current_config = INA219_ReadRegister(ina, INA219_REG_CONFIG);

	// Clear mode bits
	current_config &= ~INA219_CONFIG_MODE_MASK;

	if (on) {
		current_config |= INA219_CONFIG_MODE_POWERDOWN;
	} else {
		current_config |= INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	}

	INA219_WriteRegister(ina, INA219_REG_CONFIG, current_config);
}

