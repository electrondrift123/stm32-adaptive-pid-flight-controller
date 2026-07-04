#include "bmp280.h"

// Calibration data
static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t dig_P6, dig_P7, dig_P8, dig_P9;

// Altitude reference (startup average)
static float altitudeReference = 0;

// Raw altitude from last reading
static float rawAltitude = 0;

// Read calibration data from sensor
static bool readCalibrationData(I2C_HandleTypeDef *hi2c1){
    uint8_t data[24];
    uint8_t reg_addr = 0x88;

    if (HAL_I2C_Master_Transmit(hi2c1, BMP280_ADDRESS << 1, &reg_addr, 1, 100) != HAL_OK) return false;

    if (HAL_I2C_Master_Receive(hi2c1, BMP280_ADDRESS << 1, data, 24, 100) != HAL_OK) return false;
    
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
    
    return true;
}

// Read raw pressure and temperature from sensor
static bool readRawData(I2C_HandleTypeDef *hi2c1, uint32_t *adc_P, uint32_t *adc_T) {
    uint8_t data[6];
    uint8_t reg_addr = 0xF7;

    if (HAL_I2C_Mem_Read(hi2c1, BMP280_ADDRESS << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK) return false;

    uint32_t press_msb = data[0];
    uint32_t press_lsb = data[1];
    uint32_t press_xlsb = data[2];
    uint32_t temp_msb = data[3];
    uint32_t temp_lsb = data[4];
    uint32_t temp_xlsb = data[5];
    
    *adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    *adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
    
    return true;
}

// Calculate compensated pressure and temperature
static bool calculatePressureTemp(uint32_t adc_P, uint32_t adc_T, float *pressure, float *temperature) {
    signed long int var1, var2;
    signed long int t_fine;
    
    // Calculate temperature
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature = *temperature / 100.0;
    
    // Calculate pressure
    unsigned long int p;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    
    if (var1 == 0) {
        return false;
    }
    
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((unsigned long int)var1);
    } else {
        p = (p / (unsigned long int)var1) * 2;
    }
    
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
    
    *pressure = (double)p / 100.0; // Convert to hPa
    
    return true;
}

// Calculate altitude from pressure in cm
static float calculateAltitude(float pressure_hPa) {
    // Sea level pressure = 1013.25 hPa
    // Returns altitude in m
    return 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 1.0 / 5.255));
}

// Update sensor reading and store raw altitude
static bool updateSensorReading(I2C_HandleTypeDef *hi2c1) {
    uint32_t adc_P, adc_T;
    float pressure, temperature;
    
    if (!readRawData(hi2c1, &adc_P, &adc_T)) return false;
    if (!calculatePressureTemp(adc_P, adc_T, &pressure, &temperature)) return false;
    
    rawAltitude = calculateAltitude(pressure);
    return true;
}

bool bmp280_init(I2C_HandleTypeDef *hi2c1) { 
    uint8_t data[2];

    // Configure sensor
    data[0] = 0xF4; // reg addr
    data[1] = 0x57; // data write: oversampling x16, normal mode
    if (HAL_I2C_Mem_Write(hi2c1, BMP280_ADDRESS << 1, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, 100) != HAL_OK) return false;

    data[0] = 0xF5;
    data[1] = 0x10;
    if (HAL_I2C_Mem_Write(hi2c1, BMP280_ADDRESS << 1, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, 100) != HAL_OK) return false;
    
    // Read calibration data
    HAL_Delay(250);
    if (!readCalibrationData(hi2c1)) return false;
    
    // Calibrate altitude reference by averaging 2000 readings
    HAL_Delay(250);
    float altitudeSum = 0;
    for (int i = 0; i < 2000; i++) {
        if (!updateSensorReading(hi2c1)) return false;
        altitudeSum += rawAltitude;
        HAL_Delay(1);
    }
    altitudeReference = altitudeSum / 2000.0;
    
    return true;
}

bool bmp280_read(I2C_HandleTypeDef *hi2c1, bmp280Data_t *data) {
    if (data == NULL) return false;
    
    uint32_t adc_P, adc_T;
    float pressure, temperature;
    
    // Read raw data
    if (!readRawData(hi2c1, &adc_P, &adc_T)) return false; // is this correct?
    
    // Calculate pressure and temperature
    if (!calculatePressureTemp(adc_P, adc_T, &pressure, &temperature)) return false;
    
    // Calculate altitude
    float altitude_cm = calculateAltitude(pressure);
    float relative_altitude_cm = altitude_cm - altitudeReference;
    
    // Fill data structure
    data->pressure = pressure;
    data->temperature = temperature;
    data->altitude = relative_altitude_cm;
    
    // Store raw altitude for reference functions
    // rawAltitude = altitude_cm;
    // output in meters:
    rawAltitude = altitude_cm / 100.00f;
    
    return true;
}

void bmp280_reset_altitude_reference(void) {
    altitudeReference = rawAltitude;
}