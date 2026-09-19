/*
 * tmag5273.h
 *
 *  Created on: Sep 18, 2026
 *      Author: Will K
 */


/*
 *		Tasks:
 *      Test on hardware
 */


#ifndef TMAG5273_H_
#define TMAG5273_H_

#define TMAG5273_DEVICE_CONFIG_1        0x00

/* Bit meaning:
 * 		7 = CRC_EN;
 * 			0 disables CRC,
 * 			1 enables CRC
 * 		6-5 = MAG_TEMPCO; This is the Temperature coefficient of the magnet.
 * 			00 = 0% (No temp compensation),
 * 			01 = 0.12%/deg C (NdBFe),
 * 			10 = Reserved,
 * 			11 = 0.2%/deg C (Ceramic)
 * 		4-2 = CONV_AVG; Enables additional sampling of sensor data to reduce the noise effect (or to increase resolution)
 * 			000 = 1x average, 10.0-kSPS (3 axis) or 20-kSPS (1 axis)
 * 			001 = 2x average, 5.7-kSPS (3-axis) or 13.3-kSPS (1 axis)
 * 			010 = 4x average, 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
 * 			011 = 8x average, 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
 * 			100 = 16x average, 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
 * 			101 = 32x average, 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
 * 		1-0 = I2C_RD; Defines the I2C read mode
 * 			00 = Standard I2C 3-byte read command
 * 			01 = 1-byte I2C read command for 16bit sensor data and conversion status
 * 			10 =  1-byte I2C read command for 8 bit sensor MSB data and conversion status
 * 			11 = Reserved
 */
#define TMAG5273_DEVICE_CONFIG_2        0x01
/*
 * 		7-5 = THR_HYST; Select thresholds for the interrupt function
 * 			000 = Takes the 2's complement value of each x_THR_CONFIG register to create a magnetic threshold of the corresponding axis
 * 			001 = Takes the 7LSB bits of the x_THR_CONFIG register to create two opposite magnetic thresholds (one north, and another south) of equal magnitude.
 * 			010 = Reserved
 * 			011 = Reserved
 * 			100 = Reserved
 * 			101 = Reserved
 * 			110 = Reserved
 * 			111 = Reserved
 * 		4 = LP_LN; Selects the modes between low active current or low-noise modes
 * 			0 = Low active current mode
 * 			1 = Low noise mode
 * 		3 = I2C_GLITCH_FILTER; I2C glitch filter
 * 			0 = Glitch filter on
 * 			1 = Glitch filter off
 * 		2 = TRIGGER_MODE; Selects a condition which initiates a single conversion based off already configured registers. A running conversion completes before executing a trigger. Redundant triggers are ignored. TRIGGER_MODE is available only during the mode explicitly mentioned in OPERATING_MODE.
 *			0 = Conversion Start at I2C Command Bits, DEFAULT
 *			1 = Conversion starts through trigger signal at INT pin
 *		1-0 = OPERATING_MODE; Selects Operating Mode and updates value based on operating mode if device transitions from Wake-up and sleep mode to Standby mode.
 *			00 = Standby mode (starts new conversion at trigger event)
 *			01 = Sleep mode
 *			10 = Continuous measure mode
 *			11 = Wake-up and sleep mode (W&S mode)
 */
#define TMAG5273_SENSOR_CONFIG_1        0x02
/*		7-4 = MAG_CH_EN; Enables data acquisition of the magnetic axis channel(s)
 * 			0000 = All magnetic channels of off, DEFAULT
 * 			0001 = X channel enabled
 * 			0010 = Y channel enabled
 * 			0011 = X, Y channel enabled
 * 			0100 = Z channel enabled
 * 			0101 = Z, X channel enabled
 * 			0110 = Y, Z channel enabled
 * 			0111 = X, Y, Z channel enabled
 * 			1000 = XYX channel enabled
 * 			1001 = YXY channel enabled
 * 			1010 = YZY channel enabled
 * 			1011 = XZX channel enabled
 * 			1100 = Reserved
 * 			1101 = Reserved
 * 			1110 = Reserved
 * 			1111 = Reserved
 * 		3-0 = SLEEPTIME; Selects the time spent in low power mode between conversions when OPERATING_MODE =11b
 * 			0000 = 1ms
 * 			0001 = 5ms
 * 			0010 = 10ms
 * 			0011 = 15ms
 * 			0100 = 20ms
 * 			0101 = 30ms
 * 			0110 = 50ms
 * 			0111 = 100ms
 * 			1000 = 500ms
 * 			1001 = 1000ms
 * 			1010 = 2000ms
 * 			1011 = 5000ms
 * 			1100 = 20000ms
 */
#define TMAG5273_SENSOR_CONFIG_2        0x03
/*
 * 		7 = Reserved
 * 		6 = THRX_COUNT;  Number of threshold crossings before the interrupt is asserted
 * 			0 = 1 threshold crossing
 * 			1 = 4 threshold crossing
 * 		5 = MAG_THR_DIR; Selects the direction of threshold check. This bit is ignored when THR_HYST > 001b
 * 			0 = sets interrupt for field above the threshold
 * 			1 = sets interrupt for field below the threshold
 * 		4 = MAG_GAIN_CH; Selects the axis for magnitude gain correction value entered in MAG_GAIN_CONFIG register
 * 			0 = 1st channel is selected for gain adjustment
 * 			1 = 2nd channel is selected for gain adjustment
 * 		3-2 = ANGLE_EN;  Enables angle calculation, magnetic gain, and offset corrections between two selected magnetic channels
 * 			00 = No angle calculation, magnitude gain, and offset correction enabled
 * 			01 = X 1st, Y 2nd
 * 			10 = Y 1st, Z 2nd
 * 			11 = X 1st, Z 2nd
 * 		1 = X_Y_RANGE; Select the X and Y axes magnetic range from 2 different options.
 * 			0 =  ±40mT (TMAG5273A1) or ±133mT (TMAG5273A2), DEFAULT
 * 			1 = ±80mT (TMAG5273A1) or ±266mT (TMAG5273A2)
 * 		0 = Z_RANGE;  Select the Z axis magnetic range from 2 different options.
 * 			0 =  ±40mT (TMAG5273A1) or ±133mT (TMAG5273A2), DEFAULT
 * 			1 =  ±80mT (TMAG5273A1) or ±266mT (TMAG5273A2)
 */
#define TMAG5273_X_THR_CONFIG           0x04
/*
 * 		7-0 =  8-bit, 2's complement X axis threshold code for limit check.
 * 				The range of possible threshold entrees can be +/-128.
 * 				The threshold value in mT is calculated for A1 as (40(1+X_Y_RANGE)/128)*X_THR_CONFIG, for A2 as (133(1+X_Y_RANGE)/128)*X_THR_CONFIG.
 * 				Default 0h means no threshold comparison
 */
#define TMAG5273_Y_THR_CONFIG           0x05
/*
 * 		7-0 = Y_THR_CONFIG; 8-bit, 2's complement Y axis threshold code for limit check.
 * 				The range of possible threshold entrees can be +/-128. The threshold value in mT is calculated for A1 as (40(1+X_Y_RANGE)/128)*X_THR_CONFIG, for A2 as (133(1+X_Y_RANGE)/128)*X_THR_CONFIG.
 * 				Default 0h means no threshold comparison.
 */
#define TMAG5273_Z_THR_CONFIG           0x06
/*
 * 		7-0 = Z_THR_CONFIG;  8-bit, 2's complement Z axis threshold code for limit check.
 * 				The range of possible threshold entrees can be +/-128. The threshold value in mT is calculated for A1 as (40(1+Z_RANGE)/128)*Z_THR_CONFIG, for A2 as (133(1+Z_RANGE)/128)*Z_THR_CONFIG.
 * 				Default 0h means no threshold comparison.
 */
#define TMAG5273_T_CONFIG               0x07
/*
 * 		7-1 = T_THR_CONFIG;  Temperature threshold code entered by user.
 * 				The valid temperature threshold ranges are -41C to 170C with the threshold codes for -41C = 1Ah, and 170C = 34h.
 * 				Resolution is 8 degree C/ LSB. Default 0h means no threshold comparison.
 * 		0 = T_CH_EN;  Enables data acquisition of the temperature channel
 * 			0 = Temp channel disabled
 * 			1 = Temp channel enabled
 */
#define TMAG5273_INT_CONFIG_1           0x08
/*
 * 		7 = RSLT_INT;  Enable interrupt response on conversion complete
 * 			0 =  Interrupt is not asserted when the configured set of conversions are complete.
 * 			1 =  Interrupt is asserted when the configured set of conversions are complete.
 * 		6 = THRSLD_INT;  Enable interrupt response on a predefined threshold cross.
 * 			0 = Interrupt is not asserted when a threshold is crossed
 * 			1 = Interrupt is asserted when a threshold is crossed
 * 		5 = INT_STATE;  INT interrupt latched or pulsed.
 * 			0 =  INT interrupt latched until clear by a primary addressing the device.
 * 			1 =  INT interrupt pulse for 10us
 * 		4-2 = INT_MODE; Interrupt mode select.
 * 			000 = No interrupt
 * 			001 = Interrupt through INT
 * 			010 = Interrupt through INT except when I2C bus is busy
 * 			011 = Interrupt through SCL
 * 			100 = Interrupt through SCL except when I2C bus is busy
 * 			101 = Reserved
 * 			110 = Reserved
 * 			111 = Reserved
* 		1 = Reserved
* 		0 = MASK_INTB;  Mask INT pin when INT connected to GND
* 			0 = INT pin is enabled
* 			1 = INT pin is disabled (for wake-up and trigger functions)
 */
#define TMAG5273_MAG_GAIN_CONFIG        0x09
/*
 * 		7-0 = GAIN_VALUE;  8-bit gain value determined by a primary to adjust a Hall axis gain.
 * 				The particular axis is selected based off the settings of MAG_GAIN_CH and ANGLE_EN register bits.
 * 				The binary 8-bit input is interpreted as a fractional value in between 0 and 1 based off the formula, 'user entered value in decimal/256'.
 * 				Gain value of 0 is interpreted by the device as 1.
 */
#define TMAG5273_MAG_OFFSET_CONFIG_1    0x0A
/*
 * 		7-0 = OFFSET_VALUE_1ST; 8-bit, 2's complement offset value determined by a primary to adjust first axis offset value.
 * 				The range of possible offset valid entrees can be +/-128.
 * 				The offset value is calculated by multiplying bit resolution with the entered value.
 */
#define TMAG5273_MAG_OFFSET_CONFIG_2    0x0B
/*
 * 		7-0 = OFFSET_VALUE_2ND; 8-bit, 2's complement offset value determined by a primary to adjust second axis offset value.
 * 				The range of possible offset valid entrees can be +/-128.
 * 				The offset value is calculated by multiplying bit resolution with the entered value.
 */
#define TMAG5273_I2C_ADDRESS            0x0C
/*
 * 		7-1 = I2C_ADDRESS; 7-bit default factory I2C address is loaded from OTP during first power up.
 * 				Change these bits to a new setting if a new I2C address is required (at each power cycle these bits must be written again to avoid going back to default factory address).
 * 				Reset = 0100011 (everything else has been 0)
 * 		0 = I2C_ADDRESS_UPDATE_EN; Enable a new user defined I2C address.
 * 			0 = Disable update of I2C address
 * 			1 = Enable update of I2C address with bits (7:1)
 */
#define TMAG5273_DEVICE_ID              0x0D
/*
 * 		7-2 = Reserved
 * 		1-0 = VER; Device version indicator. Reset value of DEVICE_ID depends on the orderable part number.
 * 			00 = Reserved
 * 			01 = ±40mT and ±80mT range
 * 			10 = ±133mT and ±266mT range
 * 			11 = Reserved
 */
#define TMAG5273_MANUFACTURER_ID_LSB    0x0E
/*
 * 		7-0 = MANUFACTURER_ID_[7:0]; 8-bit unique manufacturer ID
 * 				Reset = 0110001
 */
#define TMAG5273_MANUFACTURER_ID_MSB    0x0F
/*
 * 		7-0 = MANUFACTURER_ID_[15:8]; 8-bit unique manufacturer ID
 * 				Reset = 0110110
 */


#define TMAG5273_T_MSB_RESULT           0x10
/*
 * 		7-0 = T_CH_RESULT [15:8]; T-channel data conversion results, MSB 8 bits.
 */
#define TMAG5273_T_LSB_RESULT           0x11
/*
 * 		7-0 = T_CH_RESULT [7:0]; T-channel data conversion results, LSB 8 bits.
 */
#define TMAG5273_X_MSB_RESULT           0x12
/*
 * 		7-0 = X_CH_RESULT [15:8]; X-channel data conversion results, MSB 8 bits.
 */
#define TMAG5273_X_LSB_RESULT           0x13
/*
 * 		7-0 = X_CH_RESULT [7:0]; X-channel data conversion results, LSB 8 bits.
 */
#define TMAG5273_Y_MSB_RESULT           0x14
/*
 * 		7-0 = Y_CH_RESULT [15:8]; Y-channel data conversion results, MSB 8 bits.
 */
#define TMAG5273_Y_LSB_RESULT           0x15
/*
 * 		7-0 = Y_CH_RESULT [7:0]; Y-channel data conversion results, LSB 8 bits.
 */
#define TMAG5273_Z_MSB_RESULT           0x16
/*
 * 		7-0 = Z_CH_RESULT [15:8]; Z-channel data conversion results, MSB 8 bits.
 */
#define TMAG5273_Z_LSB_RESULT           0x17
/*
 * 		7-0 = Z_CH_RESULT [7:0]; Z-channel data conversion results, LSB 8 bits.
 */
#define TMAG5273_CONV_STATUS            0x18
/*
 * 		7-5 = SET_COUNT; Rolling Count of Conversion Data Sets
 * 		4 = POR; Device powered up, or experienced power-on-reset. Bit is clear when host writes back 1.
 * 			0 = NO POR
 * 			1 = POR occurred
 * 			Reset = 1h
 * 		3-2 = Reserved
 * 		1 = DIAG_STATUS; Detect any internal diagnostics fail which include VCC UV, internal memory CRC error, INT pin error and internal clock error.
 * 			0 = No diag fail
 * 			1 = Diag fail detected
 * 		0 = RESULT_STATUS; Conversion data buffer is ready to be read.
 * 			0 = Conversion data not complete
 * 			1 = Conversion data complete
 */
#define TMAG5273_ANGLE_RESULT_MSB       0x19
/*
 * 		7-0 = ANGLE_RESULT_MSB; Angle measurement result in degree.
 * 				The data is displayed from 0 to 360 degree in 13LSB bits after combining the ANGLE_RESULT_MSB and _LSB bits.
 * 				The 4 LSB bits allocated for fraction of an angle in the format (xxxx/16).
 */
#define TMAG5273_ANGLE_RESULT_LSB       0x1A
/*
 * 		7-0 = ANGLE_RESULT_LSB;  Angle measurement result in degree.
 * 				The data is displayed from 0 to 360 degree in 13LSB bits after combining the ANGLE_RESULT_MSB and _LSB bits.
 * 				The 4 LSB bits allocated for fraction of an angle in the format (xxxx/16).
 */
#define TMAG5273_MAGNITUDE_RESULT       0x1B
/*
 * 		7-0 = MAGNITUDE_RESULT; Resultant vector magnitude (during angle measurement) result.
 * 				This value should be constant during 360 degree measurements
 */
#define TMAG5273_DEVICE_STATUS          0x1C
/*
 * 		7-5 = Reserved
 * 		4 = INTB_RB;  Indicates the level that the device is reading back from INT pin.
 * 				The reset value of DEVICE_STATUS depends on the status of the INT pin at power-up.
 * 			0 = INT pin driven low
 * 			1 = INT pin status high
 * 		3 = OSC_ER (R/W1CP);  Indicates if Oscillator error is detected.
 * 				Bit is clear when host writes back 1.
 * 			0 = No Oscillator error detected
 * 			1 = Oscillator error detected
 * 		2 = INT_ER (R/W1CP);  Indicates if INT pin error is detected.
 * 				Bit is clear when host writes back 1.
 * 			0 = No INT error detected
 * 			1 = INT error detected
 * 		1 = OTP_CRC_ER (R/W1CP); Indicates if OTP CRC error is detected.
 * 				Bit is clear when host writes back 1.
 * 			0 = No OTP CRC error detected
 * 			1 = OTP CRC error detected
 * 		0 = VCC_UV_ER (R/W1CP); Indicates if VCC undervoltage was detected.
 * 				Bit is clear when host writes back 1.
 * 			0 = No VCC UV detected
 * 			1 = VCC UV detected
 */
#endif /* TMAG5273_H_ */
