/*
 * Adafruit_ADXL345_Bits.h
 *
 * Created: 5/7/2014 11:26:01 AM
 *  Author: Michael D Pirie
 */


#ifndef SENSOR_REGISTERS_BITS_H_
#define SENSOR_REGISTERS_BITS_H_
/*==================================================================================================
   General convenience
   ----------------------------------------------------------------------------------------------*/
   enum interface_mode
   {
	   MODE_SPI,
	   MODE_I2C,
   };
/*================================================================================================*/
/*==================================================================================================
   ADXL345
   ----------------------------------------------------------------------------------------------*/
	#define ADXL_INT1    0
	#define ADXL_INT2    1

	#define ENABLE  1
	#define DISABLE 0

	#define ADXL_X_AXIS  0
	#define ADXL_Y_AXIS  1
	#define ADXL_Z_AXIS  2

	#define ADXL_FIFO_BYPASS  0
	#define ADXL_FIFO_FIFO    1
	#define ADXL_FIFO_STREAM  2
	#define ADXL_FIFO_TRIGGER 3
	// INT_ENABLE & INT_MAP & IN_SOURCE register
	#define ADXL_DATA_READY	 7
	#define ADXL_SINGLE_TAP	 6
	#define ADXL_DOUBLE_TAP  5
	#define ADXL_ACTIVITY    4
	#define ADXL_INACTIVITY	 3
	#define ADXL_FREE_FALL   2
	#define ADXL_WATERMARK   1
	#define ADXL_OVERRUN     0
	// POWER_CTL register
	#define ADXL_LINK        5
	#define ADXL_AUTO_SLEEP  4
	#define ADXL_MEASURE     3
	#define ADXL_SLEEP       2
	#define ADXL_WAKEUP_D1   1
	#define ADXL_WAKEUP_D0   0
	// ACT_INACT_CTL register
	#define ADXL_ATCITIVITY_ACDC 7
	#define ADXL_ACT_X_ENABLE    6
	#define ADXL_ACT_Y_ENABLE    5
	#define ADXL_ACT_Z_ENABLE    4
	#define ADXL_INACTIVITY_ACDC 3
	#define ADXL_INACT_X_ENABLE  2
	#define ADXL_INACT_Y_ENABLE  1
	#define ADXL_INACT_Z_ENABLE  0
	// TAP_AXES register
	#define ADXL_DBL_TAP_SUPPRESS 3
	#define ADXL_TAP_X_ENABLE     2
	#define ADXL_TAP_Y_ENABLE     1
	#define ADXL_TAP_Z_ENABLE     0
	// ACT_TAP_STATUS register
	#define ADXL_ACT_X_SOURCE    6
	#define ADXL_ACT_Y_SOURCE    5
	#define ADXL_ACT_Z_SOURCE    4
	#define ADXL_ASLEEP		     3
	#define ADXL_TAP_X_SOURCE    2
	#define ADXL_TAP_Y_SOURCE    1
	#define ADXL_TAP_Z_SOURCE    0
	// DATA_FORMAT register
	#define ADXL_SELF_TEST  7
	#define ADXL_SPI_3_WIRE 6
	#define ADXL_INT_INVERT 5
	#define ADXL_FULL_RES   3
	#define ADXL_JUSTIFY    2
	#define ADXL_RANGE_D1   1
	#define ADXL_RANGE_D2   0
	//
/*================================================================================================*/
/*================================================================================================
   LSM9DS0
   -----------------------------------------------------------------------------------------------*/
	/////// Gyro Registers ///////
   #define LSM_GYRO_REG_WHO_AM_I_G			0x0F
   #define LSM_GYRO_REG_CTRL_REG1_G			0x20
   #define LSM_GYRO_REG_CTRL_REG2_G			0x21
   #define LSM_GYRO_REG_CTRL_REG3_G			0x22
   #define LSM_GYRO_REG_CTRL_REG4_G			0x23
   #define LSM_GYRO_REG_CTRL_REG5_G			0x24
   #define LSM_GYRO_REG_REFERENCE_G			0x25
   #define LSM_GYRO_REG_STATUS_REG_G		0x27
   #define LSM_GYRO_REG_OUT_X_L_G			0x28
   #define LSM_GYRO_REG_OUT_X_H_G			0x29
   #define LSM_GYRO_REG_OUT_Y_L_G			0x2A
   #define LSM_GYRO_REG_OUT_Y_H_G			0x2B
   #define LSM_GYRO_REG_OUT_Z_L_G			0x2C
   #define LSM_GYRO_REG_OUT_Z_H_G			0x2D
   #define LSM_GYRO_REG_FIFO_CTRL_REG_G     0x2E
   #define LSM_GYRO_REG_FIFO_SRC_REG_G		0x2F
   #define LSM_GYRO_REG_INT1_CFG_G			0x30
   #define LSM_GYRO_REG_INT1_SRC_G			0x31
   #define LSM_GYRO_REG_INT1_THS_XH_G		0x32
   #define LSM_GYRO_REG_INT1_THS_XL_G		0x33
   #define LSM_GYRO_REG_INT1_THS_YH_G		0x34
   #define LSM_GYRO_REG_INT1_THS_YL_G		0x35
   #define LSM_GYRO_REG_INT1_THS_ZH_G		0x36
   #define LSM_GYRO_REG_INT1_THS_ZL_G		0x37
   #define LSM_GYRO_REG_INT1_DURATION_G	    0x38
	// Accel, Magneto Registers //
	#define LSM_MAG_REG_OUT_TEMP_L_XM		0x05
	#define LSM_MAG_REG_OUT_TEMP_H_XM		0x06
	#define LSM_MAG_REG_STATUS_REG_M		0x07
	#define LSM_MAG_REG_OUT_X_L_M			0x08
	#define LSM_MAG_REG_OUT_X_H_M			0x09
	#define LSM_MAG_REG_OUT_Y_L_M			0x0A
	#define LSM_MAG_REG_OUT_Y_H_M			0x0B
	#define LSM_MAG_REG_OUT_Z_L_M			0x0C
	#define LSM_MAG_REG_OUT_Z_H_M			0x0D
	#define LSM_MAG_REG_WHO_AM_I_XM			0x0F
	#define LSM_MAG_REG_INT_CTRL_REG_M		0x12
	#define LSM_MAG_REG_INT_SRC_REG_M		0x13
	#define LSM_MAG_REG_INT_THS_L_M			0x14
	#define LSM_MAG_REG_INT_THS_H_M			0x15
	#define LSM_MAG_REG_OFFSET_X_L_M		0x16
	#define LSM_MAG_REG_OFFSET_X_H_M		0x17
	#define LSM_MAG_REG_OFFSET_Y_L_M		0x18
	#define LSM_MAG_REG_OFFSET_Y_H_M		0x19
	#define LSM_MAG_REG_OFFSET_Z_L_M		0x1A
	#define LSM_MAG_REG_OFFSET_Z_H_M		0x1B
	#define LSM_MAG_REG_REFERENCE_X			0x1C
	#define LSM_MAG_REG_REFERENCE_Y			0x1D
	#define LSM_MAG_REG_REFERENCE_Z			0x1E
	#define LSM_MAG_REG_CTRL_REG0_XM		0x1F
	#define LSM_MAG_REG_CTRL_REG1_XM		0x20
	#define LSM_MAG_REG_CTRL_REG2_XM		0x21
	#define LSM_MAG_REG_CTRL_REG3_XM		0x22
	#define LSM_MAG_REG_CTRL_REG4_XM		0x23
	#define LSM_MAG_REG_CTRL_REG5_XM		0x24
	#define LSM_MAG_REG_CTRL_REG6_XM		0x25
	#define LSM_MAG_REG_CTRL_REG7_XM		0x26
	#define LSM_MAG_REG_STATUS_REG_A		0x27
	#define LSM_MAG_REG_OUT_X_L_A			0x28
	#define LSM_MAG_REG_OUT_X_H_A			0x29
	#define LSM_MAG_REG_OUT_Y_L_A			0x2A
	#define LSM_MAG_REG_OUT_Y_H_A			0x2B
	#define LSM_MAG_REG_OUT_Z_L_A			0x2C
	#define LSM_MAG_REG_OUT_Z_H_A			0x2D
	#define LSM_MAG_REG_FIFO_CTRL_REG		0x2E
	#define LSM_MAG_REG_FIFO_SRC_REG		0x2F
	#define LSM_MAG_REG_INT_GEN_1_REG		0x30
	#define LSM_MAG_REG_INT_GEN_1_SRC		0x31
	#define LSM_MAG_REG_INT_GEN_1_THS		0x32
	#define LSM_MAG_REG_INT_GEN_1_DURATION	0x33
	#define LSM_MAG_REG_INT_GEN_2_REG		0x34
	#define LSM_MAG_REG_INT_GEN_2_SRC		0x35
	#define LSM_MAG_REG_INT_GEN_2_THS		0x36
	#define LSM_MAG_REG_INT_GEN_2_DURATION	0x37
	#define LSM_MAG_REG_CLICK_CFG			0x38
	#define LSM_MAG_REG_CLICK_SRC			0x39
	#define LSM_MAG_REG_CLICK_THS			0x3A
	#define LSM_MAG_REG_TIME_LIMIT			0x3B
	#define LSM_MAG_REG_TIME_LATENCY		0x3C
	#define LSM_MAG_REG_TIME_WINDOW			0x3D
	#define LSM_MAG_REG_ACT_THS				0x3E
	#define LSM_MAG_REG_ACT_DUR				0x3F
/*================================================================================================*/

#endif /* SENSOR_REGISTERS_BITS_H_ */