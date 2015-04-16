/**************************************************************************/
/*!
    @file     Adafruit_ADXL345_U.h
    @author   Michael D. Pirie, K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit ADXL345 breakout board
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include "Sensor_Registers_Bits.h"

#ifndef __ADAFRUIT_ADXL345_U_H__
#define __ADAFRUIT_ADXL345_U_H__

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
/*=========================================================================*/
/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*================================================================================================*/
/*==================================================================================================
    REGISTERS
    ----------------------------------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER ( 0.004 )  // 4mg per lsb
/*================================================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
	ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
	ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
	ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
	ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
	ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
	ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
	ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
	ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
	ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
	ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
	ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
	ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
	ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
	ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

// CONVENIENCE MACROS
#define BV( bit )				   ( 1 << bit );
#define check_bit(var, pos)	(!!((var) & (1 << (pos))))
#define toggle_bit(var, pos)	((var) ^= (1 << (pos)))
#define set_bit(var, pos)		((var) |= (1 << (pos)))
#define clear_bit(var, pos)	((var) &= ~(1 << (pos)))


class Adafruit_ADXL345 : public Adafruit_Sensor
{
 public:
	Adafruit_ADXL345( interface_mode inter_face, uint8_t accelAddress, int32_t sensorID = -1 );
	bool      begin( void );
	/* Inherited members */
	void      getSensor( sensor_t* );
	uint8_t	  getDeviceID( void );
	void      getEvent( sensors_event_t* );
	int16_t	  getX( void );
	int16_t	  getY( void );
	int16_t	  getZ( void );
	/* toggle methods: these members toggle a function or an attribute of a function that have only
	 * 2 possible state, ON or OFF - ENABLE or DISABLE */
	void    toggleStandbyMode( uint8_t state );
	void    toggleSleepMode( uint8_t state, uint8_t wakeupFreq = 1 );
	void    toggleAutoSleep( uint8_t state, uint8_t threshold = 0.2, uint8_t time = 1 ); // TODO test this member
	void    toggleAxisTap( uint8_t axis, uint8_t state );
	void    toggleInterrupt( uint8_t interrupt, uint8_t state);
	void    toggleAxisActivity(uint8_t xyz, uint8_t state);
	void    toggleAxisInactivity(uint8_t axis, uint8_t state);
	/* set methods: these members set a value within the ADXL345 that determine how built in Functions
	 * behave. */
	void    setThresholdTap( uint8_t threshold );
	void    setAllOffsets( uint8_t xOffset, uint8_t yOffset, uint8_t zOffset );
	void    setDurationTap( uint8_t duration);
	void    setLatencyDoubleTap( uint8_t latency ); // TODO test this member
	void    setWindowDoubleTap( uint8_t window ); // TODO test this member
	void    setThresholdActivity( float threshold ); // TODO test this member
	void    setThresholdInactivity( uint8_t threshold ); // TODO test this member
	void    setTimeInactivity( uint8_t time ); // TODO test this member
	void    setThresholdFreeFall( uint8_t threshold ); // TODO test this member
	void    setTimeFreeFall( uint8_t time ); // TODO test this member
	void    setDataRate( dataRate_t dataRate );
	void    setInterruptMapping( uint8_t interrupt, uint8_t interruptPin ); // TODO test this method
	void    setRange( range_t range );
	void    setFIFOMode( uint8_t mode ); // TODO test this method
	void    setFIFOSamples( uint8_t samples ); // TODO implement, test this method
	/* get methods: these members return the current value stored and utilized by the ADXL345 as set
	 * by the above methods'. */
	uint8_t    getThresholdTap( void );
	uint8_t	   getDurationTap( void ); // TODO test this method
	uint8_t	   getLatencyDoubleTap( void ); // TODO test this method
	uint8_t	   getWindowDoubleTap( void ); // TODO test this method
	uint8_t	   getThresholdActivity( void ); // TODO test this method
	uint8_t	   getThresholdInactivity( void ); // TODO test this method
	uint8_t	   getTimeInactivity( void ); // TODO test this method
	uint8_t	   getThresholdFreeFall( void ); // TODO test this method
	uint8_t	   getTimeFreeFall( void ); // TODO test this method
	dataRate_t getDataRate( void );
	range_t    getRange( void );
	uint8_t	   getFIFOMode(); // TODO test this method
	uint8_t	   getInterruptState( void ); // TODO test this method
	uint8_t	   getInterruptSource( void ); // TODO test this member
	/* convenience methods for utilizing the ADXL345 */
	void    DisplayAllOffsets(void); // TODO test this method
	void    displaySensorDetails(void);
	void    displayRange( void );
	void    displayDataRate( void );
	void    disableAllInterrupt( void ); // TODO test this method
	void    displayInterruptStatus( void );
	bool    isTriggered( uint8_t interrupt ); // TODO test this method

	void       writeRegister( uint8_t reg, uint8_t value );
	uint8_t    readRegister( uint8_t reg );
	uint16_t   read16( uint8_t reg );

 private:
	int32_t _sensorID;
	uint8_t _deviceId;
	uint8_t _slaveSelectPin;
	uint8_t _fifo_ctl;
	interface_mode _interfaceMode;
}; //Adafruit_ADXL345

#endif //__ADAFRUIT_ADXL345_U_H__
