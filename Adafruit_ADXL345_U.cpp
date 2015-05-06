/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries), Michael D Pirie
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C or SPI.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
	 v2.0 - Added support for SPI communication protocol, Added support for Arduino Due.
    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/

#include <limits.h>
#include "Adafruit_ADXL345_U.h"
//#include "Sensor_Registers_Bits.h"

/*================================================================================================
   STATIC MEMBERS
   -----------------------------------------------------------------------------------------------*/
/*================================================================================================*/
/** @brief Abstract away platform differences in Arduino wire library while reading a byte via I2C.
 *
 *  @param
 *  @return
 *//*---------------------------------------------------------------------------------------------*/
static uint8_t i2cread(void) {
	#if ARDUINO >= 100
		return Wire.read();
	#else
		return Wire.receive();
	#endif
}
/*================================================================================================*/
/** @brief Abstract away platform differences in Arduino wire library while writing a byte via I2C.
 *
 *  @param
 *  @return
 *//*---------------------------------------------------------------------------------------------*/
static void i2cwrite( uint8_t x ) {
	#if ARDUINO >= 100
		Wire.write( ( uint8_t ) x );
	#else
		Wire.send( x );
	#endif
}
/*================================================================================================*/
/*================================================================================================
   PUBLIC MEMBERS
   -----------------------------------------------------------------------------------------------*/
/*================================================================================================*/
/** \brief Constructor: create an instance of the ADXL345 object.
 *
 *	The user can specify: Interface Protocol, the Address or Channel Select Pin, and provide a
 *	unique device ID.
 *
 * \param inter_face : either MODE_SPI for SPI protocol or MODE_I2C for I2C protocol.\n
 *	\param accelAddress : either the pin number for SPI channel select or the I2c address
 *	 of the device.
 *	\param sensorID : (optional default -1) a user determined unique ID number for the device.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
Adafruit_ADXL345::Adafruit_ADXL345( interface_mode inter_face, uint8_t accelAddress, int32_t sensorID ) {

	_sensorID = sensorID;
	_interfaceMode = inter_face;
	_slaveSelectPin = accelAddress;
}

/*================================================================================================*/
/** \brief Command the host controller to begin communication with the device.
 *
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
bool Adafruit_ADXL345::begin() {
	/* Communicate using SPI */
	if ( _interfaceMode == MODE_SPI )
	{
		Serial.println( "Communicating in SPI mode");
	#ifdef __SAM3X8E__
		Serial.println( "running on Arduino Due" );
		SPI.begin( _slaveSelectPin );
		SPI.setClockDivider( _slaveSelectPin, 21 );// Baud rate = MCK / ClockDivider.
		SPI.setBitOrder( _slaveSelectPin, MSBFIRST );
		SPI.setDataMode( _slaveSelectPin, SPI_MODE3 );
	#else
		Serial.println( " running on Arduino Uno/Mega" );
		SPI.begin();
		SPI.setClockDivider( SPI_CLOCK_DIV32 );// Baud rate = MCK / ClockDivider.
		SPI.setBitOrder( MSBFIRST );
		SPI.setDataMode( SPI_MODE3 );
	#endif
	}/* If using I2C communication */
	else if ( _interfaceMode == MODE_I2C )
	{
		/* Communicate using I2C */
		Serial.println( "I2C communication started" );
		Wire.begin();
	}

	_deviceId = getDeviceID();

	/* Check connection */
	if ( _deviceId != 0xE5 )
	{
		/* No ADXL345 detected ... return false */
		Serial.print( "Device ID: ");
		Serial.println( _deviceId, HEX );
		return false;
	}
	// Enable measurements
	writeRegister( ADXL345_REG_POWER_CTL, 0x08 );

	return true;
}
/*================================================================================================*/
/** \brief Calling this function will populate the supplied sensors_event_t reference with the
 * latest available sensor data. You should call this function as often as you want to update your
 * data.
 *
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::getSensor( sensor_t *sensor ) {
  /* Clear the sensor_t object */
  memset( sensor, 0, sizeof( sensor_t ) );

  /* Insert the sensor name in the fixed length char array */
  strncpy ( sensor->name, "ADXL345", sizeof( sensor->name ) - 1 );
  sensor->name[ sizeof( sensor->name )- 1 ] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value   = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution  = 0.003923F;  /*  4mg = 0.0392266 m/s^2 */
}
/*================================================================================================*/
/** \brief Retrieve the factory assigned device ID.
 *
 *	detailed description here temperature
 *
 * \return 8bit device identification number
 *//*---------------------------------------------------------------------------------------------*/
 uint8_t Adafruit_ADXL345::getDeviceID( void ) {

	Serial.print( "getting device ID... " );
	uint8_t id = readRegister( ADXL345_REG_DEVID );
	Serial.println( id, HEX );
	return id;
}
/*================================================================================================*/
/** \brief Retrieve one measurement for all three axis.
 *
 *	An inherited function from the Adafruit_Sensor library.  This member gathers a measurement for
 *	all three axis and stores the values in a struct.
 *
 * \param *event : a pointer to a struct of type sensors_event_t where measurement data will be stored.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::getEvent( sensors_event_t *event ) {
  /* Clear the event */
  memset( event, 0, sizeof( sensors_event_t ) );

  event->version   = sizeof( sensors_event_t );
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

//   event->acceleration.x = getX();
//   event->acceleration.y = getY();
//   event->acceleration.z = getZ();
}
/*================================================================================================*/
/** \brief Retrieve a measurement from the X axis
 *
 * \return X axis measurement value
 *//*---------------------------------------------------------------------------------------------*/
int16_t Adafruit_ADXL345::getX( void ) {

	return read16( ADXL345_REG_DATAX0 );
}
/*================================================================================================*/
/** \brief Retrieve a measurement from the Y axis
 *
 * \return Y axis measurement value
 *//*---------------------------------------------------------------------------------------------*/
int16_t Adafruit_ADXL345::getY( void ) {

	return read16( ADXL345_REG_DATAY0 );
}
/*================================================================================================*/
/** \brief Retrieve a measurement from the Z axis
 *
 * \return Z axis measurement value
 *//*---------------------------------------------------------------------------------------------*/
int16_t Adafruit_ADXL345::getZ( void ) {

	return read16( ADXL345_REG_DATAZ0 );
}
/*================================================================================================*/
/** \brief Enable or Disable Standby Mode
 *
 * For even lower power operation, standby mode can be used. In standby mode, current consumption
 * is reduced to 0.1 ?A (typical). In this mode, no measurements are made.
 * Standby mode is entered by clearing the measure bit (Bit D3) in the POWER_CTL register
 * (Address 0x2D). Placing the device into standby mode preserves the contents of FIFO.
 *
 *  \param state : either ENABLE or DISABLE
 *  \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::toggleStandbyMode( uint8_t state ) {

	uint8_t power_ctl;

	switch ( state )
	{
	case ENABLE:
		power_ctl = readRegister( ADXL345_REG_POWER_CTL );
		clear_bit( power_ctl, 3);
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
		break;
	case DISABLE:
		power_ctl = readRegister( ADXL345_REG_POWER_CTL );
		set_bit( power_ctl, 3);
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
		break;
	}
}
/*================================================================================================*/
/** \brief Enable or Disable the Sleep Mode function
 *
 *	A setting of 0 in the sleep bit puts the part into the normal mode of operation, and a setting
 *	of 1 places the part into sleep mode. Sleep mode suppresses DATA_READY, stops transmission of
 *	data to FIFO, and switches the sampling rate to one specified by the wakeup bits. In sleep mode,
 *	only the activity function can be used. When the DATA_READY interrupt is suppressed, the output
 *	data registers (Register 0x32 to Register 0x37) are still updated at the sampling rate set by
 *	the wakeup bits (D1:D0).
 *
 *	When clearing the sleep bit, it is recommended that the part be placed
 *	into standby mode and then set back to measurement mode with a subsequent write. This is done to
 *	ensure that the device is properly biased if sleep mode is manually disabled; otherwise,
 *	the first few samples of data after the sleep bit is cleared may have additional noise,
 *	especially if the device was asleep when the bit was cleared.
 *
 * \param state : (required) either ENABLE or DISABLE
 * \param wakeupFreq : (optional, defaults to 1) either 1, 2, 4, or 8 samples per sec. When
 * disabling SLEEP MODE the Wake Up bits will be left in there previous values.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::toggleSleepMode( uint8_t state, uint8_t wakeupFreq )
{
	// retrieve the current byte value from the POWER CONTROL register
	uint8_t power_ctl = readRegister( ADXL345_REG_POWER_CTL );

	switch ( state )
	{
	case ENABLE:
		// set the SLEEP bit
		set_bit( power_ctl, ADXL_SLEEP ); // turn on sleep mode bit

		switch ( wakeupFreq )
		{
		case 1: // 1 Hz acceleration update
			power_ctl |= 0b00000011;
			writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
			break;
		case 2: // 2 Hz acceleration update
			clear_bit( power_ctl, 0 );
			set_bit( power_ctl, 1 );
			writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
			break;
		case 4:
			clear_bit( power_ctl, 1 );
			set_bit( power_ctl, 0 );
			writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
			break;
		case 8:
			power_ctl &= 0b00;
			writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
			break;
		default:
			Serial.print( __LINE__ );
			Serial.println( " ERROR: \'wakeupFreq\' parameter is un-handled, expected 1, 2, 4, or 8" );
			delay( 2000 );
			break;
		}
		break;
	case DISABLE:
		// clear the MEASURE bit to put the device into STANDBY MODE
		clear_bit( power_ctl, ADXL_MEASURE );
		// update the POWER CONTROL register with the new byte
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );

		delayMicroseconds( 5 );

		// clear the SLEEP bit to disable SLEEP MODE
		clear_bit( power_ctl, ADXL_SLEEP );
		// update the POWER CONTROL register
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );

		delayMicroseconds( 5 );

		// set the MEASURE bit to disable STANCBY MODE and update POWER CONTROL register again.
		set_bit( power_ctl, ADXL_MEASURE ); // return the device to Measure Mode
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
		break;
	default:
		Serial.print( __LINE__ );
		Serial.println( " ERROR: un-handled parameter, Adafruit_ADXL345::toggleSleepMode" );
		break;
	}
}
/*================================================================================================*/
/** \brief Enable or Disable the Auto sleep feature.
 *
 * Additional power can be saved if the ADXL345 automatically switches to sleep mode during periods
 * of inactivity. To enable this feature, set the THRESH_INACT register (Address 0x25) and the
 * TIME_INACT register (Address 0x26) each to a value that signifies inactivity
 * (the appropriate value depends on the application), and then set the AUTO_SLEEP bit (Bit D4)
 * and the link bit (Bit D5) in the POWER_CTL register (Address 0x2D). Current consumption at the
 * sub-12.5 Hz data rates that are used in this mode is typically 23 ?A for a VS of 2.5 V.
 *
 * \param state : (required) either ENABLE or DISABLE.
 * \param threshold : (optional, default 0.2 m\s) set a level for which motion activity must remain
 * below to trigger AUTO SLEEP MODE.
 * \param time : (optional, default is 1) a time period in seconds for which inactivity must
 * persist before AUTO SLEEP is set.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::toggleAutoSleep( uint8_t state, uint8_t threshold, uint8_t time ) {

	uint8_t power_ctl = readRegister( ADXL345_REG_POWER_CTL );

	switch ( state )
	{
	case ENABLE:
		toggleStandbyMode( ENABLE );
		set_bit( power_ctl, ADXL_AUTO_SLEEP );// set AUTO_SLEEP bit in POWER_CTL register
		set_bit( power_ctl, ADXL_LINK );// set LINK bit in POWER_CTL register
		writeRegister( ADXL345_REG_THRESH_INACT, threshold );
		writeRegister( ADXL345_REG_TIME_INACT, time );
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
		toggleInterrupt( ADXL_INACTIVITY, ENABLE );
		toggleStandbyMode( DISABLE );
		break;
	case DISABLE:
		toggleStandbyMode( ENABLE );
		clear_bit( power_ctl, ADXL_AUTO_SLEEP );
		clear_bit( power_ctl, ADXL_LINK );
		writeRegister( ADXL345_REG_POWER_CTL, power_ctl );
		toggleInterrupt( ADXL_INACTIVITY, ENABLE );
		toggleStandbyMode( DISABLE );
		break;
	}
}
/*================================================================================================*/
/** \brief Turn Tap Detection on or off for a specified axis.
 *
 * \param axis : the axis for which to enable or disable tap detection.
 * \param state : either ENABLE or DISABLE.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::toggleAxisTap( uint8_t axis, uint8_t state ) {

	uint8_t reg = readRegister( ADXL345_REG_TAP_AXES );

	switch ( axis )
	{
	case ADXL_X_AXIS:
		if ( state == ENABLE )
		{
			set_bit( reg, ADXL_TAP_X_ENABLE );
		}
		else if ( state == DISABLE )
		{
			clear_bit( reg, ADXL_TAP_X_ENABLE );
		}
		break;
	case ADXL_Y_AXIS:
		if ( state == ENABLE )
		{
			set_bit( reg, ADXL_TAP_Y_ENABLE );
		}
		else if ( state == DISABLE )
		{
			clear_bit( reg, ADXL_TAP_Y_ENABLE );
		}
		break;
	case ADXL_Z_AXIS:
		if ( state == ENABLE )
		{
			set_bit( reg, ADXL_TAP_Z_ENABLE );
		}
		else if ( state == DISABLE )
		{
			clear_bit( reg, ADXL_TAP_Z_ENABLE );
		}
		break;
	default :
		Serial.println( " ERROR : un-handled parameter, ADXL345::toggleAxisTap" );
		delay( 1000 );
	}
	writeRegister( ADXL345_REG_TAP_AXES, reg );
}
/*================================================================================================*/
/** \brief Toggle ON or OFF a specified interrupt function.
 *
 *  There are 8 interrupt function:
 *  DATA_READY, SINGLE_TAP, DOUBLE_TAP, ACTIVITY, INACTIVITY, FREE_FALL, WATERMARK, OVERRUN. To
 *  learn more about each interrupt refer to the Data Sheet pg. 20 from Analog Devices.
 *	 Use the macros ENABLE or DISABLE to turn on or off respectively.
 *
 *  \param interrupt : which of the 8 interrupt function to ENABLE or DISABLE.
 *  \param state : enable or disabled.
 *  \return
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::toggleInterrupt( uint8_t interrupt, uint8_t state )
{
	uint8_t reg = readRegister(ADXL345_REG_INT_ENABLE);

	switch ( interrupt )
	{
		case ADXL_DATA_READY:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_DATA_READY);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_DATA_READY);
		}
		break;
		case ADXL_SINGLE_TAP:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_SINGLE_TAP);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_SINGLE_TAP);
		}
		break;
		break;
		case ADXL_DOUBLE_TAP:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_DOUBLE_TAP);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_DOUBLE_TAP);
		}
		break;
		break;
		case ADXL_ACTIVITY:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_ACTIVITY);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_ACTIVITY);
		}
		break;
		break;
		case ADXL_INACTIVITY:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_INACTIVITY);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_INACTIVITY);
		}
		break;
		case ADXL_FREE_FALL:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_FREE_FALL);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_FREE_FALL);
		}
		break;
		case ADXL_WATERMARK:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_WATERMARK);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_WATERMARK);
		}
		break;
		break;
		case ADXL_OVERRUN:
		if (state == ENABLE)
		{
			set_bit(reg, ADXL_OVERRUN);
		}
		else if (state == DISABLE)
		{
			clear_bit(reg, ADXL_OVERRUN);
		}
		break;
	}// switch
	writeRegister(ADXL345_REG_INT_ENABLE, reg);
	delayMicroseconds( 5 );
}


/**
 * \brief Enable or Disable Activity detection for a specified axis.
 *
 * \param axis the axis to ENABLE or DISABLE Activity detection.
 * \param state either ENABLE or DISABLE.
 *
 * \return void
 */
void Adafruit_ADXL345::toggleAxisActivity( uint8_t axis, uint8_t state )
{
	uint8_t reg = readRegister( ADXL345_REG_ACT_INACT_CTL );

	switch ( axis )
	{
		case ADXL_X_AXIS:
		if ( state == ENABLE )
		{
			set_bit(reg, ADXL_ACT_X_ENABLE);
		}
		else if (state == DISABLE)
		{
			clear_bit( reg, ADXL_ACT_X_ENABLE );
		}
		break;
		case ADXL_Y_AXIS:
		if ( state == ENABLE )
		{
			set_bit( reg, ADXL_ACT_Y_ENABLE );
		}
		else if ( state == DISABLE )
		{
			clear_bit( reg, ADXL_ACT_Y_ENABLE );
		}
		break;
		case ADXL_Z_AXIS:
		if ( state == ENABLE )
		{
			set_bit( reg, ADXL_ACT_Z_ENABLE );
		}
		else if (state == DISABLE)
		{
			clear_bit( reg, ADXL_ACT_Z_ENABLE );
		}
		break;
		default:
		Serial.println( "ERROR: unhandled parameter, Adafruit_ADXL345::toggleAxisActivity" );
		delay( 2000 );
	}// end switch
	writeRegister(ADXL345_REG_ACT_INACT_CTL, reg);
	delayMicroseconds( 5 );
}


/**
 * \brief Enable or Disable Inactivity detection for a specified axis
 *
 * \param axis which axis to ENABLE or DISABLE
 * \param state either ENABLE or DIASBLE
 *
 * \return void
 */
void Adafruit_ADXL345::toggleAxisInactivity( uint8_t axis, uint8_t state )
{
	uint8_t reg = readRegister(ADXL345_REG_ACT_INACT_CTL);

	switch (axis)
	{
		case ADXL_X_AXIS:
		if (state == ENABLE )
		{
			set_bit(reg, ADXL_INACT_X_ENABLE);
		}
		else// if (state == DISABLE)
		{
			clear_bit(reg, ADXL_INACT_X_ENABLE);
		}
		break;
		case ADXL_Y_AXIS:
		if (state == ENABLE )
		{
			set_bit(reg, ADXL_INACT_Y_ENABLE);
		}
		else// if (state == DISABLE)
		{
			clear_bit(reg, ADXL_INACT_Y_ENABLE);
		}
		break;
		case ADXL_Z_AXIS:
		if (state == ENABLE )
		{
			set_bit(reg, ADXL_INACT_Z_ENABLE);
		}
		else// if (state == DISABLE)
		{
			clear_bit(reg, ADXL_INACT_Z_ENABLE);
		}
		break;
		default:
		Serial.println("ERROR: unhandled parameter, Adafruit_ADXL345::toggleAxisInactivity");
		delay(3000);
	}// end switch
	writeRegister(ADXL345_REG_ACT_INACT_CTL, reg);
	delayMicroseconds( 5 );
}
/*================================================================================================*/
/** \brief Set the motion value that must be met to flag a tap detection.
 *
 *	detailed description here temperature
 *
 * \param threshold : the value - in meters per second - a measurement must be greater than this to
 * flag a tap detection.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setThresholdTap( uint8_t threshold ) {

	threshold = constrain(threshold, 0, 255);

	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_THRESH_TAP, threshold );
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Each Axis has an 8bit register for calibrating their measurement.
 *
 *	The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set offset adjustments
 *	in twos complement format with a scale factor of 15.6 mg/LSB (that is, 0x7F = 2 g).
 *	The value stored in the offset registers is automatically added to the acceleration data,
 *	and the resulting value is stored in the output data registers. For additional information
 *	regarding offset calibration and the use of the offset registers, refer to the Offset
 *	Calibration section.
 *
 * \param xOffset: and 8 bit value for adjusting the output of the X axis.
 * \param yOffset: and 8 bit value for adjusting the output of the Y axis.
 * \param zOffset: and 8 bit value for adjusting the output of the Z axis
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setAllOffsets( uint8_t xOffset, uint8_t yOffset, uint8_t zOffset ) {

	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_OFSX, xOffset );
	writeRegister( ADXL345_REG_OFSY, yOffset );
	writeRegister( ADXL345_REG_OFSZ, zOffset );
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Set a time duration for tap detection
 *
 *	The DUR register is eight bits and contains an unsigned time value representing the maximum
 *	time that an event must be above the THRESH_TAP threshold to qualify as a tap event.
 *	The scale factor is 625 us/LSB. A value of 0 disables the single tap/ double tap functions.
 *
 * \param duration : 8 bit value (0 - 255)
 * \return
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setDurationTap( uint8_t duration ) {

	duration = constrain(duration, 0, 255);
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_DUR, duration );
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Set a time separation for double tap detection.
 *
 * The latent register is eight bits and contains an unsigned time value representing the wait time
 * from the detection of a tap event to the start of the time window
 * (defined by the window register) during which a possible second tap event can be detected.
 * The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap function.
 *
 * \param latency : 8 bit value (0-255)
 * \return
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setLatencyDoubleTap( uint8_t latency ) {

	latency = constrain( latency, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_LATENT, latency );
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Set a time separation for double tap detection.
 *
 *	The window register is eight bits and contains an unsigned time value representing the amount of
 *	time after the expiration of the latency time (determined by the latent register)
 *	during which a second valid tap can begin. The scale factor is 1.25 ms/LSB. A value of 0 disables
 *	the double tap function.
 *
 * \param window : 8 bit value (0 - 255)
 * \return
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setWindowDoubleTap( uint8_t window ) {

	window = constrain( window, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_WINDOW, window );
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Set a motion threshold in Meters/Second that must be exceeded in order to flag for ACTIVITY.
 *
 *	The activity bit is set when acceleration greater than the value stored in the THRESH_ACT
 *	register (Address 0x24) is experienced on any participating axis, set by the ACT_INACT_CTL
 *	register (Address 0x27).
 *
 * \param threshold : 8 bit value (0 - 255) in Meters/Second for minimum motion required to flag
 * ACTIVITY.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setThresholdActivity( float threshold ) {

	threshold = constrain( threshold, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_THRESH_ACT, threshold );
	toggleStandbyMode( DISABLE );
}
 /*================================================================================================*/
/** \brief Set a motion threshold in Meters/Second to check for inactivity.
 *
 *	The inactivity bit is set when acceleration of less than the value stored in the THRESH_INACT
 *	register (Address 0x25) is experienced for more time than is specified in the TIME_INACT
 *	register (Address 0x26) on all participating axes, as set by the ACT_INACT_CTL
 *	register (Address 0x27). The maximum value for TIME_INACT is 255 sec.
 *
 * \param threshold : 8 bit value (0 - 255) in 62.5mg increments for maximum motion allowed before
 * flagging INACTIVITY.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setThresholdInactivity( uint8_t threshold ) {

	threshold = constrain( threshold, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_THRESH_INACT, threshold );
	toggleStandbyMode( DISABLE );
}
 /*================================================================================================*/
/** \brief Set a time period before flagging for INACTIVITY.
 *
 *	The inactivity bit is set when acceleration of less than the value stored in the THRESH_INACT
 *	register (Address 0x25) is experienced for more time than is specified in the TIME_INACT
 *	register (Address 0x26) on all participating axes, as set by the ACT_INACT_CTL
 *	register (Address 0x27). The maximum value for TIME_INACT is 255 sec.
 *
 * \param time : 8 bit value (0 - 255) Seconds for minimum time period before flagging for INACTIVITY.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setTimeInactivity( uint8_t time ) {

	time = constrain( time, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_TIME_INACT, time );
	toggleStandbyMode( DISABLE );
}
 /*================================================================================================*/
/** \brief Set a maximum value for all axis for detecting FREE_FALL.
 *
 *	The FREE_FALL bit is set when acceleration of less than the value stored in the THRESH_FF
 *	register (Address 0x28) is experienced for more time than is specified in the TIME_FF
 *	register (Address 0x29) on all axes (logical AND). The FREE_FALL interrupt differs from
 *	the inactivity interrupt as follows: all axes always participate and are logically AND’ed,
 *	the timer period is much smaller (1.28 sec maximum), and the mode of operation is always
 *	dc-coupled.
 *	Note that a value of 0 mg may result in undesirable behavior if the free-fall interrupt is enabled.
 * Values between 300 mg and 600 mg (0x05 to 0x09) are recommended.
 * A value of 0 may cause cause undesirable behavior.
 *
 * \param threshold : 8 bit value (0 - 255) in 62.5mg increments.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setThresholdFreeFall( uint8_t threshold ) {

	threshold = constrain( threshold, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_THRESH_FF, threshold ); // 62.5mg per increment
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief Set a time period which must be met before flagging for FREE_FALL detection.
 *
 * The scale factor is 5 ms/LSB. A value of 0 may result in undesirable behavior
 * if the free-fall interrupt is enabled. Values between 100 ms and 350 ms (0x14 to 0x46)
 * are recommended.
 *
 * \param time : 8 bit value (0 - 255) in 5ms increments.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setTimeFreeFall( uint8_t time ) {

	time = constrain( time, 0, 255 );
	toggleStandbyMode( ENABLE );
	writeRegister( ADXL345_REG_TIME_FF, time ); // 5ms per increment
	toggleStandbyMode( DISABLE );
}
/*================================================================================================*/
/** \brief
 *
 *	detailed description here temperature
 *
 * \param
 * \return
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setDataRate( dataRate_t dataRate ) {

	/* Note: The LOW_POWER bits are currently ignored and we always keep
	the device in 'normal' mode */
	writeRegister( ADXL345_REG_BW_RATE, dataRate );

}
/*================================================================================================*/
/** \brief Enable a specified interrupt and link it to one of two interrupt outputs.
 *
 * There are eight interrupts available for use and two output pins to which to read from.
 * The interrupts are:
 * DATA_READY, SINGLE_TAP, DOUBLE_TAP, ACTIVITY, INACTIVITY, FREE_FALL, WATER_MARK, OVERRUN
 * See data sheet page 20 for details about each interrupt. NOTE: if you assign more than one
 * interrupt function to the same output pin, you will not be able to distinguish which function
 * triggered.  You will need to read the ADXL345_REG_INT_SOURCE register to determine which event
 * triggered.
 *
 * \param interrupt    : one of the eight interrupts named above.
 * \param interruptPin : the output pin to route the specified interrupt event to.
 * Either INT1 (0) or INT2 (1).
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setInterruptMapping( uint8_t interrupt, uint8_t interruptPin )
{
	uint8_t int_map = readRegister( ADXL345_REG_INT_MAP );

	switch ( interrupt )
	{
	case ADXL_DATA_READY:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_DATA_READY );
		}
		else if (  interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_DATA_READY );
		}
		break;
	case ADXL_SINGLE_TAP:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_SINGLE_TAP );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_SINGLE_TAP );
		}
		break;
	case ADXL_DOUBLE_TAP:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_DOUBLE_TAP );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_DOUBLE_TAP );
		}
		break;
	case ADXL_ACTIVITY:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_ACTIVITY );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_ACTIVITY );
		}
		break;
	case ADXL_INACTIVITY:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_INACTIVITY );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_INACTIVITY );
		}
		break;
	case ADXL_FREE_FALL:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_FREE_FALL );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_FREE_FALL );
		}
		break;
	case ADXL_WATERMARK:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_WATERMARK );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_WATERMARK );
		}
		break;
	case ADXL_OVERRUN:
		if ( interruptPin == ADXL_INT1 )
		{
			clear_bit( int_map, ADXL_OVERRUN );
		}
		else if ( interruptPin == ADXL_INT2 )
		{
			set_bit( int_map, ADXL_OVERRUN );
		}
		break;
	}// switch
	writeRegister( ADXL345_REG_INT_MAP, int_map );
	delayMicroseconds( 5 );
}
/*================================================================================================*/
/** \brief Set the device measurement range
 *
 *	Choose between a measurement range of +/- 2g, 4g, 8g, and 16g
 *
 * \param range : either 2, 4, 8, or 16
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setRange( range_t range ) {

	/* Red the data format register to preserve bits */
	uint8_t format = readRegister( ADXL345_REG_DATA_FORMAT );

	/* Update the data rate */
	format &= ~0x0F;
	format |= range;

	/* Make sure that the FULL-RES bit is enabled for range scaling */
	format |= 0x08;

	/* Write the register back to the IC */
	writeRegister( ADXL345_REG_DATA_FORMAT, format );
}
/*================================================================================================*/
/** \brief Change the operating mode os the First In First Out measurement memory system.
 *
 * The ADXL345 contains patent pending technology for an embedded memory management system
 * with 32-level FIFO that can be used to minimize host processor burden.
 * This buffer has four modes: bypass, FIFO, stream, and trigger (see FIFO Modes).
 * Each mode is selected by the settings of the FIFO_MODE bits (Bits[D7:D6]) in the FIFO_CTL
 * register (Address 0x38).
 * See data sheet page 22 for details on all available FIFO operating modes.
 * There are 4 options - FIFI_BYPASS, FIFO_FIFO, FIFO_STREAM, FIFO_TRIGGER
 *
 * \param mode : the mode to command the device to use.
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::setFIFOMode( uint8_t mode ) {

	uint8_t currentMode = readRegister( ADXL345_REG_FIFO_CTL );

	switch ( mode )
	{
	case ADXL_FIFO_BYPASS:
		currentMode &= 0b00111111;
		writeRegister( ADXL345_REG_FIFO_CTL, currentMode );
		break;
	case ADXL_FIFO_FIFO:
		currentMode |= 0b11000000;
		currentMode &= 0b01111111;
		writeRegister( ADXL345_REG_FIFO_CTL, currentMode );
		break;
	case ADXL_FIFO_STREAM:
		currentMode |= 0b11000000;
		currentMode &= 0b10111111;
		writeRegister( ADXL345_REG_FIFO_CTL, currentMode );
		break;
	case ADXL_FIFO_TRIGGER:
		currentMode |= 0b11000000;
		writeRegister( ADXL345_REG_FIFO_CTL, currentMode );
		break;
	}
}

/**
 * \brief Set the trigger limit for a number of samples that will trigger Watermark interrupt.
 *
 * \param samples
 *
 * \return void
 */
void Adafruit_ADXL345::setFIFOSamples( uint8_t samples )
{
	 samples = constrain( samples, 0, 32 );
	 writeRegister( ADXL345_REG_FIFO_CTL, samples );
}

uint8_t Adafruit_ADXL345::getThresholdTap() {

	return readRegister(ADXL345_REG_THRESH_TAP);
}

uint8_t Adafruit_ADXL345::getDurationTap()
{
	return readRegister( ADXL345_REG_DUR );
}

uint8_t Adafruit_ADXL345::getLatencyDoubleTap( void )
{
	return readRegister( ADXL345_REG_LATENT );
}

uint8_t Adafruit_ADXL345::getWindowDoubleTap( void )
{
	return readRegister( ADXL345_REG_WINDOW );
}

uint8_t Adafruit_ADXL345::getThresholdActivity( void )
{
	return readRegister( ADXL345_REG_THRESH_ACT );
}

uint8_t Adafruit_ADXL345::getThresholdInactivity( void )
{
	return readRegister( ADXL345_REG_THRESH_INACT );
}

uint8_t Adafruit_ADXL345::getTimeInactivity( void )
{
	return readRegister( ADXL345_REG_TIME_INACT );
}

uint8_t Adafruit_ADXL345::getThresholdFreeFall( void )
{
	return readRegister( ADXL345_REG_THRESH_FF );
}

uint8_t Adafruit_ADXL345::getTimeFreeFall( void )
{
	return readRegister( ADXL345_REG_TIME_FF );
}
/*================================================================================================*/
/** \brief Returns the currently set data rate for the ADXL345.
 *
 * \return 8 bit value representing the current data rate.
 *//*---------------------------------------------------------------------------------------------*/
dataRate_t Adafruit_ADXL345::getDataRate( void ) {

		return ( dataRate_t )( readRegister( ADXL345_REG_BW_RATE) & 0x0F );
}
/*================================================================================================*/
/** \brief Return the current measurement range setting.
 *
 * \param
 * \return range_t : the current measurement range.
 *//*---------------------------------------------------------------------------------------------*/
range_t Adafruit_ADXL345::getRange( void ) {

		return ( range_t )( readRegister( ADXL345_REG_DATA_FORMAT ) & 0x03 );
}
/*================================================================================================*/
/** \brief Returns the current FIFO operating mode
 *  long description here
 *
 *  \return 8 bit value representing the current FIFO mode.
 *//*---------------------------------------------------------------------------------------------*/
uint8_t Adafruit_ADXL345::getFIFOMode()
{
	return readRegister( ADXL345_REG_FIFO_CTL );
}
/*================================================================================================*/
/** \brief
 *  long description here
 *
 *  \return
 *//*---------------------------------------------------------------------------------------------*/
uint8_t Adafruit_ADXL345::getInterruptState() {

	return readRegister( ADXL345_REG_INT_ENABLE );
}
/*================================================================================================*/
/** \brief Read the INT_SOURCE register to check for event flags and reset certain interrupts.
 *
 *	Bits set to 1 in this register indicate that their respective functions have triggered an event,
 *	whereas a value of 0 indicates that the corresponding event has not occurred. The DATA_READY,
 *	watermark, and overrun bits are always set if the corresponding events occur, regardless of
 *	the INT_ENABLE register settings, and are cleared by reading data from the DATAX, DATAY,
 *	and DATAZ registers. The DATA_READY and watermark bits may require multiple reads,
 *	as indicated in the FIFO mode descriptions in the FIFO section. Other bits, and the
 *	corresponding interrupts, are cleared by reading the INT_SOURCE register.
 *
 * \return 8 bit value representing the currently flagged interrupt events.
 *//*---------------------------------------------------------------------------------------------*/
uint8_t Adafruit_ADXL345::getInterruptSource() {

	return readRegister( ADXL345_REG_INT_SOURCE );
}

void Adafruit_ADXL345::DisplayAllOffsets( void )
{
	Serial.print("Offsets: X = ");Serial.print(readRegister(ADXL345_REG_OFSX), DEC);
	Serial.print(" Y = ");Serial.print(readRegister(ADXL345_REG_OFSY));
	Serial.print(" Z = ");Serial.println(readRegister(ADXL345_REG_OFSZ));
	delay(3000);
}

void Adafruit_ADXL345::displaySensorDetails(void) {

	sensor_t sensor;
	getSensor( &sensor );
	Serial.println("------------------------------------");
	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
	Serial.println("------------------------------------");
	Serial.println("");
	delay(500);
}

void Adafruit_ADXL345::displayRange( void ) {

	Serial.print  ("Range:         +/- ");

	switch( getRange() )
	{
		case ADXL345_RANGE_16_G:
		Serial.print  ( "16 " );
		break;
		case ADXL345_RANGE_8_G:
		Serial.print  ( "8 " );
		break;
		case ADXL345_RANGE_4_G:
		Serial.print  ( "4 " );
		break;
		case ADXL345_RANGE_2_G:
		Serial.print  ( "2 " );
		break;
		default:
		Serial.print  ( "?? " );
		break;
	}
	Serial.println( " g" );
}

void Adafruit_ADXL345::displayDataRate( void ) {

	Serial.print( "Data Rate:    " );

	switch( getDataRate() )
	{
		case ADXL345_DATARATE_3200_HZ:
		Serial.print  ("3200 ");
		break;
		case ADXL345_DATARATE_1600_HZ:
		Serial.print  ("1600 ");
		break;
		case ADXL345_DATARATE_800_HZ:
		Serial.print  ("800 ");
		break;
		case ADXL345_DATARATE_400_HZ:
		Serial.print  ("400 ");
		break;
		case ADXL345_DATARATE_200_HZ:
		Serial.print  ("200 ");
		break;
		case ADXL345_DATARATE_100_HZ:
		Serial.print  ("100 ");
		break;
		case ADXL345_DATARATE_50_HZ:
		Serial.print  ("50 ");
		break;
		case ADXL345_DATARATE_25_HZ:
		Serial.print  ("25 ");
		break;
		case ADXL345_DATARATE_12_5_HZ:
		Serial.print  ("12.5 ");
		break;
		case ADXL345_DATARATE_6_25HZ:
		Serial.print  ("6.25 ");
		break;
		case ADXL345_DATARATE_3_13_HZ:
		Serial.print  ("3.13 ");
		break;
		case ADXL345_DATARATE_1_56_HZ:
		Serial.print  ("1.56 ");
		break;
		case ADXL345_DATARATE_0_78_HZ:
		Serial.print  ("0.78 ");
		break;
		case ADXL345_DATARATE_0_39_HZ:
		Serial.print  ("0.39 ");
		break;
		case ADXL345_DATARATE_0_20_HZ:
		Serial.print  ("0.20 ");
		break;
		case ADXL345_DATARATE_0_10_HZ:
		Serial.print  ("0.10 ");
		break;
		default:
		Serial.print  ("???? ");
		break;
	}
	Serial.println(" Hz");
}
 /*================================================================================================*/
/** \brief Quickly disable all interrupts.
 *
 *	Each interrupt can be routed to either of two output pins.
 *
 * \param void
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::disableAllInterrupt() {
	/* Communicate using SPI */
	if ( _interfaceMode == MODE_SPI ) {
		toggleStandbyMode( ENABLE );
		writeRegister( ADXL345_REG_INT_ENABLE, 0x00 );
		toggleStandbyMode( DISABLE );
	}
	/* Communicate using I2C */
	else if ( _interfaceMode == MODE_I2C ) {
		toggleStandbyMode( ENABLE );
		writeRegister( ADXL345_REG_INT_ENABLE, 0x00 );
		toggleStandbyMode( DISABLE );
	}
}
/*================================================================================================*/
/** \brief Read the INT_SOURCE register to check for event flags and reset certain interrupts.
 *
 *	Bits set to 1 in this register indicate that their respective functions have triggered an event,
 *	whereas a value of 0 indicates that the corresponding event has not occurred. The DATA_READY,
 *	watermark, and overrun bits are always set if the corresponding events occur, regardless of
 *	the INT_ENABLE register settings, and are cleared by reading data from the DATAX, DATAY,
 *	and DATAZ registers. The DATA_READY and watermark bits may require multiple reads,
 *	as indicated in the FIFO mode descriptions in the FIFO section. Other bits, and the
 *	corresponding interrupts, are cleared by reading the INT_SOURCE register.
 *
 * \param void
 * \return void
 *//*---------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::displayInterruptStatus() {

	uint8_t status = getInterruptSource();

	Serial.print( "DATA_READY : " ); Serial.println( check_bit(status, ADXL_DATA_READY ) );
	Serial.print( "SINGLE_TAP : " ); Serial.println( check_bit(status, ADXL_SINGLE_TAP ) );
	Serial.print( "DOUBLE_TAP : " ); Serial.println( check_bit(status, ADXL_DOUBLE_TAP ) );
	Serial.print( "ACTIVITY   : " ); Serial.println( check_bit(status, ADXL_ACTIVITY ) );
	Serial.print( "INACTIVITY : " ); Serial.println( check_bit(status, ADXL_INACTIVITY ) );
	Serial.print( "FREE_FALL  : " ); Serial.println( check_bit(status, ADXL_FREE_FALL ) );
	Serial.print( "WATERMARK  : " ); Serial.println( check_bit(status, ADXL_WATERMARK ) );
	Serial.print( "OVERRUN    : " ); Serial.println( check_bit(status, ADXL_OVERRUN ) );
	delay( 1000 );
}

bool Adafruit_ADXL345::isTriggered( uint8_t interrupt )
{
	uint8_t reg = readRegister( ADXL345_REG_INT_SOURCE );
	uint8_t reg_copy = reg;

	switch ( interrupt )
	{
	case ADXL_DATA_READY:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_DATA_READY ) ) >> ADXL_DATA_READY;
		break;
		case ADXL_SINGLE_TAP:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_SINGLE_TAP ) ) >> ADXL_SINGLE_TAP;
		break;
	case ADXL_DOUBLE_TAP:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_DOUBLE_TAP ) ) >> ADXL_DOUBLE_TAP;
		break;
		case ADXL_ACTIVITY:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_ACTIVITY ) ) >> ADXL_ACTIVITY;
		break;
	case ADXL_INACTIVITY:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_INACTIVITY ) ) >> ADXL_INACTIVITY;
		break;
	case ADXL_FREE_FALL:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_FREE_FALL ) ) >> ADXL_FREE_FALL;
		break;
	case ADXL_WATERMARK:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_WATERMARK ) ) >> ADXL_WATERMARK;
		break;
	case ADXL_OVERRUN:
		reg_copy = reg;
		return ( reg_copy & ( 1 << ADXL_OVERRUN ) ) >> ADXL_OVERRUN;
		break;
	default :
		Serial.println( "ERROR: unhandled parameter, ADAFRUIT_ADXL345::isTriggered");
		break;
	}
}

/*================================================================================================*/
/*================================================================================================
   PRIVATE MEMBERS
   -----------------------------------------------------------------------------------------------*/
void Adafruit_ADXL345::writeRegister( uint8_t reg, uint8_t val )
{
	if ( _interfaceMode == MODE_SPI )
	{
	#ifdef __SAM3X8E__ // if using Arduino Due
		SPI.transfer( _slaveSelectPin, reg, SPI_CONTINUE );
		SPI.transfer( _slaveSelectPin, val, SPI_LAST);
	#else
		digitalWrite( _slaveSelectPin, LOW );
		SPI.transfer( reg );
		SPI.transfer( val );
		digitalWrite( _slaveSelectPin, HIGH );
	#endif
	}
	else if ( _interfaceMode == MODE_I2C )
	{
		Wire.beginTransmission( ADXL345_ADDRESS );
		i2cwrite( ( uint8_t ) reg );
		i2cwrite( ( uint8_t )( val ) );
		Wire.endTransmission();
	}
}

uint8_t Adafruit_ADXL345::readRegister( uint8_t reg )
{
	if ( _interfaceMode == MODE_SPI )
	{
	#ifdef __SAM3X8E__ // if using Arduino Due
		SPI.transfer( _slaveSelectPin, reg |= 0xC0, SPI_CONTINUE );
		return  SPI.transfer( _slaveSelectPin, 0x00, SPI_LAST );
	#else
		uint8_t retCode;
		digitalWrite( _slaveSelectPin, LOW );
		SPI.transfer( reg );
		retCode = SPI.transfer( 0x00 );
		digitalWrite( _slaveSelectPin, HIGH );
		return retCode;
	#endif
	}
	else if ( _interfaceMode == MODE_I2C )
	{
		Wire.beginTransmission( ADXL345_ADDRESS );
		i2cwrite( reg );
		Wire.endTransmission();
		Wire.requestFrom( ADXL345_ADDRESS, 1 );
		return ( i2cread() );
	}
}

uint16_t Adafruit_ADXL345::read16( uint8_t reg )
{
	if ( _interfaceMode == MODE_SPI )
	{
		int16_t highByte;
		int16_t lowByte;

	#ifdef __SAM3X8E__ // if using Arduino Due
		//Serial.println( "talking to Arduino Due" );
		SPI.transfer( _slaveSelectPin, reg |= 0xC0, SPI_CONTINUE );
		lowByte = SPI.transfer( _slaveSelectPin, 0x00, SPI_CONTINUE );
		highByte = SPI.transfer( _slaveSelectPin, 0x00, SPI_LAST );
		return lowByte |= ( highByte << 8 );
	#else
		digitalWrite( _slaveSelectPin, LOW );
		SPI.transfer( reg |= 0x08 );
		highByte = SPI.transfer( 0x00 );
		highByte <<= 8 ;
		highByte |= lowByte;
		digitalWrite( _slaveSelectPin, HIGH );
		return highByte;
	#endif
	}
	else if ( _interfaceMode == MODE_I2C)
	{
		Wire.beginTransmission( ADXL345_ADDRESS );
		i2cwrite( reg );
		Wire.endTransmission();
		Wire.requestFrom( ADXL345_ADDRESS, 2 );
		return ( int16_t )( i2cread() | ( i2cread() << 8 ) );
	}
}



