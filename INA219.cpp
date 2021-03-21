/******************************************************************************
* TI INA219 hi-side i2c current/power monitor Library
*
* http://www.ti.com/product/ina219
*
* 6 May 2012 by John De Cristofaro
*
*
* Tested at standard i2c 100kbps signaling rate.
*
* This library does not handle triggered conversion modes. It uses the INA219
* in continuous conversion mode. All reads are from continous conversions.
*
* A note about the gain (PGA) setting:
*	The gain of the ADC pre-amplifier is programmable in the INA219, and can
*	be set between 1/8x (default) and unity. This allows a shunt voltage 
*	range of +/-320mV to +/-40mV respectively. Something to keep in mind,
*	however, is that this change in gain DOES NOT affect the resolution
*	of the ADC, which is fixed at 1uV. What it does do is increase noise
*	immunity by exploiting the integrative nature of the delta-sigma ADC.
*	For the best possible reading, you should set the gain to the range
*	of voltages that you expect to see in your particular circuit. See
*	page 15 in the datasheet for more info about the PGA.
*
* Known bugs:
*     * may return unreliable values if not connected to a bus or at
*	bus currents below 10uA.
*
* Arduino 1.0 compatible as of 6/6/2012
*
* Dependencies:
*    * Arduino Wire library
*
* MIT license
******************************************************************************/

#include "INA219.h"

#if defined(ESP8266) || defined(ESP32)
  #define _delay_ms(ms) delayMicroseconds((ms) * 1000)
#endif

#ifdef __avr__
  #include <util/delay.h>
#endif

namespace{
// config. register bit labels
const uint8_t RST =	15;
const uint8_t BRNG = 13;
const uint8_t PG_1 = 12;
const uint8_t PG_0 = 11;
const uint8_t BADC4 = 10;
const uint8_t BADC3	= 9;
const uint8_t BADC2	= 8;
const uint8_t BADC1	= 7;
const uint8_t SADC4	= 6;
const uint8_t SADC3	= 5;
const uint8_t SADC2	= 4;
const uint8_t SADC1	= 3;
const uint8_t MODE3	= 2;
const uint8_t MODE2	= 1;
const uint8_t MODE1	= 0;
};

#define CNVR_B 1  // conversion ready bit in bus voltage register V_BUS_R 
#define OVF_B  0  // math overflow bit in bus voltage register V_BUS_R 

INA219::INA219(t_i2caddr addr): i2c_address(addr) {
}

uint8_t INA219::begin() {
  uint8_t ret;
    Wire.begin();
    ret = reset();
    if (ret)
      return ret;  // Likely failed to address/find the device
    configure();
    calibrate();

    return ret;
}

void INA219::calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected) {
    uint16_t digits;
    float min_lsb, swap;
#ifdef INA219_DEBUG
    float max_current,max_before_overflow,max_shunt_v,max_shunt_v_before_overflow,max_power,i_max_possible,max_lsb;
#endif

    r_shunt = shunt_val;

    min_lsb = i_max_expected / 32767;

    current_lsb = min_lsb;
    digits=0;

    /* From datasheet: This value was selected to be a round number near the Minimum_LSB.
     * This selection allows for good resolution with a rounded LSB.
	   * eg. 0.000610 -> 0.000700
	   */
    while( current_lsb > 0.0 ){//If zero there is something weird...
        if( (uint16_t)current_lsb / 1){
        	current_lsb = (uint16_t) current_lsb + 1;
        	current_lsb /= pow(10,digits);
        	break;
        }
        else{
        	digits++;
            current_lsb *= 10.0;
        }
    };

    swap = (0.04096)/(current_lsb*r_shunt);
    cal = (uint16_t)swap;
    power_lsb = current_lsb * 20;

#ifdef INA219_DEBUG
      i_max_possible = v_shunt_max / r_shunt;
      max_lsb = i_max_expected / 4096;
      max_current = current_lsb*32767;
      max_before_overflow =  max_current > i_max_possible?i_max_possible:max_current;

      max_shunt_v = max_before_overflow*r_shunt;
      max_shunt_v_before_overflow = max_shunt_v > v_shunt_max?v_shunt_max:max_shunt_v;

      max_power = v_bus_max * max_before_overflow;
      Serial.print("v_bus_max:     "); Serial.println(v_bus_max, 8);
      Serial.print("v_shunt_max:   "); Serial.println(v_shunt_max, 8);
      Serial.print("i_max_possible:        "); Serial.println(i_max_possible, 8);
      Serial.print("i_max_expected: "); Serial.println(i_max_expected, 8);
      Serial.print("min_lsb:       "); Serial.println(min_lsb, 12);
      Serial.print("max_lsb:       "); Serial.println(max_lsb, 12);
      Serial.print("current_lsb:   "); Serial.println(current_lsb, 12);
      Serial.print("power_lsb:     "); Serial.println(power_lsb, 8);
      Serial.println("  ");
      Serial.print("cal:           "); Serial.println(cal);
      Serial.print("r_shunt:       "); Serial.println(r_shunt, 6);
      Serial.print("max_before_overflow:       "); Serial.println(max_before_overflow,8);
      Serial.print("max_shunt_v_before_overflow:       "); Serial.println(max_shunt_v_before_overflow,8);
      Serial.print("max_power:       "); Serial.println(max_power,8);
      Serial.println("  ");
#endif
      write16(CAL_R, cal);
}

void INA219::configure(  t_range range,  t_gain gain,  t_adc  bus_adc,  t_adc shunt_adc,  t_mode mode) {
  config = 0;

  config |= (range << BRNG | gain << PG_0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);
#ifdef INA219_DEBUG
  Serial.print("Config: 0x"); Serial.println(config,HEX);
#endif
  write16(CONFIG_R, config);
}

#define INA_RESET        0xFFFF    // send to CONFIG_R to reset unit
// Returns non-zero if unable to reset.
// This is likely due to device not being functional/installed on I2C bus
// Returned error values are defined in the Wire libraries.
// Values of 2 indicates the address wasn't on the bus (NAK response)
uint8_t INA219::reset(){
  uint8_t ret = write16(CONFIG_R, INA_RESET);
  _delay_ms(5);
  return ret;
}

int16_t INA219::shuntVoltageRaw() const {
  return read16(V_SHUNT_R);
}

float INA219::shuntVoltage() const {
  float temp;
  temp = read16(V_SHUNT_R);
  return (temp / 100000);
}

uint16_t INA219::busVoltageRaw() {
  uint16_t bus_voltage_register = read16(V_BUS_R);
  _overflow = bitRead(bus_voltage_register, OVF_B);     // overflow bit
  _ready    = bitRead(bus_voltage_register, CNVR_B);    // ready bit
  return bus_voltage_register;
}


float INA219::busVoltage() {
  uint16_t temp;
  temp = busVoltageRaw();
  temp >>= 3;
  return (temp * 0.004);
}

int16_t INA219::shuntCurrentRaw() const {
  return (read16(I_SHUNT_R));
}

float INA219::shuntCurrent() const {
  return (read16(I_SHUNT_R) * current_lsb);
}

float INA219::busPower() const {
  return (read16(P_BUS_R) * power_lsb);
}

/**************************************************************************/
/*! 
    @brief  Rewrites the last config register
*/
/**************************************************************************/
void INA219::reconfig() const {
#ifdef INA219_DEBUG
  Serial.print("Reconfigure with Config: 0x"); Serial.println(config,HEX);
#endif
  write16(CONFIG_R, config);
}

/**************************************************************************/
/*! 
    @brief  Rewrites the last calibration
*/
/**************************************************************************/
void INA219::recalibrate() const {
#ifdef INA219_DEBUG
  Serial.print("Recalibrate with cal: "); Serial.println(cal);
#endif
  write16(CAL_R, cal);
}

/**************************************************************************/
/*! 
    @brief  returns conversion ready bite from last bus voltage read
    
    @note page 30:
          Although the data from the last conversion can be read at any time,
          the INA219 Conversion Ready bit (CNVR) indicates when data from
          a conversion is available in the data output registers.
          The CNVR bit is set after all conversions, averaging, 
          and multiplications are complete.
          CNVR will clear under the following conditions:
          1.) Writing a new mode into the Operating Mode bits in the 
              Configuration Register (except for Power-Down or Disable)
          2.) Reading the Power Register
          
          page 15:
          The Conversion Ready bit clears under these
          conditions:
          1. Writing to the Configuration Register, except
          when configuring the MODE bits for Power Down
          or ADC off (Disable) modes;
          2. Reading the Status Register;
          3. Triggering a single-shot conversion with the
          Convert pin.
*/
/**************************************************************************/
bool INA219::ready() const {
  return _ready;
}

/**************************************************************************/
/*! 
    @brief  returns overflow bite from last bus voltage read
    
    @note The Math Overflow Flag (OVF) is set when the Power or
          Current calculations are out of range. It indicates that
          current and power data may be meaningless.
*/
/**************************************************************************/
bool INA219::overflow() const {
  return _overflow;
}

/**********************************************************************
*             INTERNAL I2C FUNCTIONS                  *
**********************************************************************/

// write16 returns a non-zero value in the case of a transmission error.
uint8_t INA219::write16(t_reg a, uint16_t d) const {
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;
  Wire.beginTransmission(i2c_address); // start transmission to device

  #if ARDUINO >= 100
    Wire.write(a); // sends register address to read from
    Wire.write((uint8_t)d);  // write data hibyte 
    Wire.write(temp); // write data lobyte;
  #else
    Wire.send(a); // sends register address to read from
    Wire.send((uint8_t)d);  // write data hibyte 
    Wire.send(temp); // write data lobyte;
  #endif

  temp = Wire.endTransmission(); // end transmission
  _delay_ms(1);
  return temp;
}

int16_t INA219::read16(t_reg a) const {
  uint16_t ret;

  // move the pointer to reg. of interest, null argument
  write16(a, 0);
  
  Wire.requestFrom((int)i2c_address, 2);    // request 2 data bytes

  #if ARDUINO >= 100
    ret = Wire.read(); // rx hi byte
    ret <<= 8;
    ret |= Wire.read(); // rx lo byte
  #else
    ret = Wire.receive(); // rx hi byte
    ret <<= 8;
    ret |= Wire.receive(); // rx lo byte
  #endif

  return ret;
}
