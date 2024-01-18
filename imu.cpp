#include <Wire.h>
#include <imu.h>

#define D_SA0_HIGH_ADDRESS                0b0011101
#define D_SA0_LOW_ADDRESS                 0b0011110
#define DLHC_DLM_DLH_MAG_ADDRESS          0b0011110
#define DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0b0011001
#define DLM_DLH_ACC_SA0_LOW_ADDRESS       0b0011000

#define TEST_REG_ERROR -1

#define D_WHO_ID    0x49
#define DLM_WHO_ID  0x3C

IMU::IMU(void)
{
  /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
  m_min = (IMU::vector<int16_t>){-32767, -32767, -32767};
  m_max = (IMU::vector<int16_t>){+32767, +32767, +32767};

  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

bool IMU::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM303D if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_D)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(D_SA0_HIGH_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        // device responds to address 0011101 with D ID; it's a D with SA0 high
        device = device_D;
        sa0 = sa0_high;
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(D_SA0_LOW_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        // device responds to address 0011110 with D ID; it's a D with SA0 low
        device = device_D;
        sa0 = sa0_low;
      }
    }
    
    // check for LSM303DLHC, DLM, DLH if device is still unidentified or was specified to be one of these types
    if (device == device_auto || device == device_DLHC || device == device_DLM || device == device_DLH)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS, CTRL_REG1_A) != TEST_REG_ERROR)
      {
        // device responds to address 0011001; it's a DLHC, DLM with SA0 high, or DLH with SA0 high
        sa0 = sa0_high;
        if (device == device_auto)
        { 
          // use magnetometer WHO_AM_I register to determine device type
          //
          // DLHC seems to respond to WHO_AM_I request the same way as DLM, even though this
          // register isn't documented in its datasheet. Since the DLHC accelerometer address is the
          // same as the DLM with SA0 high, but Pololu DLM boards pull SA0 low by default, we'll
          // guess that a device whose accelerometer responds to the SA0 high address and whose
          // magnetometer gives the DLM ID is actually a DLHC.
          device = (testReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID) ? device_DLHC : device_DLH;
        }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DLM_DLH_ACC_SA0_LOW_ADDRESS, CTRL_REG1_A) != TEST_REG_ERROR)
      {
        // device responds to address 0011000; it's a DLM with SA0 low or DLH with SA0 low
        sa0 = sa0_low;
        if (device == device_auto)
        {
          // use magnetometer WHO_AM_I register to determine device type
          device = (testReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID) ? device_DLM : device_DLH;
        }
      }
    }
    
    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }
  
  _device = device;
  
  // set device addresses and translated register addresses
  switch (device)
  {
    case device_D:
      acc_address = mag_address = (sa0 == sa0_high) ? D_SA0_HIGH_ADDRESS : D_SA0_LOW_ADDRESS;
      translated_regs[-OUT_X_L_M] = D_OUT_X_L_M;
      translated_regs[-OUT_X_H_M] = D_OUT_X_H_M;
      translated_regs[-OUT_Y_L_M] = D_OUT_Y_L_M;
      translated_regs[-OUT_Y_H_M] = D_OUT_Y_H_M;
      translated_regs[-OUT_Z_L_M] = D_OUT_Z_L_M;
      translated_regs[-OUT_Z_H_M] = D_OUT_Z_H_M;
      break;

    case device_DLHC:
      acc_address = DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS; // DLHC doesn't have configurable SA0 but uses same acc address as DLM/DLH with SA0 high
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLHC_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLHC_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLHC_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLHC_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLHC_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLHC_OUT_Z_L_M;
      break;

    case device_DLM:
      acc_address = (sa0 == sa0_high) ? DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLM_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLM_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLM_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLM_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLM_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLM_OUT_Z_L_M;
      break;

    case device_DLH:
      acc_address = (sa0 == sa0_high) ? DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLH_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLH_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLH_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLH_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLH_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLH_OUT_Z_L_M;
      break;
  }
  
  return true;
}


void IMU::read(void){
    Wire.beginTransmission(acc_address);
    // assert the MSB of the address to get the accelerometer
    // to do slave-transmit subaddress updating.
    Wire.write(OUT_X_L_A | (1 << 7));
    last_status = Wire.endTransmission();
    Wire.requestFrom(acc_address, (byte)6);

    unsigned int millis_start = millis();
    while (Wire.available() < 6) {
      if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
      {
        did_timeout = true;
        return;
      }
    }

    byte xla = Wire.read();
    byte xha = Wire.read();
    byte yla = Wire.read();
    byte yha = Wire.read();
    byte zla = Wire.read();
    byte zha = Wire.read();

    // combine high and low bytes
    // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
    // (12-bit resolution, left-aligned). The D has 16-bit resolution
    a.x = (int16_t)(xha << 8 | xla);
    a.y = (int16_t)(yha << 8 | yla);
    a.z = (int16_t)(zha << 8 | zla);
}

byte IMU::readReg(byte reg)
{
  byte value;

  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void IMU::writeReg(byte reg, byte value)
{
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

int IMU::testReg(byte address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}

void IMU::enableDefault(void)
{

  if (_device == device_D)
  {
    // Accelerometer

    // 0x00 = 0b00000000
    // AFS = 0 (+/- 2 g full scale)
    writeReg(CTRL2, 0x00);

    // 0x57 = 0b01010111
    // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    writeReg(CTRL1, 0x57);

    // Magnetometer

    // 0x64 = 0b01100100
    // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    writeReg(CTRL5, 0x64);

    // 0x20 = 0b00100000
    // MFS = 01 (+/- 4 gauss full scale)
    writeReg(CTRL6, 0x20);

    // 0x00 = 0b00000000
    // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    writeReg(CTRL7, 0x00);
  }
  else
  {
    // Accelerometer
    
    if (_device == device_DLHC)
    {
      // 0x08 = 0b00001000
      // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
      writeReg(CTRL_REG4_A, 0x08);

      // 0x47 = 0b01000111
      // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
      writeReg(CTRL_REG1_A, 0x47);
    }
    else // DLM, DLH
    {
      // 0x00 = 0b00000000
      // FS = 00 (+/- 2 g full scale)
      writeReg(CTRL_REG4_A, 0x00);

      // 0x27 = 0b00100111
      // PM = 001 (normal mode); DR = 00 (50 Hz ODR); Zen = Yen = Xen = 1 (all axes enabled)
      writeReg(CTRL_REG1_A, 0x27);
    }
  }
}