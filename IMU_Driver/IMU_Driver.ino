
#include <Wire.h>

#define D_SA0_HIGH_ADDRESS                0b0011101
#define D_SA0_LOW_ADDRESS                 0b0011110
#define DLHC_DLM_DLH_MAG_ADDRESS          0b0011110
#define DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0b0011001
#define DLM_DLH_ACC_SA0_LOW_ADDRESS       0b0011000

#define TEST_REG_ERROR -1

#define D_WHO_ID    0x49
#define DLM_WHO_ID  0x3C

class IMU
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DLH, device_DLM, device_DLHC, device_D, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    // register addresses
    enum regAddr
    {
      TEMP_OUT_L        = 0x05, // D
      TEMP_OUT_H        = 0x06, // D

      STATUS_M          = 0x07, // D

      INT_CTRL_M        = 0x12, // D
      INT_SRC_M         = 0x13, // D
      INT_THS_L_M       = 0x14, // D
      INT_THS_H_M       = 0x15, // D

      OFFSET_X_L_M      = 0x16, // D
      OFFSET_X_H_M      = 0x17, // D
      OFFSET_Y_L_M      = 0x18, // D
      OFFSET_Y_H_M      = 0x19, // D
      OFFSET_Z_L_M      = 0x1A, // D
      OFFSET_Z_H_M      = 0x1B, // D
      REFERENCE_X       = 0x1C, // D
      REFERENCE_Y       = 0x1D, // D
      REFERENCE_Z       = 0x1E, // D

      CTRL0             = 0x1F, // D
      CTRL1             = 0x20, // D
      CTRL_REG1_A       = 0x20, // DLH, DLM, DLHC
      CTRL2             = 0x21, // D
      CTRL_REG2_A       = 0x21, // DLH, DLM, DLHC
      CTRL3             = 0x22, // D
      CTRL_REG3_A       = 0x22, // DLH, DLM, DLHC
      CTRL4             = 0x23, // D
      CTRL_REG4_A       = 0x23, // DLH, DLM, DLHC
      CTRL5             = 0x24, // D
      CTRL_REG5_A       = 0x24, // DLH, DLM, DLHC
      CTRL6             = 0x25, // D
      CTRL_REG6_A       = 0x25, // DLHC
      HP_FILTER_RESET_A = 0x25, // DLH, DLM
      CTRL7             = 0x26, // D
      REFERENCE_A       = 0x26, // DLH, DLM, DLHC
      STATUS_A          = 0x27, // D
      STATUS_REG_A      = 0x27, // DLH, DLM, DLHC

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL         = 0x2E, // D
      FIFO_CTRL_REG_A   = 0x2E, // DLHC
      FIFO_SRC          = 0x2F, // D
      FIFO_SRC_REG_A    = 0x2F, // DLHC

      IG_CFG1           = 0x30, // D
      INT1_CFG_A        = 0x30, // DLH, DLM, DLHC
      IG_SRC1           = 0x31, // D
      INT1_SRC_A        = 0x31, // DLH, DLM, DLHC
      IG_THS1           = 0x32, // D
      INT1_THS_A        = 0x32, // DLH, DLM, DLHC
      IG_DUR1           = 0x33, // D
      INT1_DURATION_A   = 0x33, // DLH, DLM, DLHC
      IG_CFG2           = 0x34, // D
      INT2_CFG_A        = 0x34, // DLH, DLM, DLHC
      IG_SRC2           = 0x35, // D
      INT2_SRC_A        = 0x35, // DLH, DLM, DLHC
      IG_THS2           = 0x36, // D
      INT2_THS_A        = 0x36, // DLH, DLM, DLHC
      IG_DUR2           = 0x37, // D
      INT2_DURATION_A   = 0x37, // DLH, DLM, DLHC

      CLICK_CFG         = 0x38, // D
      CLICK_CFG_A       = 0x38, // DLHC
      CLICK_SRC         = 0x39, // D
      CLICK_SRC_A       = 0x39, // DLHC
      CLICK_THS         = 0x3A, // D
      CLICK_THS_A       = 0x3A, // DLHC
      TIME_LIMIT        = 0x3B, // D
      TIME_LIMIT_A      = 0x3B, // DLHC
      TIME_LATENCY      = 0x3C, // D
      TIME_LATENCY_A    = 0x3C, // DLHC
      TIME_WINDOW       = 0x3D, // D
      TIME_WINDOW_A     = 0x3D, // DLHC

      Act_THS           = 0x3E, // D
      Act_DUR           = 0x3F, // D

      CRA_REG_M         = 0x00, // DLH, DLM, DLHC
      CRB_REG_M         = 0x01, // DLH, DLM, DLHC
      MR_REG_M          = 0x02, // DLH, DLM, DLHC

      SR_REG_M          = 0x09, // DLH, DLM, DLHC
      IRA_REG_M         = 0x0A, // DLH, DLM, DLHC
      IRB_REG_M         = 0x0B, // DLH, DLM, DLHC
      IRC_REG_M         = 0x0C, // DLH, DLM, DLHC

      WHO_AM_I          = 0x0F, // D
      WHO_AM_I_M        = 0x0F, // DLM

      TEMP_OUT_H_M      = 0x31, // DLHC
      TEMP_OUT_L_M      = 0x32, // DLHC


      // dummy addresses for registers in different locations on different devices;
      // the library translates these based on device type
      // value with sign flipped is used as index into translated_regs array

      OUT_X_H_M         = -1,
      OUT_X_L_M         = -2,
      OUT_Y_H_M         = -3,
      OUT_Y_L_M         = -4,
      OUT_Z_H_M         = -5,
      OUT_Z_L_M         = -6,
      // update dummy_reg_count if registers are added here!

      // device-specific register addresses

      DLH_OUT_X_H_M     = 0x03,
      DLH_OUT_X_L_M     = 0x04,
      DLH_OUT_Y_H_M     = 0x05,
      DLH_OUT_Y_L_M     = 0x06,
      DLH_OUT_Z_H_M     = 0x07,
      DLH_OUT_Z_L_M     = 0x08,

      DLM_OUT_X_H_M     = 0x03,
      DLM_OUT_X_L_M     = 0x04,
      DLM_OUT_Z_H_M     = 0x05,
      DLM_OUT_Z_L_M     = 0x06,
      DLM_OUT_Y_H_M     = 0x07,
      DLM_OUT_Y_L_M     = 0x08,

      DLHC_OUT_X_H_M    = 0x03,
      DLHC_OUT_X_L_M    = 0x04,
      DLHC_OUT_Z_H_M    = 0x05,
      DLHC_OUT_Z_L_M    = 0x06,
      DLHC_OUT_Y_H_M    = 0x07,
      DLHC_OUT_Y_L_M    = 0x08,

      D_OUT_X_L_M       = 0x08,
      D_OUT_X_H_M       = 0x09,
      D_OUT_Y_L_M       = 0x0A,
      D_OUT_Y_H_M       = 0x0B,
      D_OUT_Z_L_M       = 0x0C,
      D_OUT_Z_H_M       = 0x0D
    };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings
    vector<int16_t> m_max; // maximum magnetometer values, used for calibration
    vector<int16_t> m_min; // minimum magnetometer values, used for calibration

    byte last_status; // status of last I2C transmission

    IMU(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);

    byte readReg(byte reg);

    void writeReg(byte reg, byte value);
    byte readReg(int reg);

    void read(void);

  private:
    deviceType _device; // chip type (D, DLHC, DLM, or DLH)
    byte acc_address;
    byte mag_address;

    static const int dummy_reg_count = 6;
    regAddr translated_regs[dummy_reg_count + 1]; // index 0 not used

    unsigned int io_timeout;
    bool did_timeout;

    int testReg(byte address, regAddr reg);
};



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

IMU sensor;
char report[80];


void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.enableDefault();

}

void loop() {
  sensor.read();
  snprintf(report, sizeof(report), "A: %6d %6d %6d", sensor.a.x, sensor.a.y, sensor.a.z);
  Serial.println(report);

}
