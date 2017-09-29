#include "MPU6050.h"

//if defined, offsetCalc() use these values instead of calibrate the sensor (it takes time...)
//to calculate the offsets, run offsetCalc() without DEFAULT_OFFSETS defined.
//it will print on the serial the values after few moments.
//then copy and paste those values here, as they are (with {})
#define DEFAULT_OFFSETS {74,-321,815,88,3,20}

//Comment to prevent printings
//Remember that you need printf to obtain offsets in offsetCalc()
//#define useDebugSerial

#ifdef useDebugSerial
    Serial debugSerial(USBTX, USBRX,57600);
    #define LN "\r\n"
    #define BREAK debugSerial.printf(LN)
    #define PRINTF debugSerial.printf
    #define PRINTLN(x) debugSerial.printf(x);BREAK
#else
    #define PRINTF(...)
    #define BREAK
    #define PRINTLN(x)
#endif

/** Default constructor, uses default I2C address.
 * @see MPU6050_DEFAULT_ADDRESS
 */
MPU6050::MPU6050()
{
    devAddr = MPU6050_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MPU6050_DEFAULT_ADDRESS
 * @see MPU6050_ADDRESS_AD0_LOW
 * @see MPU6050_ADDRESS_AD0_HIGH
 */
MPU6050::MPU6050(uint8_t address)
{
    devAddr = address;
}

/** 
 * Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their sensitive settings, and sets the clock source to use the X Gyro for
 * reference, which is slightly better than the default internal clock source.
 */
void MPU6050::initialize()
{
    PRINTLN("MPU6050::initialize start");
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    int gyro_sens=MPU6050_GYRO_FS_250;
    int accel_sens=MPU6050_ACCEL_FS_4;
    PRINTF("MPU6050: gyro_sensibility: %d\n",gyro_sens);
    PRINTF("MPU6050: accel_sensibility: %d\n",accel_sens);
    setFullScaleGyroRange(gyro_sens);
    setFullScaleAccelRange(accel_sens);
    setSleepEnabled(false);
    PRINTLN("MPU6050::initialize end");
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU6050::testConnection()
{
    PRINTLN("MPU6050::testConnection start");
    uint8_t deviceId = getDeviceID();
    PRINTF("MPU6050: DeviceId = %d\n",deviceId);
    return deviceId == 0x34;
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050::reset()
{
    i2c_hw.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
uint8_t MPU6050::getClockSource()
{
    i2c_hw.readBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050::setClockSource(uint8_t source)
{
    i2c_hw.writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050::setSleepEnabled(bool enabled)
{
    i2c_hw.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050::getFullScaleGyroRange()
{
    i2c_hw.readBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050::setFullScaleGyroRange(uint8_t range)
{
    i2c_hw.writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// ACCEL_CONFIG register

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050::getFullScaleAccelRange()
{
    i2c_hw.readBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU6050::setFullScaleAccelRange(uint8_t range)
{
    i2c_hw.writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050::getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
int16_t MPU6050::getAccelerationX()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_YOUT_H
 */
int16_t MPU6050::getAccelerationY()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_ZOUT_H
 */
int16_t MPU6050::getAccelerationZ()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050::getRotation(int16_t* x, int16_t* y, int16_t* z)
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
int16_t MPU6050::getRotationX()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
int16_t MPU6050::getRotationY()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
int16_t MPU6050::getRotationZ()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050::getDeviceID()
{
    i2c_hw.readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
void MPU6050::setDeviceID(uint8_t id)
{
    i2c_hw.writeBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

/** Calculates mean values of 1000 readings of raw values (accel and gyro)
 */
void MPU6050::meanValues(int mean[])
{
    long buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
    int16_t ax,ay,az,gx,gy,gz;

    for (long i=0; i<100; i++) //throw away first 100 reads
        getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    for (long i=0; i<1000; i++) {
        getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        wait_ms(2);
        buff_ax+=ax; buff_ay+=ay; buff_az+=az;
        buff_gx+=gx; buff_gy+=gy; buff_gz+=gz;
    }
    mean[0]=buff_ax/1000; mean[1]=buff_ay/1000; mean[2]=buff_az/1000;
    mean[3]=buff_gx/1000; mean[4]=buff_gy/1000; mean[5]=buff_gz/1000;
}

/* Calibration of the sensor
 * It sets every offset, takes a few moments if doesn't use default ones
 * @see DEFAULT_OFFSETS at the top of the file
 *
 * This function runs recursively while the MPU is at rest:
 * every cycle it calculates the average raw values read, and modifies
 * accordingly the offsets
 *
 * It keeps running until:
 *    1) the avg values read by the accelerometer are <8 i.e. acel_deadzone
 *           AND
 *    2) the avg values read by the gyroscope are <1 i.e. giro_deadzone
 */
void MPU6050::offsetCalc()
{
#ifdef DEFAULT_OFFSETS
    int16_t offsets[6]=DEFAULT_OFFSETS;
    setXAccelOffset(offsets[0]); setYAccelOffset(offsets[1]); setZAccelOffset(offsets[2]);
    setXGyroOffset(offsets[3]); setYGyroOffset(offsets[4]); setZGyroOffset(offsets[5]);
    wait_ms(3);
    PRINTF("MPU6050: Using default offsets\n");
#else
    PRINTLN("MPU6050: Calibrating gyroscope .... don't move the hardware ..........");

    int16_t g_value;
    uint8_t currentAcceleroRange=getFullScaleAccelRange();
    switch (currentAcceleroRange) {
        case MPU6050_ACCEL_FS_2: //case 0
            g_value= 16384; //value of 1g in 2g sensibility
            break;
        case MPU6050_ACCEL_FS_4: //case 1
            g_value= 8192; //value of 1g in 4g sensibility
            break;
        case MPU6050_ACCEL_FS_8: //case 2
            g_value=4096; //value of 1g in 8g sensibility
            break;
        case MPU6050_ACCEL_FS_16: //case 3
            g_value= 2048; //value of 1g in 16g sensibility
            break;
    }

    //reset offsets before calibrating
    setXAccelOffset(0); setYAccelOffset(0); setZAccelOffset(0);
    setXGyroOffset(0); setYGyroOffset(0); setZGyroOffset(0);

    int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset=0;
    int acel_deadzone=8;
    int giro_deadzone=1;
    int mean[6];
    meanValues(mean);
    ax_offset=-mean[0]/8;
    ay_offset=-mean[1]/8;
    az_offset+=(g_value-mean[2])/8; //doesn't count the gravity

    gx_offset=-mean[3]/4;
    gy_offset=-mean[4]/4;
    gz_offset=-mean[5]/4;
    int ready=0;
    while (ready!=6) {
        ready=0;
        setXAccelOffset(ax_offset); setYAccelOffset(ay_offset); setZAccelOffset(az_offset);
        setXGyroOffset(gx_offset); setYGyroOffset(gy_offset); setZGyroOffset(gz_offset);

        meanValues(mean);

        PRINTF("MPU6050: ...");
        PRINTF("offsets{%d,%d,%d,%d,%d,%d}",ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset);
        PRINTF("mean{%d,%d,%d,%d,%d,%d}...\n",mean[0],mean[1],mean[2],mean[3],mean[4],mean[5]);

        if (abs(mean[0])<=acel_deadzone)
            ready++;
        else
            ax_offset-=mean[0]/acel_deadzone;

        if (abs(mean[1])<=acel_deadzone)
            ready++;
        else
            ay_offset-=mean[1]/acel_deadzone;

        if (abs(g_value-mean[2])<=acel_deadzone)
            ready++;
        else
            az_offset+=(g_value-mean[2])/acel_deadzone;

        if (abs(mean[3])<=giro_deadzone)
            ready++;
        else
            gx_offset-=mean[3]/(giro_deadzone+1);

        if (abs(mean[4])<=giro_deadzone)
            ready++;
        else
            gy_offset-=mean[4]/(giro_deadzone+1);

        if (abs(mean[5])<=giro_deadzone)
            ready++;
        else
            gz_offset-=mean[5]/(giro_deadzone+1);
    }
#endif
}

//Offsets Registers
// XA_OFFS registers
int16_t MPU6050::getXAccelOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_XA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXAccelOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_XA_OFFS_H, 2, arr);
}

// YA_OFFS register
int16_t MPU6050::getYAccelOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_YA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYAccelOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_YA_OFFS_H, 2, arr);
}

// ZA_OFFS register
int16_t MPU6050::getZAccelOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ZA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZAccelOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_ZA_OFFS_H, 2, arr);
}

// XG_OFFS registers
int16_t MPU6050::getXGyroOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXGyroOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, arr);
}

// YG_OFFS register
int16_t MPU6050::getYGyroOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYGyroOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, arr);
}

// ZG_OFFS register
int16_t MPU6050::getZGyroOffset()
{
    i2c_hw.readBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZGyroOffset(int16_t offset)
{
    int8_t zeroToSeven=offset;        //bits --------76543210
    int8_t eightToFifteen=offset>>8;    //bits 76543210--------
    uint8_t arr[2];
    arr[1]=(uint8_t)zeroToSeven;
    arr[0]=(uint8_t)eightToFifteen;
    i2c_hw.writeBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, arr);
}

float ypr_angles[3];

/*
 * @return the pointer to the array of angles
 */
float* MPU6050::getAngles(){
    return ypr_angles;    
}


float looptime=0.004;
kalman filter_x, filter_y;
bool kalman_ready, complementary_ready=false;

/** Called by a timer, sets the kalman flag to true
 */
void MPU6050::kalman_flag(){
    kalman_ready=true;    
}

/** Called by calculate_filter() when flags are true
 * It does the calculations
 */
void MPU6050::kalman_filter(){
    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    //Get raw data
    getMotion6(acc_raw,acc_raw+1,acc_raw+2,gyr_raw,gyr_raw+1,gyr_raw+2);
    
    //accelerometer ypr_angles that we will use as parameters of kalman_calculate method
    double angle_acc[2]; //1 ROLL AND 0 PITCH
    angle_acc[1]=atan2((double)acc_raw[1], (double)acc_raw[2])*57.2957786; //to convert in degrees
    angle_acc[0]=atan2(-(double)acc_raw[0], (double)acc_raw[2])*57.2957786; //to convert in degrees
    
    kalman_predict(&filter_x, (double)gyr_raw[1]/32767, looptime);
    kalman_predict(&filter_y, (double)gyr_raw[0]/32767, looptime);
    kalman_update(&filter_x, angle_acc[1]);
    kalman_update(&filter_y, angle_acc[0]);
    ypr_angles[0]=kalman_get_angle(&filter_y); //pitch angle filtered 
    ypr_angles[1]=kalman_get_angle(&filter_x); //roll angle filtered
    ypr_angles[2]=gyr_raw[2]; //gyro z raw value
}

/** Called by a timer, sets the complementary flag to true
 */
void MPU6050::complementary_flag(){
    complementary_ready=true;    
}

/** Called by calculate_filter() when flags are true
 * It does the calculations
 */
void MPU6050::complementary_filter() //loop frequency 250Hz
{
    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    static bool first_start=true;
    static float angle_pitch,angle_roll;
    float angle_pitch_acc,angle_roll_acc;
    
    //Get raw data
    getMotion6(acc_raw,acc_raw+1,acc_raw+2,gyr_raw,gyr_raw+1,gyr_raw+2);
    
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz*131)
    angle_pitch += gyr_raw[0] * 0.0000611;                                //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyr_raw[1] * 0.0000611;                                 //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) sin function is in radians
    angle_pitch += angle_roll * sin(gyr_raw[2] * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyr_raw[2] * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    long acc_total_vector = (long) sqrt((float)(acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));  //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    if(abs(acc_raw[1])<acc_total_vector)
        angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector)* 57.296;       //Calculate the pitch angle
    if(abs(acc_raw[0])<acc_total_vector)
        angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector)* -57.296;       //Calculate the roll angle

    if(!first_start) {                                                //If the IMU is already started
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
        
    } else {                                                             //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
        first_start = false;                                            //Set the IMU started flag
        ypr_angles[0]=ypr_angles[1]=0;
    }

    //To dampen the pitch and roll ypr_angles a complementary filter is used
    ypr_angles[0] = ypr_angles[0] * 0.9 + angle_roll * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    ypr_angles[1] = ypr_angles[1] * 0.9 + angle_pitch * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    ypr_angles[2] = gyr_raw[2]; //gyro z raw value
}

/** Initialize the filters and the timers for the calculations
 * @param type 0,1 for kalman or complementary first order
 * @param dt period of repeat (loop time)
 * These timers call the functions that set to true periodically (every dt) the flags of the filters
 */
void MPU6050::initialize_timer(short type, float dt){
    switch (type){
        case KALMAN:{  //kalman filter
            /*const float Q_angle  =  0.01; //0.001   //0.005
            const float Q_gyro   =  0.0003;  //0.003   //0.0003
            const float R_angle  =  0.01;  //0.03   //0.008
            */
            kalman_init(&filter_x, 0.01, 0.03, 0.008);
            kalman_init(&filter_y, 0.01, 0.03, 0.008);
            timer.attach(this, &MPU6050::kalman_flag, dt);
            break;
        }
        case COMPLEMENTARY:{  //first order complementary filter
            timer.attach(this, &MPU6050::complementary_flag, dt);
            break;
        }
        
    }
}

/** Checks if there is something ready to calculate (kalman or complementary)
 * Must run every cycle
 * Uses flags of kalman and complementary
 */
void MPU6050::calculate_filter(){
    if(kalman_ready){
        kalman_ready=false;
        kalman_filter();
    }
    else if(complementary_ready){
        complementary_ready=false;
        complementary_filter();   
    }
}
