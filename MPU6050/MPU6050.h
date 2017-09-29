#include "I2C_hw.h"
#include "kalman.h"
#include "inttypes.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

// Offsets
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

// Raw Data
// Accelerometer
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
// Gyroscope
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

// Gyroscope sensitivity
#define MPU6050_GYRO_FS_250         0x00 //250  deg/sec --- WE USE THIS ONE
#define MPU6050_GYRO_FS_500         0x01 //500  deg/sec
#define MPU6050_GYRO_FS_1000        0x02 //1000 deg/sec
#define MPU6050_GYRO_FS_2000        0x03 //2000 deg/sec

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

// Accelerometer sensitivity
#define MPU6050_ACCEL_FS_2          0x00 //+- 2  g
#define MPU6050_ACCEL_FS_4          0x01 //+- 4  g --- WE USE THIS ONE
#define MPU6050_ACCEL_FS_8          0x02 //+- 8  g
#define MPU6050_ACCEL_FS_16         0x03 //+- 16 g

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define KALMAN 0
#define COMPLEMENTARY 1

class MPU6050 {
    private:
        I2C_hw i2c_hw;
        Ticker timer;
        
    public:
        MPU6050();
        MPU6050(uint8_t address);

        void initialize();
        bool testConnection();
        void reset();
        
        uint8_t getClockSource();
        void setClockSource(uint8_t source);
        void setSleepEnabled(bool enabled);

        // GYRO_CONFIG register
        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);

        // ACCEL_CONFIG register
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);

        // ACCEL_*OUT_* registers
        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        // GYRO_*OUT_* registers
        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);
        
        // Calibration
        // Calculates mean values of 1000 readings of raw values (accel and gyro)
        void meanValues(int mean[]);
        //It sets every offset, takes a few moments if doesn't use default ones (see MPU6050.cpp)
        void offsetCalc();
        
        // Offsets Registers
        // Accelerometer
        int16_t getXAccelOffset();
        void setXAccelOffset(int16_t offset);
        int16_t getYAccelOffset();
        void setYAccelOffset(int16_t offset);
        int16_t getZAccelOffset();
        void setZAccelOffset(int16_t offset);
        // Gyroscope
        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);
        
        // Return a pointer to the angles
        float* getAngles();
        
        // Sets kalman_flag true
        void kalman_flag();
        // Sets complementary_flag true
        void complementary_flag();
        
        // These two functions are called from calculate_filter() when flags are true
        // They do the calculations
        void kalman_filter();
        void complementary_filter();
                
        /** Initialize the filters and the timers for the calculations
        * @param type 0,1 for kalman or complementary first order
        * @param dt period of repeat (loop time)
        * These timers call the functions that set to true periodically (every dt) the flags of the filters
        */
        void initialize_timer(short type, float dt);
        
        /** Checks if there is something ready to calculate (kalman or complementary)
         * Must run every cycle
         * Uses flags of kalman and complementary
         */
        void calculate_filter();
        
    private:
        uint8_t devAddr;
        uint8_t buffer[14]; //the class uses this when reading in i2c
};
