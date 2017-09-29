#include "mbed.h"
#include "MPU6050.h"
#include "esc.h"
#include "ultrasonic.h"
#include "receiverIR.h"

    //uncomment this if the drone is connected to the pc
    //#define PC
    
    #define LOOP_TIME 0.004  //duration of loops with frequency of 250Hz
    
    #define ESC_DW_LX D5
    #define ESC_UP_RX D9
    #define ESC_DW_RX D3
    #define ESC_UP_LX D6
    #define LED_IR D13
    #define PIN_IR D8
    #define ULTRA_TRIG D12
    #define ULTRA_ECHO D11

#ifdef PC
    Serial pc(USBTX, USBRX,57600);
    #define LN "\r\n"
    #define BREAK pc.printf(LN)
    
    #define PRINTF pc.printf
    #define PRINTLN(x) pc.printf(x);BREAK
#else
    #define PRINTF(...)
    #define BREAK
    #define PRINTLN(x)
#endif

MPU6050 mpu;
ESC motors(ESC_DW_RX,ESC_DW_LX,ESC_UP_LX,ESC_UP_RX);
Ultrasonic ultra(ULTRA_TRIG, ULTRA_ECHO, 0.1, 0.5);
ReceiverIR ir(PIN_IR);
DigitalIn click(USER_BUTTON);
DigitalOut led_ir(LED_IR,1);
DigitalOut led_running(D2,1); //the led on the board indicates that the loop is running
Timer time_count; //attention, this timer overflows after 30 minutes


volatile float* angles; //the array that will contain the filtered angles
static float altitude; //TODO the altitude measured by ultrasonic sensor
static float speed=0.0; //this is the baseline speed, it is shared between the motors
static float output_roll, output_pitch, output_yaw;
static float integral_roll, integral_pitch, integral_yaw=0;
static float error_val, last_roll_differential_error, last_pitch_differential_error,last_yaw_differential_error;

static float proportional_gain=0; //equals for pitch and roll
static float integral_gain=0;     //equals for pitch and roll
static float differential_gain=0; //equals for pitch and roll

//for yaw it's useless set up a complete and precise PID, even only proportional is enough
static float proportional_gain_yaw=0;
static float integral_gain_yaw=0;
static float differential_gain_yaw=0;

//the "/3" must remain, because we have three PIDs and the final value is the sum of them
//therefore you change only the value in the parentheses and it will be the maximum value
//of the pid output (max 1 because the ESCs have max speed =1)
float max_pid=(1)/3;               //equals for all PIDs
float max_pid_integral=(1)/3;

long loop_count=0;

/**
 * Catches an ir signal and takes an action accordingly
 * @input If you want to add other signals, use ir.decode_data to find
 *        the codes of your signals. Then add them in the switch structure.
 * 
 * You can always use the EMERGENCY RESET to shutdown the drone
 */
void catch_ir(){
    static uint32_t last_time=0;
    //if a signal is detected
    if (time_count.read_ms()-last_time>200) //the led stays on for 200ms
        led_ir=0;
    long data = ir.decode_data(); //decode_data returns -1 if received nothing
    if (data==-1)
        return;
    uint32_t now=time_count.read_ms();
    if (now-last_time<300) //max one signal every 300ms
        return;
    switch(data){
        case 0x19E60CF3: //increase speed received from Android
            speed+=0.05;
            PRINTF("INCREASE ");
            break;
        case 0x19E60DF2: //decrease speed received from Android
            speed-=0.05;
            PRINTF("DECREASE ");
            break;
        case 0x19E60EF1: //emergency reset received from Android
            PRINTF("TURN OFF ");
            while(1) motors.setThrottle(0); //busy waiting
        //add here other signals:
        //case xxx: ... break;
        default:
            PRINTF("UNKNOWN ");
    }
    last_time=now; //update the last time of reception 
    led_ir=1; //turn on the led
    PRINTF("ir signal:%08X\n",data);
}

/**
 * Checks every sensor and the motors
 * Runs once, on the setup
 * If connected to PC, checks only the mpu, to speed up debugging
 *
 * Period of blinking led:
 * 30ms  - Waiting for IR signal
 * 200ms - MPU6050 undetected!
 */
void check(){
    wait(1);
    if (mpu.testConnection()){
        PRINTLN("MPU6050 detected");
    }
    else{
        PRINTLN("MPU6050 undetected!");
        while(1){ led_ir=!led_ir; wait_ms(200);}
    }
    #ifndef PC
    PRINTLN("Waiting for IR signal");
    while(ir.decode_data()<0){
        led_ir=!led_ir;
        wait_ms(30);
    }
    PRINTLN("IR signal received");
    
    PRINTLN("Checking motors..");
    PRINTLN("UP");
    motors.setThrottlePure(0.2,0,0,0); wait_ms(500);
    PRINTLN("RIGHT");
    motors.setThrottlePure(0,0.2,0,0); wait_ms(500);
    PRINTLN("DOWN");
    motors.setThrottlePure(0,0,0.2,0); wait_ms(500);
    PRINTLN("LEFT");
    motors.setThrottlePure(0,0,0,0.2); wait_ms(500);
    motors.setThrottle(0);
    #endif
}

/**
 * Limit the speed in the range 0-1
 *
 * @param float& 4 speeds
 */
void limitSpeed(float& speed_dw_rx,float& speed_dw_lx,float& speed_up_lx,float& speed_up_rx){
    //speed_dw_rx
    if(speed_dw_rx<0) speed_dw_rx=0;
    else if(speed_dw_rx>1) speed_dw_rx=1;
    //speed_dw_lx
    if(speed_dw_lx<0) speed_dw_lx=0;
    else if(speed_dw_lx>1) speed_dw_lx=1;
    //speed_up_lx
    if(speed_up_lx<0) speed_up_lx=0;
    else if(speed_up_lx>1) speed_up_lx=1;
    //speed_up_rx
    if(speed_up_rx<0) speed_up_rx=0;
    else if(speed_up_rx>1) speed_up_rx=1;
}

/**
 * Runs once, when the drone is switched on
 * @input insert here KALMAN or COMPLEMENTARY filter
 */
void setup(){
    time_count.start();
    check(); //controls every sensor
    mpu.initialize(); //initialize MPU6050
    wait_ms(3);
    ultra.startUpdates(); //ultrasonic sensor starts reading
    mpu.initialize_timer(KALMAN,LOOP_TIME); //initialize the filter
    //now "angle"" points to the angles calculated every LOOP_TIME in MPU6050.cpp
    angles=mpu.getAngles(); //getAngles return a pointer
    mpu.offsetCalc(); //does nothing if defined DEFAULT_OFFSETS in MPU6050.cpp
    wait_ms(3);
    led_ir=0; //the led is turned off when setup is complete
}

 void loop(){
    loop_count++;
    led_running=!led_running; //this led keeps blinking while loop is running
    catch_ir(); //listening for a ir signal
    altitude=ultra.getCurrentDistance(); //calculates altituide
    mpu.calculate_filter();//calculates the angles
    
    float roll_setpoint=0; //the PID makes roll, pitch and yaw angles equal to..
    float pitch_setpoint=0; //...these setpoints
    float yaw_setpoint=0;
    
    //--------------------------PID CALCULATION--------------------------
    
    //roll
    error_val = angles[1] - roll_setpoint; //the error_val IS AN ANGLE
    integral_roll += integral_gain * error_val;
    if(integral_roll > max_pid_integral)
        integral_roll = max_pid_integral;
    else if(integral_roll < (max_pid_integral*-1))
        integral_roll = max_pid_integral*-1;
    output_roll = proportional_gain * error_val +
                  integral_roll +
                  differential_gain * (error_val-last_roll_differential_error);
    if(output_roll > max_pid)
        output_roll = max_pid;
    else if(output_roll < max_pid*-1)
        output_roll = max_pid*-1;
    last_roll_differential_error = error_val;
    
    //pitch
    error_val = angles[0] - pitch_setpoint; //the error_val IS AN ANGLE
    integral_pitch += integral_gain * error_val;
    if(integral_pitch > max_pid_integral)
        integral_pitch = max_pid_integral;
    else if(integral_pitch < max_pid_integral*-1)
        integral_pitch = max_pid_integral*-1;
    output_pitch = proportional_gain * error_val + integral_pitch + differential_gain * (error_val - last_pitch_differential_error);
    if(output_pitch > max_pid)
        output_pitch = max_pid;
    else if(output_pitch < max_pid*-1)
        output_pitch = max_pid*-1;
    last_pitch_differential_error = error_val;
    
    //yaw
    // in this case I want the angular speed to be zero, not the angle 
    static float yaw_input=0;
    //this is a complementary filter of the yaw angle with itself
    //131 LSB/deg/s is the LSB sensitivity at +/-250 deg/s of sensitivity
    //in the setup we initialize the MPU with 250 deg/sec of sensitivity
    //see the table for other gyro sensitivities, you have to change also the value in MPU6050.cpp, in initialize()
    // +/- 250 degrees/s  | 131 LSB/deg/s
    // +/- 500 degrees/s  | 65.5 LSB/deg/s
    // +/- 1000 degrees/s | 32.8 LSB/deg/s
    // +/- 2000 degrees/s | 16.4 LSB/deg/s
    yaw_input=(yaw_input*0.7)+((angles[2]/131)*0.3); 
    
    error_val=yaw_input - yaw_setpoint; //the error_val IS AN ANGULAR SPEED
    integral_yaw += integral_gain_yaw * error_val;
    if(integral_yaw > max_pid_integral)
        integral_yaw = max_pid_integral;
    else if(integral_yaw < max_pid_integral*-1)
        integral_yaw = max_pid_integral*-1;
    output_yaw = proportional_gain_yaw * error_val +
                 integral_yaw +
                 differential_gain_yaw * (error_val-last_yaw_differential_error);
    if(output_yaw > max_pid)
        output_yaw = max_pid;
    else if(output_yaw < max_pid*-1)
        output_yaw = max_pid*-1;
    last_yaw_differential_error = error_val;

    float speed_dw_rx, speed_dw_lx, speed_up_lx, speed_up_rx;
    float output_dw_rx, output_dw_lx, output_up_lx, output_up_rx;
    
    //output is between [-max_pid*3,max_pid*3]
    output_dw_rx=(output_pitch+output_roll-output_yaw);
    output_dw_lx=(-output_pitch+ output_roll+output_yaw);
    output_up_lx=(-output_pitch-output_roll-output_yaw);
    output_up_rx=(output_pitch-output_roll+output_yaw);
    
    //"speed" is the baseline speed, controlled by the user with the IR
    speed_dw_rx= speed+output_dw_rx;
    speed_dw_lx= speed+output_dw_lx;
    speed_up_lx= speed+output_up_lx;
    speed_up_rx= speed+output_up_rx;
    
    //limit range of speed between 0 and 1
    limitSpeed(speed_dw_rx,speed_dw_lx,speed_up_lx,speed_up_rx);
    
    //in case of debugging, it prints only once every 100.
    //otherwise the looptime would be influenced too much
    if(loop_count%100==0){
        loop_count=0;
        PRINTF("dw_Right:%f\tdw_left:%f\tup_Left:%f\tup_right:%f\n",speed_dw_rx,speed_dw_lx,speed_up_lx,speed_up_rx);    
    }
    
    motors.setThrottle(speed_dw_rx,speed_dw_lx,speed_up_lx,speed_up_rx);
    
    /*manca da impostare la velocità di base dei 4 motori...e secondo me al posto
    del throttle (riga281 brokking) bisogna settare la velocità a cui il drone
    sta sospeso in aria.
    Poi sarà questa a variare (aumenterà o diminuirà) in base alla distanza
    rilevata dal sensore ad ultrasuoni*/
}
    
int main(){
    setup();
    while(1)
        loop();
}