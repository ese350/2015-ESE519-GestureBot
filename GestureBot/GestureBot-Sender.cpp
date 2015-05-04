#include "mbed.h"
#include "IMUfilter.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include <iostream>


/* Define the various parameters used in the code
 * These parameters are used in the obtaining of
 * accelerometer and gyroscope values
 * */

#define g0 9.812865328                      //Gravity at Earth's surface in m/s/s

#define SAMPLES_GYR 2                       //Number of gyroscope samples to average

#define SAMPLES_ACC 6                       //Number of accelerometersamples to average

#define CALIBRATION_SAMPLES 128             //Number of samples to be averaged for a null bias calculation during calibration

#define GYROSCOPE_GAIN (1 / 14.375)         //ITG-3200 sensitivity is 14.375 LSB/(degrees/sec)

#define ACCELEROMETER_GAIN (0.004 * g0)     //Full scale resolution on the ADXL345 is 4mg/LSB

#define GYRO_RATE   0.005                   //Sampling gyroscope at 200Hz

#define ACC_RATE    0.005                   //Sampling accelerometer at 200Hz

#define FILTER_RATE 0.1                     //Updating filter at 40Hz

#define toDegrees(x) (x * 57.2957795)       //Convert from radians to degrees

#define toRadians(x) (x * 0.01745329252)    //Convert from degrees to radians



/* Serial pin to control to receive data over the
 * Reset pin to reset to xbee to make it ready for communication
 * Character ch is the data received from the sender system
 */
Serial xbee1(p9, p10);
DigitalOut rst1(p30);



//USB serial communication for debugging
Serial pc(USBTX, USBRX);



//LEDs for indicating direction of motion of the robot arm
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);



/* Variables that control the IMU readings
 * imufilter gives the current roll, pitch and yaw values
 * accelerometer gives the current accelerometer readings
 * gyroscope gives the current gyroscope readings
 */
IMUfilter imuFilter(FILTER_RATE, 0.3);
ADXL345 accelerometer(p5, p6, p7, p8);
ITG3200 gyroscope(p28, p27);



/* Tickers that control the reading of values from the accelerometer and gyroscope
 * accelerometerTicker controls the rate at which accelemoremeter samples are taken
 * gyroscopeTicker controls the rate at which gyroscope samples are taken
 * filterTicker controls the rate at which IMU fiilter samples are taken
 * */
Ticker accelerometerTicker;
Ticker gyroscopeTicker;
Ticker filterTicker;



//Control the opening and closing of the robot arm gripper
DigitalOut open(p15);
DigitalOut close(p12);



//Offsets for the gyroscope
double w_xBias;
double w_yBias;
double w_zBias;



//Offsets for the accelerometer
double a_xBias;
double a_yBias;
double a_zBias;



//Accumulators used for oversampling and then averaging
double a_xAccumulator = 0;
double a_yAccumulator = 0;
double a_zAccumulator = 0;
double w_xAccumulator = 0;
double w_yAccumulator = 0;
double w_zAccumulator = 0;



//Global Accelerometer and gyroscope readings for x, y, z axes
double a_x;
double a_y;
double a_z;
double w_x;
double w_y;
double w_z;



//Buffer for accelerometer readings
int readings[3];


//Number of accelerometer samples we're on
int accelerometerSamples = 0;


//Number of gyroscope samples we're on
int gyroscopeSamples = 0;



//Caliberate function
void caliberate_all(void);


//Reset all global variables
void reset_all(void);


//Set up the ADXL345 appropriately.
void initializeAccelerometer(void);


//Calculate the null bias.
void calibrateAccelerometer(void);


//Take a set of samples and average them.
void sampleAccelerometer(void);


//Set up the ITG3200 appropriately.
void initializeGyroscope(void);


//Calculate the null bias.
void calibrateGyroscope(void);


//Take a set of samples and average them.
void sampleGyroscope(void);


//Update the filter and calculate the Euler angles.
void filter(void);



int main() {

    pc.printf("Main Loop\n");

    //Reset the xbee module to make it ready for communication
    rst1 = 0;
    wait_ms(1);
    rst1 = 1;
    wait_ms(1);
    
    //Initialize inertial sensors.
    pc.printf("Started Initialization and Caliberation\n");

    led1=led2=led3=led4=0;

    initializeAccelerometer();
    initializeGyroscope();
    caliberate_all();
    pc.printf("Finished Initialization and Caliberation\n");



    while (1) {

        wait(FILTER_RATE);
        
        /*pc.printf("%d,         %d,         %d\n",(int)toDegrees(imuFilter.getRoll()),(int)toDegrees(imuFilter.getPitch()),
                                                 (int)toDegrees(imuFilter.getYaw()));*/


        
        if( (int)toDegrees(imuFilter.getRoll()) >= 30 ){
            //Move the robot hand down

            pc.printf("Down:             %d\n", (int)toDegrees(imuFilter.getRoll()) );

            xbee1.putc('d');

            led1=led2=led4=0;led3=1;

            // while( (int)toDegrees(imuFilter.getRoll()) > 6 ){


        }
        
        else if( (int)toDegrees(imuFilter.getRoll()) <= -30 ){
            //Move the robot hand up

            pc.printf("Up                    %d\n", (int)toDegrees(imuFilter.getRoll()) );

            xbee1.putc('u');

            led1=led3=led4=0;led2=1;

            //while( (int)toDegrees(imuFilter.getRoll()) < -6 )


        }
        
        else if( (int)toDegrees(imuFilter.getPitch()) >= 30 ){
            //Move the robot hand right

            pc.printf("Right                    %d\n", (int)toDegrees(imuFilter.getPitch()) );

            xbee1.putc('r');

            led1=led2=led3=0;led4=1;


            //while( (int)toDegrees(imuFilter.getPitch()) > 6 )


        }
        else if( (int)toDegrees(imuFilter.getPitch()) <= -30 ){
            //Move the robot hand left

            pc.printf("Left                      %d\n", (int)toDegrees(imuFilter.getPitch()) );

            xbee1.putc('l');

            led4=led2=led3=0;led1=1;


            //while( (int)toDegrees(imuFilter.getPitch()) < -6 )

        }
        
        else if(open == 1){
            //Open the robot gripper

            pc.printf("Open\n");

            xbee1.putc('o');

        }
        else if(close == 1){
            //Close the robot gripper

            pc.printf("Close\n");

            xbee1.putc('c');

        }
        
        else{
            //Default case:  make the robot arm stop all motion
            
            xbee1.putc('s');

            pc.printf("Not Moving               %d,         %d,         %d\n",(int)toDegrees(imuFilter.getRoll()),(int)toDegrees(imuFilter.getPitch()),
                      (int)toDegrees(imuFilter.getYaw()) );

            led1=led2=led3=led4=1;
            
        }

    }

}



//Caliberate accelerometer and gyroscope
void caliberate_all(void){

    accelerometerTicker.detach();
    gyroscopeTicker.detach();
    filterTicker.detach();
    reset_all();



    calibrateAccelerometer();
    calibrateGyroscope();

    //Set up timers

    //Accelerometer data rate is 200Hz, so we'll sample at this speed
    accelerometerTicker.attach(&sampleAccelerometer, 0.005);

    //Gyroscope data rate is 200Hz, so we'll sample at this speed
    gyroscopeTicker.attach(&sampleGyroscope, 0.005);

    //Update the filter variables at the correct rate
    filterTicker.attach(&filter, FILTER_RATE);


}



//Set up the ADXL345 appropriately.
void initializeAccelerometer(void) {

    pc.printf("Initialize Accelerometer\n");

    //Go into standby mode to configure the device
    accelerometer.setPowerControl(0x00);

    //Full resolution, +/-16g, 4mg/LSB
    accelerometer.setDataFormatControl(0x0B);

    //200Hz data rate
    accelerometer.setDataRate(ADXL345_200HZ);

    //Measurement mode
    accelerometer.setPowerControl(0x08);

    wait_ms(22);

}




//Calculate the null bias of accelerometer
void calibrateAccelerometer(void) {

    pc.printf("Calibrate Accelerometer\n");

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;

    //Take a number of readings and average them to calculate the zero offset
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        wait(ACC_RATE);

    }

    a_xAccumulator /= CALIBRATION_SAMPLES;
    a_yAccumulator /= CALIBRATION_SAMPLES;
    a_zAccumulator /= CALIBRATION_SAMPLES;

    //At 4mg/LSB, 250 LSBs is 1g
    a_xBias = a_xAccumulator;
    a_yBias = a_yAccumulator;
    a_zBias = (a_zAccumulator - 250);

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;

}




//Take a set of accelerometer samples and average them
void sampleAccelerometer(void) {


    //Have we taken enough samples?
    if (accelerometerSamples == SAMPLES_ACC) {

        //Average the samples, remove the bias, and calculate the acceleration in m/s/s
        a_x = ((a_xAccumulator / SAMPLES_ACC) - a_xBias) * ACCELEROMETER_GAIN;
        a_y = ((a_yAccumulator / SAMPLES_ACC) - a_yBias) * ACCELEROMETER_GAIN;
        a_z = ((a_zAccumulator / SAMPLES_ACC) - a_zBias) * ACCELEROMETER_GAIN;

        a_xAccumulator = 0;
        a_yAccumulator = 0;
        a_zAccumulator = 0;
        accelerometerSamples = 0;

    } else {
        //Take another sample
        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        accelerometerSamples++;

    }

}




//Set up the ITG3200 appropriately
void initializeGyroscope(void) {

    pc.printf("Initialize Gyroscope\n");

    //Low pass filter bandwidth of 42Hz
    gyroscope.setLpBandwidth(LPFBW_42HZ);

    //Internal sample rate of 200Hz. (1kHz / 5)
    gyroscope.setSampleRateDivider(4);


}



//Calculate the null bias of gyroscope
void calibrateGyroscope(void) {

    pc.printf("Caliberate Gyroscope\n");

    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

    //Take a number of readings and average them to calculate the gyroscope bias offset
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();
        wait(GYRO_RATE);

    }

    //Average the samples.
    w_xAccumulator /= CALIBRATION_SAMPLES;
    w_yAccumulator /= CALIBRATION_SAMPLES;
    w_zAccumulator /= CALIBRATION_SAMPLES;

    w_xBias = w_xAccumulator;
    w_yBias = w_yAccumulator;
    w_zBias = w_zAccumulator;

    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;


}




//Take a set of gyroscope samples and average them
void sampleGyroscope(void) {

    //Have we taken enough samples?
    if (gyroscopeSamples == SAMPLES_GYR) {

        //Average the samples, remove the bias, and calculate the angular velocity in rad/s
        w_x = toRadians(((w_xAccumulator / SAMPLES_GYR) - w_xBias) * GYROSCOPE_GAIN);
        w_y = toRadians(((w_yAccumulator / SAMPLES_GYR) - w_yBias) * GYROSCOPE_GAIN);
        w_z = toRadians(((w_zAccumulator / SAMPLES_GYR) - w_zBias) * GYROSCOPE_GAIN);

        w_xAccumulator = 0;
        w_yAccumulator = 0;
        w_zAccumulator = 0;
        gyroscopeSamples = 0;

    } else {
        //Take another sample.
        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();

        gyroscopeSamples++;

    }

}



//Update the filter and calculate the Euler angles.
void filter(void) {


    //Update the filter variables
    imuFilter.updateFilter(w_y, w_x, w_z, a_y, a_x, a_z);

    //Calculate the new Euler angles
    imuFilter.computeEuler();

}



//Reset all global variables
void reset_all(){

    w_xBias= 0;
    w_yBias= 0;
    w_zBias= 0;


    a_xBias= 0;
    a_yBias= 0;
    a_zBias= 0;

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;
    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

    a_x= 0;
    a_y= 0;
    a_z= 0;
    w_x= 0;
    w_y= 0;
    w_z= 0;
}
