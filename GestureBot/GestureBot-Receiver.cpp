#include "mbed.h"
#include <iostream>
#define on 0.8

/* Digital Out pins to control the motors
 * The 3 motors are as follows:
 * Motor 1(m1): Controls the up/down motion of the robot arm
 * Motor 2(m2): Controls the left/right motion of the robot arm
 * Motor 3(m3): Controls the open/close motion of the robot arm gripper
 */
DigitalOut m1[2] = {p20, p19};
DigitalOut m2[2] = {p18, p17};
DigitalOut m3[2] = {p16, p15};


/* Pwm pin that controls the motion of the 3 motors
 * Pwm pin is set at a period of 20ms and duty cycle of 1*/
PwmOut pwm(p21);



/* Serial pin to control to receive data over the
 * Reset pin to reset to xbee to make it ready for communication
 * Character ch is the data received from the sender system
 */
Serial xbee1(p9, p10);
DigitalOut rst1(p30);
char ch;



//USB serial communication for debugging
Serial pc(USBTX, USBRX);


int main() {

    /* Initialization
    * Set all the motor DigitalOut pins to 0 to make them stop
    */
    m1[0]=0;
    m1[1]=0;
    m2[0]=0;
    m2[1]=0;
    m3[0]=0;
    m3[1]=0;


    //Set Pwm time period and duty cycle
    pwm.period_ms(20.0f);
    pwm.write(1.0);

    //Reset the xbee module to make it ready for communication
    rst1 = 0;
    wait_ms(1);
    rst1 = 1;
    wait_ms(1);


    while(1) {

        ch = xbee1.getc();

        if(ch=='d' || ch=='u'|| ch=='l' || ch=='r' || ch=='s'  || ch=='o' || ch=='c'){
            //Filter out junk control signals

            pc.printf("%c\n", ch);


            switch(ch){

            case 'd': //Make the robot arm go down

                m1[0]=1;
                m1[1]=0;

                break;

            case 'u': //Make the robot arm go up

                m1[0]=0;
                m1[1]=1;

                break;

            case 'l': //Make the robot arm go left

                m2[0]=1;
                m2[1]=0;

                break;

            case 'r': //Make the robot arm go right

                m2[0]=0;
                m2[1]=1;

                break;

            case 'o': //Make the robot gripper open

                m3[0]=1;
                m3[1]=0;

                break;

            case 'c': //Make the robot gripper close

                m3[0]=0;
                m3[1]=1;

                break;

            case 's': //Make the robot arm stop

                m1[0]=0;
                m1[1]=0;
                m2[0]=0;
                m2[1]=0;
                m3[0]=0;
                m3[1]=0;


                break;

            default: //Default case, same as 's'

                m1[0]=0;
                m1[1]=0;
                m2[0]=0;
                m2[1]=0;
                m3[0]=0;
                m3[1]=0;


                break;
            }
        }

    }
}
