/*
Andrew Wood
6/10/2019


RoboBoat 2019 Final

This program runs on the MBED microcontroller on the 2019 ERAU RoboBoat: Phantom 2.
The program handles inputs from the onboard computer and the RC reciever. It also 
handles the input from the Onboard E-Stop button. The program inputs are thruster
position and power commands, as well as Auto/RC control states and stop commands.
The program outputs are PWM signals for the thruster servos and ESCs, and update
signals for the LED tower.
*/





#include "mbed.h"
#include "Servo.h"
#include "PwmIn.h"

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////        setup        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//Initialize back and front servos
Servo back_servo(p23);
Servo front_servo(p24);

//Initialize E-Stop Button pin
DigitalIn Ctrl(p8);

//Initialize Pins for Thruster Power
Servo front_thrust(p25);
Servo back_thrust(p26);

//Initialize RC receiver pins
PwmIn thro(p17);
PwmIn elev(p15);
PwmIn gear(p13);
PwmIn aile(p14);
PwmIn rudd(p16);
PwmIn aux(p18);

//Initialize Serial communication pathways
Serial pc(USBTX, USBRX);
Serial ledControl(p28, p27);

//Initialize Timer variable for LED update function
Timer ledTime;

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////  global variables   //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//Six float variables contain six inputs from the RC reciever
//Set all PWM values to a default of 0
float throttle_output = 0.0;
float elevation_output = 0.0;
float ESTOP_output = 0.0;
float rudder_output = 0.0;
float aileron_output = 0.0;
float aux_output = 0.0;

//Variables for Serial Communcation with Labview
volatile bool newData = false;
volatile float inputs[4];

//light tower color and bightness values
float ledColor = 0;
float ledBright = 0;

//Expired variable used to count number of loops without update from computer
unsigned int expired = 0;       

//union for light tower ledsend function
union {
  float f;
  char  bytes[4];
} float_union;

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     Functions      ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////



//Thruster Range Function adjusts range input from 0.00 - 1.00 to 0.08 - 0.92
//This range adjustment is better suited to the PWM controller for the Thruster ESCs
float ThrusterRange(float x)
{
    float y;
    y = ((x * 0.84) + 0.08);
    return y;
}





//Function Reads Serial com from computer via USB
void onSerialRx()
{
    static char serialInBuffer[100];                                    //Buffer holds serial message characters
    static int serialCount = 0;`                                        //Counter for buffer array
 
    while (pc.readable())                                               //Only attempt to read serial message if message is being recieved, stop reading when message is not being transmitted
    {
        char byteIn = pc.getc();                                        //Read character from serial message
        
        if (byteIn == 0x65)                                             //If an end of line character is found
        { 
            serialInBuffer[serialCount] == 0;                           //Null terminate the input                
            float w,x,y,z;                                              //Initialize 4 variables for intended serial message data
            if (sscanf(serialInBuffer,"%f,%f,%f,%fe",&w,&x,&y,&z) == 4) //Check if full message was recieved and correctly formatted
            { 
                inputs[0] = w;
                inputs[1] = x;
                inputs[2] = y;
                inputs[3] = z;
                newData = true;                                         //Signal that New Data is availible
            }
            serialCount = 0;                                            //Reset the buffer
        } 
        else                                                            //If end of line character is not found
        {
            serialInBuffer[serialCount] = byteIn;                       // store the character in buffer
            if (serialCount<100)
            {
                serialCount++;                                          //Incriment the counter, limited to 100 characters
            }
        }
    }
}





//Calibration Sequence
//This sequence sends the Servos and ESCs their min and max values
void Calibrate()
{
    back_servo = 0.0;
    front_servo = 0.0;
    back_thrust = 0.0;
    front_thrust = 0.0;
    wait(0.1); //ESC detects signal
    
    
    //Required ESC Calibration/Arming sequence  
    //sends longest and shortest PWM pulse to learn and arm at power on
    back_servo = 1.0;
    front_servo = 1.0;
    back_thrust = 1.0;
    front_thrust = 1.0;
    wait(0.1);
    back_servo = 0.0;
    front_servo = 0.0;
    back_thrust = 0.0;
    front_thrust = 0.0;
    wait(0.1);
    //End calibration sequence
    
    
    //Set Thrusters and Servos to center still and center positions
    front_thrust = 0.46;
    back_thrust = 0.46;
    back_servo = 0.5;
    front_servo = 0.5;
}




//sends command message to led controller for LED
void ledSend(float ledColorOut, float ledBrightOut)
{
    /* How to use:
    -First input is for the color, second for brightness
    
    -Brightness goes from 0 to 100 (float, so it includes decimal)
    
    -Color code:
        0: off
        1: red
        2: green
        3: blue
        4: yellow
        5: purple
        Anything else turns it off as a failsafe
        It's a float value but only give it whole numbers, I just didn't feel like having 2 unions
    */
    
    //initializing values
    int numsend=0;
    char buf[30];
    
    //create message
    buf[0] = 0xFF;  //header
    buf[1] = 0x00;  //message ID
    
    //take the color, and using the union, turn it into bytes
    float_union.f = ledColorOut;
    buf[2] = float_union.bytes[0];
    buf[3] = float_union.bytes[1];
    buf[4] = float_union.bytes[2];
    buf[5] = float_union.bytes[3];
    
    //do the same with brightness
    float_union.f = ledBrightOut;
    buf[6] = float_union.bytes[0];
    buf[7] = float_union.bytes[1];
    buf[8] = float_union.bytes[2];
    buf[9] = float_union.bytes[3];
    
    //send the message over serial
    while(numsend < 10)
    {            
        ledControl.putc(buf[numsend]);
        numsend++;
    }
}




//Function assigns thruster speed and angle position for Remote Control Operation
void RCsettings()
{
    //Servo Controllers (Lock front thruster forwards, steer with rear thruster)
    front_servo = 0.5;
    back_servo = rudder_output;
                        
    //Thrust Controllers
    front_thrust = aileron_output - 0.04;
    back_thrust = elevation_output - 0.04;

    //led control for manual (4: yellow)
    ledColor = 4;
    ledBright = 75;
}




//Function assigns thruster speed and angle position for Autonomous Operation
void Autosetings()
{
    if(newData)                             //If new thruster command data is availible
    {
        newData = false;                        //Reset NewData Boolean

        front_servo = inputs[0];                //Set thruster values
        front_thrust = ThrusterRange(inputs[1]) - 0.04;
        back_servo = inputs[2];
        back_thrust = ThrusterRange(inputs[3]) - 0.04;

        expired = 0;                            //Reset Expired counter
    }
    else                                    //If there is no new command data availible
    {
        expired++;                              //Count the number of loops without new data
    }



    if(expired > 300000)                    //If too many loops have passed with no new data, assume Labview/Computer crashed
    { 
        back_thrust = 0.46;                     //Set thrusters to still position
        front_thrust = 0.46;

        ledColor = 5;                           //led control for loss of labview (5: purple)
        ledBright = 100;
    }
    else                                    //Else, inticate auto is working
    {
        ledColor = 2;                       //led control for auto (2: green, 3: blue)
        ledBright = 75;
    }
}





////////////////////////////////////////////////////////////////////////////////
/////////////////////////////        Main        ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main() {
    
    Calibrate();                    //Calibrate Servos and ESCs
    
    pc.baud (115200);               //Set baud rate for Serial communcation with the Computer
    
    pc.attach(&onSerialRx);         //Attach Serial recieve channel
    
    int stopFlag = 0;               //Variable used to signal stop conditions
    
    ledTime.start();                //Starts LED timer

    ledSend(0,0);                   //Sends intial command to LED tower: LEDs off with no brightness
    
    Ctrl.mode(PullDown);            //Sets internal resistor control mode for E-Stop button to PullDown
    
    
    
    //Enable Servo to turn 180 degrees
    back_servo.calibrate(0.00085,90.0);
    front_servo.calibrate(0.00085,90.0);
    
    
    
    while(1) {
        
        //Read in all PWM signals and set them to a value between 0 and 1
        elevation_output = (elev.pulsewidth()*1000)-1;
        throttle_output = (thro.pulsewidth()*1000)-1;
        rudder_output = (rudd.pulsewidth()*1000)-1;
        aileron_output = (aile.pulsewidth()*1000)-1;
        
        //RC vs Auto PWM
        aux_output = (aux.pulsewidth()*1000)-1; // >.5 RC... <.5 Auto    
        
        //ESTOP PWM
        ESTOP_output = (gear.pulsewidth()*1000)-1;  // >.5 run... <.5 STOP
        
        
        //Reset stop flag ever loop
        stopFlag = 0;
        
        
        if(Ctrl.read() != 1){       //If E-Stop button is pressed
            stopFlag++;             //Incriment Stop Flag to 1
        }
        
        
        
        if(ESTOP_output == (-1))        //If E-stop reads -1, then the controller is turned off
        {
            front_thrust = 0.46;            //Stop both thrusters
            back_thrust = 0.46;
            ledColor = 1;                   //led control for E-stop (1: red)
            ledBright = 100;
        }
        else                            //Else, controller is turned on
        {
            if(stopFlag == 0)               //if the estop button is not pressed
            {
                if(ESTOP_output > 0.5)          //And if remote estop is not active    
                {
                    if(aux_output > 0.5)            //RC Settings
                    {
                        RCsettings();
                    }
                    else                            //Auto Settings
                    {   
                        Autosettings();            
                    }
                }
                else                            //Else, remote E-stop is active, stop
                {
                    //Set thrusters to still position
                    back_thrust = 0.46;                     
                    front_thrust = 0.46;
                    
                    //led control for estop (1: red)
                    ledColor = 1;
                    ledBright = 100;
                }  
            }
            else                            //Else, E-stop button is pressed
            {
                //Set thrusters to still position
                back_thrust = 0.46;                     
                front_thrust = 0.46;
                
                //led control for estop (1: red)
                ledColor = 1;
                ledBright = 100;
            }
        }
        
        
        //only update LED tower every half second to prevent explosions
        if(ledTime > 0.5)           
        {
            ledSend(ledColor,ledBright);
            ledTime.reset();
        }
        
               
    }//End While
}//End Main

