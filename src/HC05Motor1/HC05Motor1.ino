//2020.11.22
//Configure THE PWM control pin
const int PWM2A = 11;      //M1 motor
const int PWM2B = 3;       //M2 motor  
const int PWM0A = 6;       //M3 motor 
const int PWM0B = 5;       //M4 motor

const int DIR_CLK = 4;     // Data input clock line
const int DIR_EN = 7;      //Equip the L293D enabling pins
const int DATA = 8;        // USB cable
const int DIR_LATCH = 12;  // Output memory latch clock
//Define the pin of ultrasonic obstacle avoidance sensor
const int Trig = A2;       //A2 is defined as the pin Trig connected to the ultrasonic sensor
const int Echo = A3;       //A3 is defined as the pin Echo connected to the ultrasonic sensor
//Define motion state
const int Forward = 39;    //39存放到Forward
const int Back = 216;      //216存放到Back
const int Left = 57;       //57存放到Left变量中
const int Right = 198;     //The right amount of change
const int Stop = 0;        //Parking variable
//Set the default speed between 1 and 255
int Speed1 = 180; //PWM0B -M3
int Speed2 = 180; //PWM0A -M4
int Speed3 = 180; //PWM2A -M3 
int Speed4 = 180; //PWM2B -M4

int distance = 0;           //Variables for storing ultrasonic sensor measurements
char serialData;            //把串口接收的数据存放serialData中
char cmd;                   //Store bluetooth receive commands

void setup() 
{
    Serial.begin(9600);//Set the serial port baud rate 9600

    //Configure as output mode
    pinMode(DIR_CLK,OUTPUT);
    pinMode(DATA,OUTPUT);
    pinMode(DIR_EN,OUTPUT);
    pinMode(DIR_LATCH,OUTPUT);
    pinMode(PWM0B,OUTPUT);
    pinMode(PWM0A,OUTPUT);
    pinMode(PWM2A,OUTPUT);
    pinMode(PWM2B,OUTPUT);

    pinMode(Trig,OUTPUT);//The Trig pin connected to the ultrasound is set to output mode
    pinMode(Echo,INPUT);//The Echo pin connected to the ultrasound is set to input mode
    
    void Motor(int Dri,int Speed1,int Speed2,int Speed3,int Speed4);
    int SR04(int Trig,int Echo);
    void AvoidingObstacles();
    void HC05();
} 

void loop()
{
    distance = SR04(Trig,Echo);       //Acquisition of ultrasonic distance

    HC05(); //Call the Bluetooth car control function
}

 
/* Function name: Motor();
* Function: Change the movement direction and speed of the car through the entrance parameters
* Entry parameter 1: Dri car movement direction
* Entry parameters 2~3: Speed1~Speed4 motor speed, value range 0~255
* Dri value description (forward :39; Back: 216; Left translation: 57; Right translation: 198; Stop: 0;
* Right rotation: 106; Left rotation: 149)
* Return value: None
 */
void Motor(int Dir,int Speed1,int Speed2,int Speed3,int Speed4)
{
    analogWrite(PWM2A,Speed1); //Motor PWM speed regulation
    analogWrite(PWM2B,Speed2); //Motor PWM speed regulation
    analogWrite(PWM0A,Speed3); //Motor PWM speed regulation
    analogWrite(PWM0B,Speed4); //Motor PWM speed regulation
    
    digitalWrite(DIR_LATCH,LOW); //DIR_LATCH sets the low level and writes the direction of motion in preparation

    shiftOut(DATA,DIR_CLK,MSBFIRST,Dir);//Write Dir motion direction value

    digitalWrite(DIR_LATCH,HIGH);//DIR_LATCH sets the high level and outputs the direction of motion
}

/*
Function name: SR04()
Function: Obtain ultrasonic ranging data
Entry parameters: Trig, Echo
Function return value: cm
*/
int SR04(int Trig,int Echo)
{
    float cm = 0;

    digitalWrite(Trig,LOW);     //Trig is set to low level
    delayMicroseconds(2);       //Wait 2 microseconds
    digitalWrite(Trig,HIGH);    //Trig is set to high level
    delayMicroseconds(15);      //Wait 15 microseconds
    digitalWrite(Trig,LOW);     //Trig is set to low

    cm = pulseIn(Echo,HIGH)/58.8; //Convert the ranging time to CM
    cm = (int(cm * 100.0))/100.0; //Leave 2 as a decimal
    Serial.print("Distance:");    //Character Distance displayed in serial port monitor window:
    Serial.print(cm); 
    Serial.println("cm"); 

    return cm;      //Returns cm value ranging data
}
/*
* Function name: HC05()
* Function: Receive Bluetooth data, control the car movement direction
* Entry parameters: None
* Return value: None
*/
void HC05()
{
    if(Serial.available() > 0)      //Determine if the received data is greater than 0
    {
        serialData = Serial.read(); //Receiving function
        
        if     ('F' == serialData )  cmd = 'F';     //If the data received by the serial port is character F, save F to CMD
        else if('B' == serialData )  cmd = 'B';     //If the data received by the serial port is character B, save F to CMD
        else if('L' == serialData )  cmd = 'L';     //If the serial port receives data as the character L, save F to CMD
        else if('R' == serialData )  cmd = 'R';     //If the serial port receives data as the character R, save F to CMD
        else if('S' == serialData )  cmd = 'S';     //If the serial port receives data as character S, save F to CMD

        else if( serialData == '+' && Speed1 < 245)//If you receive a string plus, the speed increases
        {
            Speed1 += 10;   //We're going to increase the velocity by 10 at a time
            Speed2 = Speed1;
            Speed3 = Speed1;
            Speed4 = Speed1;
        }
        else if( serialData == '-' && Speed1 > 30)//When I receive a string -- the speed decreases
        {
            Speed1 -= 10;   //I'm going to subtract 10 at a time
            Speed2 = Speed1;
            Speed3 = Speed1;
            Speed4 = Speed1;
        }

         else if('A' == serialData) //Bluetooth received the string R, car right translation
        {
            Motor(106,Speed1,Speed2,Speed3,Speed4);      //
            delay(100);
        }
        else if('C' == serialData) //Bluetooth received the string R, car right translation
        {
            Motor(149,Speed1,Speed2,Speed3,Speed4);      //
            delay(100);
        }
    }

    if('F' == cmd)   //If Bluetooth receives the string F, the dolly moves forward and enables obstacle avoidance
    {      
       AvoidingObstacles();//The ultrasonic obstacle avoidance function is called to realize the obstacle avoidance function
    }
    else if('B' == cmd)     //Bluetooth receives string B, car backs up
    {   
        Motor(Back,Speed1,Speed2,Speed3,Speed4);
    }
    else if('L' == cmd)    //Bluetooth received the string L, car left translation
    {              
        Motor(Left,Speed1,Speed2,Speed3,Speed4); 
    }
    else if('R' == cmd)     //Bluetooth received the string R, car right translation
    {
        Motor(Right,Speed1,Speed2,Speed3,Speed4);      //right translation   
    }
   
    else if('S' == serialData)      //When the string S is received, the cart stops moving
    { 
        Motor(Stop,0,0,0,0); 
    }
}

void AvoidingObstacles()
{
    if((distance > 20 ) || cmd == 'F')//If the distance is greater than 20cm or bluetooth receives a command equal to F
    {
        delay(100);//Delay of 100 ms
        if(distance > 20)//Again, determine if the distance is really greater than 20cm
        {
            Motor(Forward,Speed1,Speed2,Speed3,Speed4); //Call forward function   
        }
        else //Otherwise the distance is less than 20
        {
            Motor(Back,Speed1,Speed2,Speed3,Speed4);//retreat
            delay(500);
            Motor(106,Speed1,Speed2,Speed3,Speed4);//Turn left to change the direction of the car
            delay(500);
        }
    } 
}
