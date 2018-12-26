//version control
#define DBGPOS
#define DBGSPEED
//headers
#include <MsTimer2.h>
//basics
#define STD 20
#define BAUD 115200
#define PERIOD 20
//pin mapping: motor - uno
#define ENCODER_L_A 2
#define ENCODER_L_B 4
#define ENCODER_R_A 3
#define ENCODER_R_B 5
//pin mapping: driver - uno
#define PWM_L 9
#define PWM_R 10
//pin mapping: infrared - uno
#define L1 15 //A1, LEFTMOST
#define L2 16 //A2
#define L3 17 //A3
#define MI 18 //A4
#define R1 19 //A5, RIGHTMOST
#define R2 6
#define R3 7
//pin mapping: votage - uno
#define VOLT 14 //A0
//global variables
bool runFlag;
int IR[7];
int posErr;
int specialTurn;
int encoderLeft, encoderRight;
int setpoint_L, setpoint_R;

void setup()
{
    TCCR1B = TCCR1B & B11111000 | B00000001; //stops motor from humming
    //init encoder pins
    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);
    //init IR pins
    pinMode(L1, INPUT);
    pinMode(L2, INPUT);
    pinMode(L3, INPUT);
    pinMode(MI, INPUT);
    pinMode(R1, INPUT);
    pinMode(R2, INPUT);
    pinMode(R3, INPUT);
    //interrupts
    attachInterrupt(0, getEncoder_L, CHANGE);
    attachInterrupt(1, getEncoder_R, CHANGE);
    MsTimer2::set(PERIOD, control);
    //init threads
    Serial.begin(BAUD);
    MsTimer2::start();
    runFlag = true;
}

void loop()
{
    if (Serial.available())
    {
        int command = Serial.read();
        if (command == '0')
        {
            runFlag = !runFlag;
        }
    }
    //checkVolt();
}

void control()
{
    int posOutput, speedOutput_L, speedOutput_R;
    readIR();
    if (specialTurn != 0)
    {
        switch (specialTurn)
        {
        default:;
        }
    }
    else
    {
        posOutput = posPID(0, posErr);
        setpoint_L = STD - posOutput;
        setpoint_R = STD + posOutput;
        speedOutput_L = speedPid_L(setpoint_L, encoderLeft);
        speedOutput_R = speedPid_R(setpoint_R, encoderRight);
#ifdef DBGSPEED
        Serial.print(encoderLeft);
        Serial.print(" ");
        Serial.println(encoderRight);
#endif
        if (runFlag == true)
        {
            setMotorSpeed(speedOutput_L, speedOutput_R);   
        }
        else
        {
            stop();
        }
    }
    encoderLeft = 0;
    encoderRight = 0;
}

void readIR()
{
    IR[0] = digitalRead(L1);
    IR[1] = digitalRead(L2);
    IR[2] = digitalRead(L3);
    IR[3] = digitalRead(MI);
    IR[4] = digitalRead(R3);
    IR[5] = digitalRead(R2);
    IR[6] = digitalRead(R1);
#ifdef DBGPOS
    for (int i = 0; i <= 6; ++i)
    {
        Serial.print(IR[i]);
        Serial.print(" ");
    }
#endif
    for (int i = 0; i <= 6; ++i)
    {
        if (IR[i] == 0)
        {
            posErr = 3 - i;
            break;
        }
    }
#ifdef DBGPOS
    Serial.println(posErr);
#endif
    specialTurn = 0;
}

void setMotorSpeed(int left, int right)
{
    if (left > 0)
    {
        left = map(left, 0, 255, 127, 255);
    }
    else
    {
        left = map(left, -255, 0, 0, 127);
    }
    if (right > 0)
    {
        right = map(right, 0, 255, 127, 255);
    }
    else
    {
        right = map(right, -255, 0, 0, 127);
    }
    analogWrite(PWM_L, left);
    analogWrite(PWM_R, right);
}

void stop()
{
    analogWrite(PWM_L, 127);
    analogWrite(PWM_R, 127);
}

void getEncoder_L()
{
    if (digitalRead(ENCODER_L_A) == LOW)
    {
        if (digitalRead(ENCODER_L_B) == LOW)
        {
            encoderLeft++;
        }
        else
        {
            encoderLeft--;
        }
    }
    else
    {
        if (digitalRead(ENCODER_L_B) == LOW)
        {
            encoderLeft--;
        }
        else
        {
            encoderLeft++;
        }
    }
}

void getEncoder_R()
{
    if (digitalRead(ENCODER_R_A) == LOW)
    {
        if (digitalRead(ENCODER_R_B) == LOW)
        {
            encoderRight--;
        }
        else
        {
            encoderRight++;
        }
    }
    else
    {
        if (digitalRead(ENCODER_R_B) == LOW)
        {
            encoderRight++;
        }
        else
        {
            encoderRight--;
        }
    }
}

int posPID(int set, int current)
{
    const float K = 3, Ti = 200, Td = 0, T = PERIOD;
    static int err_1 = 0, err_2 = 0, err_3 = 0, u = 0;
    err_1 = set - current;
    float q0 = K * (1 + T / Ti + Td / T);
    float q1 = -K * (1 + 2 * Td / T);
    float q2 = K * Td / T;
    u = u + q0 * err_1 + q1 * err_2 + q2 * err_3;
    err_3 = err_2;
    err_2 = err_1;
    if (abs(u) >= 120)
    {
        u = 120 * (u / u);
    }
    return (int)u;
}

int speedPid_L(int set, int current)
{
    const float K = 6.5, Ti = 180, Td = 0, T = PERIOD;
    static int err_1 = 0, err_2 = 0, err_3 = 0, u = 0;
    err_1 = set - current;
    float q0 = K * (1 + T / Ti + Td / T);
    float q1 = -K * (1 + 2 * Td / T);
    float q2 = K * Td / T;
    u = u + q0 * err_1 + q1 * err_2 + q2 * err_3;
    err_3 = err_2;
    err_2 = err_1;
    if (abs(u) >= 255)
    {
        u = 255 * (u / u);
    }
    return (int)u;
}

int speedPid_R(int set, int current)
{
    const float K = 6.5, Ti = 180, Td = 0, T = PERIOD;
    static int err_1 = 0, err_2 = 0, err_3 = 0, u = 0;
    err_1 = set - current;
    float q0 = K * (1 + T / Ti + Td / T);
    float q1 = -K * (1 + 2 * Td / T);
    float q2 = K * Td / T;
    u = u + q0 * err_1 + q1 * err_2 + q2 * err_3;
    err_3 = err_2;
    err_2 = err_1;
    if (abs(u) >= 255)
    {
        u = 255 * (u / u);
    }
    return (int)u;
}

void checkVolt()
{
    static int count = 0, sum = 0;
    float result;
    sum += analogRead(VOLT);
    if (++count >= 200)
    {
        result = sum / 200 * (5.0 / 1023);
        if (result <= 3.3)
        {
            runFlag = false;
        }
        Serial.println(result);
        count = sum = 0;
    }
}
