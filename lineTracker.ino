//version control
#define ORIGIN
#define DEGUB
//headers
#include <MsTimer2.h>
//basics
#define STD 35
#define BAUD 9600
#define PERIOD 20
//pin mapping: motor - uno
#define ENCODER_L_A 2
#define ENCODER_L_B 4
#define ENCODER_R_A 3
#define ENCODER_R_B 5
//pin mapping: l298n - uno
#define PWM_L 11
#define IN1_L 12
#define IN2_L 13
#define PWM_R 10
#define IN1_R 9
#define IN2_R 8
//pin mapping: infrared - uno
#define L1 15
#define L2 16
#define L3 17
#define MI 18
#define R1 19
#define R2 6
#define R3 7
//pin mapping: votage - uno
#define VOLT 14
//global variables
bool runFlag;
int IR[7];
int posErr;
int specialTurn;
int encoderLeft, encoderRight;
int setpoint_L, setpoint_R;
//globals used in ORIGIN
float target1 = 2.0, target2 = 2.0;
float t1, t2;
float velocity1, velocity2;

void setup()
{
    TCCR1B = TCCR1B & B11111000 | B00000001; //see cookbook p589
    //init encoder pins
    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);
    //init l298n pins
    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(IN1_L, INPUT);
    pinMode(IN1_R, INPUT);
    pinMode(IN2_L, INPUT);
    pinMode(IN2_R, INPUT);
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
    runFlag = false;
}

void loop()
{
#ifdef DEGUB
    Serial.print(velocity1);
    Serial.print(F("\t"));
    Serial.println(velocity2);
#else
    if (Serial.available())
    {
        int command = Serial.read();
        if (command == '0')
        {
            runFlag = !runFlag;
        }
    }
    checkVolt();
#endif
}

void control()
{
    if (runFlag == false)
    {
        stop();
        return;
    }
#ifndef ORIGIN
    int posOutput, speedOutput_L, speedOutput_R;
    readIR();
    if (specialTurn)
    {
        switch (specialTurn)
        {
            /*UNFINISHED*/
        }
        specialTurn = 0;
    }
    else
    {
        posOutput = posPID(0,posErr);
        setpoint_L = STD - posOutput;
        setpoint_R = STD + posOutput;
        speedOutput_L = speedPid_L(setpoint_L, encoderLeft);
        speedOutput_R = speedPid_R(setpoint_R, encoderRight);
        setMotorSpeed(speedOutput_L, speedOutput_R);
    }
    encoderLeft = encoderRight = 0;
#else
    if (digitalRead(MI) == LOW)
    {
        target1 = target2 = 6;
    }
    if (digitalRead(L1) == LOW)
    {
        target1 = 6 * 0.9;
        target2 = 6 * 0.5;
    }
    if (digitalRead(R1) == LOW)
    {
        target1 = 6 * 0.5;
        target2 = 6 * 0.9;
    }
    if (digitalRead(L2) == LOW)
    {
        target1 = 6 * 0.75;
        target2 = 6 * 0.15;
    }
    if (digitalRead(R2) == LOW)
    {
        target1 = 6 * 0.15;
        target2 = 6 * 0.75;
    }
    if (digitalRead(L3) == LOW)
    {
        target1 = 6 * 0.65;
        target2 = 0;
    }
    if (digitalRead(L3) == LOW)
    {
        target1 = 0;
        target2 = 6 * 0.65;
    }
    target1 = target1 * 0.6 + t1 * 0.4;
    target2 = target2 * 0.6 + t2 * 0.4;
    velocity1 = (encoderLeft / 780) * 3.1415 * 2 * (1000 / PERIOD);
    velocity1 = (encoderRight / 780) * 3.1415 * 2 * (1000 / PERIOD);
    encoderLeft = encoderRight = 0;
    setMotorSpeed(speedPid_L(target1, velocity1), speedPid_R(target2, velocity2));
    t1 = target1;
    t2 = target2;
#endif
#ifdef DEGUB
    velocity1 = encoderLeft;
    velocity2 = encoderRight;
    encoderLeft = encoderRight = 0;
    int output1 = speedPid_L(20, velocity1);
    int output2 = speedPid_R(20, velocity2);
    if (output1 > 0)
    {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, HIGH);
        analogWrite(PWM_L, output1);
    }
    else
    {
        digitalWrite(IN1_L, HIGH);
        digitalWrite(IN2_L, LOW);
        analogWrite(PWM_L, abs(output1));
    }
    if (output2 > 0)
    {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, HIGH);
        analogWrite(PWM_R, output2);
    }
    else
    {
        digitalWrite(IN1_R, HIGH);
        digitalWrite(IN2_R, LOW);
        analogWrite(PWM_R, abs(output2));
    }
#endif
}

void readIR()
{
    IR[0] = digitalRead(L1);
    IR[1] = digitalRead(L2);
    IR[2] = digitalRead(L3);
    IR[3] = digitalRead(MI);
    IR[4] = digitalRead(R1);
    IR[5] = digitalRead(R2);
    IR[6] = digitalRead(R3);
}

void setMotorSpeed(int left, int right)
{
    if (left > 0)
    {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, HIGH);
        analogWrite(PWM_L, left);
    }
    else
    {
        digitalWrite(IN1_L, HIGH);
        digitalWrite(IN2_L, LOW);
        analogWrite(PWM_L, abs(left));
    }
    if (right > 0)
    {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, HIGH);
        analogWrite(PWM_R, right);
    }
    else
    {
        digitalWrite(IN1_R, HIGH);
        digitalWrite(IN2_R, LOW);
        analogWrite(PWM_R, abs(right));
    }
}

void stop()
{
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
}

void getEncoder_L()
{
    if (digitalRead(ENCODER_L_A) == LOW)
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
    else
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
    const float K = 0.5, Ti = 2000, Td = 4, T = PERIOD;
    static int err_1 = 0, err_2 = 0, err_3 = 0, u = 0;
    err_1 = set - current;
    float q0 = K * (1 + T / Ti + Td / T);
    float q1 = -K * (1 + 2 * Td / T);
    float q2 = K * Td / T;
    u = u + q0 * err_1 + q1 * err_2 + q2 * err_3;
    err_3 = err_2;
    err_2 = err_1;
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
#ifndef ORIGIN
    if (++count >= 200)
    {
        result = sum * 0.05371 / 200;
        if (result <= 9.9)
        {
            runFlag = false;
        }
        Serial.println(result);
        count = sum = 0;
    }
#else
    if (++count >= 200)
    {
        result = sum * 200 * (5.0 / 1023);
        if (result <= 3.3)
        {
            runFlag = false;
        }
        Serial.println(result);
        count = sum = 0;
    }
#endif
}