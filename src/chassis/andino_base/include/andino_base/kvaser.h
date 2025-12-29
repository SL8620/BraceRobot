#ifndef ROS_KVASER_CAN_DRIVER_KVASER_CAN_H
#define ROS_KVASER_CAN_DRIVER_KVASER_CAN_H
#include <iostream>
#include "canlib.h"
#include <cmath>
#include <chrono>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <thread>

#define PI 3.14159265358979323846

#define DBPI 6.28318530717958647692  // 2*PI


struct Encoder
{
    int count;
    int AbsZeroPos;
};

struct MOTOR
{
    long id;
    Encoder encoder;
    double InitPos;
    double In;
    double Wn;
    double Kt_inv;
    double direction;
    bool connect;
    double px;
    double vx;
    double tc;
    double jv;
};


struct CANMessage
{
    long COB_ID;
    unsigned int DLC;
    uint8_t Byte[8];
};

class Kvaser
{
public:
    Kvaser(int channel_number);
    ~Kvaser();
    int canInit(int channel_number);
    int canSend(CANMessage* canMeassage);
    int canReceive(CANMessage* canMeassage);
    int checkStatus(const std::string& id);
    int canRelease();
    canHandle handle;
    canStatus status;
    CANMessage Tx;
    CANMessage Rx;
    enum ErrorType
    {
        INIT_ERROR = 1,
        CONNECT_ERROR = 2,
        DISCONNECT_ERROR = 3,
        SEND_MESSAGE_ERROR = 4,
        RELEASE_ERROR = 5,
    };
};

class KvaserForElmo: public Kvaser
{
public:
    enum Mode
    {
        TORQUE_MODE = 1, // 力矩/电流模式
        SPEED_MODE = 2, // 速度模式
        MICRO_STEPPING_MODE = 3,
        DUAL_FEEDBACK_POSITION_MODE = 4,
        POSITION_MODE = 5 // 位置模式
    };

    KvaserForElmo(int channel_number, int TNumOfNodes, MOTOR* TpNode, const char* TNameOfNodes);

    ~KvaserForElmo();

    int netInit();

    int connectMotor(MOTOR* m);

    int modeChoose(MOTOR* M, Mode mode);

    int motorEnable(MOTOR* m);

    int motorDisable(MOTOR* m);

    void DisableMotors();

    int torqueMode(MOTOR* m, double torque);

    int SpeedMode(MOTOR* m, double speed);

    int PositionMode(MOTOR* m, double position, double speed);

    double GetPosition(MOTOR* m);

    double GetVelocity(MOTOR* m);

    int GetStatusWord(MOTOR* m);

    int stop(MOTOR* m);
    void stopAll();

    int motorReset(MOTOR* m);
    void ResetMotors(double WaitTime);

    void PosInit(double WaitTime);

    void int2byte(uint8_t* temp, int32_t temp_value)
    {
        temp[3] = temp_value >> 24;
        temp[2] = (temp_value & 0x00FF0000) >> 16;
        temp[1] = (temp_value & 0x0000FF00) >> 8;
        temp[0] = temp_value & 0x000000FF;
    }

    void float2byte(uint8_t* temp, float temp_value)
    {
        int data = *(int*)&temp_value;
        temp[3] = (data >> 24);
        temp[2] = ((data << 8) >> 24);
        temp[1] = ((data << 16) >> 24);
        temp[0] = ((data << 24) >> 24);
    }

    int byte2int(uint8_t* temp)
    {
        return (temp[3] << 24 | temp[2] << 16 | temp[1] << 8 | temp[0]);
    }

    double cnt2rad(int cnt, MOTOR* m) 
    { 
        return (double)(cnt)*DBPI / (m->encoder.count) * m->direction; 
    }

    int rad2cnt(double rad, MOTOR* m) 
    { 
        return rad * (m->encoder.count) / DBPI * m->direction;
    }
    
    MOTOR* pNode;
    const char *NameOfNodes;
    int NumOfNodes;
};

class KvaserForGold :public KvaserForElmo
{
public:
    KvaserForGold(int channel_number, int NumOfNodes, MOTOR* pNode, const char* NameOfNodes);
    int RPDOconfig(MOTOR* m, Mode mode);
    void SendTorqueCommand(MOTOR* m, double torque);
    void SendTorqueCommandForAll();
    void SendSpeedCommand(MOTOR* m, double speed);
    void SendSpeedCommandForAll();
    void SendPositionCommand(MOTOR* m, double position);
    int TPDOconfigPXVX(MOTOR* m, uint8_t T = 2);
    void GetPositionAndVelocity();
};

class KvaserForSimplIQ :public KvaserForElmo
{
public:
    KvaserForSimplIQ(int channel_number, int TNumOfNodes, MOTOR* TpNode, const char* TNameOfNodes);
    int RPDOconfig(MOTOR* m, Mode mode);
    void SendTorqueCommand(MOTOR* m, double torque);
    void SendTorqueCommandForAll();
    void SendSpeedCommand(MOTOR* m, double speed);
    void SendSpeedCommandForAll();
    void SendPositionCommand(MOTOR* m, double position);
    int TPDOconfigPXVX(MOTOR* m, uint8_t T = 2);
    void GetPositionAndVelocity();
};

class ANGLE
{
public:
    ANGLE();
    ANGLE(double Trad);
    double rad;
    double _sin_;
    double _cos_;
    void operator+=(const ANGLE& p);
    void operator-=(const ANGLE& p);
    void operator*=(const ANGLE& p);
    void operator/=(const ANGLE& p);
    void operator=(const ANGLE& p);
    double operator()();
};
ANGLE operator+(const ANGLE& a, const ANGLE& b);
ANGLE operator-(const ANGLE& a, const ANGLE& b);
ANGLE operator*(const ANGLE& a, const ANGLE& b);
ANGLE operator/(const ANGLE& a, const ANGLE& b);


class WriteDataIntoText
{
private:
    int n, num;
    std::ofstream outfile;
public:
    double* DataLine;
    WriteDataIntoText(int Tn, const char* PathOfText);
    ~WriteDataIntoText();
    void write();
};

class ReadDataFromText
{
private:
    int n, num;
    std::ifstream myfile;
public:
    double* DataLine;
    ReadDataFromText(int Tn, const char* PathOfText);
    ~ReadDataFromText();
    void read();
    void Data2Arry(double* arr);
    void PrintData();
};

class AdmittanceInterface
{
private:
    double Madm;
    double Dadm;
    double v_before;
    double fl;
public:
    AdmittanceInterface(double TMadm, double TDadm);
    double compute(double tau, double dt);

};


void EndPosition(double* xpos, ANGLE* q);

void EndOrientation(double* xrot, ANGLE* q);


class ConcurrencyControl
{
private:
    KvaserForSimplIQ* pKvaser;
    volatile bool StopThread;
    std::thread thr;
public:
    ConcurrencyControl(KvaserForSimplIQ* TpKvaser);
    void run();
    void stop();
    void func();
};


class Get_ea
{
private:
    double ea_bf[3] = { 0 }, ea[3] = { 0 }, dea[3] = { 0 };
    double K[3] = { 0 }, D[3] = { 0 };
    int num=0;
public:
    void operator()(double* ea_overline, double* P, double* Fe, double dt);
};

#endif //ROS_KVASER_CAN_DRIVER_KVASER_CAN_H
