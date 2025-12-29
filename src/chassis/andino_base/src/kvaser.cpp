#include "andino_base/kvaser.h"

//Kvaser
/*构造*/
Kvaser::Kvaser(int channel_number)
{ 
    Tx = { 0x000,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    Rx = { 0x000,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    canInit(channel_number);
}
/*析构*/
Kvaser::~Kvaser()
{
    canRelease();
}
/*初始化can通道*/
int Kvaser::canInit(int channel_number)
{
    // 初始化 can 驱动（重复调用只会调用一次）
    canInitializeLibrary();
    // 打开 can 通道
    handle = canOpenChannel(channel_number, 0);
    if (handle < 0)
    {
        std::cout << "can Open Channel error!" << std::endl;
        return EXIT_FAILURE;
    }
    // 若发送消息时出现 TimeOut 错误，调整波特率（一般是 canBITRATE_500K）
    status = canSetBusParams(handle, canBITRATE_1M, 0, 0, 0, 0, 0);
    if (checkStatus("canSetBusParams"))
    {
        std::cout << "can " << channel_number << " Set Bus Params error!" << std::endl;
        return EXIT_FAILURE;
    }
    // 打开 can 总线
    status = canBusOn(handle);
    if (checkStatus("canBusOn"))
    {
        std::cout << "can Bus On error!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "can " << channel_number << " initialized!" << std::endl;
    return EXIT_SUCCESS;
}
/*关闭 can 通道*/
int Kvaser::canRelease()
{
    status = canBusOff(handle);
    if (checkStatus("canBusOff"))
    {
        return EXIT_FAILURE;
    }
    status = canClose(handle);
    if (checkStatus("canClose"))
    {
        return EXIT_FAILURE;
    }
    std::cout << "can released!" << std::endl;
    return EXIT_SUCCESS;
}
/*发送 can 消息*/
int Kvaser::canSend(CANMessage* Meassage)
{
    status = canWriteWait(handle, Meassage->COB_ID, Meassage->Byte, Meassage->DLC, 0, 100);
    if (checkStatus("canWriteWait"))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
/*读取 can 通道返回数据*/
int Kvaser::canReceive(CANMessage* message)
{
    if (canReadSpecific(handle, message->COB_ID, message->Byte, &(message->DLC), NULL, NULL) == canOK)
        return EXIT_SUCCESS;
    else
        return EXIT_FAILURE;
}
/*检查 can 通道状态*/
int Kvaser::checkStatus(const std::string& id)
{
    if (status != canOK)
    {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(status, buf, sizeof(buf));
        std::cout << id << ": failed, stat = " << (int)status << " info: " << buf << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

//KvaserForElmo
/*构造*/
KvaserForElmo::KvaserForElmo(int channel_number, int TNumOfNodes, MOTOR* TpNode, const char* TNameOfNodes) :
    Kvaser(channel_number), pNode(TpNode), NameOfNodes(TNameOfNodes), NumOfNodes(TNumOfNodes)
{
    netInit();
}
/*析构*/
KvaserForElmo::~KvaserForElmo()
{
    DisableMotors();
}
/*初始化can网络*/
int KvaserForElmo::netInit()
{
    int i;
    CANMessage Tx = { 0x000,2,{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
    //CANMessage Rx = { 0x000,2,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
    int flag = canSend(&Tx);
    if (flag == EXIT_FAILURE)
    {
        std::cout << "connect failed!" << std::endl;
        return EXIT_FAILURE;
    }

    canFlushReceiveQueue(handle);
    std::cout << NameOfNodes << " ";
    for (i = 1; i <= NumOfNodes; i++)
    {
        if ((pNode + i)->connect)
        {
            std::cout << (pNode + i)->id << ' ';
        }
    }
    std::cout << "have been connected!" << std::endl;
    canFlushReceiveQueue(handle);
    return EXIT_SUCCESS;
}
/*连接电机*/

int KvaserForElmo::connectMotor(MOTOR *m)
{
    CANMessage messageS1 = { 0x000,2,{0x82, (uint8_t)m->id} };
    CANMessage messageR = { 0x700 | m->id,1,{0x00} };
    CANMessage messageS2 = { 0x000,2,{0x01, (uint8_t)m->id} };
    canFlushReceiveQueue(handle);
    canSend(&messageS1);
    while (canReceive(&messageR) == EXIT_FAILURE);
    if (canSend(&messageS2) == EXIT_FAILURE)
    {
        std::cout << NameOfNodes <<' ' << m->id << " connect failed!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << NameOfNodes <<' ' << m->id << " has been connected!" << std::endl;
    return EXIT_SUCCESS;
}


/*电机使能*/
int KvaserForElmo::motorEnable(MOTOR*m)
{
    CANMessage messageS = { m->id | 0x300 ,8,{ 'M', 'O', 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 } };//MO=1
    if (canSend(&messageS) == EXIT_FAILURE)
    {
        std::cout << NameOfNodes <<' ' << m->id << " enable failed!" << std::endl;

        return EXIT_FAILURE;
    }
    while ((GetStatusWord(m) & 0x00000010) == 0x00000000);
    std::cout << NameOfNodes << ' ' << m->id << " has been enabled!" << std::endl;

    return EXIT_SUCCESS;
}
/*电机去使能*/
int KvaserForElmo::motorDisable(MOTOR*m)
{
    CANMessage messageS = { m->id | 0x300 ,8,{ 'M', 'O', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//MO=0
    // if (canSend(&messageS) == EXIT_FAILURE)
    // {
    //     std::cout << NameOfNodes <<' ' << m->id << " disabled failed!" << std::endl;
    //     return EXIT_FAILURE;
    // }
    //while ((GetStatusWord(m) & 0x00000010) == 0x00000010);
    canSend(&messageS) ;
    std::cout << NameOfNodes <<' ' << m->id << " has been disabled!" << std::endl;
    return EXIT_SUCCESS;
}


void KvaserForElmo::DisableMotors()
{
    for (int i = 1; i <= NumOfNodes; i++)if ((pNode + i)->connect)motorDisable(pNode + i);
    std::cout<<  NameOfNodes << 's' << " have been disabled!" << std::endl;
}
/*选择电机运动模式*/
int KvaserForElmo::modeChoose(MOTOR*m, Mode mode)
{
    CANMessage Tx = { m->id | 0x300 ,8,{ 'U', 'M', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//UM
    CANMessage Rx = { m->id | 0x280 ,8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//UM
    stop(m);
    motorDisable(m);
    Tx.Byte[4] = mode;
    usleep(2000000);
    if (canSend(&Tx) == EXIT_FAILURE) { std::cout << NameOfNodes << ' ' << m->id << " choose mode failed!" << std::endl; return EXIT_FAILURE; }
    do { canReceive(&Rx);
    }
    while (!((Rx.Byte[0] == 'U') && (Rx.Byte[1] == 'M') && ((Rx.Byte[3] & 0x40) == 0) && (Rx.Byte[4] == mode)));
    motorEnable(m);
    switch (mode)
    {
    case TORQUE_MODE:
        std::cout << NameOfNodes << ' ' << m->id << " torque mode has been chosen!" << std::endl;
        break;
    case SPEED_MODE:
        std::cout << NameOfNodes << ' ' << m->id << " speed mode has been chosen!" << std::endl;
        break;
    case MICRO_STEPPING_MODE:
        std::cout << NameOfNodes << ' ' << m->id << " micro stepping mode has been chosen!" << std::endl;
        break;
    case DUAL_FEEDBACK_POSITION_MODE:
        std::cout << NameOfNodes << ' ' << m->id << " dual feedback position mode has been chosen!" << std::endl;
        break;
    case POSITION_MODE:
        std::cout << NameOfNodes << ' ' << m->id << " position mode has been chosen!" << std::endl;
        break;
    }
    return EXIT_SUCCESS;
}
/*力矩（电流）模式*/
int KvaserForElmo::torqueMode(MOTOR* m, double torque)
{
    if (torque > m->In)torque = m->In;
    if (torque < -m->In) torque = -m->In;
    CANMessage messageS = { m->id | 0x300 ,8,{ 'T', 'C', 0x00, 0x80, 0x00, 0x00, 0x00, 0x00 } };//TC
    float2byte(messageS.Byte + 4, (float)(torque * m->direction));
    if (canSend(&messageS) == EXIT_FAILURE)
    {
        std::cout << NameOfNodes << ' ' << m->id << " start torque failed!" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
/*速度模式*/
int KvaserForElmo::SpeedMode(MOTOR*m, double speed)
{
    if (speed > m->Wn) speed = m->Wn;
    if (speed < -m->Wn) speed = -m->Wn;
    CANMessage messageS = { m->id | 0x300 ,8,{ 'J', 'V', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//JV
    CANMessage messageBG = { m->id | 0x300, 4, { 'B', 'G', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//BG
    int jv = rad2cnt(speed, m);
    int2byte(messageS.Byte + 4, jv);
    if (canSend(&messageS) == EXIT_FAILURE) { std::cout << NameOfNodes << ' ' << m->id << " start speed mode failed!" << std::endl; return EXIT_FAILURE; }
    if (canSend(&messageBG) == EXIT_FAILURE) { std::cout << NameOfNodes << ' ' << m->id << " start speed mode failed!" << std::endl; return EXIT_FAILURE; }
    return EXIT_SUCCESS;
}
/*位置模式*/
int KvaserForElmo::PositionMode(MOTOR* m, double position, double speed)
{
    CANMessage messageAC = { m->id | 0x300 ,8, { 'A', 'C', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//AC
    CANMessage messageDC = { m->id | 0x300 ,8, { 'D', 'C', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//DC
    CANMessage messageSP = { m->id | 0x300 ,8, { 'S', 'P', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//SP
    CANMessage messagePA = { m->id | 0x300 ,8, { 'P', 'A', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//PA
    CANMessage messageBG = { m->id | 0x300, 4, { 'B', 'G', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//BG
    int2byte(messageAC.Byte+4, 1000000); // ac=
    int2byte(messageDC.Byte+4, 1000000); // dc=
    int2byte(messageSP.Byte+4, rad2cnt(speed, m) * m->direction); // sp=   sp>=0
    int2byte(messagePA.Byte+4, (int32_t)(rad2cnt(position, m) + m->encoder.AbsZeroPos)); // pa=
    canSend(&messageAC); 
    canSend(&messageDC); 
    canSend(&messageSP); 
    canSend(&messagePA); 
    canSend(&messageBG);
    std::cout << NameOfNodes << ' ' << m->id<< " start position mode!" << std::endl;
    return EXIT_SUCCESS;
}
/*电机复位*/
int KvaserForElmo::motorReset(MOTOR* m)
{
    modeChoose(m, POSITION_MODE);
    PositionMode(m, 0, 0.1*m->Wn);
    return EXIT_SUCCESS;
}
void KvaserForElmo::ResetMotors(double WaitTime)
{
    MOTOR* temp;
    for (temp = pNode + 1; temp <= pNode + NumOfNodes; temp++)
    {
        if (temp->connect)motorReset(temp);
    }
    usleep(WaitTime * 1000 * 1000);
    std::cout << NameOfNodes << 's' << " have been reseted!" << std::endl;
}
/*位置初始化*/
void KvaserForElmo::PosInit(double WaitTime)
{
    MOTOR* temp;
    for (temp = pNode + 1; temp <= pNode + NumOfNodes; temp++)
    {
        if (temp->connect)
        {
            modeChoose(temp, POSITION_MODE);
            PositionMode(temp, temp->InitPos, 0.1 * temp->Wn);
        }
    }
    usleep(WaitTime * 1000 * 1000);
    std::cout << "Position initialization has been completed!" << std::endl;
}
/*获取真实位置*/
double KvaserForElmo::GetPosition(MOTOR* m)
{
    CANMessage messageS = { m->id | 0x300,4,{ 'P', 'X', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//PX
    CANMessage messageR = { m->id | 0x280,8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
    canFlushReceiveQueue(handle);
    canSend(&messageS);
    do { canReceive(&messageR); } while (!((messageR.Byte[0] == 'P') && (messageR.Byte[1] == 'X') && ((messageR.Byte[3] & 0x40) == 0)));
    int data = byte2int(messageR.Byte + 4) - m->encoder.AbsZeroPos;
    double pos_rad = cnt2rad(data, m);
        // 🔍 打印反馈
    std::cout << "[Position] Motor ID: " << m->id
              << " | Raw Cnt: " << data
              << " | Position: " << pos_rad << " rad"
              << std::endl;
              return 0;
}
/*获取真实速度*/
double KvaserForElmo::GetVelocity(MOTOR* m)
{
    CANMessage messageS = { m->id | 0x300,4,{ 'V', 'X', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//VX
    CANMessage messageR = { m->id | 0x280,8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
    canFlushReceiveQueue(handle);
    canSend(&messageS);
    do { canReceive(&messageR); } while (!((messageR.Byte[0] == 'V') && (messageR.Byte[1] == 'X') && ((messageR.Byte[3] & 0x40) == 0)));
    int data = byte2int(messageR.Byte + 4);
    double vel_rad = cnt2rad(data, m);
        std::cout << "[Velocity] Motor ID: " << m->id
              << " | Raw Cnt/s: " << data
              << " | Velocity: " << vel_rad << " rad/s"
              << std::endl;
            return 0;
}
/*停止运行*/
int KvaserForElmo::stop(MOTOR* m)
{
    CANMessage Tx = { m->id | 0x300 ,8,{ 'S', 'T', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//ST
    CANMessage Rx = { m->id | 0x280 ,8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//ST
    canSend(&Tx);
    do { canReceive(&Rx); } while (!((Rx.Byte[0] == 'S') && (Rx.Byte[1] == 'T') && ((Rx.Byte[3] & 0x40) == 0)));
    return EXIT_SUCCESS;
}
void KvaserForElmo::stopAll()
{
    CANMessage Tx = { 0x000 ,8,{ 'S', 'T', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };//ST
    MOTOR* temp = pNode;
    int i;
    for (i = 0; i < NumOfNodes; i++)
    {
        temp++;
        Tx.COB_ID = temp->id | 0x300;
        status = canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
        checkStatus("canWrite");
    }
    status = canWriteSync(handle, 100);
    checkStatus("canWriteSync");
}
/*读状态寄存器*/
int KvaserForElmo::GetStatusWord(MOTOR* m)
{
    CANMessage messageS = { m->id | 0x300,4,{ 'S', 'R', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };//SR
    CANMessage messageR = { m->id | 0x280,8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
    canFlushReceiveQueue(handle);
    canSend(&messageS);
    do { canReceive(&messageR); } while (!((messageR.Byte[0] == 'S') && (messageR.Byte[1] == 'R') && ((messageR.Byte[3] & 0x40) == 0)));
    return byte2int(messageR.Byte + 4);
}

//KvaserForGold KvaserForSimplIQ
/*构造*/
KvaserForGold::KvaserForGold(int channel_number, int NumOfNodes, MOTOR* pNode, const char* NameOfNodes):KvaserForElmo(channel_number, NumOfNodes, pNode, NameOfNodes)
{
}

/*配置驱动器RPDO*/
int KvaserForGold::RPDOconfig(MOTOR* m, Mode mode)
{
    uint8_t H_mode = 0x00;
    uint8_t L_mode = 0x00;
    uint8_t num_mode = 0x00;
    uint8_t nnnn = 0x00;
    unsigned int L = 8;
    switch(mode)
    {
    case POSITION_MODE:
        H_mode = 0x60; L_mode = 0x7A; num_mode = 0x01; nnnn = 0x20; L = 6; break;
    case SPEED_MODE:
        H_mode = 0x60; L_mode = 0xFF; num_mode = 0x03; nnnn = 0x20; L = 6; break;
    case TORQUE_MODE:
        H_mode = 0x60; L_mode = 0x71; num_mode = 0x04; nnnn = 0x10; L = 4; break;
    default:std::cout << NameOfNodes << ' ' << m->id << " RPDO config error" << '\n'; return EXIT_FAILURE;
    }
    
    CANMessage Tx[9];// Rx[9], Receive;
    Tx[0] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x14, 0x01, (uint8_t)m->id, 0x02, 0x00, 0x80 } };//关闭TPDO通道
    Tx[1] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x80 } };//清空通道
    Tx[2] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60 } };//配置6040(控制字）
    Tx[3] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x16, 0x02, nnnn, 0x00, L_mode, H_mode } };//配置mode
    Tx[4] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00 } };//循环发送
    Tx[5] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00 } };//开启两个通道
    Tx[6] = { 0x600 | m->id, 8,{ 0x22, 0x00, 0x14, 0x01, (uint8_t)m->id, 0x02, 0x00, 0x00 } };//打开通道
    Tx[7] = { 0x600 | m->id, 8,{ 0x22, 0x60, 0x60, 0x00, num_mode, 0x00, 0x00, 0x00 } };//配置模式
    Tx[8] = { 0x200 | m->id, L,{ 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    /*Rx[0] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00 } };
    Rx[1] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Rx[2] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x16, 0x01, 0x00, 0x00, 0x00, 0x00 } };
    Rx[3] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x16, 0x02, 0x00, 0x00, 0x00, 0x00 } };
    Rx[4] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x14, 0x02, 0x00, 0x00, 0x00, 0x00 } };
    Rx[5] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Rx[6] = { 0x580 | m->id, 8,{ 0x60, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00 } };
    Rx[7] = { 0x580 | m->id, 8,{ 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Rx[8] = { 0x180 | m->id, 2,{ 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };*/
    canFlushReceiveQueue(handle);
    for (int i = 0; i < 9; i++)
    {
        if (canSend(Tx + i) == EXIT_FAILURE)
        {
            std::cout << NameOfNodes << ' ' << m->id << " RPDO config error" << '\n';
            return EXIT_FAILURE;
        }
    }
    canFlushReceiveQueue(handle);
    std::cout << NameOfNodes << ' ' << m->id << " RPDO config success" << '\n';
    return EXIT_SUCCESS;
}
void KvaserForGold::SendTorqueCommand(MOTOR* m,double torque)
{
    Tx = { 0x200 | m->id,4,{0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    double current = torque * m->Kt_inv * m->direction;
    if (current > m->In) current = m->In;
    if (current < -m->In)  current = -m->In;
    int16_t current_permillage = (int16_t)(current * 1000.0 / (m->In * 1.4142135623731));
    Tx.Byte[2] = current_permillage & 0x00FF;
    Tx.Byte[3] = current_permillage >> 8;
    canSend(&Tx);
}
void KvaserForGold::SendTorqueCommandForAll()
{
    Tx = { 0x200 ,4,{0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    double current;
    int16_t current_permillage;
    MOTOR* temp;
    for (temp = pNode + 1; temp <= pNode + NumOfNodes; temp++)
    {
        if (temp->connect)
        {
            Tx.COB_ID = 0x200 | temp->id;
            current = temp->tc * temp->Kt_inv * temp->direction;
            if (current > temp->In) current = temp->In;
            if (current < -temp->In)  current = -temp->In;
            current_permillage = (int16_t)(current * 1000.0 / (temp->In * 1.4142135623731));
            Tx.Byte[2] = current_permillage & 0x00FF;
            Tx.Byte[3] = current_permillage >> 8;
            status = canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
            checkStatus("canWrite");
        }
    }
    status = canWriteSync(handle, 100);
    checkStatus("canWriteSync");
}
void KvaserForGold::SendSpeedCommand(MOTOR* m, double speed)
{
    CANMessage Tx = { 0x200 | m->id, 6,{ 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    if (speed > m->Wn) speed = m->Wn;
    if (speed < -m->Wn) speed = -m->Wn;
    int jv = rad2cnt(speed, m);
    int2byte(Tx.Byte + 2, jv);
    canSend(&Tx);
}

void KvaserForGold::SendSpeedCommandForAll()
{
    CANMessage Tx = { 0x200 ,6,{0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    double speed;
    int jv;
    MOTOR* temp;
    for (temp = pNode + 1; temp <= pNode + NumOfNodes; temp++)
    {
        if (temp->connect)
        {
            Tx.COB_ID = 0x200 | temp->id;
            speed = temp->jv;
            if (speed > temp->Wn) speed = temp->Wn;
            if (speed < -temp->Wn)  speed = -temp->Wn;
            jv = rad2cnt(speed, temp);
            int2byte(Tx.Byte + 2, jv);
            status = canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
            checkStatus("canWrite");
        }
    }
    status = canWriteSync(handle, 100);
    checkStatus("canWriteSync");
}

void KvaserForGold::SendPositionCommand(MOTOR* m, double position)
{
    CANMessage Tx = { 0x200 | m->id, 6,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Tx.Byte[0] = 0x0F;
    status = canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
    checkStatus("canWrite");
    int pa = rad2cnt(position, m) + m->encoder.AbsZeroPos;
    Tx.Byte[0] = 0x3F;
    int2byte(Tx.Byte + 2, pa);
    status = canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
    checkStatus("canWrite");
    status = canWriteSync(handle, 100);
    checkStatus("canWriteSync");
}



/*配置驱动器TPDO*/
int KvaserForGold::TPDOconfigPXVX(MOTOR* m, uint8_t T)
{
    CANMessage Tx[8], Rx[8], Receive;
    uint8_t CH = 0x03;//0x00-0x03
    uint16_t HL = (uint16_t)(0x480 | m->id);
    uint8_t H = HL >> 8;
    uint8_t L = HL & 0x00FF;
    Tx[0] = { 0x600 | m->id, 8,{ 0x22, CH, 0x18, 0x01, L,    H,    0x00, 0x80 } };//关闭TPDO通道
    Tx[1] = { 0x600 | m->id, 8,{ 0x22, CH, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00 } };//清空TPDO1通道
    Tx[2] = { 0x600 | m->id, 8,{ 0x22, CH, 0x1A, 0x01, 0x20, 0x00, 0x69, 0x60 } };//配置6069(速度）
    Tx[3] = { 0x600 | m->id, 8,{ 0x22, CH, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60 } };//配置6064(位置）
    Tx[4] = { 0x600 | m->id, 8,{ 0x22, CH, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00 } };//开启两个通道
    Tx[5] = { 0x600 | m->id, 8,{ 0x22, CH, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00 } };//循环发送
    Tx[6] = { 0x600 | m->id, 8,{ 0x22, CH, 0x18, 0x05, T,    0x00, 0x00, 0x00 } };//发送周期默认2ms
    Tx[7] = { 0x600 | m->id, 8,{ 0x22, CH, 0x18, 0x01, L,    H,    0x00, 0x00 } };//打开TPDO
    
    Rx[0] = { 0x580 | m->id, 8,{ 0x60, CH, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00 } };
    Rx[1] = { 0x580 | m->id, 8,{ 0x60, CH, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Rx[2] = { 0x580 | m->id, 8,{ 0x60, CH, 0x1A, 0x01, 0x00, 0x00, 0x00, 0x00 } };
    Rx[3] = { 0x580 | m->id, 8,{ 0x60, CH, 0x1A, 0x02, 0x00, 0x00, 0x00, 0x00 } };
    Rx[4] = { 0x580 | m->id, 8,{ 0x60, CH, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    Rx[5] = { 0x580 | m->id, 8,{ 0x60, CH, 0x18, 0x02, 0x00, 0x00, 0x00, 0x00 } };
    Rx[6] = { 0x580 | m->id, 8,{ 0x60, CH, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00 } };
    Rx[7] = { 0x580 | m->id, 8,{ 0x60, CH, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00 } };


    Receive = { 0x580 | m->id, 8,{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };

    int i, j;
    for (i = 0; i < 8; i++)
    {
        canFlushReceiveQueue(handle);
        memset(Receive.Byte, 0x00, 8 * sizeof(uint8_t));
        canSend(Tx + i);
        while (canReceive(&Receive) == EXIT_FAILURE);
        for (j = 0; j < 8; j++)
        {
            if (Receive.Byte[j] != Rx[i].Byte[j])
            {
                std::cout << NameOfNodes << ' ' << m->id << " TPDOPXVX config error" << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    std::cout << NameOfNodes << ' ' << m->id << " TPDO configPXVX success" << std::endl;
    return EXIT_SUCCESS;
}

void KvaserForGold::GetPositionAndVelocity()
{
    if (!pNode || NumOfNodes <= 0)
        return;

    CANMessage Rx;
    Rx.DLC = 8; // 每个 PDO 帧 8 字节

    for (int i = 0; i < NumOfNodes; i++)
    {
        MOTOR* motor = &pNode[i];
        if (!motor->connect)
            continue;

        // 1️⃣ 设置 TPDO1 默认 COB-ID = 0x180 + NodeID
        Rx.COB_ID = 0x180 + motor->id;

        // 2️⃣ 接收 CAN 帧（非阻塞或带超时）
        int timeout_ms = 50; // 50ms 超时
        auto start_time = std::chrono::steady_clock::now();
        int ret;
        while ((ret = canReceive(&Rx)) != EXIT_SUCCESS)
        {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms)
            {
                std::cerr << "⚠️ 电机 " << motor->id << " TPDO 超时" << std::endl;
                break;
            }
        }

        if (ret != EXIT_SUCCESS)
            continue; // 本次循环跳过

        // 3️⃣ 打印原始 CAN 数据（调试用）
        std::cout << "电机 " << motor->id << " 原始 CAN 数据: ";
        for (unsigned int k = 0; k < Rx.DLC; ++k)
            std::cout << std::hex << static_cast<int>(Rx.Byte[k]) << " ";
        std::cout << std::dec << std::endl;

        // 4️⃣ 解析速度和位置
        // 假设 TPDO1 前 4 字节速度，后 4 字节位置
        int data_vx = byte2int(Rx.Byte);         // 前 4 字节速度
        int data_px = byte2int(Rx.Byte + 4);     // 后 4 字节位置

        motor->vx = cnt2rad(data_vx, motor);                       // 脉冲转 rad/s
        motor->px = cnt2rad(data_px - motor->encoder.AbsZeroPos, motor); // 脉冲转 rad

        // 5️⃣ 打印解析结果
        std::cout << "解析后: 速度 = " << motor->vx
                  << " rad/s, 位置 = " << motor->px << " rad" << std::endl;
    }
}

