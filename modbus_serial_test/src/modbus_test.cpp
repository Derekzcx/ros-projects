/*
本程序用于实现基于modbus协议的串口通讯节点设计,并订阅上位机的指令信息
 本程序针对的电机驱动为 雷赛的数字步进电机驱动 DM2C-RS556,采用的通讯方式位RS485,modbus协议
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <modbus/modbus.h>
#include <string.h>
#include <string>
#include <sstream> // stringstream的头文件
#include <vector>

const int Slave_Adress = 0x01;

const int Pulses_Per_Revolut_Register = 0x0001;    // 脉冲数/转,默认10000,范围200-51200,单位P/R
const int Dir_Register =                0x0007;    // 电机运转方向,0为正,1位负,,默认为 0

const int DI2_Mode_Register =           0x0147;    //DI2输入口的寄存器地址
const int DI_In_Enable =                0x0080;    //使能输入，配置成常闭就是使能
const int DI_In_Disable =               0x0000;    //无效输入
const int DI_In_Emergency_Stop =        0x0022;    //强制急停

const int DI_State_Register =           0x0179;    //DI1模式配置
const int Motor_Current_Register =      0x0191;    //电机峰值电流
                                                                
const int Trigger_Register =            0x6002;    //触发寄存器  //触发方式,0x01P(p=0-15),p端定位
                                                                // 0x020,回零 ; 0x040,急停; 0x021 当前位置手动设零
const int Start_P0 =                    0x0010;    //路径0触发运行  
const int Set_Position_0 =              0x0021;    //回零触发,设定当前位置为0 
const int Back_0 =                      0x0020;    //回0
const int Stop_Now =                    0x0040;    //急停

const int Move_Mode_Path_0_Register =   0x6200;    //运动路径0寄存器 
const int Position_Mode =               0x0001;    //位置模式, y用于路径0寄存器的0-3bit位的设置,0,1(位置定位),2(速度定位),3
const int Speed_Mode =                  0x0002;    //速度模式

const int Position_H =                   0x6201;    //位置H寄存器
const int Position_L =                   0x6202;    //位置L寄存器
const int Speed_Register =               0x6203;    //速度寄存器,用于设置速度  //单位 rpm
const int Acceleration_Register =        0x6204;    //加速度寄存器,  //单位 ms/1000rpm
const int Deceleration_Register =        0x6205;    //减速度寄存器,  //单位ms/1000rpm

typedef struct st_port_info{
    char m_port[21];    // 保存串口驱动端口, 若 "/dev/ttyUSB0"
    int  m_baud;        // 波特率
    char m_parity;      // 奇偶校验位,N无,E偶,O奇
    int  m_data_bit;    // 数据位,允许值5,6,7,8
    int  m_stop_bit;    // 停止位,允许值1,2
} port_info;

class Modbus_Connect{
private:
    port_info m_mb_info; // 接口配置
    modbus_t* m_mb;      // modbus接口d
    int m_slave_adress;  // 从机地址
public:
    bool m_port_state;  // 串口状态
    uint16_t m_state_registers[64];// 保存读取的保持寄存器的结果
    bool m_dirction_flag; //电机方向, false为上, true为下
    Modbus_Connect(){
        m_mb = NULL;
        m_port_state = false;
        m_dirction_flag = false;
        memset(m_state_registers, 0, sizeof(m_state_registers));
    }
    Modbus_Connect(port_info &info, int slave, bool direction_flag = false){
        m_mb_info = info;
        m_slave_adress = slave;
        m_port_state = false;
        m_dirction_flag = direction_flag;
        memset(m_state_registers, 0, sizeof(m_state_registers));
    }
    // 初始化modbus连接
    bool init_modbus(const port_info &info, int slave, bool direction_flag = false){
        m_mb_info = info;
        m_slave_adress = slave;
        m_dirction_flag = direction_flag;

        m_mb = modbus_new_rtu(m_mb_info.m_port, m_mb_info.m_baud, m_mb_info.m_parity,m_mb_info.m_data_bit,m_mb_info.m_stop_bit);
        if(m_mb == NULL) { 
            modbus_free(m_mb);
            return false;
        }
        modbus_set_slave(m_mb,m_slave_adress);
        modbus_rtu_set_serial_mode(m_mb, MODBUS_RTU_RS485);
        modbus_rtu_set_rts(m_mb, MODBUS_RTU_RTS_UP);// 设置rts位up
        return true;
    }
    // 发起连接
    bool Connect(){ 
        int ret = modbus_connect(m_mb);
        if(ret == -1) {
            modbus_free(m_mb);
            return false;
        }
        m_port_state = true;
        return true;
    }
    // 初始化电机
    bool init_motor(){
        int ret1 = modbus_write_register(m_mb, Pulses_Per_Revolut_Register, 1000);
        int ret2 = modbus_write_register(m_mb, Motor_Current_Register, 50);
        int ret3 = modbus_write_register(m_mb, Dir_Register, m_dirction_flag);//设置电机方向
        int ret4 = modbus_write_register(m_mb, DI2_Mode_Register, DI_In_Enable|DI_In_Emergency_Stop);//DI2输入有效信号后即停
        if(ret1 == -1 || ret2 == -1 || ret3 == -1 || ret4 == -1) return false;
        return true;
    }
    // 停止电机运转
    bool Stop_motor(){
        int ret = -1;
        if(m_port_state == true){
            ret = modbus_write_register(m_mb, Trigger_Register, Stop_Now);
        }
        if(ret == -1) return false;
        return true;
    }
    // 电机速度模式运行
    bool Run_motor_speed_mode(uint16_t speed){
        if(m_port_state == false) return false;
        memset(m_state_registers, 0, sizeof(m_state_registers));
        int ret = modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);
        if(ret == -1) return false;
        // printf("m_state_registers[0] = %x\n", m_state_registers[0]);
        // printf("m_state_registers[0] & 0x02 = %x\n", m_state_registers[0] & 0x02);
        // printf("DI_In_Disable | DI_In_Emergency_Stop = %x\n", DI_In_Disable | DI_In_Emergency_Stop);
        // printf("DI_In_Enable | DI_In_Emergency_Stop = %x\n", DI_In_Enable | DI_In_Emergency_Stop);
        if(bool(m_state_registers[0] & 0x02) && !m_dirction_flag){ // //如果当前在最低点，那么当前的急停已经使能要先设置成失能
            if(modbus_write_register(m_mb, DI2_Mode_Register, DI_In_Disable | DI_In_Emergency_Stop)==-1) return false;
            //启动
            if(modbus_write_register(m_mb, Move_Mode_Path_0_Register, Speed_Mode) == -1) return false;
            if(modbus_write_register(m_mb, Speed_Register, speed) == -1) return false;
            if(modbus_write_register(m_mb, Trigger_Register, Start_P0)) return false; 
            //等待低电平到
            memset(m_state_registers, 0, sizeof(m_state_registers));
            modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);

            while (!(m_state_registers[0] & 0x02)){
                memset(m_state_registers, 0, sizeof(m_state_registers));
                modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);
            }
            // 启动高点平急停
            if(modbus_write_register(m_mb, DI2_Mode_Register, DI_In_Enable | DI_In_Emergency_Stop)==-1) return false;
        }
        if(modbus_write_register(m_mb, Move_Mode_Path_0_Register, Speed_Mode) == -1) return false;
        if(modbus_write_register(m_mb, Speed_Register, speed) == -1) return false;
        if(modbus_write_register(m_mb, Trigger_Register, Start_P0)==-1) return false;
        return true;
    }


    // 绝对位置模式启动
    bool Run_motor_pos_mode(uint16_t pos, uint16_t speed, uint16_t acc){
        if(m_port_state == false) return false;
        memset(m_state_registers, 0, sizeof(m_state_registers));
        int ret = modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);
        if(ret == -1) return false;

        if(bool(m_state_registers[0] & 0x02) && ! m_dirction_flag){
            if(modbus_write_register(m_mb, DI2_Mode_Register, DI_In_Disable | DI_In_Emergency_Stop)==-1) return false;
            // 启动
            if(modbus_write_register(m_mb, Move_Mode_Path_0_Register, Speed_Mode) == -1) return false;
            if(modbus_write_register(m_mb, Speed_Register, speed) == -1) return false;
            if(modbus_write_register(m_mb, Trigger_Register, Start_P0)) return false; 
            // 等待低电平
            memset(m_state_registers, 0, sizeof(m_state_registers));
            modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);

            while (!(m_state_registers[0] & 0x02)){
                memset(m_state_registers, 0, sizeof(m_state_registers));
                modbus_read_registers(m_mb, DI_State_Register, 1, m_state_registers);
            }
            // 启动高点平急停
            if(modbus_write_register(m_mb, DI2_Mode_Register, DI_In_Enable | DI_In_Emergency_Stop)==-1) return false;
        }
        if(modbus_write_register(m_mb, Move_Mode_Path_0_Register, Position_Mode) == -1 ) return false;
        if(modbus_write_register(m_mb, Position_H, pos>>16) == -1) return false;
        if(modbus_write_register(m_mb, Position_L, (pos & 0xFFFF)) == -1) return false;
        if(modbus_write_register(m_mb, Speed_Register, speed) == -1) return false;
        if(modbus_write_register(m_mb, Acceleration_Register, acc)==-1) return false;
        if(modbus_write_register(m_mb, Deceleration_Register, acc) == -1) return false;
        if(modbus_write_register(m_mb, Trigger_Register, Start_P0) == -1) return false;
        
        return true;
    }
    // 手动设置回0位置
    bool Set_zero_position(){
        if(m_port_state == false) return false;
        int ret = modbus_write_register(m_mb, Trigger_Register, Set_Position_0);
        if(ret == -1) return false;
        return true;
    }
    // 改变电机方向
    bool Change_motor_direction(){
        if(m_port_state == false) return false;
        m_dirction_flag = !m_dirction_flag;
        int ret = modbus_write_register(m_mb, Dir_Register, m_dirction_flag);
        if(ret == -1) return false;
    }
    // 回到0位置
    bool back_zero_position(){
        if(m_port_state == false) return false;
        int ret = modbus_write_register(m_mb, Trigger_Register, Back_0);
        if(ret == -1) return false;
        return true;
    }
    ~Modbus_Connect(){
        modbus_close(m_mb);
        modbus_free(m_mb);
    }
};
/*
    约定:指令组成位 string,包括: [str1:str2] str1 代表功能码,str2包含功能参数
*/

/***************************************************/
enum class Funs{
    Stop,             // 急停码
    Change_direction, // 改变允许方向
    Speed_mode,       // 速度模式运行
    Pos_mode,         // 速度模式运行
    Set_zero,         // 设置归0位
    Back_zero         // 归0
};

port_info info_data;
Modbus_Connect mb;
ros::Publisher pub;             // 发布者用于的发布电机的初始化状态
ros::Subscriber sub;       // 订阅急停的下发指令 
std_msgs::String pub_state_msgs;

/********************** 回调函数 *****************************/
void Callback_Function_deal(const std_msgs::String &msg){
    //[str1:str2], 如 1:2,100,10
    std::string strs = msg.data;
    //printf("strs: %s\n", strs.c_str());
    std::string str1 = (msg.data).substr(0, 1);
    //printf("str1: %s\n", str1.c_str());
    std::string str2 = (msg.data).substr(2, (msg.data).size()-2);
    //printf("str2: %s\n", str2.c_str());
    Funs fun_id = (Funs)atoi(str1.c_str());
    // 判断指令
    switch (fun_id)
    {
    case Funs::Stop :{ 
        if(!mb.Stop_motor()){
            pub_state_msgs.data = "mb.Stop_motor() failed";
            pub.publish(pub_state_msgs);
            return;
        } 
        break;
    }
    case Funs::Change_direction :{
        if(!mb.Change_motor_direction()){
            pub_state_msgs.data = "mb.Change_motor_direction() failed";
            pub.publish(pub_state_msgs);
            return;
        }
        break;
    }
    case Funs::Speed_mode :{
        uint16_t speed = (uint16_t)atoi(str2.c_str());
        if(!mb.Run_motor_speed_mode(speed)){
            char errors[51];
            sprintf(errors, "mb.Run_motor_speed_mode(%d) failed", speed);
            pub_state_msgs.data = errors;
            pub.publish(pub_state_msgs);
            return;
        }
        break;
    }
    case Funs::Pos_mode :{
        std::stringstream ss(str2);
        std::vector<std::string> args;
        std::string tmp;
        while(std::getline(ss, tmp, ',')){
            args.emplace_back(tmp);
        }
        if(args.size() != 3){
            pub_state_msgs.data = "The number of Pos_mode's args is not enough";
            pub.publish(pub_state_msgs);
            return;
        }
        uint16_t pos_arg = (uint16_t)atoi(args[0].c_str());
        uint16_t speed_arg = (uint16_t)atoi(args[1].c_str());
        uint16_t acc_arg = (uint16_t)atoi(args[2].c_str());
        if(!mb.Run_motor_pos_mode(pos_arg,speed_arg, acc_arg)){
            char errors[61];
            sprintf(errors, "mb.Run_motor_pos_mode(%d,%d,%d) failed", pos_arg, speed_arg, acc_arg);
            pub_state_msgs.data = errors;
            pub.publish(pub_state_msgs);
            return;
        }
        break;
    }
    case Funs::Set_zero :{
        if(!mb.Set_zero_position()){
            pub_state_msgs.data = "mb.Set_zero_position() failed";
            pub.publish(pub_state_msgs);
            return;
        }
        break;
    }
    case Funs::Back_zero :{
        if(!mb.back_zero_position()){
            pub_state_msgs.data = "mb.back_zero_position() failed";
            pub.publish(pub_state_msgs);
            return;
        }
        break;
    }
    default:
        pub_state_msgs.data = "funtion_id do not exist";
        pub.publish(pub_state_msgs);
        break;
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "modbus_test");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::String>("/motor_states",100);
    sub = nh.subscribe("/ui_command",1, Callback_Function_deal);
    
    /*参数开放开放出去给launch文件*/
    std::string tmp_port, parity;
    int baud, data_bit, stop_bit;
    nh.param<std::string>("port",tmp_port, "/dev/ttyS0");
    nh.param<int>("baud",baud, 34800);
    nh.param<std::string>("parity",parity, "N");
    nh.param<int>("data_bit", data_bit, 8);
    nh.param<int>("stop_bit", stop_bit, 1);
    
    memset(info_data.m_port, 0, sizeof(info_data.m_port));
    strcpy(info_data.m_port, tmp_port.c_str());//端口根据情况而变
    info_data.m_baud = baud;
    info_data.m_parity = parity[0];
    info_data.m_data_bit = data_bit;
    info_data.m_stop_bit = stop_bit;
    // 初始化modbus
    if(!mb.init_modbus(info_data, Slave_Adress)){
        ROS_INFO("Modbus_Connect init_modbus() failed error: %s",modbus_strerror(errno));
        return -1;
    }
    if(!mb.Connect()){
        ROS_INFO("Modbus_Connect Connect() failed error: %s",modbus_strerror(errno));
        return -1;
    }
    if(!mb.init_motor()){
        ROS_INFO("Modbus_Connect init_motor() failed error: %s",modbus_strerror(errno));
        return -1;
    }
    sleep(0.1);
    pub_state_msgs.data = "modbus and motor have inited successfully";
    pub.publish(pub_state_msgs);
    sleep(1);
    ros::spin();
    return 0;
}


