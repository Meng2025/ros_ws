#include <ros/ros.h>
#include <serial_pkg/serial_commu.h>

Arm::Arm()
{
    serial::Timeout waittime = serial::Timeout::simpleTimeout(1000);

    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(waittime);
 
    try
    {
        //打开串口
        sp.open();
    }

    catch(serial::IOException& e)
    {
        ROS_ERROR("串口打开错误，检查硬件连接.");
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO("/dev/ttyUSB0 串口已打开.");
    }
    else
    {
        ROS_ERROR("无法打开串口.");
    }

    joint[0] = 0.0;
    joint[1] = 0.0;
    joint[2] = 0.0;
    joint[3] = 0.0;
    joint[4] = 0.0;
}

Arm::~Arm()
{
    sp.close();
    ROS_INFO("/dev/ttyUSB0 串口已关闭");
}


void Arm::set(uint8_t id, float degree)
{
    uint16_t servo_degree = (uint16_t)(degree/3.141593*180.0*12.33333 + 2048);
    uint16_t run_time = abs(servo_degree - servo_get1(id));
    // ROS_INFO("%d %d %d",id,servo_degree,run_time);

    ros::Duration(0.01).sleep();
    servo_set1(id, servo_degree, run_time);
}


void Arm::set(float degree_list[])
{
    uint16_t current_position[5];
    uint16_t set_position[5];

    set_position[0] = (uint16_t)(degree_list[0]/3.141593*180.0*12.33333 + 2048);
    set_position[1] = (uint16_t)(-degree_list[1]/3.141593*180.0*12.33333 + 2048);
    set_position[2] = (uint16_t)(-degree_list[2]/3.141593*180.0*12.33333 + 2048);
    set_position[3] = (uint16_t)(-degree_list[3]/3.141593*180.0*12.33333 + 2048);
    set_position[4] = (uint16_t)(-degree_list[4]/3.141593*180.0*12.33333 + 2048);

    servo_get5(current_position);

    uint16_t run_time = 0;
    uint16_t max_err = 0;

    for(int i=0;i<5;i++)
    {
        if (abs(current_position[i]-set_position[i])>max_err)
        {
            max_err = abs(current_position[i]-set_position[i]);
        }
    }

    run_time = max_err*2;

    servo_set5(set_position, run_time);
}


void Arm::get(float degree_list[])
{
    ROS_INFO("%d %d %d %d %d",servo_get1(1),servo_get1(2)\
            ,servo_get1(3),servo_get1(4),servo_get1(5));
}

void Arm::get(uint8_t id)
{
    ROS_INFO("%d",servo_get1(id));
}

void Arm::gripper(float degree)
{

}






uint16_t Arm::servo_get1(uint8_t id)
{
    std::string recive_data;

serial_send:
    sp.flushInput();                        /* 清除接收缓存区 */

    /* 发送查询指令 */
    uint8_t data_send[8] = {0xff, 0xff};    /* 帧头 */            
    data_send[2] = id;                      /* 舵机ID */
    data_send[3] = 0x04;                    /* 数据长度 */
    data_send[4] = 0x02;                    /* 指令类型：读 */
    data_send[5] = 0x38;
    data_send[6] = 0x02;
    data_send[7] =  ~(data_send[2] + data_send[3] \
                    + data_send[4] + data_send[5] \
                    + data_send[6] );       /* 和校验 */
    sp.write(data_send,8);

    /* 接收回传数据 */
    recive_data = sp.read(8);
    if( (recive_data.length() != 8) || (sp.available()!=0) )
    {
        ROS_WARN("串口接收数据长度错误，尝试重新接收.");
        goto serial_send;
    }
 
    /* 数据解析 */
    if ( ((recive_data[0]&0xff) == 0xff) && ((recive_data[1]&0xff) == 0xf5))
    {
        uint8_t check_sum = (~((recive_data[2]&0xff) + (recive_data[3]&0xff) +\
                            (recive_data[4]&0xff) + (recive_data[5]&0xff) +\
                            (recive_data[6]&0xff)))&0xff;

        if (check_sum==(recive_data[7]&0xff))
        {
            uint16_t degree = ((recive_data[5]&0xff)<<8) + (recive_data[6]&0xff);

            return degree;
        }
        else
        {
            ROS_ERROR("校验错误，尝试重新接收");
            goto serial_send;
        }
    }
    else
    {
        ROS_ERROR("帧头错误，尝试重新接收");
        goto serial_send;
    }
}

void Arm::servo_get5(uint16_t position[])
{
    position[0] = servo_get1(1);
    position[1] = servo_get1(2);
    position[2] = servo_get1(3);
    position[3] = servo_get1(4);
    position[4] = servo_get1(5);  
}


void Arm::servo_set1(uint8_t id, uint16_t position, uint16_t run_time)
{
    uint8_t data_send[11] = {0xff, 0xff};   /* 帧头 */            
    data_send[2] = id;                      /* 舵机ID */
    data_send[3] = 0x07;                    /* 数据长度 */
    data_send[4] = 0x03;                    /* 指令类型：写 */
    data_send[5] = 0x2a;
    data_send[6] = (position>>8)&0xff;        /* 转角高8位 */
    data_send[7] = position&0xff;             /* 转角低8位 */
    data_send[8] = (run_time>>8)&0xff;      /* 执行时间高8位 */
    data_send[9] = run_time&0xff;           /* 执行时间低8位 */
    data_send[10] = (~(data_send[2] + data_send[3] + data_send[4] + \
                    data_send[5] + data_send[6] + data_send[7] + \
                    data_send[8] + data_send[9]))&0xff;
                                            /* 和校验 */

    sp.write(data_send,11);
}


void Arm::servo_set5(uint16_t servo_position[],uint16_t run_time)
{
    uint8_t data_send[33] = {0xff, 0xff};   /* 帧头 */

    data_send[2] = 0xfe;                    /* 广播ID */
    data_send[3] = 29;                      /* 数据长度 */               
    data_send[4] = 0x83;                    /* 指令类型：同步写 */

    data_send[5] = 0x2a;                    /* 参数组1 */
    data_send[6] = 0x04;

    data_send[7] = 0x01;                    /* 舵机1 */
    data_send[8] = (servo_position[0]>>8)&0xff;
    data_send[9] = servo_position[0]&0xff;
    data_send[10] = (run_time>>8)&0xff;
    data_send[11] = run_time&0xff;

    data_send[12] = 0x02;                   /* 舵机2 */
    data_send[13] = (servo_position[1]>>8)&0xff;
    data_send[14] = servo_position[1]&0xff;
    data_send[15] = data_send[10];
    data_send[16] = data_send[11];

    data_send[17] = 0x03;                   /* 舵机3 */
    data_send[18] = (servo_position[2]>>8)&0xff;
    data_send[19] = servo_position[2]&0xff;
    data_send[20] = data_send[10];
    data_send[21] = data_send[11];

    data_send[22] = 0x04;                   /* 舵机4 */
    data_send[23] = (servo_position[3]>>8)&0xff;
    data_send[24] = servo_position[3]&0xff;
    data_send[25] = data_send[10];
    data_send[26] = data_send[11];

    data_send[27] = 0x05;                   /* 舵机5 */
    data_send[28] = (servo_position[4]>>8)&0xff;
    data_send[29] = servo_position[4]&0xff;
    data_send[30] = data_send[10];
    data_send[31] = data_send[11];

    data_send[32] = 0x00;

    for(int i=2; i<32; i++)
    {
        data_send[32] += data_send[i];
    }
    data_send[32] = ~data_send[32];
    
    sp.write(data_send,33);
}



