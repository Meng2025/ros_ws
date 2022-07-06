#ifndef __SERIAL_COMMU_H
#define __SERIAL_COMMU_H

#include <serial/serial.h>


class Arm {

public :
    Arm(void);
    ~Arm(void);

public : /* 机械臂相关函数 */
    void set(uint8_t id, float degree);
    void set(float degree_list[]);
    void get(float degree_list[]);
    void get(uint8_t id);
    void gripper(float degree);
    void toque(bool status);

public : /* 关节舵机相关函数 */
    uint16_t servo_get1(uint8_t id);
    void servo_get5(uint16_t position[]);
    void servo_set1(uint8_t id, uint16_t position, uint16_t run_time);
    void servo_set5(uint16_t servo_position[],uint16_t run_time);


    uint16_t servo_position[5];

    float joint[5];

    
private :           /* 机械臂私有控制 */
    serial::Serial sp;
    
};




#endif

