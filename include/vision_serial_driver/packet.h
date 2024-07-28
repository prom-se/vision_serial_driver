#ifndef PACKET_H
#define PACKET_H

#include <cstdint>

#pragma pack(2)

namespace vision
{
    struct VisionMsg
    {
        uint8_t head;
        uint8_t fire;    // 开火标志
        float aim_yaw;   // 目标Yaw
        float aim_pitch; // 目标Pitch
    };

    struct RobotMsg
    {
        uint8_t head;
        uint8_t foe_color;  // 敌方颜色 0-blue 1-red
        float robot_yaw;    // 自身Yaw
        float robot_pitch;  // 自身Pitch
        float muzzle_speed; // 弹速
    };

    union VisionPack
    {
        VisionMsg msg;
        uint8_t bytes[sizeof(VisionMsg)];
    };

    union RobotPack
    {
        RobotMsg msg;
        uint8_t bytes[sizeof(RobotMsg)];
    };
}
#pragma pack()

#endif // PACKET_H