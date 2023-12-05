//
// Created by promise on 11/3/23.
//

#ifndef SERIAL_PACKET_H
#define SERIAL_PACKET_H

#include <cstdint>
#include <vector>
#include <algorithm>

struct visionMsg{
    float aimYaw;               // 目标Yaw
    float aimPitch;             // 目标Pitch
};

struct robotMsg{
    uint8_t mode;               // 自瞄模式 0-装甲板 1-小符 2-大符
    uint8_t foeColor;           // 敌方颜色 0-blue 1-red
    float robotYaw;             // 自身Yaw
    float robotPitch;           // 自身Pitch
    float muzzleSpeed;          // 弹速
};

union visionArray{
    struct visionMsg msg;
    uint8_t array[sizeof(struct visionMsg)];
};

union robotArray{
    struct robotMsg msg;
    uint8_t array[sizeof(struct robotMsg)];
};


#endif //SERIAL_PACKET_H