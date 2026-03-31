//
// Created by CORE on 2026/3/14.
//

#include "Bsp/Inc/bsp_can.h"
#include "Config/can_id.h"

#define LOGIC_PITCH  (CAN_STDID_GIMBAL_PITCH_FB - 0x200)
#define LOGIC_YAW  (CAN_STDID_GIMBAL_YAW_FB - 0x200)

void devices_global_ctrl(int16_t yaw, int16_t pitch)
{
    int16_t motor_currents[8] = {0};
    motor_currents[LOGIC_YAW] = yaw;
    motor_currents[LOGIC_PITCH] = pitch;
    bsp_ctrl_motor(motor_currents);
}

void devices_general_ctrl(uint16_t txid, int16_t current, uint8_t slot)
{
    switch (slot)
    {
    case 1:
            dji_motor_tx(txid, current, 0, 0, 0);
            break;
    case 2:
            dji_motor_tx(txid, 0, current, 0, 0);
            break;
    case 3:
            dji_motor_tx(txid, 0, 0, current, 0);
            break;
    case 4:
            dji_motor_tx(txid, 0, 0, 0, current);
            break;
    }

}
