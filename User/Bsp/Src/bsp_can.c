//include...


#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"


/**
 * @brief       初始化can接受相关配置
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @return true 初始化成功
 * @return false 失败
 */
bool bsp_can_init(void)
{

    //TODO:
    //这里放一些初始化的代码，配置can外设
}

/**
 * @brief       can发送函数，一个原始的封装，目前只支持标准帧
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 要发送的canid
 * @param data  需要发送的数据
 * @param dlc   dlc长度
 * @return true  发送成功
 * @return false 发送失败
 */
bool bsp_tx(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    //TODO:
    //这里需要写hal库的发送函数
}

/**
 * @brief       对外的can读取函数
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 
 * @param frame 
 * @return true 
 * @return false 
 */
bool bsp_can_rx(uint16_t can_id, CanRxFrame *frame)
{

}
/**
 * @brief       CAN中断函数，发送中断后会调用该函数
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 */
static void can_ISR(void)
{
    
}

/**
 * @brief       检查指定id通讯是否正常，超时1000ms返回false
 * 
 * @date        2026-04-09
 * @author      Rui.
 * 
 * @param can_id 
 * @return true 
 * @return false 
 */
bool can_check(uint16_t can_id)
{
    //TODO:
    //这里写检测通讯的函数    
}

//TODO:
//这里补充can的中断函数定义，直接在此处定义，覆盖弱定义