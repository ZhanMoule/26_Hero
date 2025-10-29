/**
*   @file alg_crc.h
*   @brief 
*   @author Wenxin HU
*   @date 25-7-13
*   @version 0.5
*   @note
*
*   @TODO 目前只提供了CRC-CCITT的计算函数,剩余的CRC8和CRC16
*/
#ifndef ALG_CRC_H
#define ALG_CRC_H

#include <stdint.h>
#include <stdlib.h>

/**
 * @brief 用于计算缓冲区内数据的CRC-CCITT校验和
 * @param crc CRC初始值
 * @param buffer 缓冲区指针
 * @param len 缓冲区长度
 * @return CRC校验和
 */
uint16_t Crc_Ccitt_Calculate(uint16_t crc, uint8_t const *buffer, size_t len);

#endif //ALG_CRC_H
