// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
/*
  Size of the total logged dataset in bits:
 | PACKET # | TIME | FSR_HEEL | FSR_TOE | IMU_1 | IMU_2 | IMU_3 |
 |    32    |  32  |    16    |    16   |  240  |  240  |  240  |
 = 816 bits = 102 Bytes
 Note:
 - Should pad until 128 Bytes for best logging performance.
*/
#ifndef ExFatLogger_h
#define ExFatLogger_h

#include "MLX90393.h" //From https://github.com/tesshellebrekers/arduino-MLX90393 forked from Theodore Yapo

MLX90393 mlx0;
MLX90393 mlx1;
MLX90393 mlx2;
MLX90393 mlx3;

uint8_t mlx0_i2c = 0x0C;
uint8_t mlx1_i2c = 0x0D;
uint8_t mlx2_i2c = 0x0E;
uint8_t mlx3_i2c = 0x0F;

// Collection of data custom for application
// Note: delta is NOT part of data_t, it's computed during conversion based on "t"
struct data_t
{

  //uint32_t utime;
  uint32_t t;
  //uint32_t dt;
  uint32_t rtcyear;
  uint32_t rtcmonth;
  uint32_t rtcday;
  uint32_t rtchour;
  uint32_t rtcminute;
  uint32_t rtcsecond;

//  MLX90393::txyz data[24];
  
  MLX90393::txyz data0; //four x floats (uint32_t)=4 bytes = 16 bytes total
  MLX90393::txyz data1; 
  MLX90393::txyz data2; 
  MLX90393::txyz data3; 

  MLX90393::txyz data5;
  MLX90393::txyz data6;
  MLX90393::txyz data7;
  MLX90393::txyz data8;

  MLX90393::txyz data10;
  MLX90393::txyz data11;
  MLX90393::txyz data12;
  MLX90393::txyz data13;

  MLX90393::txyz data15;
  MLX90393::txyz data16;
  MLX90393::txyz data17;
  MLX90393::txyz data18;

  MLX90393::txyz data20;
  MLX90393::txyz data21;
  MLX90393::txyz data22;
  MLX90393::txyz data23;

  
  // Variable to fill in the transfer to 512 Bytes
  // (7*4+20*4*4) = 348 bytes. (512-348)/2=82
  uint16_t whitespace[82];
};
#endif // ExFatLogger_h
