#ifndef __LQ_I2C_RDWR_H__
#define __LQ_I2C_RDWR_H__ 

#include "lq_drv_common.h"

int i2c_read_regs       (struct ls_i2c_dev *dev, u8 reg, void *val, int len);   /* 从寄存器读取数据 */
u8  i2c_read_reg_byte   (struct ls_i2c_dev *dev, u8 reg);                       /* 使用 I2C 向寄存器读取一个字节 */

s32 i2c_write_regs      (struct ls_i2c_dev *dev, u8 reg, u8 *buf, u8 len);      /* 向寄存器写数据 */
s32 i2c_write_reg       (struct ls_i2c_dev *dev, u8 reg, u8 buf);               /* 向寄存器写入一个字节数据 */

#endif
