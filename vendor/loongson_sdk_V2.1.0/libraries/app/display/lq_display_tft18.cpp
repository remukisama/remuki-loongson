#include "lq_display_tft18.hpp"
#include "lq_assert.hpp"
#include "lq_common.hpp"

/* 屏幕初始化结构体 */
static spi_display_t tft18_spi;

/********************************************************************************
 * @brief   批量数据赋值.
 * @param   s     : 需要赋值的空间头指针.
 * @param   c     : 想要赋值的值(uint16_t类型).
 * @param   count : 想要赋值的数据量.
 * @return  无
 * @example memset16(buf, U16RED, 10);
 * @note    none
 ********************************************************************************/
static void memset16(void *s, uint16_t c, size_t count)
{
    uint16_t *p = (uint16_t*)s;
    for (size_t i = 0; i < count; i++)
    {
        p[i] = __builtin_bswap16(c);
    }
}

/********************************************************************************
 * @brief   TFT18 屏幕初始化
 * @param   type ： 0-->横屏  1-->竖屏
 * @return  无
 * @note    如果修改管脚 需要修改驱动模块初始化的管脚
 * @example lq_tft18_drv_init(1);
 ********************************************************************************/
void lq_tft18_drv_init(uint8_t type)
{
    if (tft18_spi.fb && tft18_spi.fb!= MAP_FAILED)
    {
        munmap(tft18_spi.fb, TFT18_FB_SIZE);
        tft18_spi.fb = NULL;
    }
    if (tft18_spi.fd > 0)
    {
        close(tft18_spi.fd);
        tft18_spi.fd = -1;
    }
    // 获取屏幕文件描述符
    tft18_spi.fd = open(TFT18_DEV_NAME, O_RDWR);
    if (tft18_spi.fd < 0)
    {
        lq_log_error("Open file %s error\n", TFT18_DEV_NAME);
    }
    // 映射屏幕缓冲区
    tft18_spi.fb = (uint16_t*)mmap(NULL, TFT18_FB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, tft18_spi.fd, 0);
    if (tft18_spi.fb == MAP_FAILED)
    {
        lq_log_error("mmap failed\n");
        close(tft18_spi.fd);
        return;
    }
    // 选择横竖屏显示
    switch (type)
    {
        case 0 : ioctl(tft18_spi.fd, IOCTL_TFT18_L_INIT);   break;
        case 1 : ioctl(tft18_spi.fd, IOCTL_TFT18_V_INIT);   break;
        default: lq_log_error("lq_tft18_dri_init error\n"); break;
    }
    struct Pixel {
        uint8_t tft18_w;
        uint8_t tft18_h; 
    };
    struct Pixel pix;
    ssize_t read_bytes = read(tft18_spi.fd, &pix, sizeof(pix));
    if (read_bytes != sizeof(pix))
    {
        lq_log_error("read failed\n");
        close(tft18_spi.fd);
        return;
    }
    tft18_spi.height = pix.tft18_h;
    tft18_spi.width  = pix.tft18_w;
    // lq_log_info("TFT18_Init: type = %d, H = %d, W = %d\n", type, tft18_spi.height, tft18_spi.width);
}
                                                                                                                                                                                                                 
/********************************************************************************
 * @brief   修改指定坐标的数据
 * @param   x     ：横坐标
 * @param   y     ：纵坐标
 * @param   color ：颜色
 * @return  无
 * @note    起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @example lq_tft18_drv_data_mod(10, 20, U16YELLOW);
 ********************************************************************************/
static void lq_tft18_drv_data_mod(uint8_t x, uint8_t y, lq_display_color_t color)
{
    tft18_spi.fb[y * tft18_spi.width + x] = __builtin_bswap16(color);
}

/********************************************************************************
 * @brief   全屏显示单色画面
 * @param   color ：填充的颜色
 * @return  无
 * @note    起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @example lq_tft18_drv_cls(U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_cls(lq_display_color_t color_dat)
{
    memset16(tft18_spi.fb, color_dat, TFT18_SSIZE);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief   填充指定区域
 * @param   xs ：起始x
 * @param   ys ：起始y
 * @param   xe ：结束x
 * @param   ys ：结束y
 * @param   color ：填充的颜色
 * @return  无
 * @note    起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @example lq_tft18_drv_fill_area(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_fill_area(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color_dat)
{
    uint8_t i, j;
    for (j = 0; j < (ye - ys + 1); j++)
        for (i = 0; i < (xe - xs + 1); i++)
            lq_tft18_drv_data_mod(xs + i, ys + j, color_dat);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief   画点
 * @param   x ：x
 * @param   y ：y
 * @param   color_dat ：颜色
 * @return  无
 * @note    起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @example lq_tft18_drv_draw_dot(10, 20, U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_draw_dot(uint8_t x, uint8_t y, lq_display_color_t color)
{
    lq_tft18_drv_data_mod(x, y, color);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief   画线
 * @param   xs ：起始x
 * @param   ys ：起始y
 * @param   xe ：结束x
 * @param   ys ：结束y
 * @param   color_dat ：颜色
 * @return  无
 * @note    起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @example lq_tft18_drv_draw_line(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_draw_line(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color)
{
    int i, ds;
    int dx, dy, inc_x, inc_y;
    int xerr = 0, yerr = 0; // 初始化变量
    // 如果输入的坐标超出范围，则不会画线
    if ((xs > tft18_spi.width - 1) || (ys < 0) || (ys > tft18_spi.height - 1))
        return;
    if ((xe > tft18_spi.width - 1) || (ye < 0) || (ye > tft18_spi.height - 1))
        return;
    if (xs == xe)   // 如果是画直线则只需要对竖直坐标计数
    {
        for (i = 0; i < (ye - ys + 1); i++)
            lq_tft18_drv_data_mod(xs, ys + i, color);
    }
    else if (ys == ye)  // 如果是水平线则只需要对水平坐标计数
    {
        for (i = 0; i < (xe - xs + 1); i++)
            lq_tft18_drv_data_mod(xs + i, ys, color);
    }
    else    // 如果是斜线，则重新计算，使用画点函数画出直线
    {
        dx = xe - xs;   // 计算坐标增量
        dy = ye - ys;
        if (dx > 0)
            inc_x = 1;  // 设置单步方向
        else
        {
            inc_x = -1;
            dx = -dx;
        }
        if (dy > 0)
            inc_y = 1;  // 设置单步方向
        else
        {
            inc_y = -1;
            dy = -dy;
        }
        if (dx > dy)
            ds = dx;    // 选取基本增量坐标值
        else
            ds = dy;
        for (i = 0; i <= ds + 1; i++)   // 画线输出
        {
            lq_tft18_drv_data_mod(xs, ys, color);
            xerr += dx;
            yerr += dy;
            if (xerr > ds)
            {
                xerr -= ds;
                xs += inc_x;
            }
            if (yerr > ds)
            {
                yerr -= ds;
                ys += inc_y;
            }
        }
    }
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    画矩形边框
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color_dat ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      lq_tft18_drv_draw_rectangle(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_draw_rectangle(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color_dat)
{
    lq_tft18_drv_draw_line(xs, ys, xs, ye, color_dat);    // 画矩形左边
    lq_tft18_drv_draw_line(xe, ys, xe, ye, color_dat);    // 画矩形右边
    lq_tft18_drv_draw_line(xs, ys, xe, ys, color_dat);    // 画矩形上边
    lq_tft18_drv_draw_line(xs, ye, xe, ye, color_dat);    // 画矩形下边
}

/********************************************************************************
 * @brief   画圆
 * @param   x ：圆心x   (0-127)
 * @param   y ：圆心y   (0-159)
 * @param   r ：半径    (0-128)
 * @param   color_dat ：颜色
 * @return  无
 * @note    圆心坐标不要超出屏幕范围
 * @example lq_tft18_drv_draw_circle(50, 50, 30, U16YELLOW);
 ********************************************************************************/
void lq_tft18_drv_draw_circle(uint8_t x, uint8_t y, uint8_t r, lq_display_color_t color_dat)
{
    uint8_t dx, dy = r;
    if ((x > tft18_spi.width - 1) || (y > tft18_spi.height - 1))
        return;
    for (dx = 0; dx <= r; dx++)
    {
        while ((r * r + 1 - dx * dx) < (dy * dy))
            dy--;
        if (is_value_in_range<uint16_t>(x + dx, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y - dy, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x + dx, y - dy, color_dat);
        if (is_value_in_range<uint16_t>(x - dx, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y - dy, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x - dx, y - dy, color_dat);
        if (is_value_in_range<uint16_t>(x - dx, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y + dy, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x - dx, y + dy, color_dat);
        if (is_value_in_range<uint16_t>(x + dx, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y + dy, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x + dx, y + dy, color_dat);
        
        if (is_value_in_range<uint16_t>(x + dy, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y - dx, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x + dy, y - dx, color_dat);
        if (is_value_in_range<uint16_t>(x - dy, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y - dx, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x - dy, y - dx, color_dat);
        if (is_value_in_range<uint16_t>(x - dy, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y + dx, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x - dy, y + dx, color_dat);
        if (is_value_in_range<uint16_t>(x + dy, 0, tft18_spi.width) && is_value_in_range<uint16_t>(y + dx, 0, tft18_spi.height))
            lq_tft18_drv_data_mod(x + dy, y + dx, color_dat);
    }
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief   液晶字符输出(6*8字体)
 * @param   x: 0 - 20	(行)
 * @param   y: 0 - 19	(列)
 * @param   word_color: 字体颜色
 * @param   back_color: 背景颜色
 * @return  无
 * @note    内部调用
 ********************************************************************************/
static void lq_tft18_drv_p6x8(uint8_t x, uint8_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                lq_tft18_drv_data_mod(x * 6 + i, y * 8 + j, word_color);
            else
                lq_tft18_drv_data_mod(x * 6 + i, y * 8 + j, back_color);
        }
    }
}

/********************************************************************************
 * @brief   液晶字符输出(8*8字体)
 * @param   x:0 - 15	(行)
 * @param   y:0 - 19	(列)
 * @param   word_color: 字体颜色
 * @param   back_color: 背景颜色
 * @return  无
 * @note    内部调用
 ********************************************************************************/
static void lq_tft18_drv_p8x8(uint8_t x, uint8_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        lq_tft18_drv_data_mod(x * 8, y * 8 + j, back_color);
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                lq_tft18_drv_data_mod(x * 8 + i + 1, y * 8 + j, word_color);
            else
                lq_tft18_drv_data_mod(x * 8 + i + 1, y * 8 + j, back_color);
        }
        lq_tft18_drv_data_mod(x * 8 + 7, y * 8 + j, back_color);
    }
}

/********************************************************************************
 * @brief   液晶字符输出(8*16字体)
 * @param   x: 0 -15   (行)
 * @param   y: 0 -9  	 (列)
 * @param   word_color: 字体颜色
 * @param   back_color: 背景颜色
 * @return  无
 * @note    内部调用
 ********************************************************************************/
static void lq_tft18_drv_p8x16(uint8_t x, uint8_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 16; j++)
    {
        for (i = 0; i < 8; i++)
        {
            if ((Font_code16[c_dat - 32][j]) & (0x01 << i))
                lq_tft18_drv_data_mod(x * 8 + i, y * 16 + j, word_color);
            else
                lq_tft18_drv_data_mod(x * 8 + i, y * 16 + j, back_color);
        }
    }
}

/********************************************************************************
 * @brief    液晶字符串输出(6*8字体)
 * @param    x: 0 - 20 (行)
 * @param    y: 0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      lq_tft18_drv_p6x8_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_tft18_drv_p6x8_str(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_tft18_drv_p6x8(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    液晶字符串输出(8*8字体)
 * @param    x:0 - 15 (行)
 * @param    y:0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      lq_tft18_drv_p8x8_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_tft18_drv_p8x8_str(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_tft18_drv_p8x8(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    液晶字符串输出(8*16字体)
 * @param    x: x: 0 -15   (行)
 * @param    y: y: 0 -9  	 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      lq_tft18_drv_p8x16_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_tft18_drv_p8x16_str(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_tft18_drv_p8x16(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    液晶汉字字符串输出(16*16字体)
 * @param    x: 0 - 7	(行)
 * @param    y: 0 - 9	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     汉字只能是字库里的 字库没有的需要自行添加
 * @see      lq_tft18_drv_p16x16_cstr(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_tft18_drv_p16x16_cstr(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint8_t wm = 0, ii = 0, i, j;
    int adder = 1;
    while (s_dat[ii] != '\0')
    {
        wm = 0;
        adder = 1;
        while (hanzi_Idx[wm] > 127)
        {
            if (hanzi_Idx[wm] == (uint8_t)s_dat[ii])
            {
                if (hanzi_Idx[wm + 1] == s_dat[ii + 1])
                {
                    adder = wm * 16;
                    break;
                }
            }
            wm += 2;
        }

        if (adder != 1) // 显示汉字
        {
            for (j = 0; j < 32; j++)
            {
                for (i = 0; i < 8; i++)
                {
                    if ((hanzi16x16[adder]) & (0x80 >> i))
                    {
                        lq_tft18_drv_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), word_color);
                    }
                    else
                    {
                        lq_tft18_drv_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), back_color);
                    }
                }
                adder += 1;
            }
        } else {}   // 显示空白字符
        x  += 1;    // 上下方向
        ii += 2;
    }
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    TFT18屏 unsigned char 灰度数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_tft18_drv_road(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle)
{
    uint64_t i, j;
    uint16_t color;
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            /* 将灰度转化为 RGB565 */
            color = (Pixle[j * wide + i] >> 3) << 11;
            color |= (Pixle[j * wide + i] >> 2) << 5;
            color |= Pixle[j * wide + i] >> 3;
            lq_tft18_drv_data_mod(wide_start + i, high_start + j, (lq_display_color_t)color);
        }
    }
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}

/********************************************************************************
 * @brief    TFT18屏 unsigned char 二值化数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_tft18_drv_binroad(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle)
{
    uint8_t i, j;
    /* 显示图像 */
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            if (Pixle[j * wide + i])
                lq_tft18_drv_data_mod(wide_start + i, high_start + j, U16WHITE); /* 显示 */
            else
                lq_tft18_drv_data_mod(wide_start + i, high_start + j, U16BLACK);
        }
    }
    lq_tft18_drv_flush();
}

#ifdef LQ_HAVE_OPENCV
/********************************************************************************
 * @brief    TFT18屏彩色数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    img        ： 显示图像的Mat对象
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_tft18_drv_road_color(uint8_t wide_start, uint8_t high_start, const cv::Mat &img)
{
    if (img.empty() || img.type() != CV_8UC3) {
        lq_log_error("图像数据为空或格式不是彩色图像\n");
        return;
    }
    uint16_t img_high = img.rows;
    uint16_t img_wide = img.cols;
    uint16_t x, y;
    uint16_t color;

    for (y = 0; y < img_high; y++) {
        const uint8_t *row_ptr = img.ptr<uint8_t>(y);
        for (x = 0; x < img_wide; x++) {
            uint8_t r = row_ptr[x * 3 + 2]; // OpenCV默认BGR格式
            uint8_t g = row_ptr[x * 3 + 1];
            uint8_t b = row_ptr[x * 3 + 0];
            color = (((r >> 3) & 0x1F) << 11) | (((g >> 2) & 0x3F) << 5) | ((b >> 3) & 0x1F);
            lq_tft18_drv_data_mod(wide_start + x, high_start + y, (lq_display_color_t)color);
        }
    }
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}
#endif

/********************************************************************************
 * @brief    TFT18屏 刷新屏幕
 * @param    无
 * @return   无
 * @note     无
 ********************************************************************************/
void lq_tft18_drv_flush()
{
    ioctl(tft18_spi.fd, IOCTL_TFT18_FLUSH);
}
