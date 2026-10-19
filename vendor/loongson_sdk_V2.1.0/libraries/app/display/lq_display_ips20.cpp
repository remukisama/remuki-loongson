#include "lq_display_ips20.hpp"
#include "lq_assert.hpp"
#include "lq_common.hpp"

/* 屏幕初始化结构体 */
static spi_display_t ips20_spi;

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
 * @brief   IPS20 屏幕初始化
 * @param   type ： 0-->横屏  1-->竖屏
 * @return  无
 * @note    如果修改管脚 需要修改驱动模块初始化的管脚
 * @example lq_ips20_drv_init(1);
 ********************************************************************************/
void lq_ips20_drv_init(uint8_t type)
{
    if (ips20_spi.fb && ips20_spi.fb != MAP_FAILED)
    {
        munmap(ips20_spi.fb, IPS20_FB_SIZE);
        ips20_spi.fb = NULL;
    }
    if (ips20_spi.fd > 0)
    {
        close(ips20_spi.fd);
        ips20_spi.fd = -1;
    }
    // 打开屏幕设备文件
    ips20_spi.fd = open(IPS20_DEV_NAME, O_RDWR);
    if (ips20_spi.fd < 0)
    {
        lq_log_error("Open file %s error\n", IPS20_DEV_NAME);
    }
    // 映射屏幕帧缓冲
    ips20_spi.fb = (uint16_t*)mmap(NULL, IPS20_FB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, ips20_spi.fd, 0);
    if (ips20_spi.fb == MAP_FAILED)
    {
        lq_log_error("mmap failed");
        close(ips20_spi.fd);
        return;
    }
    // 选择横竖屏显示
    switch (type)
    {
        case 0 : ioctl(ips20_spi.fd, IOCTL_IPS20_L_INIT);   break;
        case 1 : ioctl(ips20_spi.fd, IOCTL_IPS20_V_INIT);   break;
        default: lq_log_error("lq_ips20_dri_init error\n"); break;
    }
    struct Pixel {
        uint16_t ips20_w;
        uint16_t ips20_h;
    };
    struct Pixel pix;
    ssize_t read_bytes = read(ips20_spi.fd, &pix, sizeof(pix));
    if (read_bytes != sizeof(pix))
    {
        lq_log_error("read failed\n");
        close(ips20_spi.fd);
        return;
    }
    ips20_spi.height = pix.ips20_h;
    ips20_spi.width  = pix.ips20_w;
    lq_log_info("IPS20_Init: type = %d, H = %d, W = %d\n", type, ips20_spi.height, ips20_spi.width);
}

/********************************************************************************
 * @brief   修改指定坐标的数据
 * @param   x     ：横坐标
 * @param   y     ：纵坐标
 * @param   color ：颜色
 * @return  无
 * @example lq_ips20_drv_data_mod(10, 20, U16YELLOW);
 ********************************************************************************/
static void lq_ips20_drv_data_mod(uint16_t x, uint16_t y, lq_display_color_t color)
{
    ips20_spi.fb[y * ips20_spi.width + x] = __builtin_bswap16(color);
}

/********************************************************************************
 * @brief   全屏显示单色画面
 * @param   color ：填充的颜色
 * @return  无
 * @example lq_ips20_drv_cls(U16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_cls(lq_display_color_t color_dat)
{
    memset16(ips20_spi.fb, color_dat, IPS20_SSIZE);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief   填充指定区域
 * @param   xs ：起始x
 * @param   ys ：起始y
 * @param   xe ：结束x
 * @param   ys ：结束y
 * @param   color ：填充的颜色
 * @return  无
 * @example lq_ips20_drv_fill_area(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_fill_area(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color_dat)
{
    uint16_t i, j;
    for (j = 0; j < (ye - ys + 1); j++)
        for (i = 0; i < (xe - xs + 1); i++)
            lq_ips20_drv_data_mod(xs + i, ys + j, color_dat);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief   画点
 * @param   x ：x
 * @param   y ：y
 * @param   color_dat ：颜色
 * @return  无
 * @example lq_ips20_drv_draw_dot(10, 20, U16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_draw_dot(uint16_t x, uint16_t y, lq_display_color_t color)
{
    lq_ips20_drv_data_mod(x, y, color);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief   画线
 * @param   xs ：起始x
 * @param   ys ：起始y
 * @param   xe ：结束x
 * @param   ys ：结束y
 * @param   color_dat ：颜色
 * @return  无
 * @example lq_ips20_drv_draw_line(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_draw_line(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color)
{
    int32_t i, ds;
    int32_t dx, dy, inc_x, inc_y;
    int32_t xerr = 0, yerr = 0;
    // 如果输入的坐标超出范围, 则不会画线
    if ((xs > ips20_spi.width - 1) || (ys > ips20_spi.height - 1))
        return;
    if ((xe > ips20_spi.width - 1) || (ye > ips20_spi.height - 1))
        return;
    if (xs == xe)
    {
        uint16_t y_min = ys, y_max = ye;
        if (y_min > y_max) { uint16_t t = y_min; y_min = y_max; y_max = t; }
        for (i = y_min; i <= y_max; i++)
            lq_ips20_drv_data_mod(xs, (uint16_t)i, color);
    }
    else if (ys == ye)
    {
        uint16_t x_min = xs, x_max = xe;
        if (x_min > x_max) { uint16_t t = x_min; x_min = x_max; x_max = t; }
        for (i = x_min; i <= x_max; i++)
            lq_ips20_drv_data_mod((uint16_t)i, ys, color);
    }
    else
    {
        int32_t x = xs;
        int32_t y = ys;
        dx = (int32_t)xe - (int32_t)xs;
        dy = (int32_t)ye - (int32_t)ys;

        if (dx > 0)
            inc_x = 1;
        else
        {
            inc_x = -1;
            dx = -dx;
        }
        if (dy > 0)
            inc_y = 1;
        else
        {
            inc_y = -1;
            dy = -dy;
        }
        if (dx > dy)
            ds = dx;
        else
            ds = dy;

        for (i = 0; i <= ds + 1; i++)
        {
            lq_ips20_drv_data_mod((uint16_t)x, (uint16_t)y, color);
            xerr += dx;
            yerr += dy;
            if (xerr > ds)
            {
                xerr -= ds;
                x += inc_x;
            }
            if (yerr > ds)
            {
                yerr -= ds;
                y += inc_y;
            }
        }
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    画矩形边框
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color_dat ：颜色
 * @return   无
 * @see      lq_ips20_drv_draw_rectangle(10, 20, 30, 40, U16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_draw_rectangle(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, lq_display_color_t color_dat)
{
    lq_ips20_drv_draw_line(xs, ys, xs, ye, color_dat);
    lq_ips20_drv_draw_line(xe, ys, xe, ye, color_dat);
    lq_ips20_drv_draw_line(xs, ys, xe, ys, color_dat);
    lq_ips20_drv_draw_line(xs, ye, xe, ye, color_dat);
}

/********************************************************************************
 * @brief   画圆
 * @param   x ：圆心x   (0-127)
 * @param   y ：圆心y   (0-159)
 * @param   r ：半径    (0-128)
 * @param   color_dat ：颜色
 * @return  无
 * @note    圆心坐标不要超出屏幕范围
 * @example lq_ips20_drv_draw_circle(50, 50, 30, u16YELLOW);
 ********************************************************************************/
void lq_ips20_drv_draw_circle(uint16_t x, uint16_t y, uint16_t r, lq_display_color_t color_dat)
{
    int16_t dx, dy = r;
    if ((x > ips20_spi.width - 1) || (y > ips20_spi.height - 1))
        return;

    for (dx = 0; dx <= r; dx++)
    {
        while ((r * r + 1 - dx * dx) < (dy * dy))
            dy--;
        if (is_value_in_range<uint16_t>(x + dx, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y - dy, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x + dx, y - dy, color_dat);
        if (is_value_in_range<uint16_t>(x - dx, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y - dy, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x - dx, y - dy, color_dat);
        if (is_value_in_range<uint16_t>(x - dx, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y + dy, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x - dx, y + dy, color_dat);
        if (is_value_in_range<uint16_t>(x + dx, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y + dy, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x + dx, y + dy, color_dat);

        if (is_value_in_range<uint16_t>(x + dy, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y - dx, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x + dy, y - dx, color_dat);
        if (is_value_in_range<uint16_t>(x - dy, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y - dx, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x - dy, y - dx, color_dat);
        if (is_value_in_range<uint16_t>(x - dy, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y + dx, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x - dy, y + dx, color_dat);
        if (is_value_in_range<uint16_t>(x + dy, 0, ips20_spi.width) && is_value_in_range<uint16_t>(y + dx, 0, ips20_spi.height))
            lq_ips20_drv_data_mod(x + dy, y + dx, color_dat);
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
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
static void lq_ips20_drv_p6x8(uint16_t x, uint16_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint16_t i, j;
    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                lq_ips20_drv_data_mod(x * 6 + i, y * 8 + j, word_color);
            else
                lq_ips20_drv_data_mod(x * 6 + i, y * 8 + j, back_color);
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
static void lq_ips20_drv_p8x8(uint16_t x, uint16_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint16_t i, j;
    for (j = 0; j < 8; j++)
    {
        lq_ips20_drv_data_mod(x * 8, y * 8 + j, back_color);
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                lq_ips20_drv_data_mod(x * 8 + i + 1, y * 8 + j, word_color);
            else
                lq_ips20_drv_data_mod(x * 8 + i + 1, y * 8 + j, back_color);
        }
        lq_ips20_drv_data_mod(x * 8 + 7, y * 8 + j, back_color);
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
static void lq_ips20_drv_p8x16(uint16_t x, uint16_t y, uint8_t c_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint16_t i, j;
    for (j = 0; j < 16; j++)
    {
        for (i = 0; i < 8; i++)
        {
            if ((Font_code16[c_dat - 32][j]) & (0x01 << i))
                lq_ips20_drv_data_mod(x * 8 + i, y * 16 + j, word_color);
            else
                lq_ips20_drv_data_mod(x * 8 + i, y * 16 + j, back_color);
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
 * @see      lq_ips20_drv_p6x8_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_ips20_drv_p6x8_str(uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_ips20_drv_p6x8(x++, y, *s_dat++, word_color, back_color);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    液晶字符串输出(8*8字体)
 * @param    x:0 - 15 (行)
 * @param    y:0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      lq_ips20_drv_p8x8_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_ips20_drv_p8x8_str(uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_ips20_drv_p8x8(x++, y, *s_dat++, word_color, back_color);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    液晶字符串输出(8*16字体)
 * @param    x: x: 0 -15   (行)
 * @param    y: y: 0 -9  	 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      lq_ips20_drv_p8x16_str(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_ips20_drv_p8x16_str(uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    while (*s_dat)
        lq_ips20_drv_p8x16(x++, y, *s_dat++, word_color, back_color);
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    液晶汉字字符串输出(16*16字体)
 * @param    x: 0 - 7	(行)
 * @param    y: 0 - 9	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     汉字只能是字库里的 字库没有的需要自行添加
 * @see      lq_ips20_drv_p16x16_cstr(1, 1, "123456", U16YELLOW, U16RED);
 ********************************************************************************/
void lq_ips20_drv_p16x16_cstr(uint16_t x, uint16_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color)
{
    uint16_t wm = 0, ii = 0, i, j;
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

        if (adder != 1) // 显示汉字s
        {
            for (j = 0; j < 32; j++)
            {
                for (i = 0; i < 8; i++)
                {
                    if ((hanzi16x16[adder]) & (0x80 >> i))
                    {
                        lq_ips20_drv_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), word_color);
                    }
                    else
                    {
                        lq_ips20_drv_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), back_color);
                    }
                }
                adder += 1;
            }
        }
        x  += 1;
        ii += 2;
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    IPS20屏 unsigned char 灰度数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_ips20_drv_road(uint16_t wide_start, uint16_t high_start, uint16_t high, uint16_t wide, uint8_t *Pixle)
{
    uint16_t i, j, color;
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            color  = (Pixle[j * wide + i] >> 3) << 11;
            color |= (Pixle[j * wide + i] >> 2) << 5;
            color |= (Pixle[j * wide + i] >> 3);
            lq_ips20_drv_data_mod(wide_start + i, high_start + j, (lq_display_color_t)color);
        }
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

/********************************************************************************
 * @brief    IPS20屏 unsigned char 二值化数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_ips20_drv_binRoad(uint16_t wide_start, uint16_t high_start, uint16_t high, uint16_t wide, uint8_t *Pixle)
{
    uint16_t i, j;
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            if (Pixle[j * wide + i])
                lq_ips20_drv_data_mod(wide_start + i, high_start + j, U16WHITE);
            else
                lq_ips20_drv_data_mod(wide_start + i, high_start + j, U16BLACK);
        }
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}

#ifdef LQ_HAVE_OPENCV
/********************************************************************************
 * @brief    IPS20屏彩色数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    img        ： 显示图像的Mat对象
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 ********************************************************************************/
void lq_ips20_drv_road_color(uint16_t wide_start, uint16_t high_start, const cv::Mat &img)
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
            lq_ips20_drv_data_mod(wide_start + x, high_start + y, (lq_display_color_t)color);
        }
    }
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}
#endif

/********************************************************************************
 * @brief    IPS20屏 刷新屏幕
 * @param    无
 * @return   无
 * @note     无
 ********************************************************************************/
void lq_ips20_drv_flush()
{
    ioctl(ips20_spi.fd, IOCTL_IPS20_FLUSH);
}
