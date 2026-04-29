#ifndef __KEY_H__
#define __KEY_H__

extern char image_show_flag;
extern char car_start;

// 函数原型声明
void emergency_stop();          // 电机紧急停止
void signal_handler(int signum); // 信号处理回调

void Key_Get(void);
void Key_KZ(void);
void image_show(void);
#endif // !__KEY_H__