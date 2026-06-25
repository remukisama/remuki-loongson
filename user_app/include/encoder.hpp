#ifndef __ENCODER_H__
#define __ENCODER_H__

float encoder_num_L(void);
float encoder_num_R(void);
bool encoder_health_ok(void);
int encoder_invalid_count_L(void);
int encoder_invalid_count_R(void);
float encoder_raw_last_L(void);
float encoder_raw_last_R(void);

#endif // !__ENCODER_H__
