#ifndef __IMAGE_CV_H__
#define __IMAGE_CV_H__



extern char img[60][80];
extern char img3[60][80];
extern char img4[60][160];
extern char img5[120][160];

bool image_cv_Init(void);
bool image_cv_IsReady(void);
bool image_cv_zip(void);
void image_cv_Deinit(void);

#endif // !__
