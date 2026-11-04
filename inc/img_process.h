#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

extern uint8_t image[MT9V03X_H][MT9V03X_W];
extern uint8_t image_filter[MT9V03X_H][MT9V03X_W];
extern short hist_gram[256];
extern uint8_t left_line[MT9V03X_H];
extern uint8_t mid_line[MT9V03X_H];
extern uint8_t right_line[MT9V03X_H];
extern int8_t number_left_line[MT9V03X_H] ;
extern int8_t number_right_line[MT9V03X_H] ;
void get_hist_gram(unsigned char *_img,
				   const unsigned short _rows,
				   const unsigned short _cols,
				   short *_hist_gram);
unsigned char get_threshold_otsu(const short *_hist_gram);
void binaryzation_process(unsigned char *_img,
						  const unsigned short _rows,
						  const unsigned short _cols,
						  const unsigned int _threshold_value);
void filter(uint8_t *_src_pixel_mat,uint8_t *_src_filter_mat, uint8_t _src_rows, uint8_t _src_cols);

void paint_line();
void auxiliary_process(const uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
					   unsigned char _threshold_val,
					   uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line);

#endif
