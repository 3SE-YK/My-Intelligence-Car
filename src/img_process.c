/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */

#include "img_process.h"
#include "string.h"
#include <stdlib.h>
uint8_t image[MT9V03X_H][MT9V03X_W];
uint8_t image_filter[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
int8_t number_left_line[MT9V03X_H] ;
int8_t number_right_line[MT9V03X_H] ;
/*
 *@func: 二值化函数
 */
void binaryzation_process(unsigned char *_img,
                          const unsigned short _rows,
                          const unsigned short _cols,
                          const unsigned int _threshold_value)
{
    unsigned short _size = _rows * _cols;
    for (unsigned short _i = 0; _i < _size; ++_i)
    {
        if (*(_img + _i) > _threshold_value)
            *(_img + _i) = 255;
        else
            *(_img + _i) = 0;
    }
    // 绘制边框，将图像边缘清零
    for (int i = 0; i < _cols; i++)
    {
        _img[i] = 0;
        _img[1 * _cols + i] = 0;
        _img[2 * _cols + i] = 0;
        _img[(_rows - 1) * _cols + i] = 0;
    }
    for (int i = 0; i < _rows; ++i)
    {
        _img[i * _cols + 0] = 0;
        _img[i * _cols + 1] = 0;
        _img[(i+1) * _cols - 1] = 0;
        _img[(i+1) * _cols - 2] = 0;
    }
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(unsigned char *_img,
                   const unsigned short _rows,
                   const unsigned short _cols,
                   short *_hist_gram)
{
    memset(_hist_gram, 0, 512);
    for (unsigned short _i = 0; _i < _rows; ++_i)
    {
        for (unsigned short _j = 0; _j < _cols; ++_j)
        {
            _hist_gram[_img[_i * _cols + _j]]++;
        }
    }
}

/*
 *@func: 大津法求阈值
 */
/* Otsu 阈值分割：输入 256 级灰度直方图，返回最优阈值。
 * 原理：最大化类间方差 sigma_B^2 = w0*w1*(u0-u1)^2，
 *       等价于最小化类内方差（总方差恒定）。
 */
unsigned char get_threshold_otsu(const short *hist)
{
    // 获取到的参数是一张灰度分布直方图
    if (hist == NULL)
        return 0;

    /* 1. 压缩有效灰度范围，跳过两端计数为 0 的灰度级 */
    int gmin = 0, gmax = 255;
    while (gmin < 256 && hist[gmin] == 0)
        gmin++; // 找到第一个不为零的灰度值
    while (gmax > gmin && hist[gmax] == 0)
        gmax--; // 找到最后一个不为零的灰度值

    if (gmin >= 256)
        return 0; /* 空直方图 */
    if (gmin == gmax)
        return (unsigned char)gmin; /* 单一灰度 */
    if (gmin + 1 == gmax)
        return (unsigned char)gmin; /* 两个灰度 */

    /* 2. 全图总量：像素总数与灰度加权和 */
    long total_count = 0; /* 像素总数 N       */
    long total_sum = 0;   /* 加权和 Σ i*h(i)  */
    for (int g = gmin; g <= gmax; g++)
    {
        total_count += hist[g]; // 因为实际上图像中每一个像素的灰度都算做一个，所以灰度直方图的纵坐标的求和便可以表示像素总量
        total_sum += (long)hist[g] * g;
    }

    /* 3. 遍历阈值，用前缀和增量维护"背景"统计，找最大类间方差。
     *    这里数学公式做了一点点修改，具体看代码 */
    long back_count = 0, back_sum = 0;
    double best_sigma = -1.0;
    int threshold = gmin;

    for (int g = gmin; g < gmax; g++)
    {
        back_count += hist[g];
        back_sum += (long)hist[g] * g;
        if (back_count == 0) /* 防御：背景为空时该阈值无意义 */
            continue;

        long fore_count = total_count - back_count;
        if (fore_count == 0) /* 防御：前景为空 */
            break;

        long fore_sum = total_sum - back_sum;

        /* 数学公式 */
        double sigma = (double)back_sum * back_sum / back_count + (double)fore_sum * fore_sum / fore_count;

        if (sigma > best_sigma)
        {
            best_sigma = sigma;
            threshold = g;
        }
    }
    return (unsigned char)threshold;
}

// 寻线函数
void auxiliary_process(const uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line)
{

    for (int i = 0; i < MT9V03X_H; ++i)
    {
        _left_line[i] = 0;
        _right_line[i] = _src_cols - 1;
    }
    _left_line[_src_rows - 2] = 0;
    _right_line[_src_rows - 2] = _src_cols - 1;
    // 寻找底部的中点
    for (int i = 2; i <= _src_cols/2; ++i)
    {
        // 寻左线
        if (_src_pixel_mat[(_src_rows - 2) * _src_cols + i] == 0 &&
            _src_pixel_mat[(_src_rows - 2) * _src_cols + i - 1] == 0 &&
            _src_pixel_mat[(_src_rows - 2) * _src_cols + i + 1] == 255)
        {
            _left_line[_src_rows - 2] = i;
            break;
        }
    }
    for (int i = _src_cols -3; i > _src_cols/2 + 1; --i)
    {
        // 寻右线
        if (_src_pixel_mat[(_src_rows - 2) * _src_cols + i] == 0 &&
            _src_pixel_mat[(_src_rows - 2) * _src_cols + i - 1] == 255 &&
            _src_pixel_mat[(_src_rows - 2) * _src_cols + i + 1] == 0)
        {
            _right_line[_src_rows - 2] = i;
            break;
        }
    }
    for (int i = 0; i < MT9V03X_H; ++i)
    {
        number_left_line[i] = 0;
        number_right_line[i] = 0;
    }
    // 八邻域，自下而上寻找边线
    //寻左线
    int i = _src_rows - 2;
    while (i > 2 )
    {
        if(_left_line[i] + number_left_line[i] < 2)
        {
            _left_line[i-1] = 2;
            --i;
            continue;
        }
        else if(_left_line[i] + number_left_line[i] > _src_cols - 3)
        {
            _left_line[i-1] = _src_cols - 3;
            --i;
            continue;
        }
        // 先判断上一行是否存在跳变点
        int point_wite = 0;
        bool is_find = 0;
        for (int j = -1; j < 2; ++j)
        {
            if (_src_pixel_mat[(i - 1) * _src_cols + _left_line[i] + number_left_line[i] + j] == 255)
            {
                point_wite++;
            }
            else
            {
                if(_src_pixel_mat[(i - 1) * _src_cols + _left_line[i] + number_left_line[i] + j + 1] == 255)
                {
                    _left_line[i-1] = _left_line[i] + number_left_line[i] + j ;
                    --i;
                    is_find = 1;
                    break;
                }
            }

        }
        if(is_find) continue;
        if (point_wite == 0 || point_wite == 1)
        {
            number_left_line[i]++;
        }
        else if (point_wite == 3 || point_wite == 2)
        {
            number_left_line[i]--;
        }
 
    }
    //寻右线
    i = _src_rows - 2;
    while (i > 2 )
    {
        if(_right_line[i] + number_right_line[i] < 2)
        {
            _right_line[i-1] = 2;
            --i;
            continue;
        }
        else if(_right_line[i] + number_right_line[i] > _src_cols - 3)
        {
            _right_line[i-1] = _src_cols - 3;
            --i;
            continue;
        }
        bool is_find = 0;
        // 先判断上一行是否存在跳变点
        int point_wite = 0;
        for (int j = 1; j > -2; --j)
        {
            if (_src_pixel_mat[(i - 1) * _src_cols + _right_line[i] + number_right_line[i] + j] == 0)
            {
                if(_src_pixel_mat[(i - 1) * _src_cols + _right_line[i] + number_right_line[i] + j - 1] == 255)
                {
                    _right_line[i-1] =  _right_line[i] + number_right_line[i] + j;
                    --i;
                    is_find = 1;
                    break;
                }
            }
            else
            {
                point_wite++;
            }
        }
        if(is_find)continue;
        if ( point_wite == 0||point_wite == 1)
        {
            number_right_line[i]--;
        }
        else if (point_wite == 3 ||point_wite == 2)
        {
            number_right_line[i]++;
        }
    }
    //绘制中线
    for(int i = _src_rows - 2 ; i > 3; --i)
    {
        _mid_line[i] = (_left_line[i] + _right_line[i])/2; 
    }
}

/* 形态学滤波：3x3 结构元
 * 约定：一个点周围八个邻域，白色的点超过5，这个点设置为白色，否则为黑色
*/
void filter(uint8_t *_src_pixel_mat,uint8_t *_src_filter_mat, uint8_t _src_rows, uint8_t _src_cols)
{

    for(int i = 1 ; i < _src_rows - 1; ++i)
    {
        for(int j = 1;j<_src_cols - 1;++j)
        {
            short num_wite = 0;
            for(int k = -1 ; k < 2 ; ++k)
            {
                num_wite += _src_pixel_mat[i * _src_cols + j +k] / 255;
                num_wite += _src_pixel_mat[(i-1) * _src_cols + j +k] / 255;
                num_wite += _src_pixel_mat[(i+1) * _src_cols + j +k] / 255;
            }
            if(num_wite > 5)
            {
                _src_filter_mat[i * _src_cols + j] = 255;
            }
            else{
                _src_filter_mat[i * _src_cols + j] = 0;
            }
        }
    }
}
//绘制边线
void paint_line()
{
    for(int i = MT9V03X_H - 1 ; i > 2 ; --i)
    {
        
        int j = MIN(0,number_left_line[i]);
        int k = j;
        while(j <= MAX(0,number_left_line[i]) )
        {
            if((j - k)>5) break;
            tft180_draw_point((left_line[i] + j)*0.75,i,RGB565_RED);
            tft180_draw_point((left_line[i] + j)*0.75 + 2,i,RGB565_RED);
            ++j;
        }
        j = MIN(0,number_right_line[i]);
        k = j;
        while(j <= MAX(0,number_right_line[i]) )
        {
            if((j - k)>5) break;
            tft180_draw_point((right_line[i] + j)*0.75,i,RGB565_BLUE);
            tft180_draw_point((right_line[i] + j)*0.75 - 2,i,RGB565_BLUE);
            ++j;
        }
        tft180_draw_point(mid_line[i]*0.75,i,RGB565_GREEN);

    }
    
}

