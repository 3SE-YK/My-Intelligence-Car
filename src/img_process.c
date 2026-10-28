/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */

#include "img_process.h"
#include "string.h"
#include <stdlib.h>

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
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(unsigned char *_img,
                   const unsigned short _rows,
                   const unsigned short _cols,
                   short *_hist_gram)
{
    memset(_hist_gram, 0, 256);
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

uint8_t line_valid[MT9V03X_H];      // 1 = 本行中线可信，0 = 不可信（元素/丢线）

void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line)
{
    const uint8_t cols = _src_cols, rows = _src_rows;
    uint8_t mid = cols >> 1;

    /* 1. 底行中心压黑线：挪到最近的白底（找到即停） */
    const uint8_t *bottom = _src_pixel_mat + (rows - 1) * cols;
    if (bottom[mid] < _threshold_val) {
        int16_t cr = -1, cl = -1;
        for (int16_t j = mid; j < cols - 2; j++)
            if (bottom[j] >= _threshold_val && bottom[j+1] >= _threshold_val && bottom[j+2] >= _threshold_val) { cr = j; break; }
        for (int16_t j = mid; j >= 2; j--)
            if (bottom[j] >= _threshold_val && bottom[j-1] >= _threshold_val && bottom[j-2] >= _threshold_val) { cl = j; break; }
        if (cr >= 0 && cl >= 0) mid = (cr - mid < mid - cl) ? cr : cl;
        else                    mid = (cr >= 0) ? cr : (cl >= 0) ? cl : mid;
    }

    /* 2. 逐行扫线 */
    uint8_t width = 0;                          // 最近一次双边命中的赛道宽度
    for (int16_t i = rows - 1; i >= 0; i--) {
        const uint8_t *row = _src_pixel_mat + i * cols;
        int16_t l = -1, r = -1;

        for (int16_t c = mid; c >= 2; c--)                    // 向左找三连暗
            if (row[c] < _threshold_val && row[c-1] < _threshold_val && row[c-2] < _threshold_val) { l = c; break; }
        for (int16_t c = mid; c <= cols - 3; c++)             // 向右找三连暗
            if (row[c] < _threshold_val && row[c+1] < _threshold_val && row[c+2] < _threshold_val) { r = c; break; }

        if (l >= 0 && r >= 0)            { width = r - l; mid = (l + r) >> 1; line_valid[i] = 1; }
        else if (l >= 0 && width)        { mid = MIN(cols - 1, l + (width >> 1)); line_valid[i] = 0; }
        else if (r >= 0 && width)        { mid = MAX(0, r - (width >> 1));        line_valid[i] = 0; }
        else                             { /* 双边丢失：mid 保持上一行，不重置 */ line_valid[i] = 0; }

        _left_line[i]  = (l >= 0) ? (uint8_t)l : 0;
        _right_line[i] = (r >= 0) ? (uint8_t)r : cols - 1;
        _mid_line[i]   = mid;
    }
}


/* 形态学滤波：3x3 结构元
 * 约定：二值图中 1 = 亮（白线），0 = 暗（背景）
 * 若你的二值化结果相反（线为 0），把 erode/dilate 的取值逻辑互换即可
 */

/* 腐蚀：3x3 邻域全为 1 才输出 1。孤立噪点被清除，亮区收缩一圈 */
void erode(const uint8_t *src, uint8_t *dst, int h, int w)
{
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            uint8_t v = 1;
            for (int dy = -1; dy <= 1 && v; dy++)
            {
                int ny = y + dy;
                if (ny < 0 || ny >= h)
                {
                    v = 0;
                    break;
                }
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    if (nx < 0 || nx >= w || src[ny * w + nx] == 0)
                    {
                        v = 0;
                        break;
                    }
                }
            }
            dst[y * w + x] = v;
        }
    }
}

/* 膨胀：3x3 邻域有 1 则输出 1。把腐蚀时收缩的线恢复原宽，并可接断线 */
void dilate(const uint8_t *src, uint8_t *dst, int h, int w)
{
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            uint8_t v = 0;
            for (int dy = -1; dy <= 1 && !v; dy++)
            {
                int ny = y + dy;
                if (ny < 0 || ny >= h)
                    continue;
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    if (nx >= 0 && nx < w && src[ny * w + nx] != 0)
                    {
                        v = 1;
                        break;
                    }
                }
            }
            dst[y * w + x] = v;
        }
    }
}
