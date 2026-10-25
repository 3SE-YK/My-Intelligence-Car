/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"
#include <stdlib.h>

/*
 *@func: 二值化函数
 */
void binaryzation_process(unsigned char* _img, 
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
void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram)
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
unsigned char get_threshold_otsu(const short* _hist_gram)
{
    int X, Y, amount = 0;
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += _hist_gram[Y] * Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_back += _hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line)
{
    uint8_t mid_point = _src_cols >> 1;
    if (_src_pixel_mat[(_src_rows - 1) * _src_cols + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        int16_t _tmp_left_mid = 0;
        int16_t _tmp_right_mid = _src_cols;
        for (int16_t j = (_src_cols >> 1); j < _src_cols - 2; ++j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 2] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
            }
        }

        for (int16_t j = (_src_cols >> 1); j > 2; --j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 2] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    for (int i = _src_rows - 1; i >= 0; --i)
    {
        uint8_t cur_point = mid_point;
       // 扫描左线
        while (cur_point - 2 > 0)
        {
            _left_line[i] = 0;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 1] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 2] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while (cur_point + 2 < _src_cols)
        {
            _right_line[i] = _src_cols - 1;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 1] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 2] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (_right_line[i] + _left_line[i]) >> 1;
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
}


/**
 * @brief 3x3 腐蚀（二值图优化版，参数顺序与工程内其他函数一致：先 H 后 W）
 * @param image  输入图像（只读，0=黑 255=白，大小 w*h）
 * @param filter 输出图像（必须与 image 是不同的缓冲！）
 * @param h      图像高度（行数），调用处传 MT9V03X_H (120)
 * @param w      图像宽度（每行像素数），调用处传 MT9V03X_W (188)
 */
void erode(const uint8_t *image, uint8_t *filter, int h, int w)
{
    for (int y = 0; y < h; y++)          // y 遍历"行"，上限是 H
    {
        for (int x = 0; x < w; x++)      // x 遍历"列"，上限是 W
        {
            // 中心是黑的直接输出黑，跳过邻域扫描（背景占大多数，省一半工作量）
            if (image[y * w + x] == 0)
            {
                filter[y * w + x] = 0;
                continue;
            }

            int all_white = 1;

            for (int j = -1; j <= 1 && all_white; j++)
            {
                int yy = y + j;
                if (yy < 0 || yy >= h) { all_white = 0; break; }

                for (int i = -1; i <= 1 && all_white; i++)
                {
                    int xx = x + i;
                    if (xx < 0 || xx >= w) { all_white = 0; break; }

                    if (image[yy * w + xx] == 0)
                        all_white = 0;
                }
            }

            filter[y * w + x] = all_white ? 255 : 0;
        }
    }
}
