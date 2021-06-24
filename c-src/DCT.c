#include <math.h>

float DCT(int * pixel, int row_index, int column_index){
    float result = 0;
    for (int i = 0; i < 8; i ++){
        for (int j = 0; j < 8; j ++){
            float tmp = cos(acos(-1) * row_index / 16 * (2 * i + 1)) * cos(acos(-1) * column_index / 16 * (2 * j + 1));
            tmp = tmp * pixel[i * 8 + j];
            result = result + tmp;
        }
    }
    result = (row_index == 0)? result / sqrt(2): result;
    result = (column_index == 0)? result / sqrt(2): result;
    result = result / 4;
    return result;
}