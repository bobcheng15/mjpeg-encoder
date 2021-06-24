#include "firmware.h"

#define MAX_INT                 2147483647

// Here is the data offset
// We place DNN data in these addresses
#define WAR_INPUT_OFFSET 			0x00010000
#define ORCHESTRATED_INPUT_OFFSET 	0x00020000
#define SOFT_MJPEG_OUTPUT_OFFSET	0x00080000
#define HARD_MJPEG_OUTPUT_OFFSET	0x00090000

// MJPEG_MMAP's address mapping
/*
* FUNCTION: DCT
* Parameters:
*   pixel: the pointer to the array that stores the 8 * 8 pixel block
*   row_index: the row index of the block in the original image
*   column_index: the column index of the block in the original image
* Returns: 
*   result: the coefficient of the frequency componenet
*/

float DCT(int * pixel, int row_index, int column_index){
    float result = 0;
    for (int i = 0; i < 8; i ++){
        for (int j = 0; j < 8; j ++){
            float tmp = cosine(row_index / 16 * (2 * i + 1)) * cosine(column_index / 16 * (2 * j + 1));
            tmp = tmp * pixel[i * 8 + j];
            result = result + tmp;
        }
    }
    result = (row_index == 0)? result / sqrt(2): result;
    result = (column_index == 0)? result / sqrt(2): result;
    result = result / 4;
    return result;
}

/* Function: cosine
* Parameters:
*   radian: the input radian to the cosine function ()
*   
*
*/
float cosine(float radian){


}