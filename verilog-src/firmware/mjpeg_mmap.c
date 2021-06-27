#include "firmware.h"

#define MAX_INT                 2147483647

// Here is the data offset
// We place DNN data in these addresses
#define RAW_IMAGE_OFFSET 			0x00010000
#define UNFOLDED_IMAGE_OFFSET    	0x00020000
#define SOFT_MJPEG_OUTPUT_OFFSET	0x00080000
#define HARD_MJPEG_OUTPUT_OFFSET	0x00090000

/* Function: cosine
* Description:
*   This function computes the consine value of the corresponding input radian
* Parameters:
*   radian: the coefficient of PI times 16 of the input radian to the cosine function
*/
int cosine(int radian){
    int lut[9] = {1000, 981, 924, 831, 707, 556, 383, 195, 0};
    int converted_rad = radian % 32;
    converted_rad = (converted_rad > 16)? 32 - converted_rad: converted_rad;
    int invert = (converted_rad > 8)? -1: 1;
    converted_rad = (converted_rad > 8)? 16 - converted_rad: converted_rad;
    return invert * lut[converted_rad];
}

// MJPEG_MMAP's address mapping
/*
* FUNCTION: DCT
* Description:
*   This function performs 2D DCT on the input data using 8 x 8 blocks
* Parameters:
*   input_data: the pointer to the array that stores the input_data
*   output_data: the pointer to the array that stores the result;
*   input_shape: the dimension of `input_data`
*   num_coef: the number of frequency componenet reserved in the result
* Returns:
*   result: the coefficient of the frequency componenet
*/

void DCT(volatile int8_t * input_data, volatile int32_t * output_data , int * input_shape, int num_coef){
    int height = input_shape[0];
	int width = input_shape[1];
	int yuv = input_shape[3];
    int num_pixel = input_shape[2];
    for (int h = 0; h < height; h ++){
        for (int w = 0; w < width; w ++){
            for (int c = 0; c < yuv; c ++){
                print_dec(h);
                print_dec(w);
                print_dec(c);
                print_str("\n");
                for (int row_index = 0; row_index < num_coef; row_index ++){
                    for (int column_index  = 0; column_index < num_coef; column_index ++){
                        int result = 0;
                        for (int i = 0; i < 8; i ++){
                            for (int j = 0; j < 8; j ++){
                                int tmp = cosine(row_index * (2 * i + 1)) * cosine(column_index * (2 * j + 1));
                                int index_i = ((h)*width + w)*num_pixel*yuv + (i * 8 + j) * yuv + c;
                                print_dec(index_i);
                                print_str("\n");
                                print_sign(*(input_data + index_i));
                                print_str("\n");
                                tmp = tmp * *(input_data + index_i);
                                result = result + tmp;
                            }
                        }
                        int index_o = ((h)*width + w)*num_pixel*yuv + (row_index * 8 + column_index) * yuv + c;
                        result = result / 4;
                        *(output_data + index_o) = result;
                    }
                }
            }
        }
    }
    // result = (row_index == 0)? result / sqrt(2): result;
    // result = (column_index == 0)? result / sqrt(2): result;
}

/* Function: unfold
* Description:
*   This function orchestrates the raw input data such that pixels in the
*   same yuv channel are placed in neighboring addresses
* Parameters:
*   input_shape: the dimension of `input_data`
*   input_data: the pointer to the array that contains the raw input_data
*   output_data: the pointer to the array that stores the orchestrated data
* Example:
*   input_shape = {90, 160, 64, 3}
*   output_shape = {90, 160, 3, 64}
*/
void unfold(int *input_shape, volatile int8_t* input_data, volatile int8_t* output_data)
{
	int height = input_shape[0];
	int width = input_shape[1];
	int num_pixel = input_shape[2];
	int yuv = input_shape[3];

	for (int h = 0; h < height; h ++){
        for (int w = 0; w < width; w ++){
			for (int m=0; m < num_pixel; m++){
				for (int n=0; n<yuv; n++){
					int index_o = ((h)*width + w)*num_pixel*yuv + n*num_pixel + m;
					int index_i = ((h)*width + w)*num_pixel*yuv + m*yuv + n;
					*(output_data + index_o) = *(input_data + index_i);
				}
			}
        }
    }
}

void MJPEG_SOFT(void){
    int start, stop;
    int input_shape[4] = {4, 4, 64, 3};
    // setup pointer to the input and output data addresses
    volatile int8_t* input_data = RAW_IMAGE_OFFSET;
    volatile int32_t* output_data = SOFT_MJPEG_OUTPUT_OFFSET;
    print_dec(*(input_data + 3));
    print_str("\n");
    // performs DCT
    start = tick();
    DCT(input_data, output_data, input_shape, 2);
    stop = tick();
    // print result
    print_str("\nThe result of soft_conv is:\n");
	int output_size = input_shape[0] * input_shape[1] * input_shape[2] * input_shape[3];
	for(int i=0; i< output_size; i++){
		print_dec(i);
		print_str(": ");
		print_sign(*(output_data + i));
		print_str("\n");
	}
	print_str("\nThe time usage: ");
	print_dec(stop - start);
	print_str("\n");
}

void mjpeg_mmap(void){
    // software version
    MJPEG_SOFT();
}
