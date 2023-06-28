#ifndef SYLED_H_
#define SYLED_H_

#include <stdint.h>
#include <datatypes.h>


/*
 *     BattIn   BattOut   Status   DType   D1.3    D1.2    D1.1    D1.0    D2.2    D2.1    D2.0
 *       1       2          3         4       5       6       7       8       9      10      11
 * S1A  100%     100%NW    Alert1    W     d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S2B   80%     100%N     Alert2   KMH    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S3C   60%     50%N      BLE1     PER    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S4D   40%     0%NE      BLE2    VOLT    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S5E   20%     0%SE      Charg  Comma    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S6F  100%S    50%S        .     Time    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S7G  100%SW    .          .        .    d1.3    d1.2    d1.1    d1.0    d2.2    d2.1    d2.0
 * S8H    .       .          .        .     .       .       .       .       .       .       .
 */

/*
There are tree types of commands

DATA COMMAND                        // sets the address mode to auto or fixed
DISPL COMMAND                       // sets the display on or off and the brightness
ADDR COMMAND + VAL1 + .. VALN       // sets the address and the value to be displayed
                                    // ATTN this is a multibyte command
*/


#define TM1640_DATA_COMMAND             0x40
#define TM1640_DISP_COMMAND             0x80
#define TM1640_ADDR_COMMAND             0xC0

#define TM1640_DATA_CINC	            0x00
#define TM1640_DATA_CSET_FIXADDR	    0x04

// Address Command is used to set the

#define TM1640_DISP_COFF      	        0x00
#define TM1640_DISP_CON      	        0x08
#define TM1640_DISP_BRIGHTNESS_4        0x03
#define TM1640_DISP_BRIGHTNESS_8        0x07

#define SIZE_MATRIX_COL		            11
#define SIZE_MATRIX_ROW		            8

// Battery status
#define SLED_BATT_COLUM				     0x03
#define SLED_BATT_NONE					(0x00)
#define SLED_BATT_100				    (0x00_1F<<0)
#define SLED_BATT_80				    (0x00_0F<<0)
#define SLED_BATT_60				    (0x00_07<<0)
#define SLED_BATT_40				    (0x00_03<<0)
#define SLED_BATT_20				    (0x00_01<<0)
#define SLED_BATT_0 				    (0x00_00<<0)
#define SLED_BATT_HULL 				    (0x3F_60<<0)

// Battery status
#define SLED_STATUS_COLUM				 0x04
#define SLED_BATT_NONE					(0x00)
#define SLED_BATT_100				    (0x00_1F<<0)
#define SLED_BATT_80				    (0x00_0F<<0)
#define SLED_BATT_60				    (0x00_07<<0)
#define SLED_BATT_40				    (0x00_03<<0)
#define SLED_BATT_20				    (0x00_01<<0)
#define SLED_BATT_0 				    (0x00_00<<0)
#define SLED_BATT_HULL 				    (0x3F_60<<0)

// Decimal Type
#define SLED_DTYPE_COLUMN				 0x08
#define SLED_DTYPE_NONE					(0x00)
#define SLED_DTYPE_W					(1<<0)
#define SLED_DTYPE_KMH					(1<<1)
#define SLED_DTYPE_PERCENT              (1<<2)
#define SLED_DTYPE_VOLT					(1<<3)
#define SLED_DTYPE_TIME					(1<<4)





// Segment bit positions for 7 Segment display using the LM1640 mapping for TM1640
// Modify this table for different 'bit-to-segment' mappings. The ASCII character defines and the FONT_7S const table below 
// will be adapted automatically according to the bit-to-segment mapping. Obviously this will only work when the segment
// mapping is identical for every digit position. This will be the case unless the hardware designer really hates software developers.
//
//            A
//          -----
//         |     |     
//       F |     | B    
//         |  G  |     
//          -----
//         |     |     
//       E |     | C    
//         |     |     
//          -----   * DP
//            D  
//
#define S7_A    0x0001
#define S7_B    0x0002
#define S7_C    0x0004
#define S7_D    0x0008
#define S7_E    0x0010
#define S7_F    0x0020
#define S7_G    0x0040 
#define S7_DP   0x0080 

//48 0x30  Digits
#define C7_0    (S7_A | S7_B | S7_C | S7_D | S7_E | S7_F)
#define C7_1    (S7_B | S7_C)
#define C7_2    (S7_A | S7_B | S7_D | S7_E | S7_G)
#define C7_3    (S7_A | S7_B | S7_C | S7_D | S7_G)
#define C7_4    (S7_B | S7_C | S7_F | S7_G)
#define C7_5    (S7_A | S7_C | S7_D | S7_F | S7_G)
#define C7_6    (S7_A | S7_C | S7_D | S7_E | S7_F | S7_G)
#define C7_7    (S7_A | S7_B | S7_C)
#define C7_8    (S7_A | S7_B | S7_C | S7_D | S7_E | S7_F | S7_G)
#define C7_9    (S7_A | S7_B | S7_C | S7_D | S7_F | S7_G)
#define C7_NULL  0x00

extern const unsigned char ledsegments[SIZE_MATRIX_COL];


#define ULED_MSB   4

// Function prototype for intToDigits()
size_t intToDigits(uint32_t number, uint8_t* digits);
size_t countDigits(uint32_t number);


typedef struct {
	float v_in;
	int32_t tacho_value;
	int32_t mc_interface_get_battery_level;
} led_data_t;



#endif /* SYLED_H_ */
