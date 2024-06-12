#include "conf_general.h"
#include "syled.h"


const unsigned char ledsegments[SIZE_MATRIX_COL] = {
    C7_0,    // Batt 0
    C7_1,    // Batt 1
    C7_2,    // Status Header 2
    C7_3,    // Status Side 3
    C7_4,    // digit 4
    C7_5,    // digit 5
    C7_6,    // digit 6
    C7_7,    // digit 7
    C7_8,    // digit 8
    C7_9,    // digit 9
    C7_NULL  // NULL
};

const unsigned char upperpos[4] = {
    7,      // digit 0
    6,      // digit 1
    5,      // digit 2
    4       // digit 3
};


size_t intToDigits(uint32_t number, uint8_t* digits) {
    size_t length = 0;
    size_t maxLength = countDigits(number);

    while (number > 0 && length < maxLength) {
        digits[length] = ledsegments[number % 10];
        number /= 10;
        length++;
    }

    // Reverse the order of digits
    for (size_t i = 0; i < length / 2; i++) {
        uint8_t temp = digits[i];
        digits[i] = digits[length - i - 1];
        digits[length - i - 1] = temp;
    }

    return length;
}


size_t countDigits(uint32_t number) {
    size_t count = 0;
  
    if (number == 0) {
        return 1;  // Special case for number 0
    }
  
    while (number != 0) {
        number /= 10;
        count++;
    }
  
    return count;
}