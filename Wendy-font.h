#ifndef Wendy3x5_H
#define Wendy3x5_H

#define Wendy3x5_WIDTH 3
#define Wendy3x5_HEIGHT 5

/*
 * added to allow fontname to match header file name. 
 * as well as keep the old name for backward compability
 */

#define WendyFont3x5 Wendy3x5

static const uint8_t Wendy3x5[] PROGMEM = {
//static const uint8_t Wendy3x5[] = {
    
    // font data
        0x00, 0x00, 0x00,  // (space)
        0x00, 0x17, 0x00,  // !
        0x03, 0x00, 0x03,  // "
        0x1F, 0x0A, 0x1F,  // # 0x0A, 0x1F, 0x0A,
        0x16, 0x13, 0x1A,  // 0x
        0x09, 0x04, 0x12,  // %
        0x0A, 0x15, 0x1A,  // &
        0x00, 0x03, 0x00,  // '
        0x00, 0x0E, 0x11,  // (
        0x11, 0x0E, 0x00,  // )
        0x0E, 0x1F, 0x15,  // * €
        0x04, 0x0E, 0x04,  // +
        0x10, 0x08, 0x00,  // , 0x0C, 0x1C, 0x00,
        0x04, 0x04, 0x04,  // -
        0x00, 0x10, 0x00,  // .
        0x18, 0x04, 0x03,  // /
        0x1F, 0x11, 0x1F,  // 0
        0x12, 0x1F, 0x10,  // 1  0x02, 0x1F, 0x00,
        0x1D, 0x15, 0x17,  // 2
        0x15, 0x15, 0x1F,  // 3
        0x0F, 0x08, 0x1E,  // 4
        0x17, 0x15, 0x1D,  // 5
        0x1F, 0x15, 0x1D,  // 6
        0x01, 0x01, 0x1F,  // 7
        0x1F, 0x15, 0x1F,  // 8
        0x17, 0x15, 0x1F,  // 9
        0x00, 0x0A, 0x00,  // :
        0x10, 0x0A, 0x00,  // ; 0x00, 0x1A, 0x00,
        0x04, 0x0A, 0x11,  // <
        0x0A, 0x0A, 0x0A,  // =
        0x11, 0x0A, 0x04,  // >
        0x00, 0x15, 0x07,  // ?
        0x1F, 0x15, 0x17,  // @
        0x1F, 0x05, 0x1F,  // A
        0x1F, 0x15, 0x1B,  // B
        0x1F, 0x11, 0x11,  // C
        0x1F, 0x11, 0x0E,  // D
        0x1F, 0x15, 0x15,  // E
        0x1F, 0x05, 0x01,  // F
        0x0E, 0x15, 0x1D,  // G 0x1F, 0x11, 0x1D
        0x1F, 0x04, 0x1F,  // H
        0x11, 0x1F, 0x11,  // I
        0x08, 0x10, 0x0F,  // J
        0x1F, 0x04, 0x1B,  // K
        0x1F, 0x10, 0x10,  // L
        0x1F, 0x01, 0x1F,  // M 0x1F, 0x06, 0x1F,
        0x1F, 0x06, 0x1F,  // N 0x1C, 0x04, 0x1C,
        0x1F, 0x11, 0x1F,  // O
        0x1F, 0x05, 0x07,  // P
        0x0E, 0x19, 0x1E,  // Q
        0x1F, 0x05, 0x1B,  // R
        0x16, 0x15, 0x0D,  // S  0x17, 0x15, 0x1D
        0x01, 0x1F, 0x01,  // T
        0x1F, 0x10, 0x1F,  // U
        0x0F, 0x10, 0x0F,  // V
        0x1F, 0x0C, 0x1F,  // W
        0x1B, 0x04, 0x1B,  // X
        0x17, 0x14, 0x1F,  // Y
        0x19, 0x15, 0x13,  // Z
        0x1F, 0x11, 0x00,  // [
        0x03, 0x04, 0x18,  // BackSlash
        0x00, 0x11, 0x1F,  // ]
        0x06, 0x01, 0x06,  // ^
        0x10, 0x10, 0x10,  // _
        0x01, 0x01, 0x02,  // `
        0x18, 0x14, 0x1C,  // a
        0x1F, 0x14, 0x1C,  // b
        0x1C, 0x14, 0x14,  // c
        0x1C, 0x14, 0x1F,  // d
        0x0C, 0x1A, 0x14,  // e
        0x04, 0x1E, 0x05,  // f
        0x06, 0x15, 0x0F,  // g 0x17, 0x15, 0x1E
        0x1F, 0x04, 0x1C,  // h
        0x00, 0x1D, 0x00,  // i
        0x08, 0x10, 0x0D,  // j
        0x1F, 0x0C, 0x1A,  // k
        0x00, 0x1F, 0x00,  // l
        0x1C, 0x04, 0x1C,  // m 0x18, 0x0C, 0x18
        0x18, 0x04, 0x18,  // n
        0x1C, 0x14, 0x1C,  // o 0x1E, 0x12, 0x1E
        0x1E, 0x05, 0x07,  // p 0x1F, 0x05, 0x07,
        0x06, 0x05, 0x1F,  // q 0x07, 0x05, 0x1F,
        0x18, 0x04, 0x04,  // r
        0x14, 0x1E, 0x0A,  // s 0x12, 0x15, 0x09,
        0x00, 0x1F, 0x02,  // t 0x02, 0x1F, 0x02,
        0x0C, 0x10, 0x1C,  // u 0x1C, 0x10, 0x1C,
        0x0C, 0x10, 0x0C,  // v 0x0C, 0x10, 0x0C,
        0x1E, 0x1C, 0x1E,  // w 0x0C, 0x18, 0x0C
        0x14, 0x08, 0x14,  // x
        0x16, 0x18, 0x06,  // y
        0x04, 0x1C, 0x10,  // z
        0x04, 0x0E, 0x11,  // {
        0x00, 0x1F, 0x00,  // |
        0x11, 0x0E, 0x04,  // }
        0x02, 0x05, 0x02,  // ~ 0x02, 0x04, 0x02
        0x0C, 0x1A, 0x15,  // è 128 0x80       
        0x0D, 0x1A, 0x14,  // é 127 0x7f (del)
        0x0D, 0x1A, 0x15,  // ê
        0x1F, 0x1F, 0x1F   // 
};

/*
static const uint8_t Wendy3x6[] PROGMEM =
                       {0x00,0x00,0x00, // Space   0x20
                        0x00,0x5C,0x00, // !     
                        0x0C,0x00,0x0C, // "
                        0x7C,0x28,0x7C, // #
                        0x7C,0x44,0x7C, // 0x                         
                        0x24,0x10,0x48, // %
                        0x28,0x54,0x08, // &
                        0x00,0x0C,0x00, // '                          
                        0x38,0x44,0x00, // (                          
                        0x44,0x38,0x00, // )                          
                        0x20,0x10,0x08, // //                         
                        0x10,0x38,0x10, // +                          
                        0x80,0x40,0x00, // ,                          
                        0x10,0x10,0x10, // -                          
                        0x00,0x40,0x00, // .                          
                        0x20,0x10,0x08, // /     
                        0x38,0x44,0x38, // 0    0x30                      
                        0x00,0x7C,0x00, // 1                          
                        0x64,0x54,0x48, // 2                          
                        0x44,0x54,0x28, // 3                          
                        0x1C,0x10,0x7C, // 4                          
                        0x4C,0x54,0x24, // 5                          
                        0x38,0x54,0x20, // 6                          
                        0x04,0x74,0x0C, // 7                          
                        0x28,0x54,0x28, // 8                          
                        0x08,0x54,0x38, // 9                          
                        0x00,0x50,0x00, // :                          
                        0x80,0x50,0x00, // ;                          
                        0x10,0x28,0x44, // <                          
                        0x28,0x28,0x28, // =                   
                        0x44,0x28,0x10, // >                         
                        0x04,0x54,0x08, // ?                          
                        0x38,0x4C,0x5C, // @    0x40                           
                        0x78,0x14,0x78, // A                          
                        0x7C,0x54,0x28, // B                          
                        0x38,0x44,0x44, // C                          
                        0x7C,0x44,0x38, // D                          
                        0x7C,0x54,0x44, // E                          
                        0x7C,0x14,0x04, // F                          
                        0x38,0x44,0x34, // G                          
                        0x7C,0x10,0x7C, // H                          
                        0x00,0x7C,0x00, // I                          
                        0x20,0x40,0x3C, // J                          
                        0x7C,0x10,0x6C, // K                          
                        0x7C,0x40,0x40, // L                          
                        0x7C,0x08,0x7C, // M                          
                        0x7C,0x04,0x7C, // N                          
                        0x7C,0x44,0x7C, // O                          
                        0x7C,0x14,0x08, // P    0x50                  
                        0x38,0x44,0x78, // Q                          
                        0x7C,0x14,0x68, // R                          
                        0x48,0x54,0x24, // S                          
                        0x04,0x7C,0x04, // T                          
                        0x7C,0x40,0x7C, // U                          
                        0x3C,0x40,0x3C, // V                          
                        0x7C,0x20,0x7C, // W                          
                        0x6C,0x10,0x6C, // X                          
                        0x1C,0x60,0x1C, // Y                          
                        0x64,0x54,0x4C, // Z                          
                        0x7C,0x44,0x00, // [                          
                        0x08,0x10,0x20, // \                          
                        0x44,0x7C,0x00, // ]                          
                        0x08,0x04,0x08, // ^                          
                        0x80,0x80,0x80, // _                          
                        0x04,0x08,0x00, // ` 0x60                  
//        0x18, 0x14, 0x1C,  // a
        B00011000, 
        B00100100,
        B00011000,

        B00011110, 
        B00101000,
        B00011000,
        
//      0x1F, 0x14, 0x1C,  // b
        0x1C, 0x14, 0x14,  // c
        0x1C, 0x14, 0x1F,  // d
        0x0C, 0x1A, 0x14,  // e
        0x04, 0x1E, 0x05,  // f
        0x17, 0x15, 0x1E,  // g
        0x1F, 0x04, 0x1C,  // h
        0x00, 0x1D, 0x00,  // i
        0x08, 0x10, 0x0D,  // j
        0x1F, 0x0C, 0x1A,  // k
        0x00, 0x1F, 0x00,  // l
        0x1C, 0x04, 0x1C,  // m 0x18, 0x0C, 0x18
        0x18, 0x04, 0x18,  // n
        0x1C, 0x14, 0x1C,  // o 0x1E, 0x12, 0x1E
        0x1E, 0x05, 0x07,  // p 0x1F, 0x05, 0x07,
        0x07, 0x05, 0x1F,  // q 0x07, 0x05, 0x1F,
        0x1E, 0x04, 0x04,  // r
        0x12, 0x15, 0x09,  // s
        0x00, 0x1F, 0x02,  // t 0x02, 0x1F, 0x02,
        0x1C, 0x10, 0x1C,  // u
        0x0C, 0x10, 0x0C,  // v
        0x0C, 0x18, 0x0C,  // w
        0x14, 0x08, 0x14,  // x
        0x16, 0x18, 0x06,  // y
        0x04, 0x1C, 0x10,  // z
        0x04, 0x0E, 0x11,  // {
        0x00, 0x1F, 0x00,  // |
        0x11, 0x0E, 0x04,  // }
        0x02, 0x04, 0x02,  // ~
        0x1F, 0x1F, 0x1F   // 

};
*/

#endif
