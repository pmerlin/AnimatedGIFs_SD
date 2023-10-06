#if !defined(FASTLED_MATRIX)
#define FASTLED_MATRIX

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))


#define adagfxswap(a, b) { a = a ^ b; b = a ^ b; a = a ^ b; }

#if !defined(ESP8266)
  #define swap(a, b) adagfxswap(a, b)
#endif


/* *** LED color table *** */
//BGR
/*
#define  BLACK  CRGB:Black
#define  GREEN  CRGB::Green //0x00FF00
#define  LGREEN CRGB::LightGreen // 0x007700
#define  RED    CRGB::Red // 0xFF0000
#define  BLUE   CRGB::Blue // 0x0000FF
#define  YELLOW CRGB::Yellow //0xFFFF00
#define  LBLUE  CRGB::LightBlue //0x00FFFF*/

/*
#define BLACK 0x000000
#define GREEN 0x00FF00
#define LGREEN 0x007700
#define RED 0xFF0000
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define LBLUE 0x00FFFF
#define PURPLE 0xFF00FF
#define LPURPLE 0x770077
#define WHITE 0xFFFFFF
#define LWHITE 0x777777
*/

uint16_t BLACK = (uint16_t)0x000000;
uint16_t GREEN = (uint16_t)0x00FF00;
uint16_t LGREEN = (uint16_t)0x007700;
uint16_t RED = (uint16_t)0xFF0000;
uint16_t BLUE = (uint16_t)0x0000FF;
uint16_t YELLOW = (uint16_t)0xFFFF00;
uint16_t LBLUE = (uint16_t)0x00FFFF;
uint16_t PURPLE = (uint16_t)0xFF00FF;
uint16_t LPURPLE = (uint16_t)0x770077;
uint16_t WHITE = (uint16_t)0xFFFFFF;
uint16_t LWHITE = (uint16_t)0x777777;

uint16_t myWHITE = (uint16_t)0xFFFFFF;
uint16_t myBLACK = (uint16_t)0x000000;
uint16_t myGREY =  (uint16_t)0x888888;



//uint32_t PrintCol[2] = {CRGB::Yellow, CRGB::Red};
uint32_t PrintCol[2] = {CRGB::Black, CRGB::Grey};
uint32_t CalCol[2] = {CRGB::DarkBlue, CRGB::DarkBlue};
uint32_t WhiteCol[2] = {CRGB::White, CRGB::White};
uint32_t BlackCol[2] = {CRGB::Black, CRGB::Black};

int cursX, cursY;
//uint32_t cursCol[2] = {CRGB::Yellow, CRGB::Red};
uint32_t cursCol[2] = {CRGB::Red, CRGB::White};

uint8_t letter_width= 3;

#endif

#include "fastled-matrix-impl.h"
