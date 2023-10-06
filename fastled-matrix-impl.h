#include "Wendy-font.h"

/* UTF-8 to ISO-8859-1/ISO-8859-15 mapper.
 * Return 0..255 for valid ISO-8859-15 code points, 256 otherwise.
*/

static inline unsigned int to_latin9(const unsigned int code) {
    /* Code points 0 to U+00FF are the same in both. */
    if (code < 256U)
        return code;
    switch (code) {
    case 0x0152U: return 188U; /* U+0152 = 0xBC: OE ligature */
    case 0x0153U: return 189U; /* U+0153 = 0xBD: oe ligature */
    case 0x0160U: return 166U; /* U+0160 = 0xA6: S with caron */
    case 0x0161U: return 168U; /* U+0161 = 0xA8: s with caron */
    case 0x0178U: return 190U; /* U+0178 = 0xBE: Y with diaresis */
    case 0x017DU: return 180U; /* U+017D = 0xB4: Z with caron */
    case 0x017EU: return 184U; /* U+017E = 0xB8: z with caron */
    case 0x20ACU: return 164U; /* U+20AC = 0xA4: Euro */
    default:      return 256U;
    }
}

size_t utf8_to_latin9(char *const output, const char *const input, const size_t length) {
    unsigned char             *out = (unsigned char *)output;
    const unsigned char       *in  = (const unsigned char *)input;
    const unsigned char *const end = (const unsigned char *)input + length;

    while (in < end)
        if (*in < 128)
            *(out++) = *(in++); // Valid codepoint 
        else
        if (*in != 0xC3)
            in++;               // 10000000 .. 10111111 are invalid 
        else
        {
          in++;

          if (*in == 0xa9) // é
            *(out++) = 127;
          else
          if (*in == 0xa8) // è
            *(out++) = 128;
          else
          if (*in == 0xa0) // à
            *(out++) = 'a'; //128;
          else
          if (*in == 0xaa) // ê
            *(out++) = 'e'; //129;
          else  
          if (*in == 0xf9) // ù
            *(out++) = 'u'; //131;
          else  
          if (*in == 0xe7) // ç
            *(out++) = 'c'; // 132;
          in++;  
        }

    /* Terminate the output string. */
    *out = '\0';
    return (size_t)(out - (unsigned char *)output);
}

void drawPixel(uint8_t n, uint32_t color)
{
  leds[n] = CRGB(color);
}

void drawPixel(uint8_t n, uint8_t r, uint8_t g, uint8_t b)
{
  leds[n] = CRGB(r, g, b);
}

void fillScreen(uint32_t color)
{
  uint16_t i;
  for (i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(color);
}

void clearDisplay()
{
  fillScreen(0x000000);
}

/*
void copyScreen()
{
  uint16_t i;
  for (i = 0; i < NUM_LEDS; i++)
    leds[i] = leds2[i];
}
*/

#ifdef VERTICAL
uint16_t XY(uint8_t x, uint8_t y)
{
   //VERT+RIGHT
  if (x & 0x01)
    return ( (HEIGHT - 1 - y) + (x * HEIGHT));
  else
    return (x * HEIGHT + y);
}

#else

uint16_t XY(uint8_t x, uint8_t y)
{
  

  //ROTATION
  //  t = x;
  //  x = WIDTH  - 1 - y;
  //  y = t;

/*  
  uint16_t t = 0;
  switch(rotation) {
   case 1:
    t = x;
    x = WIDTH  - 1 - y;
    y = t;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    t = x;
    x = y;
    y = HEIGHT - 1 - t;
    break;
  }
*/

//RIGHT Matrix correction
    x= WIDTH-1-x;

  
//ZIGZAG
  if( y & 0x01)
    return ( (y * WIDTH) + (WIDTH-1-x) );
  else
    return ( (y * WIDTH) + x);


//NORMAL
//  return (x + y * WIDTH);

}
#endif

void drawPixel(int16_t x, int16_t y, uint32_t color, bool transparent)
{
  if(transparent == true && color == 0x0) return;
 
  if ((x < 0) || (y < 0) || (x >= WIDTH) || (y >= HEIGHT))
    return;
  leds[XY(x, y)] = CRGB(color);
}

void drawPixel(int16_t x, int16_t y, uint32_t color)
{
  if ((x < 0) || (y < 0) || (x >= WIDTH) || (y >= HEIGHT))
    return;
  leds[XY(x, y)] = CRGB(color);
}

void drawPixel(int16_t x, int16_t y, CRGB color)
{
  if ((x < 0) || (y < 0) || (x >= WIDTH) || (y >= HEIGHT))
    return;
  leds[XY(x, y)] = color;
}


void drawPixelDouble(uint8_t x, uint8_t y, uint32_t col)
{
  drawPixel((x << 1), (y << 1), col);
  drawPixel((x << 1) + 1, (y << 1), col);
  drawPixel((x << 1), (y << 1) + 1, col);
  drawPixel((x << 1) + 1, (y << 1) + 1, col);
}

uint32_t getPixel(uint8_t n)
{
  return (leds[n].r << 16) + (leds[n].g << 8) + leds[n].b;
}

///////////////

// Bresenham's algorithm - thx wikpedia
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    adagfxswap(x0, y0);
    adagfxswap(x1, y1);
  }

  if (x0 > x1) {
    adagfxswap(x0, x1);
    adagfxswap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void drawFastHLine(int16_t x, int16_t y, int16_t w, uint32_t color) {
  // Update in subclasses if desired!
  drawLine(x, y, x+w-1, y, color);
}

void drawFastVLine(int16_t x, int16_t y, int16_t h, uint32_t color) {
  // Update in subclasses if desired!
  drawLine(x, y, x, y+h-1, color);
}

// Draw a rectangle
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color) {
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y+h-1, w, color);
  drawFastVLine(x, y, h, color);
  drawFastVLine(x+w-1, y, h, color);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color) {
  // Update in subclasses if desired!
  for (int16_t i=x; i<x+w; i++) {
    drawFastVLine(i, y, h, color);
  }
}



///////////////

void printTextPos(const char text[], int16_t pos, int16_t xoffset, int16_t yoffset, uint32_t color[2])
{
  uint16_t curX;
  int8_t lx;
  int16_t i;
  uint16_t textsize;
  uint8_t minusculeY = 0;
  uint8_t col;

//  static int16_t pos=-99;


  textsize = strlen(text);
  for (curX = xoffset, i = pos / letter_width; i < textsize && curX < WIDTH; i++)
  {

/*    
    if (i>=4)
{    
 sprintf(zl, "pos: %d, curX: %d, lx:%d, i:%d ", pos, curX, lx, i);
  Serial.println(zl);      
}
*/
    // some letters are 1y below

    if (text[i] == 'g' || text[i] == 'j' || text[i] == 'p' || text[i] == 'q' || text[i] == 'y' || text[i] == ',')
      minusculeY = 1;
    else
      minusculeY = 0;

    //Loop until width of letter is reached
    for (lx = pos % letter_width; lx < letter_width && curX < WIDTH; lx++, pos++, curX++)
    {     
      if (i >= 0 && pos >= 0)
      {
        if (lx == 3) // to add space between letter for letter_width=4
          col = 0;
        else if ( (text[i] - 32) < 98 && text[i] > 31)
          col = pgm_read_byte_near(&Wendy3x5[(text[i] - 32) * 3 + lx]);
//          col = Wendy3x5[(text[i] - 32) * 3 + lx];
        else
          col = 0xFF;
        /*
if(i==0) {
        sprintf(zl, "Print pos: %d, curX: %d, lx:%d, i:%d ", pos, curX, lx, i);
        Serial.println(zl);
}*/
        for (uint8_t ly = 0; ly < 5; ly++)
        {
          if (bitRead(col, ly))
            drawPixel(curX, yoffset + ly + minusculeY, color[i % 2]);
          //        else
          //          drawPixel(curX, yoffset+ly, BLACK);
        }
      }
      /*
      else
      {
        sprintf(zl, "pos: %d, curX: %d, lx:%d, i:%d ", pos, curX, lx, i);
        Serial.println(zl);      
      }
    */
    }
  }
  //Serial.println("End");
}





/*
void scroll8(const char *text, uint8_t lx, uint8_t ly, uint32_t color[2])
{
  uint16_t size;
  static int16_t pos = -31;
  static bool pos_init = false;
  static int8_t delta = 1;
  static int width = WIDTH - lx;



  if (pos_init == false)
  {
    size = strlen(text)*letter_width;
    if ( size < WIDTH) // no scroll
    {
      pos = -(WIDTH - size) / 2;
      pos_init = true;
      delta = 0;
    }
    else // scroll
    {
      pos = -2 * letter_width;
      pos_init = true;
   
    }
  }
    
  printTextPos(text, pos, lx, ly, color);
  //  if(--x > -(size) ) x=0;
  EVERY_N_MILLISECONDS(150) //150
  {

    //sprintf(zl, "Print pos: %d, delta: %d, size %d", pos, delta, size);
    //Serial.println(zl);
    //Serial.println(text);

    pos = pos + delta;
    if (delta == 1 && ((size > width && pos > size - width + letter_width) || (size < width && pos == 0)))
      delta = -1;
    else if (delta == -1 && ((size > width && pos == -letter_width) || (size <= width && pos == -width + size)))
      delta = 1;
  }
}
*/
void setCursor(uint8_t x, uint8_t y)
{
  cursX = x;
  cursY = y;
}

void setColor(uint32_t color[2])
{
  cursCol[0] = color[0];
  cursCol[1] = color[1];
}

bool pos_init = false;

void print(const char *text, uint8_t lx, uint8_t ly, uint32_t color[2])
{
  uint16_t size = strlen(text)*letter_width;  // size of the text in pixel
  static int16_t pos = -31+lx; //Starting position
  static int8_t delta =1 ;  // direction -1 1 0 means no moving

  if (pos_init == false) // 1st call for each text
  {
    size = strlen(text)*letter_width;

    if ( size < (WIDTH - lx) ) // fit on screen without scroll 
    {
      pos = -(WIDTH - size-lx) / 2 ;
//    sprintf(zl, "INIT Print pos: %d, size %d, xoffset %d, %s", pos, size, lx, text);
//    Serial.println(zl);

      delta = 0;
    }
    else 
    {
      pos = -2 * letter_width;
      delta = 1;
    }
    pos_init = true;
  }

  printTextPos(text, pos, lx, ly, color);
  //  if(--x > -(size) ) x=0;
  EVERY_N_MILLISECONDS(150) //150
  {

//    sprintf(zl, "Print pos: %d, delta: %d, size %d, xoffset %d, %s", pos, delta, size, lx, text);
//    Serial.println(zl);

    pos = pos + delta;
    if (delta == 1 && ((size > WIDTH && pos > size - WIDTH + letter_width +lx ) || (size < (WIDTH) && pos >= lx+ letter_width )))
      delta = -1;
    else if (delta == -1 && ((size > WIDTH && pos <= -letter_width)  || (size <= WIDTH && pos == -WIDTH + size - + letter_width )))
      delta = 1;
  }
}

void scroll( const char *text, uint8_t lx, uint8_t ly, uint32_t color[2])
{
 //static char text[] = "Les produits IT les plus demandes en cette periode de confinement force, pour cause de Covid-19, seront-ils encore en rupture de stock fin avril 2020 ? Leurs prix vont-ils encore augmenter ?";
//  static char text[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ,;:!?,;:!?/\éà@à/\ ";

 /*
 char text[100];
 for(int i=0;i<94; i++) text[i]=32+i;
  text[94]=0;
*/
  
  static uint16_t size = strlen(text) * letter_width;
  static int16_t pos =  -2 * letter_width;;

  printTextPos(text, pos , lx, ly, color);
  //  if(--x > -(size) ) x=0;
  EVERY_N_MILLISECONDS(150) //150
  {
    if (pos++ > (size))
      pos = -2 * letter_width;
  }
}

void print(const char *text)
{
  print(text, cursX, cursY, cursCol);
}

void show()
{
  FastLED.show();
}

void showBuffer()
{
  FastLED.show();
}

