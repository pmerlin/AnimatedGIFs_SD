#include "sprites.h"

void fixPal(CRGB pal[], uint8_t maxPal)
{
  int i;
  uint8_t t;

  for (i = 0; i < maxPal; i++) // 11
  {
    t = pal[i][0];
    pal[i][0] = pal[i][2];
    pal[i][2] = t;
  }
}


void drawSprite(const Sprite &sprite, bool transparent=true)
{
  uint8_t idx, p;

  for (uint8_t y = 0; y < sprite.h; y++)
  {
    for (uint8_t x = 0; x < sprite.w; x++)
    {

      if (sprite.dx >= 0)
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h))* ((sprite.w+1)/2)+   x/2]);
      else
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h + 1))*((sprite.w+1)/2) - 1 - x/2] );

        if (sprite.dx >= 0)
          if (x%2==1)
            p = idx & 0x0F;
          else 
            p = idx >>4; 
        else
          if (x%2==1)
            p = idx >>4;
          else 
            p = idx & 0x0F;
      
      if ((x + sprite.x) < WIDTH && (x + sprite.x) >= 0 && (y + sprite.y) >= 0 && (y + sprite.y) < HEIGHT)
        if(transparent==false || (transparent==true && p) ) leds[XY(x  + sprite.x, y + sprite.y)] = sprite.pal[p];   
    }
    
  }
}

void drawSpriteOnSprite(const Sprite &sprite, const Sprite &sprite2, bool transparent=true, int8_t revDelta=0 )
{
  uint8_t idx, p;
  int8_t delta=0;
  

  for (uint8_t y = 0; y < sprite.h; y++)
  {
    for (uint8_t x = 0; x < sprite.w; x++)
    {

      if (sprite2.dx >= 0)
      {
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h))* ((sprite.w+1)/2)+   x/2]);
      }
      else
      {
        delta=revDelta;
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h + 1))*((sprite.w+1)/2) - 1 - x/2] );
      }

        if (sprite.dx >= 0)
        {
          if (x%2==1)
            p = idx & 0x0F;
          else 
            p = idx >>4; 
        }
        else
        { 
          if (x%2==1)
            p = idx >>4;
          else 
            p = idx & 0x0F;
        }


      if ((x + sprite.x+sprite2.x) < WIDTH && (x + sprite.x+sprite2.x) >= 0 && (y + sprite.y+sprite2.y) >= 0 && (y + sprite.y+sprite2.y) < HEIGHT)
        if(transparent==false || (transparent==true && p) ) leds[XY(x  + sprite.x+sprite2.x+delta, y + sprite.y+sprite2.y)] = sprite.pal[p]; 

    }
  }
}

//template <uint8_t N>
void drawSprite(const Sprite &sprite, int dx, int dy, bool transparent=true)
{
  uint8_t idx, p;

  for (uint8_t y = 0; y < sprite.h; y++)
  {
    for (uint8_t x = 0; x < sprite.w; x++)
    {


      if (sprite.dx >= 0)
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h))* ((sprite.w+1)/2)+   x/2]);
      else
        idx = pgm_read_byte_near( &sprite.data[ (y + (sprite.nb * sprite.h + 1))*((sprite.w+1)/2) - 1 - x/2] );

        if (sprite.dx >= 0)
          if (x%2==1)
            p = idx & 0x0F;
          else 
            p = idx >>4; 
        else
          if (x%2==1)
            p = idx >>4;
          else 
            p = idx & 0x0F;
      
      if ((x + sprite.x+dx) < WIDTH && (x + sprite.x+dx) >= 0 && (y + sprite.y+dy) >= 0 && (y + sprite.y+dy) < HEIGHT)
        if(transparent==false || (transparent==true && p) ) leds[XY(x  + sprite.x+dx, y + sprite.y+dy)] = sprite.pal[p];   
    }
  }
}

//template <uint8_t N>
/*
void setSpritePixel(Sprite &sprite, uint8_t sx, uint8_t sy, uint8_t colidx)
{
  
  uint8_t rx = sx / 2;
  uint16_t index=(sy + (sprite.nb * sprite.h))*sprite.w+   rx;

  if (sx % 2)
  {
    sprite.data[index] = sprite.data[index] & 0xf0;
    sprite.data[index] = sprite.data[index] | colidx;
  }
  else
  {
    sprite.data[index] = sprite.data[index] & 0x0f;
    sprite.data[index] = sprite.data[index] | colidx << 4;
  }
 
}
*/

//template <uint8_t N>
/*
void rotateEye(Sprite &sprite, char eye)
{
  static uint8_t lr = 0, rr = 0;
  uint8_t y, r;

  //  if(r++==5) r=0;

  if (eye == 'l')
  {
    y = 0;
    if (lr++ == 4)
      lr = 0;
    r = lr;
  }
  else if (eye == 'r')
  {
    y = 3;
    if (rr++ == 4)
      rr = 0;
    r = rr;
  }
  else
    return;

  switch (r)
  {
  case 0:
    setSpritePixel(sprite, 2 + y, 4, 0x00);
    setSpritePixel(sprite, 2 + y, 3, 0x0e);
    break;
  case 1:
    setSpritePixel(sprite, 2 + y, 3, 0x00);
    setSpritePixel(sprite, 3 + y, 3, 0x0e);
    break;
  case 2:
    setSpritePixel(sprite, 3 + y, 3, 0x00);
    setSpritePixel(sprite, 3 + y, 4, 0x0e);
    break;
  default:
    setSpritePixel(sprite, 3 + y, 4, 0x00);
    setSpritePixel(sprite, 2 + y, 4, 0x0e);
  }
}

//template <uint8_t N>
void rotateEyes(Sprite &sprite)
{
  rotateEye(sprite, 'l');
  rotateEye(sprite, 'r');
}
*/
//template <uint32_t N> void moveSprite(Sprite<N> &sprite) {
//template <uint8_t N>
void moveSprite(Sprite &sprite)
{
//  if (sprite.dx != REVERSE)  
  sprite.x += sprite.dx;
  sprite.y += sprite.dy;

  switch (sprite.effect)
  {     
    case BOTHWAY:
      sprintf(zl, "Sprite %d/%d\n",sprite.nb, sprite.max );
      Serial.println(zl);

      if(sprite.mode==FWD)
      {
        if (++sprite.nb == sprite.max) 
        {
            sprite.nb--;
            sprite.mode=REV;
        }
            
      }
      else /* REVerse*/
      {
          if (--sprite.nb == 0) sprite.mode=FWD;
      }
      break;

    case SLOW:
      if (++sprite.mode==6)
      {
        sprite.mode=0;
        if (++sprite.nb == sprite.max) sprite.nb = 0;
      }
      break;


    default:
      if (++sprite.nb == sprite.max) sprite.nb = 0;
  }

//  if (sprite.dx != REVERSE) 
  {
    if (sprite.dx > 0 && sprite.x == MAXX)
      sprite.dx = -sprite.dx;
    if (sprite.dx < 0 && sprite.x == MINX)
      sprite.dx = -sprite.dx;
  }
}
