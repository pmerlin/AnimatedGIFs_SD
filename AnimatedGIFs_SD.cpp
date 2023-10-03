// please read credits at the bottom of file
#ifndef min
#define min(a, b) (((a) <= (b)) ? (a) : (b))
#endif

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <avr/pgmspace.h> 
#include <arduino.h>
#include <FastLED.h>
#include <WiFiManager.h>

/* #include <Ticker.h> */
#include <Timezone.h>
#include <ArduinoOTA.h>
#include <FS.h>
//#include <LittleFS.h>



#define WIDTH 16
#define HEIGHT 16
#define LZWMAXBITS 10
#define NUM_LEDS (WIDTH * HEIGHT)
#define BORDER_WIDTH 1


//#define FILESYSTEM LittleFS
#define FILESYSTEM SPIFFS

#define LOCK_VERSION       2

//#define USE_PALETTE565

#include "GifDecoder_Impl.h"
#include "LzwDecoder_Impl.h"
//template class GifDecoder<480, 320, 12>;   // .kbv tell the world.
template class GifDecoder<WIDTH, HEIGHT, LZWMAXBITS>;   // .kbv tell the world.
#include "FilenameFunctions.h"    //defines USE_SPIFFS


#define DISPLAY_TIME_SECONDS 30 //
#define NUMBER_FULL_CYCLES     10  //
//#define GIFWIDTH             16  //228 fails on COW_PAINT.  Edit class_implementation.cpp
#define FLASH_SIZE      512*1024  //     


#define BRIGHTNESS 20
#define COLOR_ORDER GRB
#define FAST_LED_CHIPSET WS2811
#define FASTLED_DATA_PIN D6 // (D1)
#define ACCESS_POINT "wemos16x16"

TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       // Central European Standard Time
Timezone CE(CEST, CET);
TimeChangeRule *tcr;

CRGB leds[NUM_LEDS];
CRGB colorsBACK[NUM_LEDS];
unsigned long prevTime;

int backgroundMode = 3; // plasma 1
int timeMode = 0; //1
unsigned long animLength = DISPLAY_TIME_SECONDS; // play each gif for 20 seconds

//uint8_t display_draw_time=50; //10-50 is usually fine
//float isrDelay = 0.002;


#include "fastled-matrix.h"


/*  template parameters are maxGifWidth, maxGifHeight, lzwMaxBits

    The lzwMaxBits value of 12 supports all GIFs, but uses 16kB RAM
    lzwMaxBits can be set to 10 or 11 for small displays, 12 for large displays
    All 32x32-pixel GIFs tested work with 11, most work with 10
*/

GifDecoder<WIDTH, HEIGHT, LZWMAXBITS> decoder;

#define USE_SPIFFS
#define SD_CS D5

#if defined(USE_SPIFFS)
#define GIF_DIRECTORY "/"     //ESP8266 SPIFFS
#define DISKCOLOUR   CYAN
#else
#define GIF_DIRECTORY "/gifs"
//#define GIF_DIRECTORY "/gifs32"
//#define GIF_DIRECTORY "/gifsdbg"
#define DISKCOLOUR   BLUE
#endif


// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

int num_files;
long rowCount;  //.kbv
long plotCount; //.kbv
long skipCount; //.kbv
long lineTime;  //.kbv
int32_t parse_start; //.kbv


#include "gif16.h"

#define M0(x) {x, #x, sizeof(x)}
typedef struct {
    const unsigned char *data;
    const char *name;
    uint32_t sz;
} gif_detail_t;
gif_detail_t gifs[] = {
#if FLASH_SIZE >= 1024 * 1024      //Teensy4.0, ESP32, F767, L476

//    M0(llama_driver_gif),          //758945
    M0(teakettle_128x128x10_gif),  // 21155
    M0(bottom_128x128x17_gif),     // 51775
    M0(globe_rotating_gif),        // 90533

    //    M0(mad_man_gif),               //711166
    M0(mad_race_gif),              //173301
    //    M0(marilyn_240x240_gif),       // 40843
    //    M0(cliff_100x100_gif),   //406564
#elif FLASH_SIZE >= 512 * 1024     // Due, F446, ESP8266
//    M0(teakettle_128x128x10_gif),  // 21155
//    M0(bottom_128x128x17_gif),     // 51775
//    M0(globe_rotating_gif),        // 90533
    //M0(mad_race_gif),              //173301
    //    M0(marilyn_240x240_gif),       // 40843
//    M0(horse_128x96x8_gif),        //  7868
    M0(PiXEL),
    M0(WiFiLogo),
#elif FLASH_SIZE >= 256 * 1024     //Teensy3.2, Zero
    M0(teakettle_128x128x10_gif),  // 21155
    M0(bottom_128x128x17_gif),     // 51775
    //    M0(globe_rotating_gif),        // 90533
    M0(mad_race_gif),              //173301
    M0(horse_128x96x8_gif),        //  7868
#else
    M0(teakettle_128x128x10_gif),  // 21155
    M0(globe_rotating_gif),        // 90533
    M0(bottom_128x128x17_gif),     // 51775
    M0(irish_cows_green_beer_gif), // 29798
    M0(horse_128x96x8_gif),        //  7868
#endif
};




void configModeCallback (WiFiManager *myWiFiManager) {
  // generate QRCode and display it
/*
  QRCode qrcode;
  uint8_t qrcodeBytes[qrcode_getBufferSize(LOCK_VERSION)];
  // 29x29
  qrcode_initText(&qrcode, qrcodeBytes, LOCK_VERSION, ECC_LOW, "WIFI:S:" ACCESS_POINT ";;");
  display.clearDisplay();
  uint8_t d = (WIDTH - qrcode.size) / 2;
  for (uint8_t y = 0; y < qrcode.size; ++y) {
    for (uint8_t x = 0; x < qrcode.size; ++x) {
      if (qrcode_getModule(&qrcode, x, y)) {
        display.drawPixelRGB888(x + d, y + d, 255, 255, 255);
      }
    }
  }
  display.showBuffer();
  */
}

unsigned int NTP_PORT = 2390;      // local port to listen for UDP packets
const char* ntpServerName = "time.google.com";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
WiFiUDP udpNTP;

void sendNTPpacket(IPAddress& address) {
  if (WiFi.isConnected()) {
    byte packetBuffer[ NTP_PACKET_SIZE];
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
      
    udpNTP.beginPacket(address, 123); //NTP requests are to port 123
    udpNTP.write(packetBuffer, NTP_PACKET_SIZE);
    udpNTP.endPacket();
  }
}

const time_t DEFAULT_TIME = 0;

time_t getNtpTime() {
  if (WiFi.isConnected()) {
    IPAddress timeServerIP; // time.nist.gov NTP server address
    WiFi.hostByName(ntpServerName, timeServerIP);
    while (udpNTP.parsePacket() > 0) ; // discard any previously received packets
    sendNTPpacket(timeServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = udpNTP.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
 //       Serial.println("Receive NTP Response");
        byte packetBuffer[ NTP_PACKET_SIZE];
        udpNTP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        return secsSince1900 - 2208988800UL;
      }
    }
  }
//  Serial.println("No NTP Response :-(");
  return DEFAULT_TIME; // return 0 if unable to get the time
}

void initOTA() {
  ArduinoOTA.setHostname("WemosMatrix16");
  ArduinoOTA.setPassword("wemos");
  ArduinoOTA.onStart([]() { 
    clearDisplay();
    showBuffer();
  });
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int realprogress = (progress / (total / 100));
    char buffer[8];
    sprintf(buffer, "%.2d %%", realprogress);
    setCursor(0, 0);
    clearDisplay();
    print(buffer);
    showBuffer();
  });
  ArduinoOTA.onError([](ota_error_t error) {});
  ArduinoOTA.begin();
}

ESP8266WebServer webServer(80);       // Create a webserver object that listens for HTTP request on port 80

String fileList() {
      String str = "";
  Dir dir = FILESYSTEM.openDir("/");
  while (dir.next()) {
    str += dir.fileName();
    str += " -> ";
    str += dir.fileSize();
    str += " b\r\n";
  }
  return str;
}

void sendOk(String text = "Ok") {
  webServer.send(200, "text/plain", text);
}

//unsigned long lastFrameTime = 0;




const uint8_t *g_gif;
uint32_t g_seek;
bool fileSeekCallback_P(unsigned long position) {
    g_seek = position;
    return true;
}

unsigned long filePositionCallback_P(void) {
    return g_seek;
}

int fileReadCallback_P(void) {
    return pgm_read_byte(g_gif + g_seek++);
}

int fileReadBlockCallback_P(void * buffer, int numberOfBytes) {
    memcpy_P(buffer, g_gif + g_seek, numberOfBytes);
    g_seek += numberOfBytes;
    return numberOfBytes; //.kbv
}

void screenClearCallback(void) {
    //    tft.fillRect(0, 0, 128, 128, 0x0000);
}

bool openGifFilenameByIndex_P(const char *dirname, int index)
{
    gif_detail_t *g = &gifs[index];
    g_gif = g->data;
    g_seek = 0;

    Serial.print("Flash: ");
    Serial.print(g->name);
    Serial.print(" size: ");
    Serial.println(g->sz);

    return index < num_files;
}
void updateScreenCallback(void) {
    ;
}

void drawPixelCallback(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue) {
//    tft.drawPixel(x, y, tft.color565(red, green, blue));
    drawPixel(XY(x,y),red, green, blue);
    plotCount++;
    rowCount = 1;
    Serial.print("Pix ");
}

void drawLineCallback24(int16_t x, int16_t y, uint8_t *buf, int16_t w, rgb_24 *palette, int16_t skip) {
    uint8_t pixel;
    bool first;
    int32_t t = micros();
/*
    if (y >= tft.height() || x >= tft.width() ) return;
    if (x + w > tft.width()) w = tft.width() - x;
*/    
    if (w <= 0) return;
    int16_t endx = x + w - 1;
    rgb_24 buf24[w];
    for (int i = 0; i < w; ) {
        int n = 0;
        while (i < w) {
            pixel = buf[i++];
            if (pixel == skip) {
                skipCount++;
                break;
            }
            buf24[n++] = palette[pixel];
        }
        if (n) {
//            tft.setAddrWindow(x + i - n, y, endx, y);
            first = true;
//            tft.pushColors(buf565, n, first);
            for (int x=0; x<n; x++)
              drawPixel(XY(x+i-n,y),buf24[x].red, buf24[x].green, buf24[x].blue);
        }
    }
    plotCount += w;  //count total pixels (including skipped)
    rowCount += 1;   //count number of drawLines
    lineTime += micros() - t;
}


void drawLineCallback565(int16_t x, int16_t y, uint8_t *buf, int16_t w, uint16_t *palette565, int16_t skip) {
    uint8_t pixel;
    bool first;
    int32_t t = micros();
/*
    if (y >= tft.height() || x >= tft.width() ) return;
    if (x + w > tft.width()) w = tft.width() - x;
*/    
    if (w <= 0) return;
    int16_t endx = x + w - 1;
    uint16_t buf565[w];
    for (int i = 0; i < w; ) {
        int n = 0;
        while (i < w) {
            pixel = buf[i++];
            if (pixel == skip) {
                skipCount++;
                Serial.println("Skip");
                break;
            }
            buf565[n++] = palette565[pixel];
        }
        if (n) {
//            tft.setAddrWindow(x + i - n, y, endx, y);
            first = true;
//            tft.pushColors(buf565, n, first);
            for (int x=0; x<n; x++)
            {
                uint8_t r = ((((buf565[x] >> 11) & 0x1F) * 527) + 23) >> 6;
                uint8_t g = ((((buf565[x] >> 5) & 0x3F) * 259) + 33) >> 6;
                uint8_t b = (((buf565[x] & 0x1F) * 527) + 23) >> 6;

                drawPixel(XY(x+i-n,y),r, g, b);
            }
    

        }
    }
    plotCount += w;  //count total pixels (including skipped)
    rowCount += 1;   //count number of drawLines
    lineTime += micros() - t;
//    Serial.print("Lin ");
}

/*
void printCenter(String str, int16_t y) {
#if 0
  int16_t  x1, y1;
  uint16_t w, h;
  display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(((WIDTH - w) / 2) - x1, y);
#else
int16_t x = 4;
#if 1
  display.setTextColor(myWHITE);
 display.setCursor(x - 1, y);
  display.print(str);
  yield();
 display.setCursor(x + 1, y);
  display.print(str);
  yield();
 display.setCursor(x, y - 1);
  display.print(str);
  yield();
display.setCursor(x, y + 1);
  display.print(str);
#endif
yield();
  display.setTextColor(myBLACK);
  display.setCursor(x, y);
  display.print(str);
#endif
}
*/

/*
#if 0
uint16_t XY(uint8_t x, uint8_t y) {
  return y * WIDTH + x;
}
#else
// alternate (zigzag)
uint16_t XY(uint8_t x, uint8_t y) {
  uint16_t i;
  if( y & 0x00) {
      // Odd rows run backwards
      uint8_t reverseX = (WIDTH - 1) - x;
      i = (y * WIDTH) + reverseX;
    } else {
      // Even rows run forwards
      i = (y * WIDTH) + x;
    }
    return i;
}
#endif
*/

void displayFastLED(CRGB* colors) {
    for (uint16_t y = 0; y < HEIGHT; ++y) {
      yield();
      for (uint16_t x = 0; x < WIDTH; ++x) {
        CRGB& color = colors[XY(x, y)];
//        display.drawPixelRGB888(x, y, color.r, color.g, color.b);
//        leds[XY(x, y)]=color.r << 16 | color.g << 8 | color.b;
        leds[XY(x, y)]=colors[XY(x, y)];

      }
    }
}

void displayTime() {
   //display.setTextColor(myWHITE);
    time_t utc = now();
    time_t local = CE.toLocal(utc, &tcr);
    char buffer[4];
    sprintf(buffer, "%.2d", hour(local));
    print(buffer, 0,7,cursCol);
   sprintf(buffer, "%.2d", minute(local));     
    print(buffer, 0,7,cursCol);   
}


uint16_t PlasmaTime = 0;
uint16_t PlasmaShift = (random8(0, 5) * 32) + 64;

#define PLASMA_X_FACTOR     24
#define PLASMA_Y_FACTOR     24
#define TARGET_FRAME_TIME   25  // Desired update rate, though if too many leds it will just run as fast as it can!

uint32_t  LoopDelayMS = TARGET_FRAME_TIME;
unsigned long  LastLoop = millis() - LoopDelayMS;

void backgroundSwirl() {
  if ( (millis() - LastLoop) >= LoopDelayMS) { //abs
    LastLoop = millis();
    uint8_t blurAmount = beatsin8(2,10,255);
    blur2d( colorsBACK, WIDTH, HEIGHT, blurAmount);
   yield();
    // Use two out-of-sync sine waves
    uint8_t  i = beatsin8( 27, BORDER_WIDTH, HEIGHT-BORDER_WIDTH);
    uint8_t  j = beatsin8( 41, BORDER_WIDTH, WIDTH-BORDER_WIDTH);
    // Also calculate some reflections
    uint8_t ni = (WIDTH-1)-i;
    uint8_t nj = (WIDTH-1)-j;
   yield();
    // The color of each point shifts over time, each at a different speed.
    uint16_t ms = millis();  
    colorsBACK[XY( i, j)] += CHSV( ms / 11, 200, 255);
    colorsBACK[XY( j, i)] += CHSV( ms / 13, 200, 255);
    colorsBACK[XY(ni,nj)] += CHSV( ms / 17, 200, 255);
    colorsBACK[XY(nj,ni)] += CHSV( ms / 29, 200, 255);
    colorsBACK[XY( i,nj)] += CHSV( ms / 37, 200, 255);
    colorsBACK[XY(ni, j)] += CHSV( ms / 41, 200, 255);
  }
}

void backgroundPlasma2() {
   if ((millis() - LastLoop) >= LoopDelayMS) {
    LastLoop = millis();
   // Fill background with dim plasma
    for (int16_t y = 0; y < HEIGHT; y++) {
     for (int16_t x = 0; x < WIDTH; x++) {
        yield(); // secure time for the WiFi stack of ESP8266
        int16_t r = sin16(PlasmaTime) / 256;
        int16_t h = sin16(x * r * PLASMA_X_FACTOR + PlasmaTime) + cos16(y * (-r) * PLASMA_Y_FACTOR + PlasmaTime) + sin16(y * x * (cos16(-PlasmaTime) / 256) / 2);
        colorsBACK[XY(x, y)] = CHSV((uint8_t)((h / 256) + 128), 200, 200);
      }
    }
    uint16_t OldPlasmaTime = PlasmaTime;
    PlasmaTime += PlasmaShift;
    if (OldPlasmaTime > PlasmaTime)
      PlasmaShift = (random8(0, 5) * 32) + 64;
  }
}

boolean imageLoaded = false;


void loadNewImage(const char* url) {
  // create file from url
  if (WiFi.isConnected()) {
    WiFiClient wclient;
    HTTPClient client;
    if (client.begin(wclient, url)) {
      if (client.GET() > 0) {
        int len = client.getSize();
//        Serial.printf("size: %d\n", len);
        uint8_t buff[256] = { 0 };       
        File file = FILESYSTEM.open("/image.gif", "w");
        if (!file) {
          Serial.println("There was an error opening the file for writing");
          return;
        } 
        while (client.connected() && (len > 0 || len == -1)) {
          // read up to 128 byte
          int c = wclient.readBytes(buff, std::min((size_t)len, sizeof(buff))); 
 //          Serial.printf("readBytes: %d\n", c);
          if (!c) {
 //           Serial.println("read timeout");
          }
          // write it to File
          file.write(buff, c);

          if (len > 0) {
            len -= c;
          }  
        }
        file.close();
      }  
    }
    client.end();
    Serial.println("Gif Downloaded");
    imageLoaded=true;
  }
  else Serial.println("Wifi not connected");
}

unsigned long backgroundMillis = 0;

void backgroundGIF() {
/*
   if ((millis() - backgroundMillis) >= (animLength * 1000)) { // load new image
         backgroundMillis = millis();
         loadNewImage("http://merlinux.free.fr/gifs/gif2.php");
   }
*/


//   playGIFFrame();

    static unsigned long futureTime, cycle_start, nextFrameTime, frame_time, frames;

    //    int index = random(num_files);
    static int index = -1;

    int32_t now = millis();
    if (now >= futureTime || decoder.getCycleNo() > NUMBER_FULL_CYCLES) {
        //new images
        int good;
        if (1)
        {
            loadNewImage("http://merlinux.free.fr/gifs/gif2.php");
            good= openGifFilenameByName("/image.gif");
        }
        else
        {
            if (g_gif) good = (openGifFilenameByIndex_P(GIF_DIRECTORY, index) >= 0);
            else good = (openGifFilenameByIndex(GIF_DIRECTORY, index) >= 0);

                if (++index >= num_files) {
                index = 0;
                
            }

        }



        char buf[100];
        int32_t frameCount = decoder.getFrameCount();
        if (frameCount > 0) {   //complete animation sequence
            int32_t framedelay = decoder.getFrameDelay_ms();
            int32_t cycle_design = framedelay * frameCount;
            int32_t cycle_time = now - cycle_start;
            int32_t percent = (100 * cycle_design) / cycle_time;
            int32_t skipcent = plotCount ? (100 * skipCount) / (plotCount) : 0;
            int32_t avg_wid = rowCount ? plotCount / rowCount : 0;
            int32_t avg_ht = rowCount / frames;
            percent = (100 * frames * framedelay) / (frame_time / 1000);
            if (percent > 100) percent = 100;
         sprintf(buf, "%3d frames @%3dms %3d%% [%3d] typ: %3dx%-3d ",
                    frameCount, framedelay, percent, frames,
                    avg_wid, avg_ht);
            Serial.print(buf);
         float map = 0.001 / frames;
            char ft[10], dt[10];
            dtostrf(frame_time * map, 5, 1, ft);
            dtostrf(lineTime * map, 5, 1, dt);
            sprintf(buf, "avg:%sms draw:%sms %d%%", ft, dt, skipcent);
            Serial.println(buf);
        }
        skipCount = plotCount = rowCount = lineTime = frames = frame_time = 0L;
        cycle_start = now;
        // Calculate time in the future to terminate animation
        futureTime = now + (DISPLAY_TIME_SECONDS * 1000);



        if (good >= 0) {
            /*
            tft.fillScreen(g_gif ? MAGENTA : DISKCOLOUR);
            tft.fillRect(GIFWIDTH, 0, 1, tft.height(), WHITE);
            tft.fillRect(278, 0, 1, tft.height(), WHITE);
*/
            decoder.startDecoding();

        }
    }

    parse_start = micros();
    decoder.decodeFrame();
    frame_time += micros() - parse_start; //count it even if housekeeping block

    if (decoder.getFrameNo() != 0) {  //don't count the header blocks.
        frames++;
        nextFrameTime = now + decoder.getFrameDelay_ms();
        while (millis() <= nextFrameTime) yield();
    }

    yield();

}














// Setup method runs once, when the sketch starts
void setup() {
    char msg[80];
    Serial.begin(115200);

  #if defined(ESP8266)
	Serial.println();
	Serial.print( F("Heap: ") ); Serial.println(system_get_free_heap_size());
	Serial.print( F("Boot Vers: ") ); Serial.println(system_get_boot_version());
	Serial.print( F("CPU: ") ); Serial.println(system_get_cpu_freq());
	Serial.print( F("SDK: ") ); Serial.println(system_get_sdk_version());
	Serial.print( F("Chip ID: ") ); Serial.println(system_get_chip_id());
	Serial.print( F("Flash ID: ") ); Serial.println(spi_flash_get_id());
	Serial.print( F("Flash Size: ") ); Serial.println(ESP.getFlashChipRealSize());
	Serial.print( F("Vcc: ") ); Serial.println(ESP.getVcc());
	Serial.println();
  #endif
//  fixPal(sprites_pal, sizeof(sprites_pal) / sizeof(CRGB)); //16

    while (!Serial) ;
    Serial.println("\nAnimatedGIFs_SD");
/*    
    tft.begin(tft.readID());
    tft.setRotation(1);
    tft.fillScreen(BLACK);
*/  
    FastLED.addLeds<FAST_LED_CHIPSET, FASTLED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
    FastLED.setBrightness(BRIGHTNESS);

    leds[0] = CRGB(255, 0, 0); //0,1
    leds[1] = CRGB(0, 255, 0); //0,2
    leds[2] = CRGB(0, 255, 0); //0,3
    leds[3] = CRGB(0, 0, 255);
    leds[4] = CRGB(0, 0, 255);
    leds[5] = CRGB(0, 0, 255);
    drawPixel(0,0, RED);
    drawPixel(0,1, GREEN);
    drawPixel(1,1, GREEN);
    drawPixel(0,2, BLUE);
    drawPixel(1,2, BLUE);
    drawPixel(2,2, BLUE);
    FastLED.show(); 
    Serial.println("initmatrix");
    delay(1000);

    setCursor(0, 0);
    setColor(cursCol);
    pos_init == false;
    printTextPos("Wifi..",0,0,0,cursCol);

    FastLED.show();
    Serial.println("Wifi");
    delay(1000);


#if defined(USE_PALETTE565)
    decoder.setDrawLineCallback(drawLineCallback565);
#else
    decoder.setDrawLineCallback(drawLineCallback24);
#endif
    decoder.setScreenClearCallback(screenClearCallback);
    decoder.setUpdateScreenCallback(updateScreenCallback);
    decoder.setDrawPixelCallback(drawPixelCallback);

/*
// Playing Progmem gif
    decoder.setFileSeekCallback(fileSeekCallback_P);
    decoder.setFilePositionCallback(filePositionCallback_P);
    decoder.setFileReadCallback(fileReadCallback_P);
    decoder.setFileReadBlockCallback(fileReadBlockCallback_P);
    g_gif = gifs[0].data;


        decoder.setFileSeekCallback(fileSeekCallback);
        decoder.setFilePositionCallback(filePositionCallback);
        decoder.setFileReadCallback(fileReadCallback);
        decoder.setFileReadBlockCallback(fileReadBlockCallback);

        SPIFFS.begin();
*/





    int ret = initSdCard(SD_CS);
    if (ret == 0) {
        Serial.println("Using ");
        decoder.setFileSeekCallback(fileSeekCallback);
        decoder.setFilePositionCallback(filePositionCallback);
        decoder.setFileReadCallback(fileReadCallback);
        decoder.setFileReadBlockCallback(fileReadBlockCallback);
        num_files = enumerateGIFFiles(GIF_DIRECTORY, true);
    }
    if (ret != 0 || num_files == 0) {
        if (num_files == 0) sprintf(msg, "No GIF files on SD card");
        else sprintf(msg, "No SD card on CS:%d", SD_CS);
        Serial.println(msg);
        decoder.setFileSeekCallback(fileSeekCallback_P);
        decoder.setFilePositionCallback(filePositionCallback_P);
        decoder.setFileReadCallback(fileReadCallback_P);
        decoder.setFileReadBlockCallback(fileReadBlockCallback_P);
        g_gif = gifs[0].data;
        for (num_files = 0; num_files < sizeof(gifs) / sizeof(*gifs); num_files++) {
            Serial.println(gifs[num_files].name);
        }
    }

/*
    openGifFilenameByName("/PiXEL.gif");
    decoder.startDecoding();
    int32_t now = millis();
    while (decoder.getCycleNo() <= 1) {
        static unsigned long futureTime, cycle_start, nextFrameTime, frame_time, frames;
//        char buf[100];
//        int32_t frameCount = decoder.getFrameCount();
//        skipCount = plotCount = rowCount = lineTime = frames = frame_time = 0L;    
        decoder.decodeFrame();     
    
        parse_start = micros();
        decoder.decodeFrame();
        frame_time += micros() - parse_start; //count it even if housekeeping block

        if (decoder.getFrameNo() != 0) {  //don't count the header blocks.
            frames++;
            nextFrameTime = now + decoder.getFrameDelay_ms();
            while (millis() <= nextFrameTime) yield();
        }
        FastLED.show();
        yield();
    }

    Serial.println("End Boot ");
*/

    sprintf(msg, "Animated GIF files Found: %d", num_files);
    if (num_files < 0) sprintf(msg, "No directory on %s", GIF_DIRECTORY);
    if (num_files == 0) sprintf(msg, "No GIFs on %s", GIF_DIRECTORY);
    Serial.println(msg);
//    if (num_files > 0) return;
//    tft.fillScreen(RED);
//    tft.println(msg);

//  FILESYSTEM.begin();
//  gif.begin(LITTLE_ENDIAN_PIXELS);
  /*
  display.begin(16);
  display.setFastUpdate(true);
  display.setBrightness(254);
*/
  Serial.println("Setup Wifi");
  WiFiManager wifiManager;
  // wifiManager.resetSettings();
  // TODO: increase in production
  wifiManager.setConfigPortalTimeout(60);
  wifiManager.setDebugOutput(true);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setMinimumSignalQuality(20);
  wifiManager.autoConnect(ACCESS_POINT);

 if (WiFi.isConnected()) {
    udpNTP.begin(NTP_PORT);
    initOTA();

   webServer.on("/intensity", HTTP_GET, []() {
      String value = webServer.arg("value");
      FastLED.setBrightness(value.toInt());
      sendOk();
    });
   webServer.on("/animlen", HTTP_GET, []() {
      String value = webServer.arg("value");
      animLength = value.toInt();
      sendOk();
    });
   webServer.on("/bmode", HTTP_GET, []() {
      String mode = webServer.arg("value");
      backgroundMode = mode.toInt();
      sendOk();
    });
    webServer.on("/tmode", HTTP_GET, []() {
      String mode = webServer.arg("value");
      timeMode = mode.toInt();
      sendOk();
    });
    webServer.on("/format", HTTP_GET, []() { boolean ret = FILESYSTEM.format(); sendOk(ret ? "Success" : "Failure"); });
    webServer.on("/list", HTTP_GET, []() { sendOk(fileList()); });
    webServer.begin();


    setCursor(0, 0);
//    display.setTextColor(myWHITE);

    prevTime = millis();
    pos_init = false;
    do
    {
      yield();
      fillScreen(0);
      print(WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "No Wifi", 0, 2, cursCol);
      show();
    } while (millis() - prevTime < 5000);



    delay(2000);
    setSyncInterval(3600);
    setSyncProvider(getNtpTime);
    delay(4000);
  }
//  while (1) delay(10);  //does a yield()
}








void loop2() {
    static unsigned long futureTime, cycle_start, nextFrameTime, frame_time, frames;

    //    int index = random(num_files);
    static int index = -1;

    int32_t now = millis();
    if (now >= futureTime || decoder.getCycleNo() > NUMBER_FULL_CYCLES) {
        char buf[100];
        int32_t frameCount = decoder.getFrameCount();
        if (frameCount > 0) {   //complete animation sequence
            int32_t framedelay = decoder.getFrameDelay_ms();
            int32_t cycle_design = framedelay * frameCount;
            int32_t cycle_time = now - cycle_start;
            int32_t percent = (100 * cycle_design) / cycle_time;
            int32_t skipcent = plotCount ? (100 * skipCount) / (plotCount) : 0;
            int32_t avg_wid = rowCount ? plotCount / rowCount : 0;
            int32_t avg_ht = rowCount / frames;
            percent = (100 * frames * framedelay) / (frame_time / 1000);
            if (percent > 100) percent = 100;

            sprintf(buf, "%3d frames @%3dms %3d%% [%3d] typ: %3dx%-3d ",
                    frameCount, framedelay, percent, frames,
                    avg_wid, avg_ht);
            Serial.print(buf);

            float map = 0.001 / frames;
            char ft[10], dt[10];
            dtostrf(frame_time * map, 5, 1, ft);
            dtostrf(lineTime * map, 5, 1, dt);
            sprintf(buf, "avg:%sms draw:%sms %d%%", ft, dt, skipcent);
            Serial.println(buf);
        }
        skipCount = plotCount = rowCount = lineTime = frames = frame_time = 0L;

        cycle_start = now;
        // Calculate time in the future to terminate animation
        futureTime = now + (DISPLAY_TIME_SECONDS * 1000);

        if (++index >= num_files) {
            index = 0;
        }

        int good;
        if (g_gif) good = (openGifFilenameByIndex_P(GIF_DIRECTORY, index) >= 0);
        else good = (openGifFilenameByIndex(GIF_DIRECTORY, index) >= 0);
        if (good >= 0) {
            /*
            tft.fillScreen(g_gif ? MAGENTA : DISKCOLOUR);
            tft.fillRect(GIFWIDTH, 0, 1, tft.height(), WHITE);
            tft.fillRect(278, 0, 1, tft.height(), WHITE);
*/
            decoder.startDecoding();

        }
    }

    parse_start = micros();
    decoder.decodeFrame();
    FastLED.show();
    yield();
    frame_time += micros() - parse_start; //count it even if housekeeping block
    if (decoder.getFrameNo() != 0) {  //don't count the header blocks.
        frames++;
        nextFrameTime = now + decoder.getFrameDelay_ms();
        while (millis() <= nextFrameTime) yield();
    }
    yield();

}



void loop() {
   ArduinoOTA.handle();
   webServer.handleClient();
   switch (backgroundMode) {
    case 0: // blank
      clearDisplay(); 
      break;

     case 1: // plasma
        backgroundPlasma2();
        displayFastLED(colorsBACK);
        break;

      case 2: // fastled anim
         backgroundSwirl();
        displayFastLED(colorsBACK);
        break;

      case 3: // GIF
       backgroundGIF();
//       displayFastLED(colorsBACK);
//       Serial.println("loop");
       break;

       case 4: // uniform color
       fill_solid(colorsBACK, NUM_LEDS, CRGB::HotPink);
        displayFastLED(colorsBACK);
        default:
         break;
      }
      yield();
   if (timeMode == 1) 
      displayTime();
    FastLED.show();
//    delay(20);
}











/*
    Animated GIFs Display Code for SmartMatrix and 32x32 RGB LED Panels

    Uses SmartMatrix Library for Teensy 3.1 written by Louis Beaudoin at pixelmatix.com

    Written by: Craig A. Lindley

    Copyright (c) 2014 Craig A. Lindley
    Refactoring by Louis Beaudoin (Pixelmatix)

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
    This example displays 32x32 GIF animations loaded from a SD Card connected to the Teensy 3.1
    The GIFs can be up to 32 pixels in width and height.
    This code has been tested with 32x32 pixel and 16x16 pixel GIFs, but is optimized for 32x32 pixel GIFs.

    Wiring is on the default Teensy 3.1 SPI pins, and chip select can be on any GPIO,
    set by defining SD_CS in the code below
    Function     | Pin
    DOUT         |  11
    DIN          |  12
    CLK          |  13
    CS (default) |  15

    This code first looks for .gif files in the /gifs/ directory
    (customize below with the GIF_DIRECTORY definition) then plays random GIFs in the directory,
    looping each GIF for DISPLAY_TIME_SECONDS

    This example is meant to give you an idea of how to add GIF playback to your own sketch.
    For a project that adds GIF playback with other features, take a look at
    Light Appliance and Aurora:
    https://github.com/CraigLindley/LightAppliance
    https://github.com/pixelmatix/aurora

    If you find any GIFs that won't play properly, please attach them to a new
    Issue post in the GitHub repo here:
    https://github.com/pixelmatix/AnimatedGIFs/issues
*/

/*
    CONFIGURATION:
    - If you're using SmartLED Shield V4 (or above), uncomment the line that includes <SmartMatrixShieldV4.h>
    - update the "SmartMatrix configuration and memory allocation" section to match the width and height and other configuration of your display
    - Note for 128x32 and 64x64 displays with Teensy 3.2 - need to reduce RAM:
      set kRefreshDepth=24 and kDmaBufferRows=2 or set USB Type: "None" in Arduino,
      decrease refreshRate in setup() to 90 or lower to get good an accurate GIF frame rate
    - Set the chip select pin for your board.  On Teensy 3.5/3.6, the onboard microSD CS pin is "BUILTIN_SDCARD"
*/
