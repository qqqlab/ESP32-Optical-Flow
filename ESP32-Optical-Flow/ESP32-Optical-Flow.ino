/*======================================================================================
ESP32-Camera Optical Flow Sensor

Uses 3 frame buffers: one buffer grabs latest camera image while at the same time two 
previous buffers are used for flow calculation

This program uses a slightly modified Adaptive Rood Pattern Search for Block Matching. 
See https://en.wikipedia.org/wiki/Block-matching_algorithm

ESP32 single core at 240MHz does 12M block matching pixel operation per second, 
approx. 3.5ms for 240x176
========================================================================================
MIT License

Copyright (c) 2024 qqqlab https://github.com/qqqlab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
======================================================================================*/


//======================================================================================
// CONFIG
//======================================================================================

// Config for OV3660 M5Stack Timer Camera X (in Arduino IDE select board: M5Stack-Timer-CAM)

#define LED_PIN 2
//#define BUTTON_PIN 37 //not used

//----------------------
// I2C interface config
//----------------------
#define OF_I2C_SDA_PIN 4
#define OF_I2C_SCL_PIN 13
#define OF_I2C_ADDR 0x55

//----------------------
// Block matching config
//----------------------
#define BM_RESOLUTION_DPP 0.22 //Resolution in degrees/pixel (dpp) - Measured: 0.22 ddp, Specs: FOV 66.5 deg diagonal, QXGA:2048x1535 -> 2560 pix dia -> 0.026 dpp; HQVGA 240x176 ~= 1/8 QXGA -> 0.21 dpp
#define BM_FRAMERATE_S 0.040 //Frame rate in seconds - 40ms / 25 fps
#define BM_PMAX 15 //maximum block match search distance
#define BM_BSTEP 2 //step size through block

//----------------------
// Camera config
//----------------------
//#define XCLK_FREQ_HZ 27000000 //OV3660 datasheet: allowed input freq 6-27MHz, XCLK=27Mhz HQVGA dt=33ms 30fps but with dropped frames "E (6328) cam_hal: FB-SIZE: 38400 != 42240"
#define XCLK_FREQ_HZ 22500000 //OV3660 datasheet: allowed input freq 6-27MHz, XCLK=22.5Mhz HQVGA dt=40ms 25fps works fine

//#define FRAME_SIZE FRAMESIZE_QCIF    // 176x144  = 25344 pixels
#define FRAME_SIZE FRAMESIZE_HQVGA     // 240x176  = 42240 -> fb_count=3 ok, 
//#define FRAME_SIZE FRAMESIZE_240X240 // 240x240  = 57600 -> fb_count=2 ok, fb_count=3 fails

//CAMERA_MODEL_M5STACK_PSRAM - OV3660 M5Stack Timer Camera X
#define CAM_PIN_PWDN    -1 
#define CAM_PIN_RESET   15 
#define CAM_PIN_XCLK    27
#define CAM_PIN_SIOD    25
#define CAM_PIN_SIOC    23
#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3       5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      35
#define CAM_PIN_D0      32
#define CAM_PIN_VSYNC   22
#define CAM_PIN_HREF    26
#define CAM_PIN_PCLK    21

/*
//WROVER-KIT PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    21
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      19
#define CAM_PIN_D2      18
#define CAM_PIN_D1       5
#define CAM_PIN_D0       4
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22
*/



//=================================================================================
// CAMERA
//=================================================================================
#include "esp_camera.h"

#define TAG ""

/* driver/include/sensor.h
typedef enum {
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_YUV422,    // 2BPP/YUV422
    PIXFORMAT_YUV420,    // 1.5BPP/YUV420
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
    PIXFORMAT_RGB888,    // 3BPP/RGB888
    PIXFORMAT_RAW,       // RAW
    PIXFORMAT_RGB444,    // 3BP2P/RGB444
    PIXFORMAT_RGB555,    // 3BP2P/RGB555
} pixformat_t;

typedef enum {
    FRAMESIZE_96X96,    // 96x96    =  9216 pixels
    FRAMESIZE_QQVGA,    // 160x120  = 19200
    FRAMESIZE_QCIF,     // 176x144  = 25344
    FRAMESIZE_HQVGA,    // 240x176  = 42240
    FRAMESIZE_240X240,  // 240x240  = 57600
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    // 3MP Sensors
    FRAMESIZE_FHD,      // 1920x1080
    FRAMESIZE_P_HD,     //  720x1280
    FRAMESIZE_P_3MP,    //  864x1536
    FRAMESIZE_QXGA,     // 2048x1536
    // 5MP Sensors
    FRAMESIZE_QHD,      // 2560x1440
    FRAMESIZE_WQXGA,    // 2560x1600
    FRAMESIZE_P_FHD,    // 1080x1920
    FRAMESIZE_QSXGA,    // 2560x1920
    FRAMESIZE_INVALID
} framesize_t;
*/

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = XCLK_FREQ_HZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAME_SIZE, 

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 3, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_DRAM,
    .grab_mode = CAMERA_GRAB_LATEST, //Sets when buffers should be filled

    .sccb_i2c_port = 0,  // If pin_sccb_sda is -1, use the already configured I2C bus by number    
};

int of_dx = 0; //accumulated dx value
int of_dy = 0; //accumulated dy value
int of_cnt = 0; //number of frames processed


camera_fb_t * fb_last = NULL;
//sensor_t * esp_camera_sensor_get(void);

esp_err_t camera_init(){
    //power up the camera if PWDN pin is defined
    if(CAM_PIN_PWDN != -1){
        pinMode(CAM_PIN_PWDN, OUTPUT);
        digitalWrite(CAM_PIN_PWDN, LOW);
    }

    //initialize the camera
    uint32_t free0 = ESP.getFreeHeap();  //esp_get_free_heap_size();
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    Serial.printf("esp_camera_init allocated %d bytes (%d per fb), free=%d\n", free0-ESP.getFreeHeap(), (free0-ESP.getFreeHeap()) / camera_config.fb_count, ESP.getFreeHeap());

    //get initial frame buffer
    fb_last = esp_camera_fb_get();

    //warm up
    for(int i=0; i<10; i++) {
      camera_capture();
    }
    of_dx = 0;
    of_dy = 0;
    of_cnt = 0;

    return ESP_OK;
}

esp_err_t camera_capture(){
  //get next frame
  camera_fb_t * fb = esp_camera_fb_get(); 
  if (!fb) {
    ESP_LOGE(TAG, "Camera Capture Failed");
    return ESP_FAIL;
  }

  //process image
  process_image(fb->width, fb->height, fb->format, fb->buf, fb->len, fb_last->buf);

  //return the previous frame buffer back to the driver for reuse
  esp_camera_fb_return(fb_last); 

  //make current frame buffer the previous bufffer
  fb_last = fb;
  
  return ESP_OK;
}

void process_image(int w, int h, pixformat_t format, uint8_t *buf, int len, uint8_t *buf_last) {
/*  
    static uint32_t ts = 0;
    static int cnt = -1;
    static uint32_t disp_ts = 0;
    uint32_t now = micros();
    cnt++;
    if(cnt==0) ts = now;
    if(now - disp_ts > 1000000) {
      disp_ts = now;
      uint32_t dt = (cnt ? (now - ts)/cnt : 0);
      float fps = (cnt ? 1.0e6/dt : 0);
      ts = now;
      Serial.printf("%d*%d len:%d dt:%d fps:%.2f\n", w, h, len, dt, fps);
      cnt = -1;
    }
*/

    static uint32_t ts_last = micros();
    uint32_t now = micros();

    if(buf && buf_last) {
      //process_ES(w, h, buf, buf_last);
      process_ARPS(w, h, buf, buf_last);
      //process_9Blocks(w, h, buf, buf_last); 
    }

    Serial.printf("cam_dt:%d\t", (int)(now - ts_last) );      
    Serial.println();

    ts_last = now;
}

void process_ARPS(int w, int h, uint8_t *buf, uint8_t *buf_last) {
  //Serial.printf("[ARPS] ");
  static int8_t dx = 0;
  static int8_t dy = 0;
  int p = BM_PMAX;
  int bstep = BM_BSTEP;
  int bx = p;
  int by = p;
  int bw = w - 2*p;
  int bh = h - 2*p;

  uint32_t dt = micros();
  bm_ARPS(w, buf_last, buf, bx, by, bw, bh, bstep, p, &dx, &dy); //use previous frame dx,dy as guess for current frame dx,dy
  dt = micros() - dt;
  Serial.printf("of_dx:%d\tof_dy:%d\tof_cnt:%d\t", (int)of_dx, (int)of_dy, (int)of_cnt );      
  Serial.printf("dx:%d\tdy:%d\t", dx, dy);
  //Serial.printf("MAV:%d\t", bm_MAV(w, buf, bx, by, bw, bh, bstep) );
  //Serial.printf("buf1:%08X\tbuf2:%08X\t", (int)buf_last, (int)buf );
  //Serial.printf("w:%d\t:h:%d\tlen:%d\t", w, h, len);
  Serial.printf("arps_dt:%d\t", dt);

  of_dx += dx;
  of_dy += dy;
  of_cnt++;
}


void process_ES(int w, int h, uint8_t *buf, uint8_t *buf_last) {
  Serial.printf("[ES] ");
  int8_t dx,dy;
  int p = BM_PMAX;
  int bstep = BM_BSTEP;
  int bx = p;
  int by = p;
  int bw = w - 2*p;
  int bh = h - 2*p;

  if(p>6) p=6; //restict p to keep things running

  uint32_t dt = micros();
  bm_ES(w, buf_last, buf, bx, by, bw, bh, bstep, p, &dx, &dy);
  dt = micros() - dt;
  //Serial.printf("of_dx:%d\tof_dy:%d\tof_cnt:%d\t", (int)of_dx, (int)of_dy, (int)of_cnt );      
  Serial.printf("dx:%d\tdy:%d\t", dx, dy);
  //Serial.printf("buf1:%08X\tbuf2:%08X\t", (int)buf_last, (int)buf );
  //Serial.printf("w:%d\t:h:%d\tlen:%d\t", w, h, len);
  Serial.printf("Mpix/s:%.2f\t", (w-2*p) * (h-2*p) * (p*2+1) * (p*2+1) / (float)dt / bstep / bstep );     
  Serial.printf("es_dt:%d\t", dt);

  //of_dx += dx;
  //of_dy += dy;
  //of_cnt++;
}

//run ARPS on each 1/9th part of the frame
void process_9Blocks(int w, int h, uint8_t *buf, uint8_t *buf_last) {
  uint32_t dt = micros();
  float dxsum = 0;
  float dysum = 0;

  int8_t dx,dy;
  int p = BM_PMAX;
  int bstep = BM_BSTEP;

  int bw = (w - 2*p)/3;
  int bh = (h - 2*p)/3;


  for(uint16_t j=0;j<3;j++) {
    Serial.println();
    for(uint16_t i=0;i<3;i++) {
      int bx = p + i*bw;
      int by = p + j*bh;
      bm_ARPS(w, buf_last, buf, bx, by, bw, bh, bstep, p, &dx, &dy);
      dxsum += dx;
      dysum += dy;
      Serial.printf("%+d,%+d,%d\t", dx, dy, bm_MAV(w, buf, bx, by, bw, bh, bstep));
    }
    
  }
  dxsum /= 9;
  dysum /= 9;   
  dt = micros() - dt;      
  Serial.printf("dx:%+.2f dy:%+.2f dt2:%d\n\n", dxsum, dysum, dt);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Optical Flow\n");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  i2c_setup();

  while(camera_init() != ESP_OK) {
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100); 
    digitalWrite(LED_PIN, HIGH);
  }

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  camera_capture();
}

//=================================================================================
// BLOCK MATCHING ALGORITHM
//=================================================================================

//Exhaustive Search
void bm_ES(uint16_t w, uint8_t* buf1, uint8_t* buf2, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep, int8_t p, int8_t* dx_io, int8_t* dy_io) {
	uint32_t min_sum = 4000000000;
	int min_dx = -99;
	int min_dy = -99;
	for(int dy=-p;dy<=p;dy++) {
		for(int dx=-p;dx<=p;dx++) {
			uint32_t sum = bm_SAD(w, buf1, buf2, bx, by, bw, bh, bstep, dx, dy);
			if(min_sum > sum) {
				min_sum = sum;
				min_dx = dx;
				min_dy = dy;
			}
      //Serial.printf("%9d ",sum);
		}
    //Serial.println();
	}
			
	*dx_io = min_dx;
	*dy_io = min_dy;

  //Serial.printf("min:%d\t",min_sum);
}


bool bm_done[2*(BM_PMAX)+1][2*(BM_PMAX)+1];

//helper macro to check dx,dy: marks point as done in bm_done, and updates minimum
#define BM_CHECK(dx, dy) \
if(-p<=dx && dx<=p && -p<=dy && dy<=p && !bm_done[p+dx][p+dy]) { \
  bm_done[p+dx][p+dy] = true; \
  sum = bm_SAD(w, buf1, buf2, bx, by, bw, bh, bstep, dx, dy); \
  /*Serial.printf("\nSAD:%d %d %d %c\t",dx,dy,sum,(min_sum > sum ? '*' : ' '));*/ \
  if(min_sum > sum) { \
    min_sum = sum; \
    min_dx = dx; \
    min_dy = dy; \
  } \
}

//Adaptive Rood Pattern Search - dx_io,dy_io is input of intinial guess, and output of found dx,dy
void bm_ARPS(uint16_t w, uint8_t* buf1, uint8_t* buf2, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep, int8_t p, int8_t* dx_io, int8_t* dy_io) {
  memset(bm_done,0,sizeof(bm_done));

	int8_t min_dx = 0;
	int8_t min_dy = 0;
  uint32_t min_sum = 4000000000;
  uint32_t sum;
  int8_t current_dx;
  int8_t current_dy; 

  //limit p to pmax
  if(p > BM_PMAX) p = BM_PMAX;

  //constrain dx_io,dy_io
  if(*dx_io < -p) *dx_io = -p;
  if(*dx_io > p) *dx_io = p;
  if(*dy_io < -p) *dy_io = -p;
  if(*dy_io > p) *dy_io = p;

  //initial guess S = max(abs(dx,dy))
  int S = abs(*dx_io);
  if(S < abs(*dy_io)) S = abs(*dy_io);
  //if(S > p) S = p; //this is already done

  //start with minimum step 2
  if(S<2) S=2;

  //check previous minimum (addition to ARPS)
  BM_CHECK( *dx_io, *dy_io);

  //check center
  BM_CHECK( 0, 0);

  //4 point "+" at S from center
  BM_CHECK(+S, 0);
  BM_CHECK(-S, 0);
  BM_CHECK( 0,+S);
  BM_CHECK( 0,-S);
 
  //4 point "x" at S from center (addition to ARPS)
  //BM_CHECK(-S, -S);
  //BM_CHECK(-S, +S);
  //BM_CHECK(+S, -S);
  //BM_CHECK(+S, +S);

  //search with S=1 around minimum until center stays minimum
  do {
    current_dx = min_dx;
    current_dy = min_dy;

    //4 point "+" around current minimum
    BM_CHECK(current_dx + 1, current_dy);
    BM_CHECK(current_dx - 1, current_dy);
    BM_CHECK(current_dx    , current_dy + 1);
    BM_CHECK(current_dx    , current_dy - 1);

    //4 point "x" around current minimum (addition to ARPS)
    //BM_CHECK(current_dx - 1, current_dy - 1);
    //BM_CHECK(current_dx - 1, current_dy + 1);
    //BM_CHECK(current_dx + 1, current_dy - 1);
    //BM_CHECK(current_dx + 1, current_dy + 1);
  } while(current_dx != min_dx || current_dy != min_dy);

	*dx_io = min_dx;
	*dy_io = min_dy;

  //Serial.printf("min:%d\t",min_sum);  
}


//Sum Absolute Differences, b..,dx,dy relative to buf1
uint32_t bm_SAD(uint16_t w, uint8_t* buf1, uint8_t* buf2, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep, int8_t dx, int8_t dy) {	
  uint32_t sum = 0;
  for(int y=by; y<by+bh; y+=bstep) {
    int i1 = y*w + bx;
    int i2 = (y+dy)*w + bx+dx;
    for(int x=0; x<bw; x+=bstep) {
      sum += abs((int16_t)buf1[i1] - (int16_t)buf2[i2]);
      i1 += bstep;
      i2 += bstep;   
    }
  }
  return sum;
}


//Mean Absolute Variance: sum(abs(pix - avg(pix)))/n
uint32_t bm_MAV(uint16_t w, uint8_t* buf1, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep) {	
  int jmin = by*w + bx; //start pixel
  int jmax = jmin + bh*w; //end pixel
  int n = (bw/bstep) * (bh/bstep); //number of pixels

  //calc average
  uint32_t sum = 0;
  for(int j=jmin; j<jmax; j+=bstep*w) {
    for(int i=j; i<j+bw; i+=bstep) {
      sum += buf1[i];
    }
  }
  uint8_t avg = (sum + n / 2) / n;

  //calc sum
  sum = 0;
  for(int j=jmin; j<jmax; j+=w) {
    for(int i=j; i<j+bw; i++) {
      sum += abs(buf1[i]-avg);
    }
  }
  return (sum + n / 2) / n;
}


//=================================================================================
// I2C
//=================================================================================
#include <Wire.h>

enum i2c_reg_e {
  REG_DX = 0,  //x-flow in pixels, reading DX copies DX,DY,CNT to shadow registers
  REG_DY = 1,  //y-flow in pixels
  REG_CNT = 2, //number of samples, reading CNT resets DX,DY,CNT

  REG_WAI1 = 0x80, //who am i -> 0x6F 'o' 
  REG_WAI2 = 0x81, //who am i -> 0x66 'f' 
  REG_RES_H = 0x82, //resolution in degrees per pixel / 65536
  REG_RES_L = 0x83,
  REG_RATE_H = 0x84, //frame rate in seconds / 65536
  REG_RATE_L = 0x85,
};

uint8_t i2c_reg = 0;

int8_t int8_saturate(int v) {
  if(v>127) return 127;
  if(v<-128) return -128;
  return v;
}

uint8_t uint8_saturate(int v) {
  if(v>255) return 255;
  if(v<0) return 0;
  return v;
}

void i2c_RxHandler(int numBytes)
{
  uint8_t idx = 0;
  while(Wire.available()) {
    uint8_t b = Wire.read();
    if(idx == 0) {
      i2c_reg = b;
    }else{
      switch( (i2c_reg_e)i2c_reg ) {
        default:
          break;
      } 
    }
    idx++;
  }
}

void i2c_TxHandler(void)
{
  //shadow register values
  static int dx = 0;
  static int dy = 0;
  static int cnt = 0;
  switch( (i2c_reg_e)i2c_reg ) {      
    case REG_DX:
      //load shadow values
      dx = of_dx;
      dy = of_dy;
      cnt = of_cnt;
      Wire.write(int8_saturate(dx));
      //fall thru
    case REG_DY:
      Wire.write(int8_saturate(dy));
      //fall thru
    case REG_CNT:
      Wire.write(uint8_saturate(cnt));
      //update optical flow
      of_dx -= dx;
      of_dy -= dy;
      of_cnt -= cnt;
      break;
    case REG_WAI1:
      Wire.write('o');
      //fall thru
    case REG_WAI2:
      Wire.write('f');
      //fall thru
    case REG_RES_H:
      Wire.write( (uint8_t) ((uint32_t)((BM_RESOLUTION_DPP)*65536+0.5) >> 8) );
      //fall thru
    case REG_RES_L:
      Wire.write( (uint8_t) ((uint32_t)((BM_RESOLUTION_DPP)*65536+0.5) & 0xff) );
      //fall thru
    case REG_RATE_H:
      Wire.write( (uint8_t) ((uint32_t)((BM_FRAMERATE_S)*65536+0.5) >> 8) );
      //fall thru
    case REG_RATE_L:
      Wire.write( (uint8_t) ((uint32_t)((BM_FRAMERATE_S)*65536+0.5) & 0xff) );
      break;     
    default:
      break;       
  }
  //i2c_reg++; //don't increment reg as we don't know how many bytes were written to master
}

void i2c_setup() {
  Wire.begin(OF_I2C_ADDR, OF_I2C_SDA_PIN, OF_I2C_SCL_PIN, 1000000);
  Wire.onRequest(i2c_TxHandler);
  Wire.onReceive(i2c_RxHandler);
}
