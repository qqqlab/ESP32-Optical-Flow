/*======================================================================================
Demo program to process data from ESP32-Camera Optical Flow Sensor
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

#include "Wire.h"

class OpticalFlow {
public:
  uint8_t i2c_addr = 0x55;
  float resolution_dpp = 1;
  float framerate_s = 0;
  float dx = 0;
  float dy = 0;

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

  bool setup(TwoWire *i2c) {
    _i2c = i2c;
    _i2c->beginTransmission(i2c_addr);
    _i2c->write(REG_WAI1);
    _i2c->endTransmission(false);
    _i2c->requestFrom(i2c_addr, (uint8_t)6);
    if(_i2c->read() != 0x6F) return false; //REG_WAI1
    if(_i2c->read() != 0x66) return false; //REG_WAI2
    resolution_dpp = (_i2c->read()*256 + _i2c->read()) / 65536.0; // REG_RES_H,REG_RES_L
    framerate_s = (_i2c->read()*256 + _i2c->read()) / 65536.0;    //REG_RATE_H,REG_RATE_L
    return true;
  }

  bool update() {
    int8_t dxnew, dynew;
    if(!read(&dxnew, &dynew)) return false;
    dx = resolution_dpp * dxnew;
    dy = resolution_dpp * dynew;
    return true;
  }

  bool read(int8_t* dx, int8_t* dy){
    _i2c->beginTransmission(i2c_addr);
    _i2c->write(REG_DX);
    _i2c->endTransmission(false);
    _i2c->requestFrom(i2c_addr, (uint8_t)3);
    *dx = _i2c->read(); //REG_DX
    *dy = _i2c->read(); //REG_DY
    uint8_t cnt = _i2c->read(); //REG_CNT
    return (cnt>0);
  }

private:
  TwoWire *_i2c;
};

OpticalFlow of;

void setup() {
  Serial.begin(115200);

  //Note: need to call Wire.begin() before of.setup()
  Wire.begin(); //use default wire pins, this should work on any Arduino board
  //Wire.begin(4, 13, 1000000); //set sda,scl,frequency (use this to test with another M5Stack Timer Cam X)

  while(!of.setup(&Wire)) {
    Serial.println("Optical Flow sensor not found."); 
    delay(1000); 
  }
}

void loop() {
  static uint32_t ts = micros();
  if(of.update()) {
    uint32_t tsnew = micros();
    Serial.printf("dt:%d rate:%f res:%f dx:%f dy:%f\n", tsnew-ts, of.framerate_s, of.resolution_dpp, of.dx, of.dy);
    ts = tsnew;
  }
}
