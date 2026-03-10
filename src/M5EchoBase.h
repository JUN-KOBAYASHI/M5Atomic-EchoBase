#pragma once
#ifndef M5ECHOBASE_H
#define M5ECHOBASE_H

#include <Arduino.h>
#include <FS.h>
#include <Wire.h>
#include "es8311.h"

#ifdef ESP_IDF_VERSION
  #if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    #define USE_NEW_I2S_API 1
    #include <ESP_I2S.h>
  #else
    #define USE_NEW_I2S_API 0
    #include "driver/i2s.h"
  #endif
#else
  #define USE_NEW_I2S_API 0
#endif

class M5EchoBase {
 public:
#if USE_NEW_I2S_API
  M5EchoBase() : es_handle(nullptr) {}
#else
  M5EchoBase(i2s_port_t i2s_num = I2S_NUM_0) : es_handle(nullptr), i2s_num(i2s_num) {}
#endif

  ~M5EchoBase() {}

  bool init(int sample_rate = 16000,
            int i2c_sda = GPIO_NUM_38,
            int i2c_scl = GPIO_NUM_39,
            int i2s_di  = GPIO_NUM_7,
            int i2s_ws  = GPIO_NUM_6,
            int i2s_do  = GPIO_NUM_5,
            int i2s_bck = GPIO_NUM_8,
            TwoWire& wire = Wire);

  bool setSpeakerVolume(int volume);
  bool setMicGain(es8311_mic_gain_t gain);
  bool setMicPGAGain(bool digital_mic, uint8_t pga_gain);
  bool setMicAdcVolume(uint8_t volume);
  bool setMute(bool mute);

  int getBufferSize(int duration, int sample_rate = 0);
  int getDuration(int size, int sample_rate = 0);

  bool record(FS& fs, const char* filename, int size);
  bool record(uint8_t* buffer, int size);

  bool play(FS& fs, const char* filename);
  bool play(const uint8_t* buffer, int size);

 private:
  es8311_handle_t es_handle;

#if USE_NEW_I2S_API
  I2SClass I2S;
#else
  i2s_port_t i2s_num;
  i2s_config_t i2s_cfg;
  i2s_pin_config_t i2s_pin_cfg;
#endif

  int _i2c_sda = -1;
  int _i2c_scl = -1;
  int _i2s_di  = -1;
  int _i2s_ws  = -1;
  int _i2s_do  = -1;
  int _i2s_bck = -1;

  TwoWire* _wire = nullptr;

  bool es8311_codec_init(int sample_rate);
  bool i2s_driver_init(int sample_rate);
  bool pi4ioe_init();

  uint8_t wire_read_byte(uint8_t i2c_addr, uint8_t reg_addr);
  void wire_write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value);
};

#endif