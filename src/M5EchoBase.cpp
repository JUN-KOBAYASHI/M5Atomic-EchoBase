#include "M5EchoBase.h"
#include "es8311.h"
#include <esp_log.h>

#define TAG "EchoBase"

// PI4IOE5V6408
#define PI4IOE_ADDR           0x43
#define PI4IOE_REG_CTRL       0x00
#define PI4IOE_REG_IO_DIR     0x03
#define PI4IOE_REG_IO_OUT     0x05
#define PI4IOE_REG_IO_PP      0x07
#define PI4IOE_REG_IO_PULLUP  0x0D

// ES8311
#define ES8311_ADDR 0x18

bool M5EchoBase::init(int sample_rate,
                      int i2c_sda,
                      int i2c_scl,
                      int i2s_di,
                      int i2s_ws,
                      int i2s_do,
                      int i2s_bck,
                      TwoWire& wire) {
  _wire = &wire;

  _i2c_sda = i2c_sda;
  _i2c_scl = i2c_scl;
  _i2s_di  = i2s_di;
  _i2s_ws  = i2s_ws;
  _i2s_do  = i2s_do;
  _i2s_bck = i2s_bck;

  _wire->begin(_i2c_sda, _i2c_scl, 100000U);
  ESP_LOGI(TAG, "I2C initialized sda=%d scl=%d", _i2c_sda, _i2c_scl);

  if (!i2s_driver_init(sample_rate)) {
    ESP_LOGE(TAG, "I2S init failed");
    return false;
  }

  if (!es8311_codec_init(sample_rate)) {
    ESP_LOGE(TAG, "ES8311 init failed");
    return false;
  }

  if (!pi4ioe_init()) {
    ESP_LOGE(TAG, "PI4IOE init failed");
    return false;
  }

  setMicGain(ES8311_MIC_GAIN_0DB);
  return true;
}

bool M5EchoBase::es8311_codec_init(int sample_rate) {
  es8311_set_twowire(_wire);
  es_handle = es8311_create((i2c_port_t)0, ES8311_ADDR);  // ポート番号は未使用
  if (!es_handle) {
    ESP_LOGE(TAG, "es8311_create failed");
    return false;
  }

  es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = false,
    .mclk_frequency = 0,
    .sample_frequency = sample_rate,
  };

  if (es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_32, ES8311_RESOLUTION_32) != ESP_OK) {
    ESP_LOGE(TAG, "es8311_init failed");
    return false;
  }

  if (es8311_voice_volume_set(es_handle, 50, NULL) != ESP_OK) {
    ESP_LOGE(TAG, "es8311_voice_volume_set failed");
    return false;
  }

  if (es8311_microphone_config(es_handle, false) != ESP_OK) {
    ESP_LOGE(TAG, "es8311_microphone_config failed");
    return false;
  }

  ESP_LOGI(TAG, "ES8311 initialized sample_rate=%d", sample_rate);
  return true;
}

bool M5EchoBase::i2s_driver_init(int sample_rate) {
#if USE_NEW_I2S_API
  ESP_LOGI(TAG, "Using NEW I2S API");

  // 念のため begin 前にピン指定
  bool pin_ok = I2S.setPins(_i2s_bck, _i2s_ws, _i2s_do, _i2s_di, -1);
  if (!pin_ok) {
    ESP_LOGE(TAG, "I2S.setPins failed");
    return false;
  }

  // EchoBase は録音/再生とも使うので TX/RX を持つ標準I2Sとして開始
  bool begin_ok = I2S.begin(I2S_MODE_STD, sample_rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  if (!begin_ok) {
    ESP_LOGE(TAG, "I2S.begin failed");
    return false;
  }

  ESP_LOGI(TAG, "NEW I2S init ok rate=%d", sample_rate);
  return true;
#else
  ESP_LOGI(TAG, "Using LEGACY I2S API");

  i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  i2s_cfg.sample_rate = (uint32_t)sample_rate;
  i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_cfg.dma_buf_count = 8;
  i2s_cfg.dma_buf_len = 256;
  i2s_cfg.use_apll = false;
  i2s_cfg.tx_desc_auto_clear = false;
  i2s_cfg.fixed_mclk = 0;

  i2s_pin_cfg.mck_io_num = -1;
  i2s_pin_cfg.bck_io_num = _i2s_bck;
  i2s_pin_cfg.ws_io_num = _i2s_ws;
  i2s_pin_cfg.data_out_num = _i2s_do;
  i2s_pin_cfg.data_in_num = _i2s_di;

  if (i2s_driver_install(i2s_num, &i2s_cfg, 0, NULL) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_driver_install failed");
    return false;
  }
  if (i2s_set_pin(i2s_num, &i2s_pin_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_set_pin failed");
    return false;
  }

  i2s_zero_dma_buffer(i2s_num);
  i2s_start(i2s_num);

  ESP_LOGI(TAG, "LEGACY I2S init ok rate=%d", sample_rate);
  return true;
#endif
}

bool M5EchoBase::setSpeakerVolume(int volume) {
  if (volume < 0 || volume > 100) {
    ESP_LOGE(TAG, "speaker volume out of range");
    return false;
  }
  return es8311_voice_volume_set(es_handle, volume, NULL) == ESP_OK;
}

bool M5EchoBase::setMicGain(es8311_mic_gain_t gain) {
  return es8311_microphone_gain_set(es_handle, gain) == ESP_OK;
}

bool M5EchoBase::setMicPGAGain(bool digital_mic, uint8_t pga_gain) {
  return es8311_microphone_pgagain_config(es_handle, digital_mic, pga_gain) == ESP_OK;
}

bool M5EchoBase::setMicAdcVolume(uint8_t volume) {
  if (volume > 100) {
    ESP_LOGE(TAG, "adc volume out of range");
    return false;
  }
  return es8311_set_adc_volume(es_handle, volume) == ESP_OK;
}

uint8_t M5EchoBase::wire_read_byte(uint8_t i2c_addr, uint8_t reg_addr) {
  _wire->beginTransmission(i2c_addr);
  _wire->write(reg_addr);
  _wire->endTransmission(false);
  _wire->requestFrom(i2c_addr, (uint8_t)1);
  if (_wire->available()) {
    return _wire->read();
  }
  return 0xFF;
}

void M5EchoBase::wire_write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t value) {
  _wire->beginTransmission(i2c_addr);
  _wire->write(reg_addr);
  _wire->write(value);
  _wire->endTransmission();
}

bool M5EchoBase::pi4ioe_init() {
  wire_read_byte(PI4IOE_ADDR, PI4IOE_REG_CTRL);
  wire_write_byte(PI4IOE_ADDR, PI4IOE_REG_IO_PP, 0x00);
  wire_read_byte(PI4IOE_ADDR, PI4IOE_REG_IO_PP);

  wire_write_byte(PI4IOE_ADDR, PI4IOE_REG_IO_PULLUP, 0xFF);
  wire_write_byte(PI4IOE_ADDR, PI4IOE_REG_IO_DIR, 0x6F);
  wire_read_byte(PI4IOE_ADDR, PI4IOE_REG_IO_DIR);

  wire_write_byte(PI4IOE_ADDR, PI4IOE_REG_IO_OUT, 0xFF);
  wire_read_byte(PI4IOE_ADDR, PI4IOE_REG_IO_OUT);

  return true;
}

bool M5EchoBase::setMute(bool mute) {
  wire_write_byte(PI4IOE_ADDR, PI4IOE_REG_IO_OUT, mute ? 0x00 : 0xFF);
  return true;
}

int M5EchoBase::getBufferSize(int duration, int sample_rate) {
  if (!sample_rate) {
#if USE_NEW_I2S_API
    sample_rate = I2S.txSampleRate();
#else
    sample_rate = i2s_cfg.sample_rate;
#endif
  }
  return duration * sample_rate * 2 * 2;
}

int M5EchoBase::getDuration(int size, int sample_rate) {
  if (!sample_rate) {
#if USE_NEW_I2S_API
    sample_rate = I2S.txSampleRate();
#else
    sample_rate = i2s_cfg.sample_rate;
#endif
  }
  return size / (sample_rate * 2 * 2);
}

bool M5EchoBase::record(FS& fs, const char* filename, int size) {
  File file = fs.open(filename, FILE_WRITE);
  if (!file) {
    ESP_LOGE(TAG, "open record file failed");
    return false;
  }

  const size_t CHUNK_SIZE = 1024;
  uint8_t buffer[CHUNK_SIZE];
  size_t total = size;
  size_t done = 0;

  while (done < total) {
    size_t want = CHUNK_SIZE;
    if (done + CHUNK_SIZE > total) {
      want = total - done;
    }

    size_t got = 0;
#if USE_NEW_I2S_API
    got = I2S.readBytes((char*)buffer, want);
    if ((int)got < 0) {
      file.close();
      ESP_LOGE(TAG, "I2S.readBytes failed");
      return false;
    }
#else
    if (i2s_read(i2s_num, buffer, want, &got, portMAX_DELAY) != ESP_OK) {
      file.close();
      ESP_LOGE(TAG, "i2s_read failed");
      return false;
    }
#endif
    file.write(buffer, got);
    done += got;
  }

  file.close();
  return true;
}

bool M5EchoBase::record(uint8_t* buffer, int size) {
  size_t got = 0;
#if USE_NEW_I2S_API
  got = I2S.readBytes((char*)buffer, size);
  if ((int)got < 0) {
    ESP_LOGE(TAG, "I2S.readBytes failed");
    return false;
  }
#else
  if (i2s_read(i2s_num, buffer, size, &got, getDuration(size) * 1000 + 1000) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_read failed");
    return false;
  }
#endif
  return true;
}

bool M5EchoBase::play(FS& fs, const char* filename) {
  File file = fs.open(filename, FILE_READ);
  if (!file) {
    ESP_LOGE(TAG, "open play file failed");
    return false;
  }

  const size_t CHUNK_SIZE = 1024;
  uint8_t buffer[CHUNK_SIZE];

  while (file.available()) {
    size_t want = CHUNK_SIZE;
    if (file.available() < (int)CHUNK_SIZE) {
      want = file.available();
    }

    size_t got = file.read(buffer, want);
    if (got > 0) {
      size_t written = 0;
#if USE_NEW_I2S_API
      written = I2S.write(buffer, got);
      if ((int)written < 0) {
        file.close();
        ESP_LOGE(TAG, "I2S.write failed");
        return false;
      }
#else
      if (i2s_write(i2s_num, buffer, got, &written, portMAX_DELAY) != ESP_OK) {
        file.close();
        ESP_LOGE(TAG, "i2s_write failed");
        return false;
      }
#endif
    }
  }

  file.close();
  return true;
}

bool M5EchoBase::play(const uint8_t* buffer, int size) {
  size_t written = 0;
#if USE_NEW_I2S_API
  written = I2S.write(buffer, size);
  if ((int)written < 0) {
    ESP_LOGE(TAG, "I2S.write failed");
    return false;
  }
#else
  if (i2s_write(i2s_num, buffer, size, &written, portMAX_DELAY) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_write failed");
    return false;
  }
  i2s_zero_dma_buffer(i2s_num);
#endif
  return true;
}