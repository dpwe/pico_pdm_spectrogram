// pico_pdm_spectrogram
//
// Real-time spectrogram display from PDM mic input.
// Using ili9340C 240x320 display
//
// dan.ellis@gmail.com 2024-09-07

// Wiring:
//   Pico
//   GP2   <-- PDM Dout
//   GP3   --> PDM clk
//
//   GP16  <-- ILI9340 MISO (not used)
//   GP17  --> ILI9340 CS
//   GP18  --> ILI9340 CLK
//   GP19  --> ILI9340 MOSI
//   GP20  --> ILI9340 D/C
//   GP21  --> ILI9340 RST


#include "arm_math.h"


// -------- PDM mic input --------
#include <PDM.h>

#define PDM_DATA 2
#define PDM_CLK 3

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[128];

// Number of audio samples read
volatile int samplesRead;

void setup_pdm() {
  PDM.setDIN(PDM_DATA);
  PDM.setCLK(PDM_CLK);
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void buffer_feed(short *samples, int n_samples);

void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;

  buffer_feed(sampleBuffer, samplesRead);
}

// --------- TFT display -------

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 20
#define TFT_CS 17
#define TFT_RST 21

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup_display() {
  Serial.println("ILI9341 initialization"); 
 
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  tft.setTextColor(ILI9341_WHITE);  
  tft.setTextSize(1);

}

// ------- waveform plot -----

const int waveform_x = 0;
const int waveform_w = 320;
const int waveform_y = 0;
const int waveform_h = 240;
const int waveform_mid_y = waveform_y + waveform_h / 2;

const int LAST_POINTS_SIZE = 1024;
short last_points[LAST_POINTS_SIZE];

void setup_waveform() {

  tft.setCursor(0, 0);
  tft.println("Hello World!");

  tft.drawLine(waveform_x, waveform_mid_y,
               waveform_x + waveform_w, waveform_mid_y,
               ILI9341_LIGHTGREY);

  for (int i = 0; i < LAST_POINTS_SIZE; ++i)  last_points[i] = 0;
}

void waveform_display(short *samples, int n_samples, int color) {
  int last_x = waveform_x;
  int last_y = waveform_mid_y;
  int last_old_y = last_y;
  for (int i = 0; i < n_samples; ++i) {
    int x_val = waveform_x + (i << 0);
    // Display shows +/- 120.  Divide samples by 8, so display is ~ +/- 1000.
    int y_val = waveform_mid_y - (samples[i] >> 3);
    int old_y_val = last_points[i];
    if (i > 0) {
      tft.drawLine(last_x, last_old_y, x_val, old_y_val, ILI9341_BLACK);
      tft.drawLine(last_x, last_y, x_val, y_val, color);
    }
    last_x = x_val;
    last_y = y_val;
    last_old_y = old_y_val;
    last_points[i] = y_val;
  }
}

// ------- spectrogram plot -----------

// from https://github.com/ArmDeveloperEcosystem/audio-spectrogram-example-for-pico

// import numpy as np
// import matplotlib as mpl
//
// cmap = mpl.colormaps['magma']
// s = ''
// for i in range(256):
//   r, g, b, a = cmap(i / 256, bytes=True)
//   val565 = ((r & 0xF8) << (11 - 3)) | ((g & 0xFC) << (5 - 2)) | ((b & 0xF8) >> 3)
//   s += '0x%x, ' % val565
//   if (i + 1) % 9 == 0:
//     print(s)
//     s = ''
// print(s)

const uint16_t COLOR_MAP[256] = {
// Magma
0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x2, 
0x2, 0x22, 0x22, 0x23, 0x23, 0x823, 0x823, 0x824, 0x844, 
0x844, 0x845, 0x845, 0x845, 0x1065, 0x1066, 0x1066, 0x1066, 0x1067, 
0x1067, 0x1067, 0x1867, 0x1888, 0x1888, 0x1888, 0x1889, 0x1889, 0x2089, 
0x208a, 0x208a, 0x208a, 0x208a, 0x288b, 0x288b, 0x288b, 0x288c, 0x288c, 
0x308c, 0x308c, 0x308d, 0x306d, 0x306d, 0x386d, 0x386d, 0x386e, 0x386e, 
0x406e, 0x406e, 0x406e, 0x406e, 0x406e, 0x488f, 0x488f, 0x488f, 0x488f, 
0x488f, 0x508f, 0x508f, 0x508f, 0x508f, 0x50af, 0x58af, 0x58af, 0x58af, 
0x58af, 0x58af, 0x60cf, 0x60cf, 0x60cf, 0x60d0, 0x60d0, 0x68d0, 0x68f0, 
0x68f0, 0x68f0, 0x68f0, 0x68f0, 0x70f0, 0x70f0, 0x7110, 0x7110, 0x7110, 
0x7910, 0x7910, 0x7910, 0x7930, 0x7930, 0x8130, 0x8130, 0x8130, 0x8130, 
0x8130, 0x8950, 0x8950, 0x8950, 0x8950, 0x8950, 0x9150, 0x9150, 0x9150, 
0x9170, 0x916f, 0x996f, 0x996f, 0x996f, 0x996f, 0x996f, 0xa16f, 0xa18f, 
0xa18f, 0xa18f, 0xa18f, 0xa98f, 0xa98f, 0xa98f, 0xa9af, 0xb1af, 0xb1af, 
0xb1af, 0xb1af, 0xb1af, 0xb9af, 0xb9af, 0xb9ce, 0xb9ce, 0xb9ce, 0xc1ce, 
0xc1ce, 0xc1ce, 0xc1ee, 0xc1ee, 0xc9ee, 0xc9ee, 0xc9ee, 0xc9ee, 0xca0e, 
0xd20d, 0xd20d, 0xd20d, 0xd20d, 0xd22d, 0xd22d, 0xda2d, 0xda2d, 0xda4d, 
0xda4d, 0xda4c, 0xe24c, 0xe26c, 0xe26c, 0xe26c, 0xe28c, 0xe28c, 0xe28c, 
0xeaac, 0xeaac, 0xeaac, 0xeacb, 0xeacb, 0xeacb, 0xeaeb, 0xeaeb, 0xf30b, 
0xf30b, 0xf30b, 0xf32b, 0xf32b, 0xf34b, 0xf34b, 0xf36b, 0xf36b, 0xf38b, 
0xf38b, 0xf38b, 0xfbab, 0xfbab, 0xfbcb, 0xfbcb, 0xfbeb, 0xfbeb, 0xfc0b, 
0xfc0b, 0xfc2c, 0xfc2c, 0xfc4c, 0xfc4c, 0xfc6c, 0xfc6c, 0xfc8c, 0xfc8c, 
0xfc8c, 0xfcac, 0xfcad, 0xfccd, 0xfccd, 0xfced, 0xfced, 0xfd0d, 0xfd0d, 
0xfd2e, 0xfd2e, 0xfd4e, 0xfd4e, 0xfd6e, 0xfd6e, 0xfd6f, 0xfd8f, 0xfd8f, 
0xfdaf, 0xfdaf, 0xfdcf, 0xfdd0, 0xfdf0, 0xfdf0, 0xfe10, 0xfe10, 0xfe31, 
0xfe31, 0xfe31, 0xfe51, 0xfe51, 0xfe72, 0xfe72, 0xfe92, 0xfe92, 0xfeb2, 
0xfeb3, 0xfed3, 0xfed3, 0xfef3, 0xfef3, 0xfef4, 0xff14, 0xff14, 0xff34, 
0xff35, 0xff55, 0xff55, 0xff75, 0xff76, 0xff96, 0xff96, 0xff96, 0xffb6, 
0xffb7, 0xffd7, 0xffd7, 0xfff7, 
};

int disp_column = 0;

void add_display_column(short *values, int n_values) {
  int x = waveform_x + disp_column;
  for (int i; i < MIN(waveform_h, n_values); ++i) {
    int y = waveform_y + waveform_h - i;
    int v = MIN(255, (MAX(0, values[i] + 1024)) >> 3);  // 0..255.
    //int color = tft.color565(v, v, v);
    int color = COLOR_MAP[v];
    tft.drawPixel(x, y, color);
  }
  disp_column = (disp_column + 1) % waveform_w;
  tft.scrollTo(disp_column);
}


// -------- waveform buffer & trigger selection -------

const int RING_BUFFER_SIZE = 32768;  // Must be 2^n
short ring_buffer[RING_BUFFER_SIZE];
volatile int ring_buffer_tail = 0;

void setup_buffer() {
  for (int i = 0; i < RING_BUFFER_SIZE; ++i) {
    ring_buffer[i] = 0;
  }
}

void buffer_feed(short *samples, int n_samples) {
  for (int i = 0; i < n_samples; ++i) {
    ring_buffer[ring_buffer_tail++] = *samples++;
    ring_buffer_tail &= RING_BUFFER_SIZE - 1;
  }
}

int find_crossing(int min_size, int threshold, bool positive_going) {
  int tail = ring_buffer_tail;  // Snapshot volatile.
  int sign = 1;
  if (positive_going)  sign = -1;  // We're searching backwards, so sign is opposite what you'd expect.
  short last_sample = ring_buffer[(tail - min_size) & (RING_BUFFER_SIZE - 1)];
  for (int i = min_size + 1; i < RING_BUFFER_SIZE; ++i) {
    // Scan backwards, so we find the most recent transition with at least min_size.
    int index = (tail - i) & (RING_BUFFER_SIZE - 1) ;
    short next_sample = ring_buffer[index];
    if (((sign * last_sample) <= threshold) && ((sign * next_sample) > threshold))
      return index;
    last_sample = next_sample;
  }
  // Didn't find a crossing.
  return 0;
}

void buffer_get_latest(short *buffer, int n_samples, int scalebits) {
  // Copy most recent values into output buffer.
  int from = (ring_buffer_tail - n_samples) & (RING_BUFFER_SIZE - 1);
  for (int i = 0; i < n_samples; ++i) {
    *buffer++ = ring_buffer[from++] << scalebits;
    from &= RING_BUFFER_SIZE - 1;
  }
}

// ----- dsp ------
//#define USE_FIXEDPOINT

#ifdef USE_FIXEDPOINT

#define SAMPLE_ q15_t
#define float_to_sample(from, to, len)  arm_float_to_q15(from, to, len)
#define short_to_sample(from, to, len)  bcopy(from, to, len * sizeof(short))
#define sample_to_short(from, to, len)  bcopy(from, to, len * sizeof(short))

#define FFT_STRUCT arm_rfft_instance_q15
#define rfft_init(fft_struct, fft_size)  arm_rfft_init_q15(fft_struct, fft_size, 0, 1)
#define mult(in1, in2, out, len)  arm_mult_q15(in1, in2, out, len)
#define rfft(fft_struct, in, out) arm_rfft_q15(fft_struct, in, out)
#define cmplx_mag(in, out, len)  arm_cmplx_mag_q15(in, out, len)
#define vlog(in, out, len)  /* not implemented */
#define vscale(in, scale, out, len)  /* not implemented */
#define voffset(in, offset, out, len)  /* not implemented */

#else // floating point

#define SAMPLE_ float32_t
#define float_to_sample(from, to, len)  bcopy(from, to, len * sizeof(float32_t))
#define short_to_sample(from, to, len)  arm_q15_to_float(from, to, len)
#define sample_to_short(from, to, len)  arm_float_to_q15(from, to, len)

#define FFT_STRUCT arm_rfft_fast_instance_f32
#define rfft_init(fft_struct, fft_size)  arm_rfft_fast_init_f32	(fft_struct, fft_size)
#define mult(in1, in2, out, len)  arm_mult_f32(in1, in2, out, len)
#define rfft(fft_struct, in, out) arm_rfft_fast_f32(fft_struct, in, out, 0)
#define cmplx_mag(in, out, len)  arm_cmplx_mag_f32(in, out, len)
#define vlog(in, out, len) arm_vlog_f32(in, out, len)
#define vscale(in, scale, out, len) arm_scale_f32(in, scale, out, len)
#define voffset(in, offset, out, len) arm_offset_f32(in, offset, out, len)

#endif

const int FFT_SIZE = 1024;

SAMPLE_ input_buf[FFT_SIZE];
SAMPLE_ window_buf[FFT_SIZE];
SAMPLE_ windowed_input_buf[FFT_SIZE];

FFT_STRUCT S_;

SAMPLE_ fft_buf[FFT_SIZE * 2];
SAMPLE_ fft_mag_buf[FFT_SIZE / 2];

void hanning_window_init(SAMPLE_* window, size_t size) {
    for (size_t i = 0; i < size; i++) {
       float32_t f = 0.5 * (1.0 - arm_cos_f32(2 * PI * i / FFT_SIZE ));

       float_to_sample(&f, &window[i], 1);
    }
}

void setup_dsp() {
  // initialize the hanning window and RFFT instance
  hanning_window_init(window_buf, FFT_SIZE);
  rfft_init(&S_, FFT_SIZE);
}

int magnitude_spectrum(SAMPLE_ *in_buf, SAMPLE_ *out_buf, int len) {
  int n_out_points = FFT_SIZE / 2;
  assert(len >= n_out_points);

  //mult(window_buf, in_buf, out_buf, FFT_SIZE);
  //return FFT_SIZE / 2;

  // apply the DSP pipeline: Hanning Window + FFT
  // remove DC
  SAMPLE_ sum = 0;
  for (int i = 0; i < FFT_SIZE; ++i) sum += in_buf[i];
  voffset(in_buf, -sum / FFT_SIZE, in_buf, FFT_SIZE);
  // window & fft & mag
  mult(window_buf, in_buf, windowed_input_buf, FFT_SIZE);
  rfft(&S_, windowed_input_buf, fft_buf);
  cmplx_mag(fft_buf, out_buf, n_out_points);

  vlog(out_buf, out_buf, n_out_points);
  vscale(out_buf, (float32_t)0.005, out_buf, n_out_points);
  voffset(out_buf, (float32_t)0.01, out_buf, n_out_points);
  arm_clip_f32(out_buf, out_buf, -0.03, 0.03, n_out_points);

  if(0) {
    float32_t min_val = out_buf[0];
    float32_t max_val = out_buf[0];
    for (int i = 0; i < n_out_points; ++i) {
      if (out_buf[i] > max_val) max_val = out_buf[i];
      if (out_buf[i] < min_val) min_val = out_buf[i];
    }
    Serial.print("max_val=");
    Serial.println(max_val);
    Serial.print("min_val=");
    Serial.println(min_val);
  }
  // for debug: short-circuit the output.
  //bcopy(windowed_input_buf, out_buf, n_out_points);

  return n_out_points;
}

// -------- setup & main -----
void setup() {
  Serial.begin(9600);
  while (!Serial);

  setup_pdm();
  setup_display();
  setup_waveform();
  setup_dsp();
}

int total_samples = 0;
int last_report_sample = 0;
const int REPORT_INTERVAL_SAMPLE_S = 16000;

const int display_n_samples = 256;

int mode = 3;  // 0 = text 1 = waveform, 2 = fft, 3 = sgram

const int mag_spec_len = 512;
SAMPLE_ mag_spec[mag_spec_len];
short mag_spec_short[mag_spec_len];
short input_shorts_buf[FFT_SIZE];

void loop() {
  int this_start = find_crossing(display_n_samples, 0, true);
  if (mode == 0) {
    buffer_get_latest(input_shorts_buf, FFT_SIZE, 0);
    int n = 0;
    int sum = 0;
    long int sumsq = 0;
    for (int i = 0; i < FFT_SIZE; ++i) {
      short sample = input_shorts_buf[i];
      sum += sample;
      sumsq += sample * sample;
      ++n;
    }
    Serial.print("mean=");
    Serial.println(sum / n);
    Serial.print("rms=");
    Serial.println(sqrtf(((float)sumsq) / n));
  } else if (mode == 1) {
    buffer_get_latest(input_shorts_buf, FFT_SIZE, 0);
    waveform_display(input_shorts_buf, FFT_SIZE, ILI9341_YELLOW);
  } else if (mode == 2 || mode == 3) {
    buffer_get_latest(input_shorts_buf, FFT_SIZE, /* lshift_bits */ 0);
    short_to_sample(input_shorts_buf, input_buf, FFT_SIZE);
    int npts = magnitude_spectrum(input_buf, mag_spec, mag_spec_len);
    sample_to_short(mag_spec, mag_spec_short, npts);
    if (mode == 2) {
      waveform_display(mag_spec_short, npts, ILI9341_YELLOW);
    } else {
      add_display_column(mag_spec_short, npts);
    }
  }
}
