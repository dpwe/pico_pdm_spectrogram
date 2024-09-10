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
const uint16_t COLOR_MAP[256] = {
    0x400a, 0x402a, 0x402b, 0x402b, 0x404b, 0x404b, 0x484b, 0x486b, 0x486c,
    0x488c, 0x488c, 0x488c, 0x48ac, 0x48ac, 0x48ad, 0x48cd, 0x48cd, 0x48cd,
    0x48ed, 0x48ed, 0x48ee, 0x490e, 0x490e, 0x490e, 0x492e, 0x492e, 0x492e,
    0x494e, 0x494f, 0x494f, 0x494f, 0x496f, 0x496f, 0x496f, 0x498f, 0x498f,
    0x498f, 0x41af, 0x41b0, 0x41b0, 0x41b0, 0x41d0, 0x41d0, 0x41d0, 0x41f0,
    0x41f0, 0x41f0, 0x4210, 0x4210, 0x4210, 0x4210, 0x4230, 0x4230, 0x4231,
    0x4251, 0x4251, 0x4251, 0x4251, 0x3a71, 0x3a71, 0x3a71, 0x3a91, 0x3a91,
    0x3a91, 0x3a91, 0x3ab1, 0x3ab1, 0x3ab1, 0x3ab1, 0x3ad1, 0x3ad1, 0x3ad1,
    0x3ad1, 0x3af1, 0x3af1, 0x32f1, 0x32f1, 0x3311, 0x3311, 0x3311, 0x3311,
    0x3331, 0x3331, 0x3331, 0x3331, 0x3351, 0x3351, 0x3351, 0x3351, 0x3371,
    0x3371, 0x3371, 0x3371, 0x3391, 0x2b91, 0x2b91, 0x2b91, 0x2bb1, 0x2bb1,
    0x2bb1, 0x2bb1, 0x2bb1, 0x2bd1, 0x2bd1, 0x2bd1, 0x2bd1, 0x2bf1, 0x2bf1,
    0x2bf1, 0x2bf1, 0x2c11, 0x2c11, 0x2c11, 0x2c11, 0x2c11, 0x2431, 0x2431,
    0x2431, 0x2431, 0x2451, 0x2451, 0x2451, 0x2451, 0x2471, 0x2471, 0x2471,
    0x2471, 0x2471, 0x2491, 0x2491, 0x2491, 0x2491, 0x24b1, 0x24b1, 0x24b1,
    0x24b1, 0x24d1, 0x24d1, 0x24d1, 0x24d1, 0x24f1, 0x24f1, 0x24f1, 0x24f1,
    0x24f1, 0x2510, 0x2510, 0x2510, 0x2510, 0x2530, 0x2530, 0x2530, 0x2530,
    0x2530, 0x2550, 0x2550, 0x2550, 0x2d50, 0x2d70, 0x2d70, 0x2d70, 0x2d6f,
    0x2d8f, 0x2d8f, 0x2d8f, 0x358f, 0x358f, 0x35af, 0x35af, 0x35af, 0x35af,
    0x3daf, 0x3dce, 0x3dce, 0x3dce, 0x3dce, 0x45ee, 0x45ee, 0x45ee, 0x45ee,
    0x45ee, 0x4e0d, 0x4e0d, 0x4e0d, 0x4e0d, 0x560d, 0x562d, 0x562d, 0x562c,
    0x5e2c, 0x5e2c, 0x5e4c, 0x5e4c, 0x664c, 0x664c, 0x664b, 0x6e4b, 0x6e6b,
    0x6e6b, 0x6e6b, 0x766b, 0x766a, 0x766a, 0x7e8a, 0x7e8a, 0x7e8a, 0x7e89,
    0x8689, 0x8689, 0x86a9, 0x8ea9, 0x8ea9, 0x8ea8, 0x96a8, 0x96a8, 0x96a8,
    0x96a8, 0x9ec7, 0x9ec7, 0x9ec7, 0xa6c7, 0xa6c7, 0xa6c6, 0xaec6, 0xaec6,
    0xaee6, 0xb6e5, 0xb6e5, 0xb6e5, 0xbee5, 0xbee5, 0xbee4, 0xc6e4, 0xc6e4,
    0xc6e4, 0xcee4, 0xcf04, 0xcf03, 0xd703, 0xd703, 0xd703, 0xdf03, 0xdf03,
    0xdf03, 0xdf03, 0xe703, 0xe703, 0xe703, 0xef23, 0xef23, 0xef23, 0xf724,
    0xf724, 0xf724, 0xf724, 0xff24,
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
  voffset(out_buf, (float32_t)0.02, out_buf, n_out_points);
  arm_clip_f32(out_buf, out_buf, -0.02, 0.02, n_out_points);

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
