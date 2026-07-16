// IFFT spectral gong engine //

#include "hardware/structs/rosc.h"
#include <I2S.h>

#define I2S_DATA_PIN  9
#define I2S_BCLK_PIN  10
#define I2S_BUFFERS   4
#define SAMPLE_RATE   44100

I2S i2s_output(OUTPUT);

#define LOG2_N 7  
#define N (1 << LOG2_N)

#define I2S_WORDS N

unsigned long last_trig = 0;
const unsigned long trig_interval = 2500; 

unsigned long last_decay = 0;
const unsigned long decay_interval = 1;

inline uint16_t reverse_bits(uint16_t x, uint8_t bits) {

    uint16_t rev = 0;
    for (uint8_t i = 0; i < bits; i++) {
        if (x & (1 << i)) {
            rev |= (1 << ((bits - 1) - i));
        }
    }
    return rev;

}

static int16_t real_q15[N];
static int16_t imag_q15[N];
static int16_t audio_buffer[N];
int sample_index = N;

const float metal_frequencies[] = {0.018f, 0.035f, 0.058f, 0.072f, 0.095f, 0.125f, 0.168f, 0.205f, 0.255f, 0.310f};
const int num_bins = sizeof(metal_frequencies) / sizeof(metal_frequencies[0]);

volatile int16_t bin_amplitudes[N / 2] = {0};
float phase_accumulator[N / 2] = {0};

void get_twiddle(int k, int16_t* c, int16_t* s) {

    float angle = (2.0f * M_PI * (float)k) / (float)N;
    *c = (int16_t)(cosf(angle) * 32767.0f);
    *s = (int16_t)(sinf(angle) * 32767.0f);

}

void ifft_agnostic(int16_t* re, int16_t* im) {

    for (int i = 0; i < N; i++) {
        int j = reverse_bits(i, LOG2_N);
        if (i < j) {
            int16_t tmp = re[i]; re[i] = re[j]; re[j] = tmp;
            tmp = im[i]; im[i] = im[j]; im[j] = tmp;
        }
    }

    for (int step = 1; step < N; step <<= 1) {
        int jump = step << 1;
        int delta = N / jump;
        for (int group = 0; group < step; group++) {
            int16_t w_re, w_im;
            get_twiddle(group * delta, &w_re, &w_im);
            for (int pair = group; pair < N; pair += jump) {
                int match = pair + step;
                int32_t tr = ((int32_t)re[match] * w_re - (int32_t)im[match] * w_im) >> 15;
                int32_t ti = ((int32_t)re[match] * w_im + (int32_t)im[match] * w_re) >> 15;
                re[match] = (re[pair] - tr) >> 1;
                im[match] = (im[pair] - ti) >> 1;
                re[pair] = (re[pair] + tr) >> 1;
                im[pair] = (im[pair] + ti) >> 1;
            }
        }
    }

}

void generate_spectral_gong() {

    for (int i = 0; i < N; i++) {
        real_q15[i] = 0;
        imag_q15[i] = 0;
    }

    for (int i = 0; i < num_bins; i++) {
        int bin = (int)(metal_frequencies[i] * (float)N);
        
        if (bin > 0 && bin < N / 2) { 
            int16_t amp = bin_amplitudes[bin];

            if (amp > 10) {
                phase_accumulator[bin] += (2.0f * M_PI * (float)bin);
                if (phase_accumulator[bin] > 2.0f * M_PI) {
                    phase_accumulator[bin] = fmodf(phase_accumulator[bin], 2.0f * M_PI);
                }

                int16_t cos_p = (int16_t)(cosf(phase_accumulator[bin]) * 32767.0f);
                int16_t sin_p = (int16_t)(sinf(phase_accumulator[bin]) * 32767.0f);

                real_q15[bin] = ((int32_t)amp * cos_p) >> 15;
                imag_q15[bin] = ((int32_t)amp * sin_p) >> 15;

                real_q15[N - bin] = real_q15[bin];
                imag_q15[N - bin] = -imag_q15[bin];
            }
        }
    }

    ifft_agnostic(real_q15, imag_q15);
    uint8_t gain_shift = LOG2_N - 4; 

    for (int i = 0; i < N; i++) {
        audio_buffer[i] = real_q15[i] << gain_shift; 
    }

}

static inline uint32_t read_from_rosc() {
    uint32_t random_val = 0;
    volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

    for (int k = 0; k < 32; k++) {
        random_val = (random_val << 1) | ((*rnd_reg) & 1);
    }
    return random_val;
}

void seed_random_safe() {
    uint32_t seed = 0;
    for (int i = 0; i < 8; i++) {
        seed ^= read_from_rosc();
        delayMicroseconds(100);
    }
    srand(seed);
}

void setup() {

    seed_random_safe();

    i2s_output.setFrequency(SAMPLE_RATE);
    i2s_output.setDATA(I2S_DATA_PIN);
    i2s_output.setBCLK(I2S_BCLK_PIN);
    i2s_output.setBitsPerSample(16);
    i2s_output.setBuffers(I2S_BUFFERS, I2S_WORDS);
    i2s_output.begin();

}

void loop() {

    if (sample_index >= N) {
        generate_spectral_gong();
        sample_index = 0;
    }

    int16_t sample = audio_buffer[sample_index];

    while (!i2s_output.write16(sample, sample)) {}

    sample_index++;

}

void setup1() {

    seed_random_safe();

}

void loop1() {

    unsigned long now = millis();

    if (now - last_trig >= trig_interval) {
        last_trig = now;
        int32_t strike_energy = random(4000, 10000); 

        for (int i = 0; i < num_bins; i++) {
            int bin = (int)(metal_frequencies[i] * (float)N);
            if (bin > 0 && bin < N / 2) {
                bin_amplitudes[bin] = strike_energy / (1 + i); 
            }
        }
    }

    if (now - last_decay >= decay_interval) {
        last_decay = now;

        for (int bin = 0; bin < N / 2; bin++) {
            int16_t current_amp = bin_amplitudes[bin];

            if (current_amp > 0) {
                int16_t decay_step = 1 + (bin / 2); 

                if (current_amp <= decay_step) {
                    bin_amplitudes[bin] = 0;
                } else {
                    bin_amplitudes[bin] = current_amp - decay_step;
                }
            }
        }
    }

}