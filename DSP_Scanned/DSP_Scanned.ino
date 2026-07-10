// Scanned synthesis model //

#include "hardware/structs/rosc.h"
#include <I2S.h>

#define I2S_DATA_PIN  9
#define I2S_BCLK_PIN  10
#define I2S_BUFFERS   4
#define I2S_WORDS     64
#define SAMPLE_RATE   44100
#define BPM           135

I2S i2s_output(OUTPUT);

int16_t left_buffer[I2S_WORDS];
int16_t right_buffer[I2S_WORDS];

#define STRING_SIZE 32
#define LUT_SIZE 512

float pos[STRING_SIZE];
float vel[STRING_SIZE];
float pos_left[STRING_SIZE];
float pos_right[STRING_SIZE];

float tension = 0.05f;
float damping = 0.0003f;

float scan_phase = 0.0f;
float scan_phase_step = 0.0f;

float envelope_gain = 0.0f;
bool note_active = false;

int sample_counter = 0;
int attack_samples = 0;
int release_samples = 0;
int sustain_samples = 0;
float envelope = 1.0f;

float lut_sin[LUT_SIZE];

float get_sin_lut(float phase) {
    while (phase < 0.0f) phase += 2.0f * M_PI;
    while (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;

    float index_float = (phase / (2.0f * M_PI)) * LUT_SIZE;
    int idx_low = (int)index_float % LUT_SIZE;
    int idx_high = (idx_low + 1) % LUT_SIZE;
    float frac = index_float - idx_low;

    return (1.0f - frac) * lut_sin[idx_low] + frac * lut_sin[idx_high];
}

void trigger_note(float frequency) {
    scan_phase = 0.0f;
    scan_phase_step = (2.0f * M_PI * frequency) / (float)SAMPLE_RATE;

    for (int i = 0; i < STRING_SIZE; i++) {
        float angle = (float)i / (float)(STRING_SIZE - 1) * M_PI;
        pos[i] = sinf(angle);
        vel[i] = 0.0f;
    }

    envelope_gain = 1.0f;
    note_active = true;
    sample_counter = 0;

    attack_samples = (int)(SAMPLE_RATE * 0.002);
    release_samples = (int)(SAMPLE_RATE * 0.5);
    sustain_samples = (int)(SAMPLE_RATE * 0.1);
}

float voice_sample_rate() {
    if (!note_active) return 0.0f;

    for (int i = 0; i < STRING_SIZE; i++) {
        if (i == 0) {
            pos_left[i] = 0.0f;
            pos_right[i] = pos[1];
        } else if (i == STRING_SIZE - 1) {
            pos_left[i] = pos[STRING_SIZE - 2];
            pos_right[i] = 0.0f;
        } else {
            pos_left[i] = pos[i - 1];
            pos_right[i] = pos[i + 1];
        }
    }

    float force[STRING_SIZE];
    for (int i = 0; i < STRING_SIZE; i++) {
        force[i] = tension * (pos_left[i] - 2.0f * pos[i] + pos_right[i]);
    }

    force[0] = 0.0f;
    force[STRING_SIZE - 1] = 0.0f;

    for (int i = 0; i < STRING_SIZE; i++) {
        vel[i] += force[i];
        vel[i] *= (1.0f - damping);
        pos[i] += vel[i];
    }

    float scan_sin = get_sin_lut(scan_phase);
    float scan_index = (STRING_SIZE - 1) * (0.5f + 0.5f * scan_sin);

    int idx_low = (int)scan_index;
    int idx_high = idx_low + 1;
    if (idx_high >= STRING_SIZE) idx_high = STRING_SIZE - 1;
    float frac = scan_index - idx_low;

    float raw_audio = (1.0f - frac) * pos[idx_low] + frac * pos[idx_high];

    scan_phase += scan_phase_step;
    if (scan_phase >= 2.0f * M_PI) {
        scan_phase -= 2.0f * M_PI;
    }

    float envelope_value = 1.0f;

    if (sample_counter < attack_samples) {
        envelope_value = (float)sample_counter / (float)attack_samples;
    }
    else if (sample_counter < attack_samples + sustain_samples) {
        int sustain_pos = sample_counter - attack_samples;
        if (sustain_pos < (int)(sustain_samples * 0.2f)) {
            float fraction = (float)sustain_pos / (float)(sustain_samples * 0.2f);
            envelope_value = 1.0f - fraction * 0.2f;
        } else {
            envelope_value = 0.8f;
        }
    }
    else {
        int release_pos = sample_counter - (attack_samples + sustain_samples);
        if (release_pos < release_samples) {
            float fraction = (float)release_pos / (float)release_samples;
            envelope_value = 0.8f * (1.0f - fraction);
        } else {
            envelope_value = 0.0f;
            note_active = false;
        }
    }

    sample_counter++;

    return raw_audio * envelope_value;
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

    for (int i = 0; i < LUT_SIZE; i++) {
        lut_sin[i] = sinf(2.0f * M_PI * i / LUT_SIZE);
    }

    seed_random_safe();

    i2s_output.setFrequency(SAMPLE_RATE);
    i2s_output.setDATA(I2S_DATA_PIN);
    i2s_output.setBCLK(I2S_BCLK_PIN);
    i2s_output.setBitsPerSample(16);
    i2s_output.setBuffers(I2S_BUFFERS, I2S_WORDS);

    i2s_output.begin();

}

void loop() {

    for (uint32_t n = 0; n < I2S_WORDS; n++) {

        int16_t sample = (int16_t)(voice_sample_rate() * 8192.0f);

        left_buffer[n] = sample;
        right_buffer[n] = sample;

        i2s_output.write16(right_buffer[n], left_buffer[n]);

    }

}

void setup1() {

    seed_random_safe();

}

void loop1() {

    float target_freq = random(110, 880);

    trigger_note(target_freq);

    float beat_duration = 60.0f / BPM;
    int delay_ms = (int)(beat_duration * 1000.0f);

    delay(delay_ms);

}