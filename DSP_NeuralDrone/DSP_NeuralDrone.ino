// Neural cellular automaton drone generator - requires 250Mhz //

#include "hardware/structs/rosc.h"
#include <I2S.h>
#include <atomic>

#define I2S_DATA_PIN  9
#define I2S_BCLK_PIN  10
#define I2S_BUFFERS   4
#define I2S_WORDS     256
#define SAMPLE_RATE   22050

#define SEPARE_LR     SAMPLE_RATE / 20

#define RADIUS        8
#define INPUT_SIZE    (RADIUS * 2 + 1)
#define HIDDEN_SIZE   4
#define ITERATIONS    6

I2S i2s_output(OUTPUT);

#define AUDIO_BUF_SIZE 4096
volatile float audio_buffer[AUDIO_BUF_SIZE];
volatile uint32_t write_ptr = 0;
uint32_t read_ptr = 0;

std::atomic<float> current_freq(110.0f);
std::atomic<float> current_segment_ms(100.0f);

const float frequencies[] = {73.42f, 110.0f, 146.83f, 164.81f, 196.00f, 220.00f};
const size_t num_frequencies = sizeof(frequencies) / sizeof(frequencies[0]);

struct NeuralDrone {

    float w1[INPUT_SIZE][HIDDEN_SIZE];
    float b1[HIDDEN_SIZE];
    float w2[HIDDEN_SIZE];
    float b2;
    float state[I2S_WORDS];
    float memory[I2S_WORDS];

    void init() {
        for (int i = 0; i < INPUT_SIZE; i++) {
            for (int j = 0; j < HIDDEN_SIZE; j++) w1[i][j] = randomf(-1.0f, 1.0f);
        }
        for (int i = 0; i < HIDDEN_SIZE; i++) b1[i] = randomf(-0.5f, 0.5f);
        for (int i = 0; i < HIDDEN_SIZE; i++) w2[i] = randomf(-1.0f, 1.0f);
        b2 = 0.0f;
        for (int i = 0; i < I2S_WORDS; i++) {
            state[i] = 0.0f;
            memory[i] = 0.0f;
        }
    }

    void mutate(float amount) {
        for (int i = 0; i < INPUT_SIZE; i++) {
            for (int j = 0; j < HIDDEN_SIZE; j++) {
                w1[i][j] += random_normal(amount);
            }
        }
        for (int i = 0; i < HIDDEN_SIZE; i++) {
            w2[i] += random_normal(amount);
        }
    }

    float randomf(float min, float max) {
        return min + (max - min) * ((float)rand() / (float)RAND_MAX);
    }

    float random_normal(float stddev) {
        float u1 = (float)rand() / (float)RAND_MAX;
        float u2 = (float)rand() / (float)RAND_MAX;

        if (u1 < 1e-6f) u1 = 1e-6f;

        return stddev * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    }

};

NeuralDrone drone;

static inline float fast_tanh(float x) { return x / (1.0f + fabsf(x)); }

void generate_sine_segment(float* buf, uint32_t length, float freq, uint32_t phase_offset) {

    float phase_step = (2.0f * M_PI * freq) / (float)SAMPLE_RATE;
    uint32_t fade = (length / 2 < 2205) ? length / 2 : 2205;

    for (uint32_t i = 0; i < length; i++) {
        float t_phase = (phase_offset + i) * phase_step;
        float sample = sinf(t_phase);

        float env = 1.0f;

        if (i < fade) {
            env = (float)i / (float)fade;
        } else if (i > length - fade) {
            env = (float)(length - i) / (float)fade;
        }

        buf[i] = sample * env;
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

    drone.init();

    i2s_output.setFrequency(SAMPLE_RATE);
    i2s_output.setDATA(I2S_DATA_PIN);
    i2s_output.setBCLK(I2S_BCLK_PIN);
    i2s_output.setBitsPerSample(16);
    i2s_output.setBuffers(I2S_BUFFERS, I2S_WORDS);

    i2s_output.begin();

}

void loop() {

    uint32_t available =
        (write_ptr >= read_ptr)
        ? (write_ptr - read_ptr)
        : (AUDIO_BUF_SIZE - read_ptr + write_ptr);

    if (available >= I2S_WORDS) {
        for (uint32_t i = 0; i < I2S_WORDS; i++) {

            float out_sample = audio_buffer[read_ptr];
            read_ptr = (read_ptr + 1) % AUDIO_BUF_SIZE;

            int16_t sampleL = (int16_t)(out_sample * 256.0f);
            uint32_t r_ptr = (read_ptr >= SEPARE_LR) ? (read_ptr - SEPARE_LR) : (AUDIO_BUF_SIZE - SEPARE_LR + read_ptr);
            int16_t sampleR = (int16_t)(audio_buffer[r_ptr] * 256.0f);

            i2s_output.write16(sampleL, sampleR);
            
        }
    }

}

void setup1() {

    seed_random_safe();

}

void loop1() {

    float freq = current_freq.load();
    float segment_ms = current_segment_ms.load();

    uint32_t samples_count =
        (uint32_t)((SAMPLE_RATE * segment_ms) / 1000.0f);

    samples_count =
        ((samples_count + I2S_WORDS - 1) / I2S_WORDS) * I2S_WORDS;

    float* raw_sine =
        (float*)malloc(samples_count * sizeof(float));

    float* processed_out =
        (float*)malloc(samples_count * sizeof(float));

    if (!raw_sine || !processed_out) {
        if (raw_sine) free(raw_sine);
        if (processed_out) free(processed_out);
        return;
    }

    static uint32_t global_phase = 0;

    generate_sine_segment(
        raw_sine,
        samples_count,
        freq,
        global_phase
    );

    global_phase += samples_count;

    float total_activity = 0.0f;
    float total_chaos = 0.0f;

    for (uint32_t chunk = 0; chunk < samples_count; chunk += I2S_WORDS) {

        for (int i = 0; i < I2S_WORDS; i++) {
            drone.state[i] =
                raw_sine[chunk + i] * 0.7f +
                drone.memory[i] * 0.3f;
        }

        for (int iter = 0; iter < ITERATIONS; iter++) {

            float next_state[I2S_WORDS];

            for (int i = 0; i < I2S_WORDS; i++) {

                float l1_sum[HIDDEN_SIZE] = {0.0f};

                for (int r = -RADIUS; r <= RADIUS; r++) {

                    int idx = (i + r) % I2S_WORDS;

                    if (idx < 0) idx += I2S_WORDS;

                    float val = drone.state[idx];
                    int input_idx = r + RADIUS;

                    for (int h = 0; h < HIDDEN_SIZE; h++) {
                        l1_sum[h] +=
                            val * drone.w1[input_idx][h];
                    }
                }

                float update = 0.0f;

                for (int h = 0; h < HIDDEN_SIZE; h++) {

                    float l1_activated = fast_tanh(l1_sum[h] + drone.b1[h]);
                    update += l1_activated * drone.w2[h];

                }

                update += drone.b2;

                float res =
                    drone.state[i] + update * 0.35f;

                if (res > 1.0f) res = 1.0f;
                if (res < -1.0f) res = -1.0f;

                next_state[i] = res;
            }

            for (int i = 0; i < I2S_WORDS; i++) {
                drone.state[i] = next_state[i];
            }
        }

        float chunk_activity = 0.0f;

        for (int i = 0; i < I2S_WORDS; i++) {
            drone.memory[i] = drone.state[i];

            processed_out[chunk + i] = drone.state[i];

            chunk_activity += fabsf(drone.state[i]);
        }

        chunk_activity /= I2S_WORDS;

        total_activity += chunk_activity;

        float chunk_chaos = 0.0f;

        for (int i = 0; i < I2S_WORDS; i++) {
            float diff = drone.state[i] - chunk_activity;
            chunk_chaos += diff * diff;
        }

        total_chaos += sqrtf(chunk_chaos / I2S_WORDS);
    }

    total_activity /= (samples_count / I2S_WORDS);
    total_chaos /= (samples_count / I2S_WORDS);

    for (uint32_t i = 0; i < samples_count; i++) {

        while (((write_ptr + 1) % AUDIO_BUF_SIZE) == read_ptr) {
            delayMicroseconds(10);
        }

        audio_buffer[write_ptr] = processed_out[i];

        write_ptr = (write_ptr + 1) % AUDIO_BUF_SIZE;
    }

    int freq_idx = (int)(total_chaos * 10.0f) % num_frequencies;

    current_freq.store(frequencies[freq_idx]);

    float next_ms = 50.0f + (1.0f - total_activity) * 250.0f;

    current_segment_ms.store(next_ms);

    drone.mutate(0.01f);

    free(raw_sine);
    free(processed_out);

}