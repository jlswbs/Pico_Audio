// Chrono cavity percussion system //

#include "hardware/structs/rosc.h"
#include <I2S.h>

#define I2S_DATA_PIN  9
#define I2S_BCLK_PIN  10
#define I2S_BUFFERS   4
#define I2S_WORDS     64

#define SAMPLE_RATE   44100
#define BPM           120

I2S i2s_output(OUTPUT);

int16_t left_buffer[I2S_WORDS];
int16_t right_buffer[I2S_WORDS];

float frequency = 220.0f;
float density   = 0.1f;
float memory    = 0.3f;
float topo      = 0.3f;
float bloom     = 0.3f;
float decay     = 0.75f;

float envelope  = 0.0f;

class ChronoCavitySystem {
private:
    float sample_rate;
    float cells[16];
    float phase;
    float ring_phases[3];
    float ring_ratios[3];

public:
    void init(float sr) {
      
        sample_rate = sr;
        phase = 0.0f;

        for (int i = 0; i < 16; i++) {
            cells[i] = 0.0f;
        }
        for (int i = 0; i < 3; i++) {
            ring_phases[i] = 0.0f;
        }

        ring_ratios[0] = 1.414f;
        ring_ratios[1] = 2.718f;
        ring_ratios[2] = 4.669f;

    }

    float process_sample(float freq, float dens, float mem, float tp, float blm) {

        phase += freq / sample_rate;
        if (phase >= 1.0f) {
            phase -= 1.0f;
        }

        float virtual_index = phase * 16.0f;
        int idx_floor = (int)virtual_index;
        int idx_ceil = (idx_floor + 1) & 15;
        float frac = virtual_index - (float)idx_floor;

        float cell_charge = (1.0f - frac) * cells[idx_floor] + frac * cells[idx_ceil];

        if (tp > 0.1f) {
            int cross_idx = (idx_floor + 8) & 15;
            cell_charge += cells[cross_idx] * tp * 0.4f;
        }

        float fundamental_sine = sinf(phase * 2.0f * M_PI);

        float raw_output = sinf((fundamental_sine + cell_charge) * M_PI * (1.0f + dens * 1.5f));

        float ring_output = 0.0f;
        float ring_drive = raw_output * blm * 0.35f;

        for (int i = 0; i < 3; i++) {
            float ring_freq = freq * ring_ratios[i];
            ring_phases[i] += ring_freq / sample_rate;
            if (ring_phases[i] >= 1.0f) {
                ring_phases[i] -= 1.0f;
            }

            ring_output += sinf(ring_phases[i] * 2.0f * M_PI) * ring_drive;
        }

        float final_sample = raw_output + ring_output;

        float feedback_amount = mem * 0.65f;
        cells[idx_floor] = (cells[idx_floor] * (1.0f - feedback_amount)) + (final_sample * feedback_amount);

        envelope *= (1.0f - (0.00005f + (1.0f - decay) * 0.002f));

        return tanhf(final_sample * envelope * 0.25f);
    }

};

ChronoCavitySystem synth;

static inline uint32_t read_from_rosc() {

  uint32_t random = 0;
  volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

  for (int k = 0; k < 32; k++) {
    uint32_t bit = 0;
    while (1) {
      bit = (*rnd_reg) & 1;
      if (bit != ((*rnd_reg) & 1)) break;
    }
    random = (random << 1) | bit;
  }

  return random;

}

void seed_random_safe() {

  uint32_t seed = 0;

  for (int i = 0; i < 8; i++) {
    seed ^= read_from_rosc();
    delayMicroseconds(100);
  }

  srand(seed);
  randomSeed(seed);

}

void setup() {

  seed_random_safe();

  synth.init(SAMPLE_RATE);

  i2s_output.setFrequency(SAMPLE_RATE);
  i2s_output.setDATA(I2S_DATA_PIN);
  i2s_output.setBCLK(I2S_BCLK_PIN);
  i2s_output.setBitsPerSample(16);
  i2s_output.setBuffers(I2S_BUFFERS, I2S_WORDS);
  i2s_output.begin();

}

void loop() {

  for (uint32_t n = 0; n < I2S_WORDS; n++) {

    float val = synth.process_sample(frequency, density, memory, topo, bloom);
    int16_t sample = (int16_t)(val * 4096.0f);

    left_buffer[n] = sample;
    right_buffer[n] = sample;

    i2s_output.write16(right_buffer[n], left_buffer[n]);
  }

}

void setup1() {

  seed_random_safe();

}

void loop1() {

  frequency = random(55, 880);
  envelope = 1.0f;

  int tempo_ms = 60000 / BPM;
  delay(tempo_ms / 3);

}