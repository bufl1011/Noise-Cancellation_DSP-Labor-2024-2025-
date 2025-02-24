#include <global.h>
#include <arm_math.h>
#include <math.h>

// Definitions
#define SAMPLERATE 48000
#define N 512
#define BUFFER_SIZE (2 * N)
#define NUM_NOISE_FRAMES 200 // Anzahl der Rauschframes
#define HOP_SIZE 256      // hop size (overlap) 400


float32_t fft_input[N];
float32_t fft_output[N];
float32_t fft_magnitude[N / 2]; // Spektrale Magnitude
float32_t ifft_output[N];
float32_t P_noise[N / 2] = {0}; // Mittlere Rauschleistung
float32_t P_noisy[N / 2];       // Aktuelle Leistung
float32_t P_clean[N / 2];       // Geschätzte saubere Leistung
double fft_freqs[N / 2];        // Frequenzen
arm_rfft_fast_instance_f32 fft_instance;
float hamming_window[N];

uint32_t audio_ring_buffer[BUFFER_SIZE] = {0};
uint32_t out_ring_buffer[BUFFER_SIZE]={0};
float32_t overlap_buffer[N-HOP_SIZE]={0};

int wrt_buf_ind = 0;
int rd_buf_ind = 0;
int wrt_out_ind = 0;
int rd_out_ind  = 0;

float32_t centroid = 10000.0f;
float32_t spread = 0.0f;
float32_t sum_centroid = 0.0f; 
float32_t sum_spread = 0.0f; 
boolean_t filter_type;
boolean_t filter_type_defined = false;
int noise_cycle = 0;          // Noise cycle counter

//Notch 
float notch_coeff;
float prev_input = 0.0f, prev_output = 0.0f;
// LMS-Filter Variablen
#define STEP_SIZE 0.1f //0.008f
#define NOTCH_BANDWIDTH 10.0f
float a_hat = 0.0f;  // Fourier-Koeffizient für Cosinus
float b_hat = 0.0f;  // Fourier-Koeffizient für Sinus
float omega;         // Normalisierte Winkelgeschwindigkeit
float audiooutput[N];
// Wiener
float32_t H[N / 2];     
float32_t beta = 100;        
float32_t alpha = 1; 
float32_t noise_increment = 1; 

void process_audio();
float compute_centroid(double *fft_freqs, float32_t *fft_power, int size);
float compute_spread(double *fft_freqs, float32_t *fft_power, float centroid, int size);
float notch_filter(float input_sample);
void compute_wiener_filter();
void generate_hamming_window(float *window, int size);

int main(void) {
    // Initialisierung
    platform_init(BAUDRATE, SAMPLERATE, line_in, dma, DSTC_I2S_HANDLER_CH0, DSTC_I2S_HANDLER_CH1);
    Uart0Init(115200);


    if (arm_rfft_fast_init_f32(&fft_instance, N) != ARM_MATH_SUCCESS) {
        debug_printf("FFT initialization failed!\n");
        return -1;
    }

    for (int i = 0; i < N / 2; i++) {
        fft_freqs[i] = (double)i * SAMPLERATE / N;
    }
    generate_hamming_window(hamming_window, N);

    // Normale Verarbeitung
    while (1) {
        if (rx_buffer_full && tx_buffer_empty) {
            process_audio();
        }
    }
} 

void process_audio() {
    uint32_t *rxbuf, *txbuf;
    txbuf = (tx_proc_buffer == PING) ? dma_tx_buffer_ping : dma_tx_buffer_pong;
    rxbuf = (rx_proc_buffer == PING) ? dma_rx_buffer_ping : dma_rx_buffer_pong;

    // Daten in den Ringpuffer schreiben
    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        audio_ring_buffer[wrt_buf_ind] = *rxbuf++;
        wrt_buf_ind = (wrt_buf_ind + 1) % BUFFER_SIZE;
        if (filter_type_defined && filter_type){
            //debug_printf("LMS computed");
            float sample = (float)((int16_t)audio_ring_buffer[wrt_buf_ind])/ 32768.0f;
            //notch_filter
            float filtered_sample = notch_filter(sample);
            //LMS_filter 
            float s_hat = a_hat * arm_cos_f32(omega *i) + b_hat * arm_sin_f32(omega*i);
            float error = filtered_sample - s_hat;

            //LMS - aktualisieren 
            a_hat += STEP_SIZE * error * arm_cos_f32(omega * i);
            b_hat += STEP_SIZE * error * arm_sin_f32(omega * i);

            *txbuf++ = (int16_t)(error * 32768.0f);
        } 

    }

    int available_data = (wrt_buf_ind - rd_buf_ind + BUFFER_SIZE) % BUFFER_SIZE;
    if (available_data >= N) {
        // Nächstes Frame
        float32_t mean = 0.0f;
        for (int i = 0; i < N; i++) {
            fft_input[i] = (float32_t)((int16_t)audio_ring_buffer[(rd_buf_ind + i) % BUFFER_SIZE]) / 32768.0f;
            mean += fft_input[i];
        }
        //rd_buf_ind = (rd_buf_ind + N) % BUFFER_SIZE;
        rd_buf_ind = (rd_buf_ind + HOP_SIZE) % BUFFER_SIZE; //update rd_ind um hop-size
        mean /= N;

        for (int i = 0; i < N; i++) {
            fft_input[i] = (fft_input[i] - mean) * hamming_window[i];
        }

        // FFT
        arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

        // Leistungsspektrum berechnen
        arm_cmplx_mag_f32(fft_output, fft_magnitude, N / 2);

        

        for (int i = 0; i < N / 2; i++) {
            //arm_sqrt_f32(fft_output[2 * i] * fft_output[2 * i] + fft_output[2 * i + 1] * fft_output[2 * i + 1],&fft_magnitude[i]); 
            P_noisy[i] = fft_magnitude[i] * fft_magnitude[i];
            //sum_power += P_noisy[i];
        }


        if (noise_cycle < NUM_NOISE_FRAMES) {

            centroid = compute_centroid(fft_freqs, P_noisy, N / 2);
            spread = compute_spread(fft_freqs, P_noisy, centroid, N / 2);
            sum_centroid += centroid; 
            sum_spread += spread; 

            for (int i = 0; i < N / 2; i++) {
                P_noise[i] += noise_increment * P_noisy[i] / NUM_NOISE_FRAMES;
            }
            noise_cycle++;

        } else if(noise_cycle == NUM_NOISE_FRAMES){
            centroid = sum_centroid / NUM_NOISE_FRAMES; 
            debug_printf("Centroid: %f", centroid); 
            spread = sum_spread / NUM_NOISE_FRAMES;
            debug_printf("Spread: %f", spread); 

            if (spread > 300.0f){ 
                filter_type = false; //false: Wiener
            
            } else {
                filter_type = true; //true: LMS
                omega = 2 * PI * centroid / SAMPLERATE;
                notch_coeff = arm_cos_f32(2 * PI * centroid/ SAMPLERATE);
                
            }
            debug_printf("Filter Type: %s\n", filter_type); 
            filter_type_defined = true; 
            noise_cycle++;
        } else {
            if (!filter_type){
                //debug_printf("Wiener computed"); 
                compute_wiener_filter();
                for (int i = 0; i < N / 2; i++) {
                    fft_output[2 * i] *= H[i];
                    fft_output[2 * i + 1] *= H[i];
                }

                arm_rfft_fast_f32(&fft_instance, fft_output, ifft_output, 1);
                for (int i = 0; i < HOP_SIZE; i++) {
                    float32_t scaled_output=(ifft_output[i] + overlap_buffer[i]);
                    out_ring_buffer[wrt_out_ind]= (int16_t)(scaled_output * 32768.0f);
                    wrt_out_ind = (wrt_out_ind + 1) % BUFFER_SIZE;
                }

                for (int i = 0; i < (N - HOP_SIZE); i++) {
                    overlap_buffer[i] = ifft_output[i + HOP_SIZE];
                }
                // Ausgabe
                //for (int i = 0; i < N; i++) {
                    //*txbuf++ = (int16_t)(ifft_output[i] * 32768.0f);
                //}
                int available_data_output = (wrt_out_ind - rd_out_ind + BUFFER_SIZE) % BUFFER_SIZE;
                if (available_data_output >= DMA_BUFFER_SIZE) {
                    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
                        if (rd_out_ind != wrt_out_ind) {
                            *txbuf++ = (int16_t)(out_ring_buffer[rd_out_ind]);
                            rd_out_ind = (rd_out_ind + 1) % BUFFER_SIZE;
                        } 
                        else {
                            *txbuf++ = 0;  // Auffüllen mit Null
                        }
                    }
                }
            }

        }


    }
    tx_buffer_empty = 0;
    rx_buffer_full = 0;
}

void compute_wiener_filter() {
    for (int i = 0; i < N / 2; i++) {
        //P_clean[i] = P_noisy[i] - P_noise[i];
        //P_clean[i] = fmaxf(P_noisy[i] - P_noise[i], 0.0f);
        P_clean[i] = P_noisy[i] - P_noise[i];
        if (P_clean[i] > 0){
            H[i] = P_clean[i] / (P_clean[i] + beta * P_noise[i]);
            H[i] = powf(P_clean[i] / (P_clean[i] + beta * P_noise[i]), alpha);
        } else {
            H[i] = 0; 
        }
    }
    // Beta Adaptiv ändern
    //if (sum_power > sum_power_noise){
        //beta = sum_power *10 / sum_power_noise;  
    //}
}

float notch_filter(float input_sample) {
    // Notch-Filter Implementierung
    float output = input_sample - 2 * notch_coeff * prev_input + prev_output;
    prev_output = prev_input;
    prev_input = input_sample;
    return output;
}

float compute_centroid(double *fft_freqs, float32_t *fft_power, int size) {
    float sum_fm = 0.0f, sum_m = 0.0f;
    for (int i = 0; i < size; i++) {
        sum_fm += fft_freqs[i] * fft_power[i];
        sum_m += fft_power[i];
    }
    return sum_m > 0 ? sum_fm / sum_m : 0.0f;
} //Hauptfrequenz

float compute_spread(double *fft_freqs, float32_t *fft_power, float centroid, int size) {
    float sum_spread = 0.0f, sum_m = 0.0f;
    for (int i = 0; i < size; i++) {
        float diff = fft_freqs[i] - centroid;
        sum_spread += diff * diff * fft_power[i];
        sum_m += fft_power[i];
    }
    return sum_m > 0 ? sqrtf(sum_spread / sum_m) : 0.0f;
}//Breite um Centroid Frequenz -> groß -> Rauschen; klein -> eher single frequency


void generate_hamming_window(float *window, int size) {
    for (int i = 0; i < size; i++) {
        window[i] = 0.54 - 0.46 * arm_cos_f32(2 * PI * i / (size - 1));
    }
}