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

//SNR Parametern
float32_t snr_clean; 
float32_t noise_ratio; 
int noisy_cycle = 0; 
float32_t snr_clean_avg = 0; 
float32_t fft_filtered_magnitude[N / 2]; 
float32_t P_noisy_filtered[N / 2]; 
float32_t P_residual_noise[N / 2]; 
float32_t sum_P_residual_noise; 
float32_t sum_P_Noise; 
float32_t sum_P_clean_filtered; 
int filt_cycle = 0; 
boolean_t snr_calculated = false; 
float32_t P_clean_filtered[N / 2];
float32_t snr_filtered; 
float32_t buffer_P_noisy_filtered[N / 2]; 
float32_t sum_P_noisy_filtered;
float32_t error_buffer[DMA_BUFFER_SIZE]; 
float32_t fft_output_error[N]; 
float32_t fft_magnitude_error[N / 2]; 
float32_t P_filtered_error[N / 2]; 
float32_t buffer_P_error_filtered[N / 2]; 

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
#define MAX_FLOAT_VALUE 1e30f
float32_t H[N / 2];     
float32_t beta = 50;        // Wiener-Filter-Gewinn
float32_t alpha = 1; 
float32_t noise_increment = 1;

void process_audio();
float compute_centroid(double *fft_freqs, float32_t *fft_power, int size);
float compute_spread(double *fft_freqs, float32_t *fft_power, float centroid, int size);
float notch_filter(float input_sample);
void compute_wiener_filter();
void generate_hamming_window(float *window, int size);
void calculate_snr(const float32_t *P_clean1, const float32_t *P_noise1, float32_t *Snr_clean1);

int main(void) {
    // Initialisierung
    platform_init(BAUDRATE, SAMPLERATE, line_in, dma, DSTC_I2S_HANDLER_CH0, DSTC_I2S_HANDLER_CH1);
    Uart0Init(115200);

    GpioInit();

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
            error_buffer[i]= error; 
            *txbuf++ = (int16_t)(error * 32768.0f);
        } 

    }

    //SNR LMS
    if (filter_type_defined && filter_type){
        float32_t P_difference; 
        if (filt_cycle < 100 && snr_calculated){
            for (int i = 0; i < N; i++) {
                error_buffer[i] = error_buffer[i] * hamming_window[i];
            }
            arm_rfft_fast_f32(&fft_instance, error_buffer, fft_output_error, 0);
            arm_cmplx_mag_f32(fft_output_error, fft_magnitude_error, N / 2);
            for (int i = 0; i < N / 2; i++) {
                P_filtered_error[i] = fft_magnitude_error[i] * fft_magnitude_error[i];
                buffer_P_error_filtered[i] += P_filtered_error[i] / 100; 
                P_difference = P_filtered_error[i] - P_clean[i];
                if (P_difference > 0){
                    P_residual_noise[i] += P_difference;
                    P_residual_noise[i] /= 100;
                } else if (P_difference <= 0){
                    //P_residual_noise[i] = P_noise[i]; 
                    P_residual_noise[i] += P_noise[i] /100; 
                }
            }
            filt_cycle++; 

        } else if (filt_cycle == 100){

            for (int i= 0; i < N / 2; i++) {
                sum_P_residual_noise += P_residual_noise[i]; 
                sum_P_Noise += P_noise[i]; 
                sum_P_noisy_filtered += buffer_P_error_filtered[i]; 
            } 
            sum_P_clean_filtered = sum_P_noisy_filtered - sum_P_residual_noise; 
            debug_printf("Noise Ratio LMS: %.5f Prozent\n", 100 * sum_P_residual_noise / sum_P_Noise); 
            debug_printf("Residual Noise: %.5f\n", sum_P_residual_noise); 
            debug_printf("Original Noise: %.5f\n", sum_P_Noise); 
            debug_printf("Leistung gefiltertes Audio: %.5f\n", sum_P_noisy_filtered); 
            debug_printf("Liestung gefiltertes Nutzaudio: %.5f\n", sum_P_clean_filtered); 
            snr_filtered = 10.0f * logf(sum_P_clean_filtered / sum_P_residual_noise);
            debug_printf("SNR-Filtered: %.2f dB\n", snr_filtered); 

            // P_clean_filtered = P_noisy_filtered[i] - P_residual_noise[i]; 


            filt_cycle++; 
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

            for (int i = 0; i < N / 2; i++) {
                P_clean[i] = P_noisy[i] - P_noise[i];
            }


            //SNR ungefiltert berechnen
            if (noisy_cycle < 100){
                calculate_snr(P_clean, P_noise, &snr_clean); 
                if (snr_clean > 0) {
                    snr_clean_avg += snr_clean; 
                    noisy_cycle++;
                }
            } else if (noisy_cycle == 100){
                snr_clean_avg /= 100;
                debug_printf("O-SNR: %.2f dB\n", snr_clean_avg); 
                noisy_cycle++; 
                snr_calculated = true; 
            }
        
            if (!filter_type){ //Wiener Filter
                //debug_printf("Wiener computed"); 
                // compute_wiener_filter();

                for (int i = 0; i < N / 2; i++) {
                    if (P_clean[i] > 0){
                        H[i] = P_clean[i] / (P_clean[i] + beta * P_noise[i]);
                        H[i] = powf(P_clean[i] / (P_clean[i] + beta * P_noise[i]), alpha);
                    } else {
                        H[i] = 0; 
                    }
                    fft_output[2 * i] *= H[i];
                    fft_output[2 * i + 1] *= H[i];
                }

                //SNR Wiener 
                if (filt_cycle < 100 && snr_calculated){
                    arm_cmplx_mag_f32(fft_output, fft_filtered_magnitude, N / 2);
                    for (int i = 0; i < N / 2; i++) {
                        float32_t P_difference; 
                        P_noisy_filtered[i] = fft_filtered_magnitude[i] * fft_filtered_magnitude[i];
                        buffer_P_noisy_filtered[i] += P_noisy_filtered[i] / 100; 
                        // P_residual_noise[i] += fmaxf(P_noisy_filtered[i] - P_clean[i], 0.00f);
                        P_difference = P_noisy_filtered[i] - P_clean[i];
                        if (P_difference > 0){
                            P_residual_noise[i] += P_difference;
                            P_residual_noise[i] /= 100;
                        } else if (P_difference <= 0){
                            //P_residual_noise[i] = P_noise[i]; 
                            P_residual_noise[i] += P_noise[i] /100; 
                        }
                    }
                    filt_cycle++; 
                } else if (filt_cycle == 100){
                    for (int i= 0; i < N / 2; i++) {
                        sum_P_residual_noise += P_residual_noise[i]; 
                        sum_P_Noise += P_noise[i]; 
                        sum_P_noisy_filtered += buffer_P_noisy_filtered[i]; 
                    } 
                    sum_P_clean_filtered = sum_P_noisy_filtered - sum_P_residual_noise; 
                    debug_printf("Noise Ratio: %.5f Prozent\n", 100 * sum_P_residual_noise / sum_P_Noise); 
                    debug_printf("Gefiltertes Rauschen: %.5f\n", sum_P_residual_noise); 
                    debug_printf("Original Rauschen: %.5f\n", sum_P_Noise); 
                    debug_printf("Leistung gefiltertes Audio: %.5f\n", sum_P_noisy_filtered); 
                    debug_printf("Leistung gefiltertes Nutzaudio: %.5f\n", sum_P_clean_filtered); 
                    snr_filtered = 10.0f * logf(sum_P_clean_filtered / sum_P_residual_noise);
                    debug_printf("SNR-Filtered: %.2f dB\n", snr_filtered); 

                    // P_clean_filtered = P_noisy_filtered[i] - P_residual_noise[i]; 


                    filt_cycle++; 
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

void calculate_snr(const float32_t *P_clean1, const float32_t *P_noise1, float32_t *Snr_clean1){
     
    float32_t sum_P_clean = 0.0f;
    float32_t sum_avg_P_noise = 0.0f;
    
    for (int i= 0; i < N / 2; i++) {
        sum_P_clean += P_clean1[i]; 
        sum_avg_P_noise += P_noise1[i]; 
    }

    if (sum_avg_P_noise > 0) {
    *Snr_clean1 = 10.0f * logf(sum_P_clean / sum_avg_P_noise);
    } else {
        *Snr_clean1 = 0.3;
    }

}

