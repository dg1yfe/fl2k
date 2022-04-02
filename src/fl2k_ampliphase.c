/*
 * osmo-fl2k, turns FL2000-based USB 3.0 to VGA adapters into
 * low cost DACs
 *
 * fl2k-ampliphase
 * Copyright (C) 2022 by Felix Erckenbrecht <eligs@eligs.de>
 *
 * based on fl2k-fm code:
 * Copyright (C) 2016-2018 by Steve Markgraf <steve@steve-m.de>
 *
 * based on FM modulator code from VGASIG:
 * Copyright (C) 2009 by Bartek Kania <mbk@gnarf.org>
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include <math.h>
#include <complex.h>
#include <pthread.h>

#include "osmo-fl2k.h"


enum inputType_E { INP_REAL, INP_COMPLEX };


#define BUFFER_SAMPLES_SHIFT	16
#define BUFFER_SAMPLES		(1 << BUFFER_SAMPLES_SHIFT)
#define BUFFER_SAMPLES_MASK	((1 << BUFFER_SAMPLES_SHIFT)-1)

#define BASEBAND_BUF_SIZE		2048

fl2k_dev_t *dev = NULL;
volatile int do_exit = 0;

pthread_t iq_thread;
pthread_mutex_t cb_mutex;
pthread_mutex_t iq_mutex;
pthread_cond_t cb_cond;
pthread_cond_t iq_cond;
pthread_t dbg_thread;

FILE *file;
int8_t *itxbuf = NULL;
int8_t *qtxbuf = NULL;
int8_t *iambuf = NULL;
int8_t *qambuf = NULL;
int8_t *buf1 = NULL;
int8_t *buf2 = NULL;

uint32_t samp_rate = 96000000;

int base_freq = 1440000;
int rf_to_baseband_sample_ratio;
int input_freq = 48000;

complex float *ampbuf;
complex float *slopebuf;

long int * pdbuf;
long int * pdslopebuf;

int writepos, readpos;
int swap_iq = 0;
int ignore_eof = 0;
int debug_to_file = 0;


void usage(void)
{
    fprintf(stderr,
            "fl2k_ampliphase, a special modulator for FL2K VGA dongles\n"
            "(output comes on channels r&g)\n\n"
            "Usage:"
            "\t[-d device index (default: 0)]\n"
            "\t[-c center frequency (default: 1440 kHz)]\n"
            "\t[-s samplerate in Hz (default: 96 MS/s)]\n"
            "\t[-m modulation index (default: 1.0)]\n"
            "\t[-i input baseband sample rate (default: 48000 Hz)]\n"
            "\t[-t type of input: real/complex (default: real)]\n"
            "\t    input requirements: real    - single channel (mono)\n"
            "\t                        complex - dual channel (stereo)\n"
            "\t[-w swap I & Q (invert spectrum)]\n"
            "\t[-e ignore EOF]\n"
            "\t[-D Debug - write to file 'debug.out' instead of FL2K device]\n"
            "\tfilename (use '-' to read from stdin)\n\n"
    );
    exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
    if (CTRL_C_EVENT == signum) {
        fprintf(stderr, "Signal caught, exiting!\n");
        fl2k_stop_tx(dev);
        do_exit = 1;
        pthread_cond_signal(&iq_cond);
        return TRUE;
    }
    return FALSE;
}
#else
static void sighandler(int signum)
{
    fprintf(stderr, "Signal caught, exiting!\n");
    fl2k_stop_tx(dev);
    do_exit = 1;
    pthread_cond_signal(&iq_cond);
}
#endif

/* DDS Functions */

#ifndef M_PI
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
# define M_PI_4		0.78539816339744830962	/* pi/4 */
# define M_1_PI		0.31830988618379067154	/* 1/pi */
# define M_2_PI		0.63661977236758134308	/* 2/pi */
#endif
#define DDS_2PI		(M_PI * 2)		/* 2 * Pi */
#define DDS_3PI2	(M_PI_2 * 3)		/* 3/2 * pi */

#define TRIG_TABLE_ORDER	8
#define TRIG_TABLE_SHIFT	(32 - TRIG_TABLE_ORDER)
#define TRIG_TABLE_LEN	(1 << TRIG_TABLE_ORDER)
//#define ANG_INCR	(0xffffffff / DDS_2PI)
#define INT_2PI     (0x100000000)
#define ANG_INCR    ((float)INT_2PI) / DDS_2PI

enum waveform_E { WF_SINE, WF_RECT };

struct trigonometric_table_S {
    int     initialized;
    int16_t quadrature[TRIG_TABLE_LEN];
    int16_t inphase[TRIG_TABLE_LEN];
};

static struct trigonometric_table_S trig_table = { .initialized = 0 };

typedef struct {
    float sample_freq;
    float freq;
    /* instantaneous phase */
    unsigned long int phase;
    /* phase increment */
    unsigned long int phase_step;

    /* for phase modulation */
    long int phase_delta;
    long int phase_slope;

    /* for amplitude modulation */
    complex float amplitude;
    complex float ampslope;
} dds_t;



static inline void dds_set_freq(dds_t *dds, float freq)
{
    dds->freq = freq;
    dds->phase_step = (freq / dds->sample_freq) * 2 * M_PI * ANG_INCR;
}

static inline void dds_set_amp(dds_t *dds, complex float amplitude, complex float ampslope)
{
    dds->amplitude = amplitude;
    dds->ampslope  = ampslope;
}

static inline void dds_set_phase(dds_t *dds, long int phase_delta, long int phase_slope)
{
    dds->phase_delta = phase_delta;
    dds->phase_slope = phase_slope;
}

dds_t dds_init(float sample_freq, float freq, float phase, float amp, enum waveform_E waveform )
{
    dds_t dds;
    int i;

    dds.sample_freq = sample_freq;
    dds.phase = phase * ANG_INCR;
    dds_set_freq(&dds, freq);
    dds_set_amp(&dds, amp, 0);
    /* Initialize quadrature table, prescaled for 16 bit signed integer */
    if (!trig_table.initialized) {
        float incr = 1.0 / (float)TRIG_TABLE_LEN;
        for (i = 0; i < TRIG_TABLE_LEN; i++){
            if(waveform == WF_SINE){
                trig_table.quadrature[i]= sin(incr * i * DDS_2PI) * 32767;
                trig_table.inphase[i]   = cos(incr * i * DDS_2PI) * 32767;
            }
            else{
                /* rectangular / square output */
                trig_table.quadrature[i]= sin(incr * i * DDS_2PI) >= 0 ? 32767 : -32767;
                trig_table.inphase[i]   = cos(incr * i * DDS_2PI) >= 0 ? 32767 : -32767;
            }
        }

        trig_table.initialized = 1;
    }

    return dds;
}



static inline void dds_complex(dds_t *dds, int8_t * i, int8_t * q)
{
    int phase_idx_i, phase_idx_q;
    int32_t amp_i, amp_q;

    // get current carrier phase, add phase mod,  calculate table index
    phase_idx_i = (dds->phase - dds->phase_delta) >> TRIG_TABLE_SHIFT;
    phase_idx_q = (dds->phase + dds->phase_delta) >> TRIG_TABLE_SHIFT;

    // advance dds generator
    dds->phase += dds->phase_step;
    // wrap around properly
    dds->phase &= 0xffffffff;

    //amp = 255;
    amp_i = (int32_t) (creal(dds->amplitude) * 32767.0);  // 0..15
    amp_q = (int32_t) (cimag(dds->amplitude) * 32767.0);

    amp_i = amp_i * trig_table.inphase[phase_idx_i];      // 0..31
    amp_q = amp_q * trig_table.quadrature[phase_idx_q];   // 0..31

    *i = (int8_t) (amp_i >> 24);        // 0..31 >> 24 => 0..8
    *q = (int8_t) (amp_q >> 24);        // 0..31 >> 24 => 0..8

    /* advance modulation signal by interpolated input from baseband */
    dds->amplitude      += dds->ampslope;
    dds->phase_delta    += dds->phase_slope;
    return;
}


static inline void dds_complex_buf(dds_t *dds, int8_t *ibuf, int8_t *qbuf, int count)
{
    int i;
    for (i = 0; i < count; i++){
        dds_complex(dds, &ibuf[i], &qbuf[i]);
    }
}


/* Signal generation and some helpers */

/* Generate the radio signal using the pre-calculated amplitude information
 * in the amp buffer */
static void *iq_worker(void *arg)
{
    register float freq;
    register float tmp;
    dds_t base_signal;
    int8_t *tmp_ptr;
    uint32_t len = 0;
    uint32_t readlen, remaining;
    int buf_prefilled = 0;

    /* Prepare the oscillators */
    base_signal = dds_init(samp_rate, base_freq, 0, 1, WF_RECT);

    while (!do_exit) {
        // dds_set_amp(&base_signal, ampbuf[readpos], slopebuf[readpos]);
        /* set phase modulation value from audio input */
        dds_set_phase(&base_signal, pdbuf[readpos], pdslopebuf[readpos]);
        readpos++;
        readpos &= BUFFER_SAMPLES_MASK;

        /* check if we reach the end of the buffer */
        if ((len + rf_to_baseband_sample_ratio) > FL2K_BUF_LEN) {
            readlen = FL2K_BUF_LEN - len;
            remaining = rf_to_baseband_sample_ratio - readlen;
            /* generate signal, perform phase modulation on both paths */
            dds_complex_buf(&base_signal, &iambuf[len], &qambuf[len],readlen);

            if (buf_prefilled) {
                /* swap buffers */
                tmp_ptr = iambuf;
                iambuf  = itxbuf;
                itxbuf  = tmp_ptr;

                tmp_ptr = qambuf;
                qambuf  = qtxbuf;
                qtxbuf  = tmp_ptr;

                pthread_mutex_lock(&cb_mutex);
                pthread_cond_wait(&cb_cond, &cb_mutex);
                pthread_mutex_unlock(&cb_mutex);
            }

            dds_complex_buf(&base_signal, iambuf, qambuf, remaining);
            len = remaining;

            buf_prefilled = 1;
        } else {
            dds_complex_buf(&base_signal, &iambuf[len], &qambuf[len], rf_to_baseband_sample_ratio);
            len += rf_to_baseband_sample_ratio;
        }
        pthread_mutex_lock(&iq_mutex);
        pthread_cond_signal(&iq_cond);
        pthread_mutex_unlock(&iq_mutex);
    }

    pthread_exit(NULL);
}



static inline int writelen(int maxlen)
{
    int rp = readpos;
    int len;
    int r;

    if (rp < writepos)
        rp += BUFFER_SAMPLES;

    len = rp - writepos;

    r = len > maxlen ? maxlen : len;

    return r;
}



static inline float modulate_sample_ampliphase(const int lastwritepos, const float lastamp, const float sample, float modulationIndex)
{
    float amp;
    float slope;

    /* Calculate modulator amplitudes at this point to lessen
     * the calculations needed in the signal generator */
    amp = sample;

    /* What we do here is calculate a linear "slope" from
    the previous sample to this one. This is then used by
    the modulator to gently increase/decrease the phase
    with each sample without the need to recalculate
    the dds parameters. In fact this gives us a very
    efficient and pretty good interpolation filter. */
    slope = amp - lastamp;
    slope = slope * 1.0/ (float) rf_to_baseband_sample_ratio;
    /* set phase-delta buf*/
    /* soll:                             -45° .. +45°                               */
    /*                                   -1 .. 1 * 45°                              */
    /*                                   -1 .. 1 * 0.25 * 2 Pi                      */
    /*                                   -1 .. 1 * 0.25            *           2 Pi */
    pdbuf[writepos]        = (long int) (lastamp * modulationIndex * (float) INT_2PI);
    pdslopebuf[writepos]   = (long int) (slope   * modulationIndex * (float) INT_2PI);

    return amp;
}



void ampliphase_modulator(enum inputType_E inputType, const float modIndex)
{
    /*
     * upper path: +45° Carrier -> Phase-Mod +/-45° * Audio In -> Squarer -> Drive
     *
     * lower path: -45° Carrier -> Phase-Mod -/+45° * Audio In -> Squarer -> Drive
     *
     * Audio In |  upper    |   lower   | shift | carrier
     *     0    |  +45°     |   -45°    |   90° |   50%
     *     1    |  +90°     |   -90°    |  180° |    0%
     *    -1    |    0°     |     0°    |    0° |  100%
     *
     */
    unsigned int i;
    size_t len;
    float freq;
    float complex lastamp = 0;
    int16_t baseband_buf_real[BASEBAND_BUF_SIZE];
    int16_t baseband_buf_cplx[BASEBAND_BUF_SIZE][2];
    uint32_t lastwritepos = writepos;
    float sample;

    while (!do_exit) {
        int swap = swap_iq;
        len = writelen(BASEBAND_BUF_SIZE);
        if (len > 1) {
            if(inputType == INP_REAL){
                len = fread(baseband_buf_real, 1, len, file);
                for(i = 0 ; i < len; i++){
                    /* input is -1.0 .. +1.0 (-32768 .. 32767)
                     * transform to 0.0 .. +1.0 (0 .. 32767)
                     * put into I part of BB
                     *
                     * AM : (1 + audio) / 2
                     *       ^-carrier
                     */
                    baseband_buf_cplx[i][0] = baseband_buf_real[i] / 2 + INT16_MAX/2;
                    /* Q part of BB is zero for AM */
                    baseband_buf_cplx[i][1] = 0;
                }
            }
            else{
                len = fread(baseband_buf_cplx, 2, len, file);
            }

            if (len == 0){
                if(ferror(file)){
                    do_exit = 1;
                }
                if(!ignore_eof && feof(file)){
                    do_exit = 1;
                }
            }

            for (i = 0; i < len; i++) {
                //sample = (float) baseband_buf_cplx[i][0+swap] / 32768.0 + I * (float) baseband_buf_cplx[i][1-swap] / 32768.0;
                /* keep sample real for the moment ... complex bb to be implemented
                 * this is amplitude only */
                sample = (float) baseband_buf_cplx[i][0+swap] / 32768.0;

                /* Modulate and buffer the sample */
                lastamp = modulate_sample_ampliphase(lastwritepos, lastamp, sample, modIndex);
                lastwritepos = writepos++;
                writepos %= BUFFER_SAMPLES;
            }
        } else {
            pthread_mutex_lock(&iq_mutex);
            pthread_cond_wait(&iq_cond, &iq_mutex);
            pthread_mutex_unlock(&iq_mutex);
        }
    }
}


void fl2k_callback(fl2k_data_info_t *data_info)
{
    if (data_info->device_error) {
        fprintf(stderr, "Device error, exiting.\n");
        do_exit = 1;
        pthread_mutex_lock(&iq_mutex);
        pthread_cond_signal(&iq_cond);
        pthread_mutex_unlock(&iq_mutex);
    }

    pthread_cond_signal(&cb_cond);

    data_info->sampletype_signed = 1;
    data_info->r_buf = (char *)itxbuf;
    data_info->g_buf = (char *)qtxbuf;
}



static void *file_worker(void *arg){
    FILE * f;
    uint32_t * filebuf;
    int i;
    const size_t len = sizeof(*filebuf) * FL2K_BUF_LEN;

    f = arg;
    filebuf = malloc(len);
    if(filebuf == NULL){
        fprintf(stderr,"Error allocating debug file buffer.\n");
    }
    else{
        while(1){
            for(i=0;i<FL2K_BUF_LEN;i++){
                filebuf[i] = (itxbuf[i] & 0xff) | ((uint32_t) qtxbuf[i]) << 8;
            }
            if(fwrite(filebuf,1,sizeof(len),f) != len){
                perror("Error writing to debug file");
                break;
            }
            pthread_cond_signal(&cb_cond);
        }
    }
    do_exit = 1;
    pthread_mutex_lock(&iq_mutex);
    pthread_cond_signal(&iq_cond);
    pthread_mutex_unlock(&iq_mutex);
    return NULL;
}



int main(int argc, char **argv)
{
    int r, opt;
    uint32_t buf_num = 0;
    int dev_index = 0;
    pthread_attr_t attr;
    char *filename = NULL;
    int option_index = 0;
    int input_freq_specified = 0;
    enum inputType_E input_type = INP_REAL;
    float modulation_index = 0.25; /* maximum phase modulation 0.25 * 2 * Pi -> 45° */
    FILE * fDbg = NULL;

#ifndef _WIN32
    struct sigaction sigact, sigign;
#endif

    static struct option long_options[] =
    {
            {0, 0, 0, 0}
    };

    while (1) {
        opt = getopt_long(argc, argv, "ewDd:c:i:s:t:m:", long_options, &option_index);

        /* end of options reached */
        if (opt == -1)
            break;

        switch (opt) {
        case 0:
            break;
        case 'D':
            debug_to_file=1;
            break;
        case 'd':
            dev_index = (uint32_t)atoi(optarg);
            break;
        case 'c':
            base_freq = (uint32_t)atof(optarg);
            break;
        case 'i':
            input_freq = (uint32_t)atof(optarg);
            input_freq_specified = 1;
            break;
        case 'm':
            modulation_index = atof(optarg);
            break;
        case 's':
            samp_rate = (uint32_t)atof(optarg);
            break;
        case 't':
            /* type */
            if(strcasecmp(optarg, "complex") && strcasecmp(optarg, "real")){
                fprintf(stderr, "Unknown parameter to -t : %s", optarg);
                exit(1);
            }
            input_type = strcasecmp(optarg, "complex") == 0 ? INP_COMPLEX : INP_REAL;
            if(input_type == INP_COMPLEX){
                fprintf(stderr, "Complex baseband not yet implemented.\n");
                exit(1);
            }
            break;
        case 'w':
            swap_iq = 1;
            break;
        case 'e':
            ignore_eof = 1;
            break;
        default:
            usage();
            break;
        }
    }

    if (argc <= optind) {
        usage();
    } else {
        filename = argv[optind];
    }

    if (dev_index < 0) {
        exit(1);
    }

    if (strcmp(filename, "-") == 0) { /* Read samples from stdin */
        file = stdin;
#ifdef _WIN32
        _setmode(_fileno(stdin), _O_BINARY);
#endif
    } else {
        file = fopen(filename, "rb");
        if (!file) {
            fprintf(stderr, "Failed to open %s\n", filename);
            return -ENOENT;
        }
    }

    /* allocate I buffer */
    iambuf = malloc(FL2K_BUF_LEN);
    itxbuf = malloc(FL2K_BUF_LEN);
    /* allocate Q buffer */
    qambuf = malloc(FL2K_BUF_LEN);
    qtxbuf = malloc(FL2K_BUF_LEN);

    if (!qambuf || !qtxbuf || !iambuf || !itxbuf) {
        fprintf(stderr, "malloc error!\n");
        exit(1);
    }


    /* Baseband buffer */
    slopebuf 	= malloc(BUFFER_SAMPLES * sizeof(float complex));
    ampbuf  	= malloc(BUFFER_SAMPLES * sizeof(float complex));
    pdbuf       = malloc(BUFFER_SAMPLES * sizeof(long int));
    pdslopebuf  = malloc(BUFFER_SAMPLES * sizeof(long int));
    readpos 	= 0;
    writepos 	= 1;

    fprintf(stdout, "Samplerate:       %3.2f MHz\n", (float)samp_rate/1000000);
    fprintf(stdout, "Center frequency: %5.0f kHz\n", (float)base_freq/1000);
    if(swap_iq)
        fprintf(stdout, "Spectral inversion active.\n");
    if(ignore_eof)
        fprintf(stdout, "Ignoring EOF.\n");

    pthread_mutex_init(&cb_mutex, NULL);
    pthread_mutex_init(&iq_mutex, NULL);
    pthread_cond_init(&cb_cond, NULL);
    pthread_cond_init(&iq_cond, NULL);
    pthread_attr_init(&attr);

    if(debug_to_file){
        fDbg = fopen("debug.out","wb");
        if(fDbg == NULL){
            fprintf(stderr, "Failed to open 'debug.out' file.\n");
            perror("");
            goto out;
        }
        r = pthread_create(&dbg_thread, &attr, file_worker, fDbg);
        if (r < 0) {
            fprintf(stderr, "Error spawning debug-file worker thread.\n");
            goto out;
        }
    }
    else
    {
        fl2k_open(&dev, (uint32_t)dev_index);
        if (NULL == dev) {
            fprintf(stderr, "Failed to open fl2k device #%d.\n", dev_index);
            goto out;
        }
    }

    r = pthread_create(&iq_thread, &attr, iq_worker, NULL);
    if (r < 0) {
        fprintf(stderr, "Error spawning IQ worker thread!\n");
        goto out;
    }

    pthread_attr_destroy(&attr);
    if(!debug_to_file){
        r = fl2k_start_tx(dev, fl2k_callback, NULL, 0);

        /* Set the sample rate */
        r = fl2k_set_sample_rate(dev, samp_rate);
        if (r < 0)
            fprintf(stderr, "WARNING: Failed to set sample rate. %d\n", r);

        /* read back actual frequency */
        samp_rate = fl2k_get_sample_rate(dev);
    }

    /* Calculate needed constants */
    rf_to_baseband_sample_ratio = samp_rate / input_freq;

#ifndef _WIN32
    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigign.sa_handler = SIG_IGN;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigign, NULL);
#else
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

    ampliphase_modulator(input_type, modulation_index);

    out:
    if(!debug_to_file){
        fl2k_close(dev);
    }

    if (file != stdin)
        fclose(file);

    free(ampbuf);
    free(slopebuf);
    free(pdbuf);
    free(pdslopebuf);
    free(iambuf);
    free(qambuf);
    free(itxbuf);
    free(qtxbuf);

    return 0;
}
