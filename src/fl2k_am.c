/*
 * osmo-fl2k, turns FL2000-based USB 3.0 to VGA adapters into
 * low cost DACs
 *
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
#include <pthread.h>

#include "osmo-fl2k.h"
#include "rds_mod.h"

#define BUFFER_SAMPLES_SHIFT	16
#define BUFFER_SAMPLES		(1 << BUFFER_SAMPLES_SHIFT)
#define BUFFER_SAMPLES_MASK	((1 << BUFFER_SAMPLES_SHIFT)-1)

#define AUDIO_BUF_SIZE		2048

fl2k_dev_t *dev = NULL;
int do_exit = 0;

pthread_t am_thread;
pthread_mutex_t cb_mutex;
pthread_mutex_t am_mutex;
pthread_cond_t cb_cond;
pthread_cond_t am_cond;

FILE *file;
int8_t *txbuf = NULL;
int8_t *ambuf = NULL;
int8_t *buf1 = NULL;
int8_t *buf2 = NULL;

uint32_t samp_rate = 100000000;

int carrier_freq = 1440000;
int carrier_per_signal;
int input_freq = 44100;

double mod_index = 0.9;
double *ampbuf;
double *slopebuf; 
int writepos, readpos;

void usage(void)
{
	fprintf(stderr,
		"fl2k_fm, an FM modulator for FL2K VGA dongles\n\n"
		"Usage:"
		"\t[-d device index (default: 0)]\n"
		"\t[-c carrier frequency (default: 1440 kHz)]\n"
		"\t[-m Modulation index (default: 0.9)]\n"
		"\t[-i input audio sample rate (default: 44100 Hz for mono FM)]\n"
		"\t[-s samplerate in Hz (default: 100 MS/s)]\n"
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
		pthread_cond_signal(&am_cond);
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
	pthread_cond_signal(&am_cond);
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

#define SIN_TABLE_ORDER	8
#define SIN_TABLE_SHIFT	(32 - SIN_TABLE_ORDER)
#define SIN_TABLE_LEN	(1 << SIN_TABLE_ORDER)
#define ANG_INCR	(0xffffffff / DDS_2PI)

int16_t sine_table[SIN_TABLE_LEN];
int sine_table_init = 0;

typedef struct {
	double sample_freq;
	double freq;
	unsigned long int phase;
	unsigned long int phase_step;
	int32_t amplitude;
	double ampslope;
} dds_t;

static inline void dds_set_freq(dds_t *dds, double freq)
{
	dds->freq = freq;
	dds->phase_step = (freq / dds->sample_freq) * 2 * M_PI * ANG_INCR;
}

static inline void dds_set_amp(dds_t *dds, double amplitude, double ampslope)
{
	dds->amplitude = (int32_t) (amplitude * 32768.0);
	dds->ampslope  = ampslope;
}

dds_t dds_init(double sample_freq, double freq, double phase, double amp)
{
	dds_t dds;
	int i;

	dds.sample_freq = sample_freq;
	dds.phase = phase * ANG_INCR;
	dds_set_freq(&dds, freq);
	dds_set_amp(&dds, amp, 0);
	/* Initialize sine table, prescaled for 16 bit signed integer */
	if (!sine_table_init) {
		double incr = 1.0 / (double)SIN_TABLE_LEN;
		for (i = 0; i < SIN_TABLE_LEN; i++)
			sine_table[i] = sin(incr * i * DDS_2PI) * 32767;

		sine_table_init = 1;
	}

	return dds;
}

static inline int8_t dds_real(dds_t *dds)
{
	int tmp;
	int32_t amp;
	int8_t amp8;

	// advance dds generator
	tmp = dds->phase >> SIN_TABLE_SHIFT;
	dds->phase += dds->phase_step;
	dds->phase &= 0xffffffff;

	//amp = 255;
	amp = dds->amplitude;		// 0..15
	amp = amp * sine_table[tmp];	// 0..31
	amp8 = (int8_t) (amp >> 24);
	//dds->amplitude += dds->ampslope;
	return amp8;
}

static inline void dds_real_buf(dds_t *dds, int8_t *buf, int count)
{
	int i;
	for (i = 0; i < count; i++)
		buf[i] = dds_real(dds);
}

/* Signal generation and some helpers */

/* Generate the radio signal using the pre-calculated amplitude information
 * in the amp buffer */
static void *am_worker(void *arg)
{
	register double freq;
	register double tmp;
	dds_t carrier;
	int8_t *tmp_ptr;
	uint32_t len = 0;
	uint32_t readlen, remaining;
	int buf_prefilled = 0;

	/* Prepare the oscillators */
	carrier = dds_init(samp_rate, carrier_freq, 0, 1);

	while (!do_exit) {
		dds_set_amp(&carrier, ampbuf[readpos], slopebuf[readpos]);
		readpos++;
		readpos &= BUFFER_SAMPLES_MASK;

		/* check if we reach the end of the buffer */
		if ((len + carrier_per_signal) > FL2K_BUF_LEN) {
			readlen = FL2K_BUF_LEN - len;
			remaining = carrier_per_signal - readlen;
			dds_real_buf(&carrier, &ambuf[len], readlen);

			if (buf_prefilled) {
				/* swap buffers */
				tmp_ptr = ambuf;
				ambuf = txbuf;
				txbuf = tmp_ptr;
				pthread_cond_wait(&cb_cond, &cb_mutex);
			}

			dds_real_buf(&carrier, ambuf, remaining);
			len = remaining;

			buf_prefilled = 1;
		} else {
			dds_real_buf(&carrier, &ambuf[len], carrier_per_signal);
			len += carrier_per_signal;
		}
		pthread_cond_signal(&am_cond);
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

static inline double modulate_sample_am(int lastwritepos, double lastamp, double sample)
{
	double amp, slope;

	/* Calculate modulator amplitude at this point to lessen
	 * the calculations needed in the signal generator */
	amp = 1 - ((1+sample) * mod_index)/2;

	/* What we do here is calculate a linear "slope" from
	the previous sample to this one. This is then used by
	the modulator to gently increase/decrease the amplitude
	with each sample without the need to recalculate
	the dds parameters. In fact this gives us a very
	efficient and pretty good interpolation filter. */
	slope = amp - lastamp;
	slope /= carrier_per_signal;
	slopebuf[lastwritepos] = slope;
	ampbuf[writepos]       = amp;

	return amp;
}

void am_modulator()
{
	unsigned int i;
	size_t len;
	double freq;
	double lastamp = 0;
	int16_t audio_buf[AUDIO_BUF_SIZE];
	uint32_t lastwritepos = writepos;
	double sample;

	while (!do_exit) {
		len = writelen(AUDIO_BUF_SIZE);
		if (len > 1) {
			len = fread(audio_buf, 2, len, file);

			if (len == 0){
				if(ferror(file)){
					do_exit = 1;
				}
			}

			for (i = 0; i < len; i++) {
				sample = (double) audio_buf[i] / 32768.0;

				/* Modulate and buffer the sample */
				lastamp = modulate_sample_am(lastwritepos, lastamp, sample);
				lastwritepos = writepos++;
				writepos %= BUFFER_SAMPLES;
			}
		} else {
			pthread_cond_wait(&am_cond, &am_mutex);
		}
	}
}


void fl2k_callback(fl2k_data_info_t *data_info)
{
	if (data_info->device_error) {
		fprintf(stderr, "Device error, exiting.\n");
		do_exit = 1;
		pthread_cond_signal(&am_cond);
	}

	pthread_cond_signal(&cb_cond);

	data_info->sampletype_signed = 1;
	data_info->r_buf = (char *)txbuf;
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

#ifndef _WIN32
	struct sigaction sigact, sigign;
#endif

	static struct option long_options[] =
	{
		{0, 0, 0, 0}
	};

	while (1) {
		opt = getopt_long(argc, argv, "d:c:m:i:s:", long_options, &option_index);

		/* end of options reached */
		if (opt == -1)
			break;

		switch (opt) {
		case 0:
			break;
		case 'd':
			dev_index = (uint32_t)atoi(optarg);
			break;
		case 'c':
			carrier_freq = (uint32_t)atof(optarg);
			break;
		case 'i':
			input_freq = (uint32_t)atof(optarg);
			input_freq_specified = 1;
			break;
		case 'm':
			mod_index = atof(optarg);
			break;
		case 's':
			samp_rate = (uint32_t)atof(optarg);
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

	/* allocate buffer */
	buf1 = malloc(FL2K_BUF_LEN);
	buf2 = malloc(FL2K_BUF_LEN);
	if (!buf1 || !buf2) {
		fprintf(stderr, "malloc error!\n");
		exit(1);
	}

	ambuf = buf1;
	txbuf = buf2;

	/* Decoded audio */
	slopebuf 	= malloc(BUFFER_SAMPLES * sizeof(double));
	ampbuf  	= malloc(BUFFER_SAMPLES * sizeof(double));
	readpos 	= 0;
	writepos 	= 1;

	fprintf(stderr, "Samplerate:\t%3.2f MHz\n", (double)samp_rate/1000000);
	fprintf(stderr, "Carrier:\t%5.0f kHz\n", (double)carrier_freq/1000);
	fprintf(stderr, "Mod Index:\t%2.1f %%\n", (double)mod_index*100);

	pthread_mutex_init(&cb_mutex, NULL);
	pthread_mutex_init(&am_mutex, NULL);
	pthread_cond_init(&cb_cond, NULL);
	pthread_cond_init(&am_cond, NULL);
	pthread_attr_init(&attr);

	fl2k_open(&dev, (uint32_t)dev_index);
	if (NULL == dev) {
		fprintf(stderr, "Failed to open fl2k device #%d.\n", dev_index);
		goto out;
	}

	r = pthread_create(&am_thread, &attr, am_worker, NULL);
	if (r < 0) {
		fprintf(stderr, "Error spawning AM worker thread!\n");
		goto out;
	}

	pthread_attr_destroy(&attr);
	r = fl2k_start_tx(dev, fl2k_callback, NULL, 0);

	/* Set the sample rate */
	r = fl2k_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate. %d\n", r);

	/* read back actual frequency */
	samp_rate = fl2k_get_sample_rate(dev);

	/* Calculate needed constants */
	carrier_per_signal = samp_rate / input_freq;

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

	am_modulator();

out:
	fl2k_close(dev);

	if (file != stdin)
		fclose(file);

	free(ampbuf);
	free(slopebuf);
	free(buf1);
	free(buf2);

	return 0;
}
