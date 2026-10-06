/*
 * Copyright (C) CSR plc 2011
 * Copyright 2010, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: marco.sinigaglia@csr.com for CSR plc
 */

#define LOG_TAG "V4l2_handler_Fm"
#define LOG_NDEBUG 1

#ifdef LINUX
#define ALOGI printf
#define ALOGE printf
#else
#include "utils/Log.h"
#include <cutils/properties.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "../libfmjni/android_fm.h"
#include "v4l2_ioctl.h"
#include "rds_text.h"

//RDS
#define RDS_THREAD_ON                    1
#define RDS_THREAD_OFF                   0
#define BUFFER_RDS_SIZE                  300                // rds buff size
#define GROUP_SIZE                       8                  // group is composed by 2byte * 4 packets
//Mute
#define DEFAULT_VOLUME                  255
#define MUTE_OFF                        0
#define MUTE_ON                         1
//Scan
#define SCAN_RUN                        1
#define SCAN_STOP                       0
#define LOCKTIME                        40000           // wait 40ms for card to lock on
#define MAX_FREQS                       50              // number of max freq buffer for a full scan
#define DEFAULT_THRESHOLD               500             // threshold for scan

/*
 * RDS as it comes in: groups put together from the blocks the driver
 * reads out (a group may straddle two reads), and the texts put together
 * from the groups' segments (rds_text.c): the station name (PS, 0A/0B)
 * and its long form (Long PS, 15A), the radio text (RT, 2A/2B) and its
 * enhanced form (eRT, the open data application 0x6552). What is
 * complete is published as UTF-8, the long and enhanced forms before the
 * plain ones, and get_rds says what changed.
 */
#define RDS_ODA_ERT  0x6552

typedef struct rds_state_t {
  /* the group being put together: 4 blocks of 2 bytes */
  unsigned char group[GROUP_SIZE];
  int next_block;                     /* the block expected, 0..3 */
  unsigned int corrected;             /* the group's blocks the chip corrected */

  rds_text ps, long_ps, rt, ert;
  int rt_ab;                          /* the A/B flag: a change is new text */
  int ab_cand;                        /* a change seen in a corrected group */
  int ert_group;                      /* its group, type << 1 | B; -1: none */
  enum rds_charset ert_charset;

  char ps_out[RDS_PSN_UTF8_SIZE];
  char long_ps_out[RDS_PSN_UTF8_SIZE];
  char rt_out[RDS_RT_UTF8_SIZE];
  char ert_out[RDS_RT_UTF8_SIZE];
  char psn_pub[RDS_PSN_UTF8_SIZE];    /* as published */
  char rt_pub[RDS_RT_UTF8_SIZE];

  int af[RDS_MAX_AFS];
  int num_afs;

  unsigned short pi;                  /* the last group's */
  unsigned short pi_out;              /* published: seen in two groups running */
  short pty, tp, ta, ms;
  int changed;                        /* FMRADIO_RDS_*_CHANGED since read */
} rds_state;

/* session struct holded by the FM SE stack */
typedef struct fm_v4l2_data_t {
  int low_freq;
  int high_freq;
  int grid;
  int fd;
  int threshold;
  int freq;
  float fact;
  struct v4l2_tuner vt;
  pthread_t thread_rds;                                 /* thread used to read rds data */
  volatile char scan_band_run;                          /* flag to stop the scan, set from another thread */
  char thread_rds_run;                                  /* flag to stop the rds thread*/
  rds_state rds;
} fm_v4l2_data;

/* A new frequency: whatever RDS was gathered was another station's */
static void rds_reset(fm_v4l2_data* session)
{
  rds_state* rds = &session->rds;

  memset(rds, 0, sizeof(*rds));
  rds_text_init(&rds->ps, RDS_PSN_MAX_LENGTH, 2, 0);
  rds_text_init(&rds->long_ps, 32, 4, 1);
  rds_text_init(&rds->rt, RDS_RT_MAX_LENGTH, 4, 1);
  rds_text_init(&rds->ert, RDS_TEXT_MAX_BYTES, 4, 1);
  rds->rt_ab = -1;
  rds->ab_cand = -1;
  rds->ert_group = -1;
}


fm_v4l2_data* get_session_data(void **data) {return *data;}

/*
 * kHz to the driver's units and back. fact is 16 for a tuner in 62.5 Hz
 * units (V4L2_TUNER_CAP_LOW) and 0.016 for one in 62.5 kHz units: taken as
 * an int it was 0 for the latter, and the conversion divided by it.
 */
int get_standard_freq(int freq, float fact) {return (int) (freq / fact + 0.5f);}
int get_proprietary_freq(int freq, float fact) {return (int) (freq * fact + 0.5f);}

void* th_read_rds(void *thread_rds_info);

/*
 * Lets go of a session: the device closed, whatever state the tuner is in,
 * and the memory freed. The kernel lets /dev/radio0 be opened once, so a
 * session that keeps it would keep FM from ever starting again.
 */
static int release_session(void **data)
{
  fm_v4l2_data* session = get_session_data(data);
  int ret = 0;

  if (session == NULL)
    return 0;

  if (session->fd >= 0 && close(session->fd) < 0) {
    ALOGE("error on close: %s\n", strerror(errno));
    ret = -1;
  }

  free(session);
  *data = NULL;
  return ret;
}

static int v4l2_rx_start_func (void **data, int low_freq, int high_freq, int default_freq, int grid)
{
  char	*dev = DEFAULT_DEVICE;
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  ALOGI("low_freq %d, high_freq %d, default_freq %d, grid %d\n", low_freq, high_freq, default_freq, grid);

  session = malloc(sizeof(fm_v4l2_data));
  if (session== NULL){
      ALOGE("error on malloc");
      return -1;
  }

  memset(session, 0, sizeof(fm_v4l2_data));
  *data =  session;

  session->fd = open_dev(dev);
  if (session->fd < 0){
      ALOGE("error on open dev\n");
      goto fail;
  }

  if (get_tun_radio_cap(session->fd) !=1) {
      ALOGE("error on check tunner radio capability");
      goto fail;
  }

  session->vt.index=0;
  if ( get_v4l2_tuner(session->fd,  &session->vt) <0 ){
      ALOGE("error on get V4L tuner\n");
      goto fail;
  }

  /*
   * De-emphasis has to match the transmitters': 50 us in Europe, Russia and
   * most of the world, 75 us in the Americas and Korea. The app does not
   * pass it, so the device says which through ro.vendor.fm.deemphasis
   * (50 if unset). Set before tuning, so the first audio is right.
   */
  {
      int deemph = property_get_int32("ro.vendor.fm.deemphasis", 50);

      if (deemph != 50 && deemph != 75) {
          ALOGW("ro.vendor.fm.deemphasis=%d is neither 50 nor 75, using 50\n", deemph);
          deemph = 50;
      }
      /* Not fatal: a kernel without the control keeps its own default,
       * and FM still plays. */
      if (set_deemphasis(session->fd, deemph) < 0)
          ALOGW("de-emphasis not set, the driver's default stays\n");
  }

  session->fact = get_fact(session->fd, &session->vt);
  if ( session->fact <= 0) {
      ALOGE("error on get fact\n");
      goto fail;
  }

  session->freq = get_proprietary_freq(default_freq, session->fact);
  session->low_freq =  get_proprietary_freq(low_freq,  session->fact);
  session->high_freq =  get_proprietary_freq(high_freq,  session->fact);
  session->grid = get_proprietary_freq(grid,  session->fact);
  session->threshold = DEFAULT_THRESHOLD;
  /* a scan runs until stopped; see v4l2_scan */
  session->scan_band_run = SCAN_RUN;
  rds_reset(session);
  
  if (set_freq(session->fd, session->freq) < 0 ){
      ALOGE("error on set freq\n");
      goto fail;
  }

  if (set_volume(session->fd, DEFAULT_VOLUME)<0){
      ALOGE("error on set volume\n");
      goto fail;
  }

  if (set_mute(session->fd, MUTE_OFF) <0){
      ALOGE("error on mute\n");
      goto fail;
  }

  return 0;

fail:
  /* A start that failed leaves nothing behind: no session, no device */
  release_session(data);
  return -1;
}

/*
 * Muted first, so the radio goes quiet before it goes off, but the device
 * is closed and the session freed either way: a reset that fails half way
 * must not leave the device held.
 */
int v4l2_reset(void** session_data)
{
  fm_v4l2_data* session;
  int ret = 0;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);
  if (session == NULL)
    return 0;

  if (set_mute(session->fd, MUTE_ON) < 0) {
    ALOGE("error on mute before reset\n");
    ret = -1;
  }

  if (release_session(session_data) < 0)
    ret = -1;

  return ret;
}

int v4l2_pause(void** session_data){
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  return set_mute(session->fd, MUTE_ON);
}

int v4l2_resume(void** session_data){
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  return set_mute(session->fd, MUTE_OFF);
}

int v4l2_mute(void** session_data, int mute){
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  return set_mute(session->fd, mute);
}


int v4l2_set_frequency(void** session_data, int frequency){
  fm_v4l2_data* session;
  int ret;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  /* session->freq stays the tuner's own until the tune is through: the
   * RDS poll takes any other frequency the driver reports for an AF jump */
  ret = get_proprietary_freq(frequency, session->fact);
  if (set_freq(session->fd, ret) < 0)
      return -1;
  session->freq = ret;

  rds_reset(session);
  return frequency;
}

int v4l2_get_frequency (void ** session_data){
  fm_v4l2_data* session;
  int ret;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  ret = get_freq(session->fd);
  if (ret < 0)
      return -1;

  /* session->freq is in the driver's units; the caller gets kHz */
  session->freq = ret;

  return get_standard_freq(ret, session->fact);
}

int v4l2_get_threshold (void ** session_data){
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  return session->threshold;
 }

int v4l2_set_threshold (void ** session_data, int threshold){
   fm_v4l2_data* session;

   ALOGI("%s:\n", __FUNCTION__);
   session = get_session_data(session_data);

   session->threshold = threshold;
   return 0;
}

int v4l2_get_signal_strength (void ** session_data){
    fm_v4l2_data* session;
    int ret;

    ALOGI("%s:\n", __FUNCTION__);
    session = get_session_data(session_data);

    ret = get_signal_strength(session->fd, &session->vt);
    return ret;
}

int v4l2_is_playing_in_stereo (void ** session_data){
    fm_v4l2_data* session;
    int ret;

    ALOGI("%s:\n", __FUNCTION__);
    session = get_session_data(session_data);

    ret = get_stereo(session->fd, &session->vt);
    return ret;
}

/* The seek step, in Hz, for VIDIOC_S_HW_FREQ_SEEK */
static unsigned int seek_spacing_hz(fm_v4l2_data* session)
{
  return (unsigned int) get_standard_freq(session->grid, session->fact) * 1000;
}

/* A driver that has no hardware seek */
static int no_hw_seek(int err)
{
  return err == -ENOTTY || err == -EINVAL;
}

/*
 * The seek done in software, for a driver without a hardware one: a step at
 * a time round the band, from the current frequency, until a signal is
 * over the threshold. Once round at most, and it stops when asked. Returns
 * the frequency found, or -1 with the tuner back where it was.
 */
/*
 * Back to the frequency the scan started from. If the tune fails, the
 * tuner is wherever the scan left it, and session->freq follows it: the
 * RDS poll would take the difference for an AF jump.
 */
static void restore_freq(fm_v4l2_data* session)
{
  int freq;

  set_freq(session->fd, session->freq);
  freq = get_freq(session->fd);
  if (freq >= 0)
      session->freq = freq;
}

static int sw_scan(fm_v4l2_data* session, int upward)
{
  int increment = upward ? session->grid : -session->grid;
  int steps = (session->high_freq - session->low_freq) / session->grid + 1;
  int freqi = session->freq;
  int rate, i;

  ALOGI("Starting software scanning...\n");
  for (i = 0; i < steps && session->scan_band_run == SCAN_RUN; i++) {
      freqi += increment;
      if (freqi > session->high_freq)
          freqi = session->low_freq;
      if (freqi < session->low_freq)
          freqi = session->high_freq;

      if (set_freq(session->fd, freqi) < 0)
          break;

      usleep(LOCKTIME);                       // let it lock on
      rate = get_signal_strength(session->fd, &session->vt);
      if (rate < 0)
          break;

      ALOGI("final rate %d > %d \n", rate, session->threshold);
      if (rate > session->threshold) {
          ALOGI("Found freq, %d\n", freqi);
          session->freq = freqi;
          return get_standard_freq(freqi, session->fact);
      }
  }

  restore_freq(session);
  return -1;
}

/*
 * The next station up or down from the current one. The tuner seeks by
 * itself (VIDIOC_S_HW_FREQ_SEEK, wrapping round the band), which takes a
 * moment where stepping in software took seconds; software is only for a
 * driver without it. Returns the frequency found, or -1 -- no station,
 * stopped, or failed -- with the tuner back where it was.
 *
 * A stop asked for while the scan runs is kept until the scan ends, and
 * cleared then: setting "run" at the start would overwrite a stop that
 * came in before it. The JNI only sends a stop while it is SCANNING. A
 * hardware seek cannot be stopped; it ends at the next station.
 */
int v4l2_scan (void ** session_data, enum fmradio_seek_direction_t direction){
  fm_v4l2_data* session;
  int upward, ret, freq;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);
  upward = direction != FMRADIO_SEEK_DOWN;

  ret = hw_freq_seek(session->fd, upward, 1, seek_spacing_hz(session),
                     session->low_freq, session->high_freq);
  if (ret == 0) {
      freq = get_freq(session->fd);
      if (freq >= 0) {
          session->freq = freq;
          ret = get_standard_freq(freq, session->fact);
      } else {
          restore_freq(session);
          ret = -1;
      }
  } else if (no_hw_seek(ret)) {
      ret = sw_scan(session, upward);
  } else {
      ALOGI("hardware seek: %s\n", ret == -ENODATA ? "no station" : strerror(-ret));
      restore_freq(session);
      ret = -1;
  }

  session->scan_band_run = SCAN_RUN;
  rds_reset(session);
  ALOGI("End scan\n");
  return ret;
}

/*
 * Every station in the band, low to high, by hardware seeks one after the
 * other (or a software sweep, without them), stopping when asked between
 * two. The result ends with a 0; returns the number found, or -1. The tuner
 * goes back to where it was.
 */
int v4l2_full_scan (void ** session_data, int ** found_freqs, int ** signal_strenghts){
  fm_v4l2_data* session;
  int founded, i, ret;
  int freqi, rate, prev;
  int *temp_freq, *temp_strenght;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  *found_freqs = NULL;
  *signal_strenghts = NULL;
  founded = 0;

  temp_freq = (int *) malloc(sizeof(int) *MAX_FREQS);
  temp_strenght = (int *) malloc(sizeof(int) *MAX_FREQS);
  if (temp_freq == NULL || temp_strenght==NULL){
    ALOGE("error on allocate");
    founded = -1;
    goto out;
  }

  ALOGI("Starting full scanning...low freq:%d, high freq:%d\n", session->low_freq, session->high_freq);

  /* From the top of the band an upward seek starts again at the bottom */
  if (set_freq(session->fd, session->high_freq) < 0) {
      founded = -1;
      goto out;
  }

  prev = -1;
  while (founded < MAX_FREQS && session->scan_band_run == SCAN_RUN) {
      ret = hw_freq_seek(session->fd, 1, 0, seek_spacing_hz(session),
                         session->low_freq, session->high_freq);
      if (ret == -ENODATA)
          break;
      if (ret < 0) {
          if (prev < 0 && no_hw_seek(ret))
              goto software;
          ALOGE("hardware seek failed: %s\n", strerror(-ret));
          founded = -1;
          goto out;
      }

      freqi = get_freq(session->fd);
      if (freqi < 0) {
          founded = -1;
          goto out;
      }
      /* past the top, the seek has come round again */
      if (freqi <= prev)
          break;
      prev = freqi;

      rate = get_signal_strength(session->fd, &session->vt);
      ALOGI("Founded index %d, freq %d, signal %d\n", founded, freqi, rate);
      temp_freq[founded] = freqi;
      temp_strenght[founded] = rate < 0 ? 0 : rate;
      founded++;
  }
  goto found;

software:
  for (freqi = session->low_freq; ((freqi <= session->high_freq) && (founded < MAX_FREQS) && (session->scan_band_run==SCAN_RUN)) ; freqi += session->grid){

      ret = set_freq(session->fd, freqi);
      if (ret<0) {
         founded = -1;
         goto out;
      }

      usleep(LOCKTIME);		/* let it lock on */
      rate= get_signal_strength(session->fd, &session->vt);
      if (rate < 0) {
          founded = -1;
          goto out;
      }

      ALOGI("final rate %d > %d \n", rate ,session->threshold);

      if (rate > session->threshold){
          ALOGI("Founded index %d, freq %d\n", founded, freqi);
          temp_freq[founded]= freqi;
          temp_strenght[founded]= rate;
          founded++;
      }
  }

found:
  /* One more than found, for a 0 after the last: the caller reads the
   * frequencies up to it */
  *found_freqs = (int *) malloc(sizeof(int) * (founded + 1));
  *signal_strenghts = (int *) malloc(sizeof(int) * (founded + 1));
  if (*found_freqs == NULL || *signal_strenghts==NULL){
    ALOGE("error on allocate");
    free(*found_freqs);
    free(*signal_strenghts);
    *found_freqs = NULL;
    *signal_strenghts = NULL;
    founded = -1;
    goto out;
  }

  //copy founded frequencies
  for (i=0; i<founded; i++){
      (*found_freqs)[i] = get_standard_freq(temp_freq[i], session->fact);
      (*signal_strenghts)[i] = temp_strenght[i];
      ALOGI("Copied index %d, freq %d, signal %d\n",i, (*found_freqs)[i], (*signal_strenghts)[i] );
  }
  (*found_freqs)[founded] = 0;
  (*signal_strenghts)[founded] = 0;
  ALOGI("End full scan\n");

out:
  free(temp_freq);
  free(temp_strenght);
  restore_freq(session);
  session->scan_band_run = SCAN_RUN;

  return founded;
}

int v4l2_stop_scan(void ** session_data){
  fm_v4l2_data* session;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);
  session->scan_band_run=SCAN_STOP;
  ALOGI("Stop scan value is %d\n", session->scan_band_run);

  /* 0: the JNI takes anything else for a failure to stop */
  return 0;
}

int v4l2_set_force_mono (void ** session_data, int force_mono){
  fm_v4l2_data* session;
  int ret;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  ret = set_force_mono(session->fd, &session->vt, force_mono);
  return ret;
}

int v4l2_is_rds_data_supported (void ** session_data){
  fm_v4l2_data* session;
  int ret;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  ret = get_RDS_cap(session->fd);
  return ret;
}

int v4l2_is_tuned_to_valid_channel (void ** session_data){
  fm_v4l2_data* session;
  int signal;

  ALOGI("%s:\n", __FUNCTION__);
  session = get_session_data(session_data);

  signal = get_signal_strength(session->fd, &session->vt);
  if ( signal > session->threshold )
    return 1;
  else
    return 0;
}

/* AF codes 1..204 are 87.6..107.9 MHz, in kHz; anything else is not a
 * frequency (fillers, counts, LF/MF) */
static void rds_add_af(rds_state* rds, unsigned char code)
{
  int i, khz;

  if (code < 1 || code > 204)
    return;
  khz = 87500 + code * 100;
  for (i = 0; i < rds->num_afs; i++)
    if (rds->af[i] == khz)
      return;
  if (rds->num_afs < RDS_MAX_AFS) {
    rds->af[rds->num_afs++] = khz;
    rds->changed |= FMRADIO_RDS_AF_CHANGED;
  }
}

/* A text complete: to UTF-8 in out (size out_size) */
static void rds_text_out(const rds_text* t, enum rds_charset charset,
                         char* out, size_t out_size)
{
  rds_to_utf8(t->raw, rds_text_length(t), charset, out, out_size);
}

/* What the app gets: the long and enhanced forms if the station sends them */
static void rds_publish(rds_state* rds)
{
  const char* psn = rds->long_ps_out[0] ? rds->long_ps_out : rds->ps_out;
  const char* rt = rds->ert_out[0] ? rds->ert_out : rds->rt_out;

  if (strcmp(psn, rds->psn_pub) != 0) {
    strcpy(rds->psn_pub, psn);
    rds->changed |= FMRADIO_RDS_PS_CHANGED;
  }
  if (strcmp(rt, rds->rt_pub) != 0) {
    strcpy(rds->rt_pub, rt);
    rds->changed |= FMRADIO_RDS_RT_CHANGED;
  }
}

/* One whole group: blocks A..D, each [lsb, msb] */
static void rds_group(rds_state* rds)
{
  unsigned char* g = rds->group;
  int type = g[3] >> 4;                       /* block B msb: group type */
  int version_b = (g[3] >> 3) & 1;
  int code = (type << 1) | version_b;
  /* a text's bytes are in blocks B (its segment) and C, D */
  int suspect = (rds->corrected & 0x0e) != 0;
  unsigned char c[4];

  /* the station's identity: taken once two groups agree, an errored
   * block A otherwise giving another station's code */
  if (rds->pi == ((g[1] << 8) | g[0]) && rds->pi != rds->pi_out) {
    rds->pi_out = rds->pi;
    rds->changed |= FMRADIO_RDS_PI_CHANGED;
  }
  rds->pi = (g[1] << 8) | g[0];
  rds->tp = (g[3] >> 2) & 1;
  rds->pty = ((g[3] & 0x03) << 3) | (g[2] >> 5);

  /* blocks C and D, in order */
  c[0] = g[5]; c[1] = g[4]; c[2] = g[7]; c[3] = g[6];

  if (rds->ert_group >= 0 && code == rds->ert_group) {
    /* eRT: 32 segments of 4 bytes */
    if (rds_text_put(&rds->ert, g[2] & 0x1f, c, suspect))
      rds_text_out(&rds->ert, rds->ert_charset, rds->ert_out,
                   sizeof(rds->ert_out));
    rds_publish(rds);
    return;
  }

  switch (type) {
  case 0:                                     /* 0A/0B: PS, TA/MS, AF */
    rds->ta = (g[2] >> 4) & 1;
    rds->ms = (g[2] >> 3) & 1;
    if (rds_text_put(&rds->ps, g[2] & 0x03, c + 2, suspect)) {
      rds_text_out(&rds->ps, RDS_CHARSET_BASIC, rds->ps_out,
                   sizeof(rds->ps_out));
      /* again from the start: a name may change, segment by segment */
      rds->ps.have = 0;
    }
    if (!version_b && !(rds->corrected & 0x04)) {
      rds_add_af(rds, g[5]);
      rds_add_af(rds, g[4]);
    }
    break;

  case 2: {                                   /* 2A/2B: radio text */
    int ab = (g[2] >> 4) & 1;
    int per = version_b ? 2 : 4;              /* characters per segment */

    if (ab != rds->rt_ab) {
      /* new text: start again -- on a flag from a corrected block B,
       * only once it came twice */
      if ((rds->corrected & 0x02) && ab != rds->ab_cand) {
        rds->ab_cand = ab;
        break;
      }
      rds->rt_ab = ab;
      rds->ab_cand = -1;
      rds_text_init(&rds->rt, RDS_RT_MAX_LENGTH / (4 / per), per, 1);
    } else if (per != rds->rt.per) {
      rds_text_init(&rds->rt, RDS_RT_MAX_LENGTH / (4 / per), per, 1);
    }
    if (rds_text_put(&rds->rt, g[2] & 0x0f, version_b ? c + 2 : c, suspect))
      rds_text_out(&rds->rt, RDS_CHARSET_BASIC, rds->rt_out,
                   sizeof(rds->rt_out));
    break;
  }

  case 3:                                     /* 3A: an open data application */
    if (!version_b && !suspect && ((g[7] << 8) | g[6]) == RDS_ODA_ERT) {
      rds->ert_group = g[2] & 0x1f;
      rds->ert_charset = (g[4] & 0x01) ? RDS_CHARSET_UTF8 : RDS_CHARSET_UCS2;
    }
    break;

  case 15:                                    /* 15A: Long PS, UTF-8 */
    if (!version_b &&
        rds_text_put(&rds->long_ps, g[2] & 0x07, c, suspect))
      rds_text_out(&rds->long_ps, RDS_CHARSET_UTF8, rds->long_ps_out,
                   sizeof(rds->long_ps_out));
    break;

  default:
    break;
  }
  rds_publish(rds);
}

/* One block from the driver: [lsb, msb, block number and flags] */
static void rds_block(rds_state* rds, unsigned char lsb, unsigned char msb,
                      unsigned char info)
{
  int block = info & 0x07;

  /* uncorrectable, or not a block of the four */
  if ((info & 0x80) || block == 7) {
    rds->next_block = 0;
    return;
  }
  if (block == 4)                             /* C' is C */
    block = 2;
  if (block > 3)                              /* E: not RDS */
    return;

  if (block != rds->next_block) {
    /* a group starts again at A; anything else breaks it */
    if (block != 0) {
      rds->next_block = 0;
      return;
    }
  }

  if (block == 0)
    rds->corrected = 0;
  if (info & 0x40)                            /* the chip corrected it */
    rds->corrected |= 1u << block;
  rds->group[block * 2] = lsb;
  rds->group[block * 2 + 1] = msb;
  rds->next_block = block + 1;

  if (rds->next_block == 4) {
    rds_group(rds);
    rds->next_block = 0;
  }
}

/*
 * Reads what the driver has (the device does not block) into the RDS
 * state, fills the bundle with what is complete, and returns what changed
 * since the last call, FMRADIO_RDS_*_CHANGED; 0 for nothing new, -1 on a
 * read error.
 */
int v4l2_get_rds(void * * session_data, struct fmradio_rds_bundle_t * fmradio_rds_bundle) {
  fm_v4l2_data * session = get_session_data(session_data);
  rds_state* rds = &session->rds;
  unsigned char buf[BUFFER_RDS_SIZE];
  int bytesNum, i, changed, freq;
  int moved = 0;

  /* The driver switches to a station's alternative frequency by itself:
   * the tuner then is not where it was set. What RDS was gathered stays
   * the station's, but starts over with the new frequency's. */
  freq = get_freq(session->fd);
  if (freq >= 0 && freq != session->freq) {
    session->freq = freq;
    rds_reset(session);
    moved = 1;
  }

  for (;;) {
    bytesNum = read(session->fd, buf, sizeof(buf) - sizeof(buf) % 3);
    if (bytesNum < 0) {
      if (errno == EINTR)
        continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        break;
      ALOGE("Error on RDS read: %s\n", strerror(errno));
      return -1;
    }
    if (bytesNum == 0)
      break;
    for (i = 0; i + 2 < bytesNum; i += 3)
      rds_block(rds, buf[i], buf[i + 1], buf[i + 2]);
    if (bytesNum < (int) (sizeof(buf) - sizeof(buf) % 3))
      break;
  }

  memset(fmradio_rds_bundle, 0, sizeof(*fmradio_rds_bundle));
  fmradio_rds_bundle->pi = rds->pi_out;
  fmradio_rds_bundle->tp = rds->tp;
  fmradio_rds_bundle->pty = rds->pty;
  fmradio_rds_bundle->ta = rds->ta;
  fmradio_rds_bundle->ms = rds->ms;
  fmradio_rds_bundle->num_afs = rds->num_afs;
  memcpy(fmradio_rds_bundle->af, rds->af, sizeof(rds->af));
  strcpy(fmradio_rds_bundle->psn, rds->psn_pub);
  strcpy(fmradio_rds_bundle->rt, rds->rt_pub);

  changed = rds->changed;
  rds->changed = 0;
  if (moved)
    changed |= FMRADIO_RDS_FREQ_CHANGED;
  return changed;
}

int register_fmradio_functions(long *signature, struct fmradio_vendor_methods_t *vendor_methods)
{
    memset(vendor_methods, 0, sizeof(*vendor_methods));

    vendor_methods->set_frequency = v4l2_set_frequency;
    vendor_methods->get_frequency = v4l2_get_frequency;
    vendor_methods->get_threshold = v4l2_get_threshold;
    vendor_methods->set_threshold = v4l2_set_threshold;
    vendor_methods->is_rds_data_supported = v4l2_is_rds_data_supported;
    vendor_methods->scan = v4l2_scan;
    vendor_methods->is_tuned_to_valid_channel = v4l2_is_tuned_to_valid_channel;
    vendor_methods->full_scan = v4l2_full_scan;
    vendor_methods->stop_scan = v4l2_stop_scan;
    vendor_methods->is_playing_in_stereo = v4l2_is_playing_in_stereo;
    vendor_methods->get_signal_strength = v4l2_get_signal_strength;
    vendor_methods->rx_start = v4l2_rx_start_func;
    vendor_methods->pause=v4l2_pause;
    vendor_methods->resume=v4l2_resume;
    vendor_methods->reset=v4l2_reset;
    vendor_methods->set_force_mono=v4l2_set_force_mono;
    vendor_methods->mute=v4l2_mute;
    vendor_methods->get_rds=v4l2_get_rds;

    *signature = FMRADIO_SIGNATURE;
    return 0;
}
