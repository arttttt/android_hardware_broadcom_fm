/*
 * Copyright (C) ST-Ericsson SA 2010
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
 * Authors: johan.xj.palmaeus@stericsson.com
 *          stuart.macdonald@stericsson.com
 *          for ST-Ericsson
 */

/*
 * Native part of the generic RX FmRadio inteface
 */

#define ALOG_TAG "FmReceiverServiceNative"

// #define LOG_NDEBUG 1

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdarg.h>
#include <signal.h>
#include <pthread.h>
#include <math.h>


#include "jni.h"
#include <nativehelper/JNIHelp.h>
#include "android_fmradio_Receiver.h"
#include <utils/Log.h>


/* *INDENT-OFF* */
namespace android {


// state machine

static const ValidEventsForStates_t IsValidRxEventForState = {
  /* this table defines valid transitions. (turn off indent, we want this easy readable) */
             /* FMRADIO_STATE_ IDLE,STARTING,STARTED,PAUSED,SCANNING,EXTRA_COMMAND */

   /* FMRADIO_EVENT_START */         {true ,false,false,false,false,false},
   /* FMRADIO_EVENT_START_ASYNC */   {true ,false,false,false,false,false},
   /* FMRADIO_EVENT_PAUSE */         {false,false,true, true, false,false},
   /* FMRADIO_EVENT_RESUME */        {false,false,true, true, false,false},
   /* FMRADIO_EVENT_RESET */         {true, true, true, true, true, true },
   /* FMRADIO_EVENT_GET_FREQUENCY */ {false,false,true, true, false,false},
   /* FMRADIO_EVENT_SET_FREQUENCY */ {false,false,true, true, false,false},
   /* FMRADIO_EVENT_SET_PARAMETER */ {false,false,true, true, true, true },
   /* FMRADIO_EVENT_STOP_SCAN */     {true, true, true, true, true, true },
   /* FMRADIO_EVENT_EXTRA_COMMAND */ {true, true, true, true, true, true },
   /* Rx Only */
   /* FMRADIO_EVENT_GET_PARAMETER */ {false,false,true, true, true, true },
   /* FMRADIO_EVENT_GET_SIGNAL_STRENGTH */{false,false,true,true,false,false},
   /* FMRADIO_EVENT_SCAN */          {false,false,true, true, false,false},
   /* FMRADIO_EVENT_FULL_SCAN */     {false,false,true, true, false,false},
   // Tx Only - never allowed
   /* FMRADIO_EVENT_BLOCK_SCAN */    {false,false,false,false,false,false},
};
/*  *INDENT-ON*  */

/* Callbacks to java layer */

static void androidFmRadioRxCallbackOnStateChanged(int oldState,
                                                   int newState);
static void androidFmRadioRxCallbackOnError(void);

static void androidFmRadioRxCallbackOnStarted(void);

static void androidFmRadioRxCallbackOnScan(int foundFreq,
                                           int signalStrength,
                                           int scanDirection,
                                           bool aborted);
static void androidFmRadioRxCallbackOnFullScan(int noItems,
                                               int *frequencies,
                                               int *sigStrengths,
                                               bool aborted);
static void androidFmRadioRxCallbackOnForcedReset(enum fmradio_reset_reason_t reason);

static void androidFmRadioRxCallbackOnVendorForcedReset(enum fmradio_reset_reason_t reason);

static void androidFmRadioRxCallbackOnSignalStrengthChanged(int newLevel);

static void androidFmRadioRxCallbackOnRDSDataFound(struct
                                                   fmradio_rds_bundle_t
                                                   *t, int frequency);

static void androidFmRadioRxCallbackOnPlayingInStereo(int
                                                      isPlayingInStereo);

static void androidFmRadioRxCallbackOnExtraCommand(char* command,
                                                   struct
                                                   fmradio_extra_command_ret_item_t
                                                   *retItem);

struct FmSession_t fmTransmitterSession;

struct FmSession_t fmReceiverSession = {
    NULL,
    NULL,
    false,
    FMRADIO_STATE_IDLE,
    NULL,
    &IsValidRxEventForState,
    NULL,
    NULL,
    &fmTransmitterSession,
    NULL,
    FMRADIO_STATE_IDLE,
    false,
    false,
    false,
    &rx_tx_common_mutex,
    PTHREAD_COND_INITIALIZER,
    NULL,
};

/*
 *  function calls from java layer.
 */

static jint androidFmRadioRxGetState(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    FmRadioState_t state;

    ALOGI("androidFmRadioRxGetState, state\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    state = fmReceiverSession.state;
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return state;
}

/* common ones with tx, just forward to the generic androidFmRadioxxxxx version */

static bool
androidFmRadioRxStart(JNIEnv * env, jobject obj, int lowFreq,
                      int highFreq, int defaultFreq, int grid)
{
    ALOGI("androidFmRadioRxStart. LowFreq %d, HighFreq %d, DefaultFreq %d, grid %d.", lowFreq, highFreq, defaultFreq, grid);

    if (fmReceiverSession.jobj == NULL)
        fmReceiverSession.jobj = env->NewGlobalRef(obj);
    return androidFmRadioStart(&fmReceiverSession, FMRADIO_RX, false, lowFreq,
                               highFreq, defaultFreq, grid);
}


static bool
androidFmRadioRxStartAsync(JNIEnv * env, jobject obj, int lowFreq,
                           int highFreq, int defaultFreq, int grid)
{
  //  ALOGI("androidFmRadioRxStartAsync...");

    if (fmReceiverSession.jobj == NULL)
        fmReceiverSession.jobj = env->NewGlobalRef(obj);
    return androidFmRadioStart(&fmReceiverSession, FMRADIO_RX,true,
                               lowFreq, highFreq, defaultFreq, grid);
}

static void androidFmRadioRxPause(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
  //  ALOGI("androidFmRadioRxPause\n");

    (void)androidFmRadioPause(&fmReceiverSession);
}

static void androidFmRadioRxResume(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
  //  ALOGI("androidFmRadioRxResume\n");
    (void)androidFmRadioResume(&fmReceiverSession);
}

/* The state reset from, or a negative error: an int, not a jboolean,
 * which would take an error for a state */
static int androidFmRadioRxReset(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    int retval = 0;

  //  ALOGI("androidFmRadioRxReset");
    retval = androidFmRadioReset(&fmReceiverSession);

    if (retval >= 0 && fmReceiverSession.state == FMRADIO_STATE_IDLE &&
        fmReceiverSession.jobj != NULL) {
        env->DeleteGlobalRef(fmReceiverSession.jobj);
        fmReceiverSession.jobj = NULL;
    }

    return retval;
}

static int androidFmRadioRxMute(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, jint mute)
{
  //  ALOGI("androidFmRadioRxPause\n");

    return androidFmRadioMute(&fmReceiverSession, mute);
}

static int
androidFmRadioRxSetFrequency(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, jint frequency)
{
  //  ALOGI("androidFmRadioRxSetFrequency tuneTo:%d\n", (int) frequency);
    return androidFmRadioSetFrequency(&fmReceiverSession, (int) frequency);
}

static jint androidFmRadioRxGetFrequency(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
  //  ALOGI("androidFmRadioRxGetFrequency:\n");
    return androidFmRadioGetFrequency(&fmReceiverSession);
}

static jint androidFmRadioRxStopScan(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
  //  ALOGI("androidFmRadioRxStopScan\n");
    return androidFmRadioStopScan(&fmReceiverSession);
}

/* the rest of the calls are specific for RX */

static jint androidFmRadioRxGetSignalStrength(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    int retval = SIGNAL_STRENGTH_UNKNOWN;

  //  ALOGI("androidFmRadioRxGetSignalStrength\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    if (!androidFmRadioIsValidEventForState
        (&fmReceiverSession, FMRADIO_EVENT_GET_SIGNAL_STRENGTH)) {
        goto drop_lock;
    }

    if (fmReceiverSession.vendorMethods_p->get_signal_strength) {
        /* if in pause state temporary resume */
        androidFmRadioTempResumeIfPaused(&fmReceiverSession);

        retval =
            fmReceiverSession.vendorMethods_p->
            get_signal_strength(&fmReceiverSession.vendorData_p);

        if (retval < 0) {
            retval = SIGNAL_STRENGTH_UNKNOWN;
        } else if (retval > SIGNAL_STRENGTH_MAX) {
            retval = SIGNAL_STRENGTH_MAX;
        }
        androidFmRadioPauseIfTempResumed(&fmReceiverSession);
    }

  drop_lock:

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return retval;
}

static jboolean
androidFmRadioRxIsPlayingInStereo(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    bool retval;

  //  ALOGI("androidFmRadioRxIsPlayingInStereo:\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    /* if we haven't register we don't know yet */
    if (!fmReceiverSession.isRegistered) {
        retval = false;
        goto drop_lock;
    }
    // valid in all states
    if (fmReceiverSession.vendorMethods_p->is_playing_in_stereo != NULL) {
        retval =
            fmReceiverSession.vendorMethods_p->
            is_playing_in_stereo(&fmReceiverSession.vendorData_p);
    } else {
        retval = false;
    }

  drop_lock:

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return retval;
}

static int
androidFmRadioRxIsRDSDataSupported(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    bool retval;

 //   ALOGI("androidFmRadioRxIsRDSDataSupported:\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    /* if we haven't register we don't know yet */
    if (!fmReceiverSession.isRegistered) {
        retval = false;
        goto drop_lock;
    }
    // valid in all states
    if (fmReceiverSession.vendorMethods_p->is_rds_data_supported != NULL) {
        /* a capability bit, or -1 on error */
        retval =
            fmReceiverSession.vendorMethods_p->
            is_rds_data_supported(&fmReceiverSession.vendorData_p) > 0;
    } else {
        retval = false;
    }

  drop_lock:

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return retval;
}

static jboolean
androidFmRadioRxIsTunedToValidChannel(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    bool retval;

 //   ALOGI("androidFmRadioRxIsTunedToValidChannel:\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    /* if we haven't register we don't know yet */
    if (!fmReceiverSession.isRegistered) {
        retval = false;
        goto drop_lock;
    }
    // valid in all states
    if (fmReceiverSession.vendorMethods_p->is_tuned_to_valid_channel != NULL) {
        retval =
            fmReceiverSession.vendorMethods_p->
            is_tuned_to_valid_channel(&fmReceiverSession.vendorData_p);
    } else {
        retval = false;
    }

  drop_lock:

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return retval;
}

/*
 * Takes the session into SCANNING for a scan the state allows: STARTED or
 * PAUSED, with the library loaded. Under the lock, so nothing can reset the
 * session between the check and the scan's start.
 */
static bool androidFmRadioRxEnterScanning(enum FmRadioCommand_t event)
{
    bool ok;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    ok = androidFmRadioIsValidEventForState(&fmReceiverSession, event);
    if (ok) {
        FMRADIO_SET_STATE(&fmReceiverSession, FMRADIO_STATE_SCANNING);
        /* a stop belongs to the scan it was sent to */
        fmReceiverSession.lastScanAborted = false;
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return ok;
}

/* 0 with the frequency found, or -1 */
static int androidFmRadioRxScan(enum fmradio_seek_direction_t scanDirection, jint *frequency)
{
    int signalStrength = -1;
    int retval = -1;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    // we should still be in SCANNING mode, but we can't be 100.00 % sure since main thread released lock
    // before we could run

    if (fmReceiverSession.state != FMRADIO_STATE_SCANNING) {
        ALOGE("execute_androidFmRadioRxScan - warning, state not scanning");
    }

    /*
     * if mode has been changed to IDLE in the mean time by main thread,
     * exit the worker thread gracefully
     */
    if (fmReceiverSession.state == FMRADIO_STATE_IDLE) {
        goto drop_lock;
    }

    if (pthread_cond_signal(&fmReceiverSession.sync_cond) != 0) {
        ALOGE("execute_androidFmRadioRxScan - warning, signal failed");
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    retval =
        fmReceiverSession.vendorMethods_p->scan(&fmReceiverSession.
                                                vendorData_p,
                                                scanDirection);

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    if (retval >= 0) {
        // also get signal strength (if supported)
        if (fmReceiverSession.vendorMethods_p->get_signal_strength)
            signalStrength =
                fmReceiverSession.vendorMethods_p->
                get_signal_strength(&fmReceiverSession.vendorData_p);
    }
    /*
     * if state has changed we should keep it, probably a forced reset
     */
    if (fmReceiverSession.state != FMRADIO_STATE_SCANNING) {
        ALOGI("State changed while scanning (state now %d), keeping",
             fmReceiverSession.state);
        retval = -1;
    } else {
        FMRADIO_SET_STATE(&fmReceiverSession, FMRADIO_STATE_STARTED);
        // if we failed but we have a pending abort just read the current frequency to give a proper
        // onScan return

        if (retval < 0 && fmReceiverSession.lastScanAborted &&
            fmReceiverSession.vendorMethods_p->get_frequency) {
            retval = fmReceiverSession.vendorMethods_p->get_frequency(&fmReceiverSession.vendorData_p);
        }
    }

    fmReceiverSession.pendingPause = false;

    if (retval >= 0) {
        *frequency = retval;
    }

    /* A reset waiting for the scan to end may go on */
    if (pthread_cond_signal(&fmReceiverSession.sync_cond) != 0) {
        ALOGE("execute_androidFmRadioRxScan - signal failed");
    }

drop_lock:
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return retval >= 0 ? 0 : -1;
}

static int
androidFmRadioRxScanUp(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, jint *frequency)
{
  //  ALOGI("androidFmRadioRxScanUp\n");
    if (!androidFmRadioRxEnterScanning(FMRADIO_EVENT_SCAN)) {
        return -1;
    }
    return androidFmRadioRxScan(FMRADIO_SEEK_UP, frequency);
}

static int
androidFmRadioRxScanDown(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, jint *frequency)
{
  //  ALOGI("androidFmRadioRxScanDown\n");
    if (!androidFmRadioRxEnterScanning(FMRADIO_EVENT_SCAN)) {
        return -1;
    }
    return androidFmRadioRxScan(FMRADIO_SEEK_DOWN, frequency);
}

/*
 * Up to max stations into frequencies, in kHz; returns how many, or -1.
 * The vendor library says how many it found: that count is what is read.
 */
static int androidFmRadioRxFullScan(int *frequencies, int max)
{
    int retval = -1;
    int count = 0;
    int *frequencies_p = NULL;
    int *rssi_p = NULL;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    // we should still be in SCANNING mode, but we can't be 100.00 % sure since main thread released lock
    // before we could run

    if (fmReceiverSession.state != FMRADIO_STATE_SCANNING) {
        ALOGE("execute_androidFmRadioRxFullScan - warning, state not scanning\n");
    }

    /*
     * if mode has been changed to IDLE in the mean time by main thread,
     * exit the worker thread gracefully
     */
    if (fmReceiverSession.state == FMRADIO_STATE_IDLE) {
        goto drop_lock;
    }

    if (pthread_cond_signal(&fmReceiverSession.sync_cond) != 0) {
        ALOGE("execute_androidFmRadioRxFullScan - warning, signal failed\n");
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    retval = fmReceiverSession.vendorMethods_p->full_scan(&fmReceiverSession.
                                                    vendorData_p,
                                                    &frequencies_p,
                                                    &rssi_p);
    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    /*
     * if state has changed we should keep it, probably a forced pause or
     * forced reset
     */
    if (fmReceiverSession.state != FMRADIO_STATE_SCANNING) {
        ALOGI("State changed while scanning (state now %d), keeping\n",
             fmReceiverSession.state);
        retval = -1;
    } else {
        FMRADIO_SET_STATE(&fmReceiverSession, FMRADIO_STATE_STARTED);
    }

    if (retval >= 0 && frequencies_p != NULL) {
        for (int i = 0; i < retval && count < max; i++) {
           if (frequencies_p[i] <= 0)
                break;
           frequencies[count++] = frequencies_p[i];
        }
    }

    if (frequencies_p != NULL) {
        free(frequencies_p);
    }

    if (rssi_p != NULL) {
        free(rssi_p);
    }

    drop_lock:
    /* Wake up the main thread if it is currently waiting on the condition variable */
    if (pthread_cond_signal(&fmReceiverSession.sync_cond) != 0) {
        ALOGE("execute_androidFmRadioRxFullScan - signal failed\n");
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return retval >= 0 ? count : -1;
}

static int androidFmRadioRxStartFullScan(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, int *frequencies, int max)
{
  //  ALOGI("androidFmRadioRxStartFullScan\n");
    if (!androidFmRadioRxEnterScanning(FMRADIO_EVENT_FULL_SCAN)) {
        return -1;
    }
    return androidFmRadioRxFullScan(frequencies, max);
}

static void androidFmRadioRxSetForceMono(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj,
                                         jboolean forceMono)
{
    int retval = -1;

   // ALOGI("androidFmRadioRxSetForceMono\n");

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    if (!androidFmRadioIsValidEventForState
        (&fmReceiverSession, FMRADIO_EVENT_SET_PARAMETER)) {
        retval = FMRADIO_INVALID_STATE;
        goto drop_lock;
    }


    if (fmReceiverSession.vendorMethods_p->set_force_mono) {
        /* if in pause state temporary resume */
        androidFmRadioTempResumeIfPaused(&fmReceiverSession);

        retval =
            fmReceiverSession.vendorMethods_p->
            set_force_mono(&fmReceiverSession.vendorData_p, forceMono);

        androidFmRadioPauseIfTempResumed(&fmReceiverSession);
    } else {
        retval = FMRADIO_UNSUPPORTED_OPERATION;
    }

  drop_lock:
    if (retval == FMRADIO_INVALID_STATE) {
        THROW_INVALID_STATE(&fmReceiverSession);
    } else if (retval < 0) {
        THROW_IO_ERROR(&fmReceiverSession);
    }

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
}

static void
androidFmRadioRxSetThreshold(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj, jint threshold)
{
    int retval;

  //  ALOGI("androidFmRadioRxSetThreshold threshold:%d\n", (int) threshold);

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    if (!androidFmRadioIsValidEventForState
        (&fmReceiverSession, FMRADIO_EVENT_SET_PARAMETER)) {
        retval = FMRADIO_INVALID_STATE;
        goto drop_lock;
    }


    if (fmReceiverSession.vendorMethods_p->set_threshold) {
        /* if in pause state temporary resume */
        androidFmRadioTempResumeIfPaused(&fmReceiverSession);

        retval =
            fmReceiverSession.
            vendorMethods_p->set_threshold(&fmReceiverSession.vendorData_p,
                                          threshold);
        /* if in pause state temporary resume */
        androidFmRadioPauseIfTempResumed(&fmReceiverSession);
    } else {
        retval = FMRADIO_UNSUPPORTED_OPERATION;
    }

    if (retval == FMRADIO_INVALID_STATE) {
        THROW_INVALID_STATE(&fmReceiverSession);
    } else if (retval < 0) {
        THROW_IO_ERROR(&fmReceiverSession);
    }

  drop_lock:
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
}

static jint androidFmRadioRxGetThreshold(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj)
{
    int retval;

   // ALOGI("androidFmRadioRxGetThreshold\n");
    pthread_mutex_lock(fmReceiverSession.dataMutex_p);

    if (!androidFmRadioIsValidEventForState
        (&fmReceiverSession, FMRADIO_EVENT_GET_PARAMETER)) {
        retval = FMRADIO_INVALID_STATE;
        goto drop_lock;
    }

    if (fmReceiverSession.vendorMethods_p->get_threshold) {
        /* if in pause state temporary resume */
        androidFmRadioTempResumeIfPaused(&fmReceiverSession);
        retval =
            fmReceiverSession.
            vendorMethods_p->get_threshold(&fmReceiverSession.vendorData_p);
        androidFmRadioPauseIfTempResumed(&fmReceiverSession);
    } else {
        retval = FMRADIO_UNSUPPORTED_OPERATION;
    }
  drop_lock:

    if (retval == FMRADIO_INVALID_STATE) {
        THROW_INVALID_STATE(&fmReceiverSession);
    } else if (retval < 0) {
        THROW_IO_ERROR(&fmReceiverSession);
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return retval;
}

static jboolean androidFmRadioRxSendExtraCommand(JNIEnv * env, jobject obj,
                                                 jstring command,
                                                 jobjectArray parameters)
{
   // ALOGI("androidFmRadioRxSendExtraCommand");

/* we need to set jobj since this might be called before start */

    if (fmReceiverSession.jobj == NULL)
        fmReceiverSession.jobj = env->NewGlobalRef(obj);

    androidFmRadioSendExtraCommand(&fmReceiverSession, env, command,
                                   parameters);

    return true;
}

jboolean tune(JNIEnv *env, jobject thiz, jfloat freq)
{
    int ret = 0;
    int tmp_freq;

    tmp_freq = (int)(freq * 1000);        //Eg, 87.5 * 10 --> 875
    ret = androidFmRadioRxSetFrequency(env, thiz, tmp_freq);

    ALOGD("%s, [ret=%d]\n", __func__, ret);
    return ret <= 0 ? JNI_FALSE:JNI_TRUE;
}

/*
 * The band FMRadio works in: FmUtils.LOWEST_STATION..HIGHEST_STATION in
 * steps of STEP, 87.5-108 MHz at 100 kHz. The app has no other; a regional
 * band would have to come from both.
 */
#define FM_BAND_LOW_KHZ   87500
#define FM_BAND_HIGH_KHZ  108000
#define FM_BAND_STEP_KHZ  100

jboolean powerUp(JNIEnv *env, jobject thiz, jfloat freq)
{
    int ret = 0;
    int tmp_freq;

    /* Already on: up is resumed, and that is a success */
    if (androidFmRadioRxGetState(env, thiz) != FMRADIO_STATE_IDLE) {
        return androidFmRadioResume(&fmReceiverSession) >= 0 ?
                JNI_TRUE : JNI_FALSE;
    }

 //   ALOGI("%s, [freq=%d]\n", __func__, (int)freq);
    tmp_freq = (int)(freq * 1000);        //Eg, 87.5 * 10 --> 875
    ret = androidFmRadioRxStart(env, thiz, FM_BAND_LOW_KHZ, FM_BAND_HIGH_KHZ,
                                tmp_freq, FM_BAND_STEP_KHZ);
 //   ALOGD("%s, [ret=%d]\n", __func__, ret);
    return ret?JNI_FALSE:JNI_TRUE;
}

jint setMute(JNIEnv *env, jobject thiz, jboolean mute)
{
    int ret = 1;

    ret = androidFmRadioRxMute(env, thiz, (int)mute);
    if (ret) {
        ALOGE("%s, error, [ret=%d]\n", __func__, ret);
    }
   // ALOGD("%s, [mute=%d] [ret=%d]\n", __func__, (int)mute, ret);
    return ret?JNI_FALSE:JNI_TRUE;
}

/*
 * RDS for the app, which polls readRds() and, on the events it returns,
 * reads the station name (getPs) or the radio text (getLrText). The vendor
 * library puts PS and RT together; what was complete at the last readRds
 * is kept here, under the session lock, for the getters.
 */

/* FmService's RDS event bits */
#define RDS_EVENT_PI_CODE         0x0002
#define RDS_EVENT_PROGRAMNAME     0x0008
#define RDS_EVENT_LAST_RADIOTEXT  0x0040
#define RDS_EVENT_AF              0x0080

static struct fmradio_rds_bundle_t rdsLatest;
static bool rdsOn = true;
/* where the driver's AF switching moved the tuner, in the app's 100 kHz
 * units, until activeAf hands it over; -1 for nowhere */
static int rdsAfStation = -1;

jint isRdsSupport(JNIEnv *env, jobject thiz)
{
    /* the app wants 1 for yes; the driver gives a capability bit */
    return androidFmRadioRxIsRDSDataSupported(env, thiz) ? 1 : 0;
}

jshort readRds(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz)
{
    struct fmradio_rds_bundle_t bundle;
    int changed;
    jshort events = 0;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    if (!rdsOn ||
            !androidFmRadioIsValidEventForState(&fmReceiverSession,
                                                FMRADIO_EVENT_GET_PARAMETER) ||
            fmReceiverSession.vendorMethods_p->get_rds == NULL) {
        pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
        return 0;
    }

    changed = fmReceiverSession.vendorMethods_p->get_rds(
            &fmReceiverSession.vendorData_p, &bundle);
    if (changed >= 0) {
        rdsLatest = bundle;
        if (changed & FMRADIO_RDS_PI_CHANGED)
            events |= RDS_EVENT_PI_CODE;
        if (changed & FMRADIO_RDS_PS_CHANGED)
            events |= RDS_EVENT_PROGRAMNAME;
        if (changed & FMRADIO_RDS_RT_CHANGED)
            events |= RDS_EVENT_LAST_RADIOTEXT;
        if ((changed & FMRADIO_RDS_FREQ_CHANGED) &&
                fmReceiverSession.vendorMethods_p->get_frequency != NULL) {
            int khz = fmReceiverSession.vendorMethods_p->get_frequency(
                    &fmReceiverSession.vendorData_p);

            if (khz > 0) {
                rdsAfStation = khz / 100;
                events |= RDS_EVENT_AF;
            }
        }
    }
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    return events;
}

/* Off, readRds reports nothing and reads nothing (the app turns it off
 * with the screen) */
jint setRds(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz, jboolean rdson)
{
    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    rdsOn = rdson;
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return 0;
}

static jbyteArray rdsText(JNIEnv *env, const char *text, size_t max)
{
    char copy[RDS_RT_MAX_LENGTH + 1];
    size_t len;
    jbyteArray array;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    len = strnlen(text, max);
    memcpy(copy, text, len);
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);

    if (len == 0)
        return NULL;
    array = env->NewByteArray(len);
    if (array != NULL)
        env->SetByteArrayRegion(array, 0, len, (const jbyte *) copy);
    return array;
}

/* The radio text */
jbyteArray getLrText(JNIEnv *env, jobject __attribute__((unused)) thiz)
{
    return rdsText(env, rdsLatest.rt, RDS_RT_MAX_LENGTH);
}

/* The station name */
/* The station's PI code, 0 until known */
jint getPi(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz)
{
    jint pi;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    pi = rdsLatest.pi;
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return pi;
}

jbyteArray getPs(JNIEnv *env, jobject __attribute__((unused)) thiz)
{
    return rdsText(env, rdsLatest.psn, RDS_PSN_MAX_LENGTH);
}

/*
 * The frequency the driver's AF switching moved the tuner to, in 100 kHz
 * units, once, after readRds reported RDS_EVENT_AF; -1 if none. The switch
 * is done: the app only has to know it.
 */
jshort activeAf(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz)
{
    int station;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    station = rdsAfStation;
    rdsAfStation = -1;
    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return station;
}

jshortArray autoScan(JNIEnv *env, jobject thiz)
{
    int ret = 0;
    jshortArray scanChlarray;
    int chl_cnt = 0;
    int ScanTBL[50] = { 0 };
    short fixedTable[50];

    ret = androidFmRadioRxStartFullScan(env, thiz, ScanTBL, 50);
    if (ret < 0) {
        ALOGE("scan failed!\n");
        scanChlarray = NULL;
        goto out;
    }

    /* kHz to the app's 100 kHz units */
    for (int i = 0; i < ret; i++) {
         int val = ScanTBL[i]/100;
         if (val <= 0)
              break;
         fixedTable[chl_cnt++] = val & 0xFFFF;
       //  ALOGD("Add freq: %d", fixedTable[i]);
    }

    if (chl_cnt > 0) {
        scanChlarray = env->NewShortArray(chl_cnt);
        env->SetShortArrayRegion(scanChlarray, 0, chl_cnt, fixedTable);
    } else {
        ALOGE("cnt error, [cnt=%d]\n", chl_cnt);
        scanChlarray = NULL;
    }

out:
    ALOGD("%s, [cnt=%d] [ret=%d]\n", __func__, chl_cnt, ret);
    return scanChlarray;
}

jint switchAntenna(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz, jint __attribute__((unused)) antenna)
{
    int ret = 0;
    int ana = -1;
    ALOGD("%s: [antenna=%d] [ret=%d]\n", __func__, ana, ret);
    return ret;
}

jboolean stopScan(JNIEnv *env, jobject thiz)
{
    int ret = 0;

    ret = androidFmRadioRxStopScan(env, thiz);
    if (ret) {
        ALOGE("%s, error, [ret=%d]\n", __func__, ret);
    }
    ALOGD("%s, [ret=%d]\n", __func__, ret);
    /* FMRADIO_OK is a stop sent */
    return ret == FMRADIO_OK ? JNI_TRUE : JNI_FALSE;
}


jfloat seek(JNIEnv *env, jobject thiz, jfloat freq, jboolean isUp) //jboolean isUp;
{
    int ret = 0;
    int tmp_freq;
    jint ret_freq;
    float val;

    tmp_freq = (int)(freq * 1000);       //Eg, 87.55 * 100 --> 8755
    ret_freq = tmp_freq;

    ret = setMute(env, thiz, 1);
    if (ret) {
        ALOGE("%s, error, [ret=%d]\n", __func__, ret);
    }
  //  ALOGD("%s, [mute] [ret=%d]\n", __func__, ret);

    if (isUp) {
          ret = androidFmRadioRxScanUp(env, thiz, &ret_freq);
    } else {
          ret = androidFmRadioRxScanDown(env, thiz, &ret_freq);
    }

    if (ret) {
        ret_freq = tmp_freq; //seek error, so use original freq
    }

    val = roundf((ret_freq/1000.00F) * 100) / 100;

  //  ALOGD("%s, [freq=%f] [ret=%d]\n", __func__, val, ret);

    return val;
}

jboolean powerDown(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) thiz, jint __attribute__((unused)) type)
{
    int ret = 0;

    ret = androidFmRadioRxReset(env, thiz);

    ALOGD("%s, [ret=%d]\n", __func__, ret);
    /* The state it was in, IDLE included (already off), or an error */
    return ret >= 0 ? JNI_TRUE : JNI_FALSE;
}

static jboolean openDev(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj,
                                   jboolean __attribute__((unused)) receiveRDS) {
    return JNI_TRUE;
}

static jboolean closeDev(JNIEnv * __attribute__((unused)) env, jobject __attribute__((unused)) obj,
                                   jboolean __attribute__((unused)) receiveRDS) {
    return JNI_TRUE;
}


static JNINativeMethod gMethods[] = {
    {"openDev", "()Z", (void*)openDev },
    {"closeDev", "()Z", (void*)closeDev },
    {"powerUp", "(F)Z", (void*)powerUp },
    {"powerDown", "(I)Z", (void*)powerDown },
    {"tune", "(F)Z", (void*)tune },
    {"seek", "(FZ)F", (void*)seek },
    {"autoScan",  "()[S", (void*)autoScan },
    {"stopScan",  "()Z", (void*)stopScan },
    {"setRds",    "(Z)I", (void*)setRds  },
    {"readRds",   "()S", (void*)readRds },
    {"getPi",     "()I", (void*)getPi  },
    {"getPs",     "()[B", (void*)getPs  },
    {"getLrText", "()[B", (void*)getLrText},
    {"activeAf",  "()S", (void*)activeAf},
    {"setMute",	"(Z)I", (void*)setMute},
    {"isRdsSupport",	"()I", (void*)isRdsSupport},
    {"switchAntenna", "(I)I", (void*)switchAntenna},
};

int registerAndroidFmRadioReceiver(JavaVM * vm, JNIEnv * env)
{
    ALOGI("registerAndroidFmRadioReceiver\n");
    jclass clazz;

    pthread_mutex_lock(fmReceiverSession.dataMutex_p);
    fmReceiverSession.jvm_p = vm;

    struct bundle_descriptor_offsets_t *bundle_p =
        (struct bundle_descriptor_offsets_t *)
        malloc(sizeof(struct bundle_descriptor_offsets_t));

    clazz = env->FindClass("android/os/Bundle");
    bundle_p->mClass = (jclass) env->NewGlobalRef(clazz);
    bundle_p->mConstructor = env->GetMethodID(clazz, "<init>", "()V");
    bundle_p->mPutInt =
        env->GetMethodID(clazz, "putInt", "(Ljava/lang/String;I)V");
    bundle_p->mPutShort =
        env->GetMethodID(clazz, "putShort", "(Ljava/lang/String;S)V");
    bundle_p->mPutIntArray =
        env->GetMethodID(clazz, "putIntArray", "(Ljava/lang/String;[I)V");
    bundle_p->mPutShortArray =
        env->GetMethodID(clazz, "putShortArray",
                         "(Ljava/lang/String;[S)V");
    bundle_p->mPutString =
        env->GetMethodID(clazz, "putString",
                         "(Ljava/lang/String;Ljava/lang/String;)V");

    fmReceiverSession.bundleOffsets_p = bundle_p;

    pthread_mutex_unlock(fmReceiverSession.dataMutex_p);
    return jniRegisterNativeMethods(env,
                                    "com/android/fmradio/FmNative",
                                    gMethods, NELEM(gMethods));
}

/* *INDENT-OFF* */
};                              // namespace android
