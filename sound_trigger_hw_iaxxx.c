/*
 * Copyright (C) 2018 Knowles Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "SoundTriggerHAL"
#define LOG_NDEBUG 0

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <log/log.h>
#include <cutils/uevent.h>
#include <cutils/properties.h>
#include <math.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/timerfd.h>

#include <hardware/hardware.h>

#include "cvq_ioctl.h"
#include "sound_trigger_hw_iaxxx.h"
#include "sound_trigger_intf.h"

#define MAX_GENERIC_SOUND_MODELS    (9)
#define MAX_KEY_PHRASES             (1)
#define MAX_MODELS                  (MAX_GENERIC_SOUND_MODELS + MAX_KEY_PHRASES)

#define MAX_USERS                   (1)
#define MAX_BUFFER_MS               (3000)
#define POWER_CONSUMPTION           (0) // TBD
#define ST_HAL_VERSION              (1)

#define UEVENT_MSG_LEN              (1024)

#define OK_GOOGLE_KW_ID             (0)
#define AMBIENT_KW_ID               (1)
#define ENTITY_KW_ID                (2)
#define WAKEUP_KW_ID                (3)
#define USELESS_KW_ID               (999)

#define CVQ_ENDPOINT                    (IAXXX_SYSID_PLUGIN_1_OUT_EP_0)
#define MUSIC_BUF_ENDPOINT              (IAXXX_SYSID_PLUGIN_3_OUT_EP_1)

#define IAXXX_VQ_EVENT_STR          "IAXXX_VQ_EVENT"
#define IAXXX_RECOVERY_EVENT_STR    "IAXXX_RECOVERY_EVENT"
#define IAXXX_FW_DWNLD_SUCCESS_STR  "IAXXX_FW_DWNLD_SUCCESS"
#define IAXXX_FW_CRASH_EVENT_STR    "IAXXX_CRASH_EVENT"

#define CARD_NAME                          "iaxxx"
#define SOUND_TRIGGER_MIXER_PATH_BASE      "/vendor/etc/sound_trigger_mixer_paths"
#define SOUND_TRIGGER_MIXER_PATH_XML       "/vendor/etc/sound_trigger_mixer_paths_default.xml"

#define MAX_SND_CARD    (8)
#define RETRY_NUMBER    (10)
#define RETRY_US        (500000)

#ifdef __LP64__
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib64/hw/adnc_strm.primary.default.so"
#else
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib/hw/adnc_strm.primary.default.so"
#endif

static const struct sound_trigger_properties hw_properties = {
    "Knowles Electronics",      // implementor
    "Continous VoiceQ",         // description
    1,                          // version
    // Version UUID
    { 0x80f7dcd5, 0xbb62, 0x4816, 0xa931, {0x9c, 0xaa, 0x52, 0x5d, 0xf5, 0xc7}},
    MAX_MODELS,                 // max_sound_models
    MAX_KEY_PHRASES,            // max_key_phrases
    MAX_USERS,                  // max_users
    RECOGNITION_MODE_VOICE_TRIGGER | // recognition_mode
    RECOGNITION_MODE_GENERIC_TRIGGER,
    true,                       // capture_transition
    MAX_BUFFER_MS,              // max_capture_ms
    false,                      // concurrent_capture
    false,                      // trigger_in_event
    POWER_CONSUMPTION           // power_consumption_mw
};

struct model_info {
    void *recognition_cookie;
    void *sound_model_cookie;
    sound_model_handle_t model_handle;
    sound_trigger_uuid_t uuid;
    recognition_callback_t recognition_callback;
    sound_model_callback_t sound_model_callback;
    struct sound_trigger_recognition_config *config;
    int kw_id;
    sound_trigger_sound_model_type_t type;

    void *data;
    int data_sz;
    bool is_loaded;
    bool is_active;
    bool is_state_query;
};

struct knowles_sound_trigger_device {
    struct sound_trigger_hw_device device;
    struct model_info models[MAX_MODELS];
    sound_trigger_uuid_t authkw_model_uuid;
    pthread_t callback_thread;
    pthread_mutex_t lock;
    int opened;
    int send_sock;
    int recv_sock;
    struct sound_trigger_recognition_config *last_keyword_detected_config;

    // Information about streaming
    int is_streaming;
    void *adnc_cvq_strm_lib;
    int (*adnc_strm_open)(bool, int, int);
    size_t (*adnc_strm_read)(long, void *, size_t);
    int (*adnc_strm_close)(long);
    long adnc_strm_handle;

    sound_trigger_uuid_t hotword_model_uuid;
    sound_trigger_uuid_t sensor_model_uuid;
    sound_trigger_uuid_t ambient_model_uuid;
    sound_trigger_uuid_t chre_model_uuid;
    sound_trigger_uuid_t entity_model_uuid;
    sound_trigger_uuid_t wakeup_model_uuid;

    int last_detected_model_type;
    bool is_mic_route_enabled;
    bool is_music_playing;
    bool is_bargein_route_enabled;
    bool is_buffer_package_loaded;
    bool is_st_hal_ready;
    bool is_hmd_proc_on;
    bool is_dmx_proc_on;
    int hotword_buffer_enable;
    int music_buffer_enable;
    bool is_chre_enable;
    bool is_media_recording;

    unsigned int current_enable;

    struct audio_route *route_hdl;
    struct mixer *mixer;
    struct iaxxx_odsp_hw *odsp_hdl;

    void *audio_hal_handle;
    audio_hw_call_back_t audio_hal_cb;
    unsigned int sthal_prop_api_version;

    int snd_crd_num;
    char mixer_path_xml[NAME_MAX_SIZE];
    bool fw_reset_done_by_hal;
};

/*
 * Since there's only ever one sound_trigger_device, keep it as a global so
 * that other people can dlopen this lib to get at the streaming audio.
 */
static struct knowles_sound_trigger_device g_stdev = {
    .lock = PTHREAD_MUTEX_INITIALIZER
};

static bool check_uuid_equality(sound_trigger_uuid_t uuid1,
                                sound_trigger_uuid_t uuid2)
{
    if (uuid1.timeLow != uuid2.timeLow ||
        uuid1.timeMid != uuid2.timeMid ||
        uuid1.timeHiAndVersion != uuid2.timeHiAndVersion ||
        uuid1.clockSeq != uuid2.clockSeq) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        if(uuid1.node[i] != uuid2.node[i]) {
            return false;
        }
    }

    return true;
}

bool str_to_uuid(char* uuid_str, sound_trigger_uuid_t* uuid)
{
    if (uuid_str == NULL) {
        ALOGI("Invalid str_to_uuid input.");
        return false;
    }

    int tmp[10];
    if (sscanf(uuid_str, "%08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
            tmp, tmp+1, tmp+2, tmp+3, tmp+4, tmp+5,
            tmp+6, tmp+7, tmp+8, tmp+9) < 10) {
        ALOGI("Invalid UUID, got: %s", uuid_str);
        return false;
    }
    uuid->timeLow = (unsigned int)tmp[0];
    uuid->timeMid = (unsigned short)tmp[1];
    uuid->timeHiAndVersion = (unsigned short)tmp[2];
    uuid->clockSeq = (unsigned short)tmp[3];
    uuid->node[0] = (unsigned char)tmp[4];
    uuid->node[1] = (unsigned char)tmp[5];
    uuid->node[2] = (unsigned char)tmp[6];
    uuid->node[3] = (unsigned char)tmp[7];
    uuid->node[4] = (unsigned char)tmp[8];
    uuid->node[5] = (unsigned char)tmp[9];

    return true;
}

static int find_empty_model_slot(struct knowles_sound_trigger_device *st_dev)
{
    int i = -1;
    for (i = 0; i < MAX_MODELS; i++) {
        if (st_dev->models[i].is_loaded == false)
            break;
    }

    if (i >= MAX_MODELS) {
        i = -1;
    }

    return i;
}

static int find_handle_for_kw_id(
                        struct knowles_sound_trigger_device *st_dev, int kw_id)
{
    int i = 0;
    for (i = 0; i < MAX_MODELS; i++) {
        if (kw_id == st_dev->models[i].kw_id)
            break;
    }

    return i;
}

static bool is_any_model_active(struct knowles_sound_trigger_device *stdev) {
    int i = 0;
    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_active == true) {
            break;
        }
    }

    if (i == MAX_MODELS)
        return false;
    else
        return true;
}

static bool is_any_model_loaded(struct knowles_sound_trigger_device *stdev) {
    int i = 0;
    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_loaded == true) {
            break;
        }
    }

    if (i == MAX_MODELS)
        return false;
    else
        return true;
}

static void reg_hal_event_session(
                                struct sound_trigger_recognition_config *config,
                                sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    struct sound_trigger_event_info event_info;
    /*
     * Register config and capture_handle of trigger sound model to audio hal
     * It only register while request capturing buffer.
     */
    if (config->capture_requested && stdev->audio_hal_cb) {
        ALOGD("%s: ST_EVENT_SESSION_REGISTER capture_handle %d model %p",
            __func__, config->capture_handle, &stdev->models[handle]);
        event_info.st_ses.p_ses = (void *)&stdev->models[handle];
        event_info.st_ses.config = stdev_hotword_pcm_config;
        event_info.st_ses.capture_handle = config->capture_handle;
        event_info.st_ses.pcm = NULL;
        stdev->audio_hal_cb(ST_EVENT_SESSION_REGISTER, &event_info);
    }
}

static void dereg_hal_event_session(
                                struct sound_trigger_recognition_config *config,
                                sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    struct sound_trigger_event_info event_info;
    /*
     * Indicate to audio hal that streaming is stopped.
     * Stop capturing data from STHAL.
     */
    if (config->capture_requested && stdev->audio_hal_cb) {
        ALOGD("%s: ST_EVENT_SESSION_DEREGISTER capture_handle %d model %p",
            __func__, config->capture_handle, &stdev->models[handle]);
        event_info.st_ses.p_ses = (void *)&stdev->models[handle];
        event_info.st_ses.capture_handle = config->capture_handle;
        event_info.st_ses.pcm = NULL;
        stdev->audio_hal_cb(ST_EVENT_SESSION_DEREGISTER, &event_info);
    }
}


static char *stdev_keyphrase_event_alloc(sound_model_handle_t handle,
                                struct sound_trigger_recognition_config *config,
                                int recognition_status)
{
    char *data;
    struct sound_trigger_phrase_recognition_event *event;
    data = (char *)calloc(1,
                        sizeof(struct sound_trigger_phrase_recognition_event));
    if (!data)
        return NULL;
    event = (struct sound_trigger_phrase_recognition_event *)data;
    event->common.status = recognition_status;
    event->common.type = SOUND_MODEL_TYPE_KEYPHRASE;
    event->common.model = handle;
    event->common.capture_available = false;

    if (config) {
        unsigned int i;

        event->num_phrases = config->num_phrases;
        if (event->num_phrases > SOUND_TRIGGER_MAX_PHRASES)
            event->num_phrases = SOUND_TRIGGER_MAX_PHRASES;
        for (i = 0; i < event->num_phrases; i++)
            memcpy(&event->phrase_extras[i],
                &config->phrases[i],
                sizeof(struct sound_trigger_phrase_recognition_extra));
    }

    event->num_phrases = 1;
    event->phrase_extras[0].confidence_level = 100;
    event->phrase_extras[0].num_levels = 1;
    event->phrase_extras[0].levels[0].level = 100;
    event->phrase_extras[0].levels[0].user_id = 0;
    /*
     * Signify that all the data is comming through streaming
     * and not through the buffer.
     */
    event->common.capture_available = true;
    event->common.capture_delay_ms = 0;
    event->common.capture_preamble_ms = 0;
    event->common.audio_config = AUDIO_CONFIG_INITIALIZER;
    event->common.audio_config.sample_rate = 16000;
    event->common.audio_config.channel_mask = AUDIO_CHANNEL_IN_MONO;
    event->common.audio_config.format = AUDIO_FORMAT_PCM_16_BIT;

    return data;
}

static char *stdev_generic_event_alloc(int model_handle, void *payload,
                                    unsigned int payload_size,
                                    int recognition_status)
{
    char *data;
    struct sound_trigger_generic_recognition_event *event;

    data = (char *)calloc(1,
                        sizeof(struct sound_trigger_generic_recognition_event) +
                        payload_size);
    if (!data) {
        ALOGE("%s: Failed to allocate memory for recog event", __func__);
        return NULL;
    }

    event = (struct sound_trigger_generic_recognition_event *)data;
    event->common.status = recognition_status;
    event->common.type = SOUND_MODEL_TYPE_GENERIC;
    event->common.model = model_handle;

    /*
     * Signify that all the data is comming through streaming and
     * not through the buffer.
     */
    event->common.capture_available = true;
    event->common.audio_config = AUDIO_CONFIG_INITIALIZER;
    event->common.audio_config.sample_rate = 16000;
    event->common.audio_config.channel_mask = AUDIO_CHANNEL_IN_MONO;
    event->common.audio_config.format = AUDIO_FORMAT_PCM_16_BIT;

    if (payload && payload_size > 0) {
        ALOGD("%s: Attach payload in the event", __func__);
        event->common.data_size = payload_size;
        event->common.data_offset =
                        sizeof(struct sound_trigger_generic_recognition_event);

        memcpy((data + event->common.data_offset), payload, payload_size);
    }

    return data;
}

static void stdev_close_term_sock(struct knowles_sound_trigger_device *stdev)
{
    if (stdev->send_sock >= 0) {
        close(stdev->send_sock);
        stdev->send_sock = -1;
    }

    if (stdev->recv_sock >= 0) {
        close(stdev->recv_sock);
        stdev->recv_sock = -1;
    }
}

static int setup_package(struct knowles_sound_trigger_device *stdev,
                        struct model_info *model)
{
    int err = 0;

    if (check_uuid_equality(model->uuid, stdev->chre_model_uuid)) {
        if (!(stdev->current_enable & CHRE_MASK)) {
            err = setup_chre_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load CHRE package");
                goto exit;
            }
        }
        stdev->current_enable = stdev->current_enable | CHRE_MASK;
    } else if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid)) {
        if (!(stdev->current_enable & PLUGIN1_MASK)) {
            err = setup_hotword_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load Hotword package");
                goto exit;
            }
        }
        err = write_model(stdev->odsp_hdl, model->data, model->data_sz,
                        model->kw_id);
        if (err != 0) {
            ALOGE("Failed to write Hotword model");
            goto exit;
        }

        //setup model state.
        stdev->current_enable = stdev->current_enable | HOTWORD_MASK;
        err = set_hotword_state(stdev->odsp_hdl, stdev->current_enable);
        if (err != 0) {
            ALOGE("Failed to set Hotword state");
            goto exit;
        }
    } else if (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid)) {
        if (!(stdev->current_enable & PLUGIN1_MASK)) {
            err = setup_hotword_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load Hotword package");
                goto exit;
            }
        }
        err = write_model(stdev->odsp_hdl, model->data, model->data_sz,
                        model->kw_id);
        if (err != 0) {
            ALOGE("Failed to write Wakeup model");
            goto exit;
        }

        //setup model state.
        stdev->current_enable = stdev->current_enable | WAKEUP_MASK;
        err = set_hotword_state(stdev->odsp_hdl, stdev->current_enable);
        if (err != 0) {
            ALOGE("Failed to set Wakeup state");
            goto exit;
        }
    } else if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        if (!(stdev->current_enable & PLUGIN2_MASK)) {
            err = setup_ambient_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load Ambient package");
                goto exit;
            }
        } else {
            // tear down plugin2 for writing new model data.
            err = tear_ambient_state(stdev->odsp_hdl,
                                    stdev->current_enable);
        }
        err = write_model(stdev->odsp_hdl, model->data,
                        model->data_sz, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to write Ambient model");
            goto exit;
        }

        //setup model state.
        stdev->current_enable = stdev->current_enable | AMBIENT_MASK;
        err = set_ambient_state(stdev->odsp_hdl, stdev->current_enable);
        if (err != 0) {
            ALOGE("Failed to set Ambient state");
            goto exit;
        }

    } else if (check_uuid_equality(model->uuid, stdev->entity_model_uuid)) {
        if (!(stdev->current_enable & PLUGIN2_MASK)) {
            err = setup_ambient_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load Ambient package");
                goto exit;
            }
        } else {
            // tear down plugin2 for writing new model data.
            err = tear_ambient_state(stdev->odsp_hdl,
                                    stdev->current_enable);
        }
        err = write_model(stdev->odsp_hdl, model->data,
                        model->data_sz, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to write Entity model");
            goto exit;
        }

        //setup model state.
        stdev->current_enable = stdev->current_enable | ENTITY_MASK;
        err = set_ambient_state(stdev->odsp_hdl, stdev->current_enable);
        if (err != 0) {
            ALOGE("Failed to set Entity state");
            goto exit;
        }
    }

exit:
    return err;
}

static int setup_buffer(struct knowles_sound_trigger_device *stdev,
                        struct model_info *model,
                        bool enabled)
{
    int err = 0;
    if (enabled) {
        if ((check_uuid_equality(model->uuid, stdev->hotword_model_uuid))
            || (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid))) {

            stdev->hotword_buffer_enable++;
            if (stdev->hotword_buffer_enable > 1)
                goto exit;

            err = setup_howord_buffer(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to create the buffer plugin");
                goto exit;
            }
        } else if ((check_uuid_equality(model->uuid, stdev->ambient_model_uuid))
            || (check_uuid_equality(model->uuid, stdev->entity_model_uuid))) {

            stdev->music_buffer_enable++;
            if (stdev->music_buffer_enable > 1)
                goto exit;

            err = setup_music_buffer(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load music buffer package");
                goto exit;
            }
        }
    } else {
        if ((check_uuid_equality(model->uuid, stdev->hotword_model_uuid))
           || (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid))) {

            stdev->hotword_buffer_enable--;
            if (stdev->hotword_buffer_enable != 0)
                goto exit;

            err = destroy_howord_buffer(stdev->odsp_hdl);

            if (err != 0) {
                ALOGE("Failed to unload hotword buffer package");
                goto exit;
            }

        } else if ((check_uuid_equality(model->uuid, stdev->ambient_model_uuid))
           || (check_uuid_equality(model->uuid, stdev->entity_model_uuid))) {

            stdev->music_buffer_enable--;
            if (stdev->music_buffer_enable != 0)
                goto exit;

            err = destroy_music_buffer(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to unload music buffer package");
                goto exit;
            }
       }
    }

exit:
    return err;
}

static int destroy_package(struct knowles_sound_trigger_device *stdev,
                        struct model_info *model)
{
    int err = 0;

    if (check_uuid_equality(model->uuid, stdev->chre_model_uuid)) {
        stdev->current_enable = stdev->current_enable & ~CHRE_MASK;
        if (!(stdev->current_enable & CHRE_MASK)) {
            err = destroy_chre_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to destroy CHRE package");
                goto exit;
            }
        }
    } else if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid)) {
        err = tear_hotword_state(stdev->odsp_hdl, HOTWORD_MASK);
        if (err != 0) {
            ALOGE("Failed to tear Hotword state");
            goto exit;
        }

        err = flush_model(stdev->odsp_hdl, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to flush Hotword model");
            goto exit;
        }
        stdev->current_enable = stdev->current_enable & ~HOTWORD_MASK;

        if (!(stdev->current_enable & PLUGIN1_MASK)) {
            err = destroy_hotword_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to destroy Hotword package");
                goto exit;
            }
        }

    } else if (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid)) {
        err = tear_hotword_state(stdev->odsp_hdl, WAKEUP_MASK);
        if (err != 0) {
            ALOGE("Failed to tear Wakeup state");
            goto exit;
        }

        err = flush_model(stdev->odsp_hdl, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to flush Wakeup model");
            goto exit;
        }
        stdev->current_enable = stdev->current_enable & ~WAKEUP_MASK;

        if (!(stdev->current_enable & PLUGIN1_MASK)) {
            err = destroy_hotword_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to destroy Hotword package");
                goto exit;
            }
        }

    } else if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        err = tear_ambient_state(stdev->odsp_hdl, AMBIENT_MASK);
        if (err != 0) {
            ALOGE("Failed to tear Ambient state");
            goto exit;
        }

        err = flush_model(stdev->odsp_hdl, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to flush Ambient model");
            goto exit;
        }
        stdev->current_enable = stdev->current_enable & ~AMBIENT_MASK;

        if (!(stdev->current_enable & PLUGIN2_MASK)) {
            err = destroy_ambient_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to destroy Ambient package");
                goto exit;
            }
        }
    } else if (check_uuid_equality(model->uuid, stdev->entity_model_uuid)) {
        err = tear_ambient_state(stdev->odsp_hdl, ENTITY_MASK);
        if (err != 0) {
            ALOGE("Failed to tear Entity state");
            goto exit;
        }

        err = flush_model(stdev->odsp_hdl, model->kw_id);
        if (err != 0) {
            ALOGE("Failed to flush Entity model");
            goto exit;
        }
        stdev->current_enable = stdev->current_enable & ~ENTITY_MASK;

        if (!(stdev->current_enable & PLUGIN2_MASK)) {
            err = destroy_ambient_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to destroy Ambient package");
                goto exit;
            }
        }
    }
exit:
    return err;
}

static int set_package_route(struct knowles_sound_trigger_device *stdev,
                            sound_trigger_uuid_t uuid,
                            bool bargein)
{
    int ret = 0;
    /*
     *[TODO] Add correct error return value for package route
     * b/119390722 for tracing.
     */
    if (check_uuid_equality(uuid, stdev->chre_model_uuid))
        set_chre_audio_route(stdev->route_hdl, bargein);
    else if (check_uuid_equality(uuid, stdev->hotword_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN1_MASK) & ~HOTWORD_MASK)) {
            set_hotword_route(stdev->route_hdl, bargein);
        }
    } else if (check_uuid_equality(uuid, stdev->wakeup_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN1_MASK) & ~WAKEUP_MASK)) {
            set_hotword_route(stdev->route_hdl, bargein);
        }
    } else if (check_uuid_equality(uuid, stdev->ambient_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN2_MASK) & ~AMBIENT_MASK)) {
            set_ambient_route(stdev->route_hdl, bargein);
        }
    } else if (check_uuid_equality(uuid, stdev->entity_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN2_MASK) & ~ENTITY_MASK)) {
            set_ambient_route(stdev->route_hdl, bargein);
        }
    }

    return ret;
}

static int tear_package_route(struct knowles_sound_trigger_device *stdev,
                            sound_trigger_uuid_t uuid,
                            bool bargein)
{
    int ret = 0;
    /*
     *[TODO] Add correct error return value for package route
     * b/119390722 for tracing.
     */
    if (check_uuid_equality(uuid, stdev->chre_model_uuid))
        tear_chre_audio_route(stdev->route_hdl, bargein);
    else if (check_uuid_equality(uuid, stdev->hotword_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN1_MASK) & ~HOTWORD_MASK))
            tear_hotword_route(stdev->route_hdl, bargein);
    } else if (check_uuid_equality(uuid, stdev->wakeup_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN1_MASK) & ~WAKEUP_MASK))
            tear_hotword_route(stdev->route_hdl, bargein);
    } else if (check_uuid_equality(uuid, stdev->ambient_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN2_MASK) & ~AMBIENT_MASK))
            tear_ambient_route(stdev->route_hdl, bargein);
    } else if (check_uuid_equality(uuid, stdev->entity_model_uuid)) {
        if (!((stdev->current_enable & PLUGIN2_MASK) & ~ENTITY_MASK))
            tear_ambient_route(stdev->route_hdl, bargein);
    }

    return ret;
}

static int handle_input_source(struct knowles_sound_trigger_device *stdev,
                            bool enable)
{
    int err = 0;
    enum clock_type ct = INTERNAL_OSCILLATOR;

    if (stdev->is_music_playing == true) {
        ct = EXTERNAL_OSCILLATOR;
    }

    /*
     *[TODO] Add correct error return value for input source route
     * b/119390722 for tracing.
     */
    if (enable) {
        if (stdev->is_mic_route_enabled == false) {
            err = enable_mic_route(stdev->route_hdl, true, ct);
            if (err != 0) {
                ALOGE("Failed to enable mic route");
                goto exit;
            }
            stdev->is_mic_route_enabled = true;
        }
        if (stdev->is_music_playing == true &&
            stdev->is_bargein_route_enabled == false) {

            err = setup_aec_package(stdev->odsp_hdl);
            if (err != 0) {
                ALOGE("Failed to load AEC package");
                // We didn't load AEC package so don't setup the routes
                goto exit;
            }

            // Enable the bargein route if not enabled
            err = enable_bargein_route(stdev->route_hdl, true);
            if (err != 0) {
                ALOGE("Failed to enable buffer route");
                goto exit;
            }
            stdev->is_bargein_route_enabled = true;
        }
    } else {
        if (!is_any_model_active(stdev)) {
            ALOGD("None of keywords are active");
            if (stdev->is_music_playing == true &&
                stdev->is_bargein_route_enabled == true) {
                // Just disable the route and update the route status but retain
                // bargein status
                err = enable_bargein_route(stdev->route_hdl, false);
                if (err != 0) {
                    ALOGE("Failed to disable bargein route");
                    goto exit;
                }
                err = destroy_aec_package(stdev->odsp_hdl);
                if (err != 0) {
                    ALOGE("Failed to unload AEC package");
                    goto exit;
                }
                stdev->is_bargein_route_enabled = false;
           }
           if (stdev->is_mic_route_enabled == true) {
               err = enable_mic_route(stdev->route_hdl, false, ct);
               if (err != 0) {
                   ALOGE("Failed to disable mic route");
                   goto exit;
               }
               stdev->is_mic_route_enabled = false;
           }
       }
    }

exit:
    return err;
}

// stdev needs to be locked before calling this function
static int restart_recognition(struct knowles_sound_trigger_device *stdev)
{
    int err = 0;
    int i = 0;
    enum clock_type ct = INTERNAL_OSCILLATOR;
    /*
     * The libaudioroute library doesn't set the mixer controls if previously
     * applied values are the same or the active_count > 0, so we need to
     * teardown the route so that it can clear up the value and active_count.
     * Then we could setup the routes again.
     */

    stdev->current_enable = 0;
    stdev->hotword_buffer_enable = 0;
    stdev->music_buffer_enable = 0;

    if (stdev->is_hmd_proc_on == true) {
        power_on_proc_mem(stdev->route_hdl, false, IAXXX_HMD_ID);
        power_on_proc_mem(stdev->route_hdl, true, IAXXX_HMD_ID);
    }

    if (stdev->is_music_playing == true &&
        stdev->is_bargein_route_enabled == true) {
        ct = EXTERNAL_OSCILLATOR;
    }

    if (stdev->is_buffer_package_loaded == true) {
        err = setup_buffer_package(stdev->odsp_hdl);
        if (err != 0) {
            ALOGE("%s: Failed to restart Buffer package", __func__);
        }
    }

    if (stdev->is_mic_route_enabled == true) {
        err = enable_mic_route(stdev->route_hdl, false, ct);
        if (err != 0) {
            ALOGE("failed to tear mic route");
        }
        err = enable_mic_route(stdev->route_hdl, true, ct);
        if (err != 0) {
            ALOGE("failed to restart mic route");
        }
    }

    if (stdev->is_music_playing == true &&
        stdev->is_bargein_route_enabled == true) {
        err = setup_aec_package(stdev->odsp_hdl);
        if (err != 0) {
            ALOGE("Failed to restart AEC package");
        }

        err = enable_bargein_route(stdev->route_hdl, false);
        if (err != 0) {
            ALOGE("Failed to tear bargein route");
        }

        err = enable_bargein_route(stdev->route_hdl, true);
        if (err != 0) {
            ALOGE("Failed to restart bargein route");
        }
    }

    // Download all the keyword models files that were previously loaded
    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_active == true) {
            if (stdev->is_buffer_package_loaded == true) {
                setup_buffer(stdev, &stdev->models[i], true);
            }
            if (check_uuid_equality(stdev->models[i].uuid, stdev->hotword_model_uuid) ||
                (check_uuid_equality(stdev->models[i].uuid, stdev->wakeup_model_uuid))) {
                if ((stdev->hotword_buffer_enable) &&
                    (!(stdev->current_enable & HOTWORD_MASK) ||
                      (stdev->current_enable & WAKEUP_MASK))) {
                    tear_hotword_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                    set_hotword_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                }
            }
            if (check_uuid_equality(stdev->models[i].uuid, stdev->ambient_model_uuid) ||
                (check_uuid_equality(stdev->models[i].uuid, stdev->entity_model_uuid))) {
                if ((stdev->music_buffer_enable) &&
                    (!(stdev->current_enable & AMBIENT_MASK) ||
                      (stdev->current_enable & ENTITY_MASK))) {
                    tear_music_buffer_route(stdev->route_hdl,
                                        stdev->is_bargein_route_enabled);
                    set_music_buffer_route(stdev->route_hdl,
                                        stdev->is_bargein_route_enabled);
                }
            }
            setup_package(stdev, &stdev->models[i]);
            tear_package_route(stdev, stdev->models[i].uuid,
                            stdev->is_bargein_route_enabled);
            set_package_route(stdev, stdev->models[i].uuid,
                            stdev->is_bargein_route_enabled);
        }
    }

    // reload Oslo part after every package loaded to avoid HMD memory overlap
    // issue, b/128914464
    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_loaded == true) {
            if (check_uuid_equality(stdev->models[i].uuid,
                                    stdev->sensor_model_uuid)) {
                // setup the sensor route
                err = setup_sensor_package(stdev->odsp_hdl);
                if (err != 0) {
                    ALOGE("%s: setup Sensor package failed", __func__);
                    goto exit;
                }
                err = set_sensor_route(stdev->route_hdl, false);
                if (err != 0) {
                    ALOGE("%s: tear Sensor route fail", __func__);
                    goto exit;
                }
                err = set_sensor_route(stdev->route_hdl, true);
                if (err != 0) {
                    ALOGE("%s: Sensor route fail", __func__);
                    goto exit;
                }
            }
        }
    }
    ALOGD("%s: recovery done", __func__);
exit:
    return err;
}

// stdev needs to be locked before calling this function
static int crash_recovery(struct knowles_sound_trigger_device *stdev)
{
    int err = 0;

    power_down_all_non_ctrl_proc_mem(stdev->mixer);
    // Redownload the keyword model files and start recognition
    err = restart_recognition(stdev);
    if (err != 0) {
        ALOGE("%s: ERROR: Failed to download the keyword models and restarting"
            " recognition", __func__);
        goto exit;
    }

    // Reset the flag only after successful recovery
    stdev->is_st_hal_ready = true;

exit:
    return err;
}

static void *callback_thread_loop(void *context)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)context;
    struct pollfd fds[2];
    char msg[UEVENT_MSG_LEN];
    int exit_sockets[2];
    int err = 0;
    int i, n;
    int kwid = 0;
    struct iaxxx_get_event_info ge;
    void *payload = NULL;
    unsigned int payload_size = 0, fw_status = IAXXX_FW_IDLE;
    int fw_status_retries = 0;

    ALOGI("%s", __func__);
    prctl(PR_SET_NAME, (unsigned long)"sound trigger callback", 0, 0, 0);

    pthread_mutex_lock(&stdev->lock);

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, exit_sockets) == -1) {
        ALOGE("%s: Failed to create termination socket", __func__);
        goto exit;
    }

    stdev_close_term_sock(stdev);
    stdev->send_sock = exit_sockets[0];
    stdev->recv_sock = exit_sockets[1];

    memset(fds, 0, 2 * sizeof(struct pollfd));
    int timeout = -1; // Wait for event indefinitely
    fds[0].events = POLLIN;
    fds[0].fd = uevent_open_socket(64*1024, true);
    if (fds[0].fd == -1) {
        ALOGE("Error opening socket for hotplug uevent errno %d(%s)",
            errno, strerror(errno));
        goto exit;
    }
    fds[1].events = POLLIN;
    fds[1].fd = stdev->recv_sock;

    ge.event_id = -1;

    // Try to get the firmware status, if we fail, try for 10 times with a gap
    // of 500ms, if we are unable to get the status after that then exit
    do {
        err = get_fw_status(stdev->odsp_hdl, &fw_status);
        if (err == -1) {
            ALOGE("%s: ERROR: Failed to get the firmware status %d(%s)",
                    __func__, errno, strerror(errno));
            usleep(RETRY_US);
            fw_status_retries++;
        }
    } while (err != 0 && fw_status_retries < RETRY_NUMBER);

    if (err != 0) {
        ALOGE("%s: ERROR: Failed to get firmware status after %d tries",
                __func__, RETRY_NUMBER);
        goto exit;
    }

    if (fw_status == IAXXX_FW_ACTIVE) {
        stdev->is_st_hal_ready = false;
        // reset the firmware and wait for firmware download complete
        err = reset_fw(stdev->odsp_hdl);
        if (err == -1) {
            ALOGE("%s: ERROR: Failed to reset the firmware %d(%s)",
                    __func__, errno, strerror(errno));
        }
        stdev->fw_reset_done_by_hal = true;
    } else if (fw_status == IAXXX_FW_CRASH) {
        // Firmware has crashed wait till it recovers
        stdev->is_st_hal_ready = false;
    } else if (fw_status == IAXXX_FW_IDLE) {
        stdev->route_hdl = audio_route_init(stdev->snd_crd_num,
                                            stdev->mixer_path_xml);
        if (stdev->route_hdl == NULL) {
            ALOGE("Failed to init the audio_route library");
            goto exit;
        }

        power_down_all_non_ctrl_proc_mem(stdev->mixer);

        stdev->is_st_hal_ready = true;
    }
    pthread_mutex_unlock(&stdev->lock);

    while (1) {
        err = poll(fds, 2, timeout);

        pthread_mutex_lock(&stdev->lock);
        if (err < 0) {
            ALOGE("%s: Error in poll: %d (%s)",
                __func__, errno, strerror(errno));
            break;
        }

        if (fds[0].revents & POLLIN) {
            n = uevent_kernel_multicast_recv(fds[0].fd, msg, UEVENT_MSG_LEN);
            if (n <= 0) {
                pthread_mutex_unlock(&stdev->lock);
                continue;
            }
            for (i = 0; i < n;) {
                if (strstr(msg + i, IAXXX_VQ_EVENT_STR)) {
                    ALOGI("%s", IAXXX_VQ_EVENT_STR);

                    err = get_event(stdev->odsp_hdl, &ge);
                    if (err == 0) {
                        if (ge.event_id == OK_GOOGLE_KW_ID) {
                            ALOGD("Eventid received is OK_GOOGLE_KW_ID %d",
                                OK_GOOGLE_KW_ID);
                            kwid = OK_GOOGLE_KW_ID;
                        } else if (ge.event_id == AMBIENT_KW_ID) {
                            ALOGD("Eventid received is AMBIENT_KW_ID %d",
                                AMBIENT_KW_ID);
                            kwid = AMBIENT_KW_ID;
                            reset_ambient_plugin(stdev->odsp_hdl);
                        } else if (ge.event_id == ENTITY_KW_ID) {
                            ALOGD("Eventid received is ENTITY_KW_ID %d",
                                ENTITY_KW_ID);
                            kwid = ENTITY_KW_ID;
                        } else if (ge.event_id == WAKEUP_KW_ID) {
                            ALOGD("Eventid received is WAKEUP_KW_ID %d",
                                WAKEUP_KW_ID);
                            kwid = WAKEUP_KW_ID;
                        } else {
                            ALOGE("Unknown event id received, ignoring %d",
                                ge.event_id);
                        }
                        stdev->last_detected_model_type = kwid;
                        break;
                    } else {
                        ALOGE("get_event failed with error %d", err);
                    }
                } else if (strstr(msg + i, IAXXX_RECOVERY_EVENT_STR)) {
                    /* If the ST HAL did the firmware reset then that means
                     * that the userspace crashed so we need to reinit the audio
                     * route library, if we didn't reset the firmware, then it
                     * was genuine firmware crash and we don't need to reinit
                     * the audio route library.
                     */
                    if (stdev->fw_reset_done_by_hal == true) {
                        stdev->route_hdl = audio_route_init(stdev->snd_crd_num,
                                                            stdev->mixer_path_xml);
                        if (stdev->route_hdl == NULL) {
                            ALOGE("Failed to init the audio_route library");
                            goto exit;
                        }

                        stdev->fw_reset_done_by_hal = false;
                    }

                    ALOGD("Firmware has redownloaded, start the recovery");
                    int err = crash_recovery(stdev);
                    if (err != 0) {
                        ALOGE("Crash recovery failed");
                    }
                } else if (strstr(msg + i, IAXXX_FW_DWNLD_SUCCESS_STR)) {
                    ALOGD("Firmware downloaded successfully");
                    stdev->is_st_hal_ready = true;
                    power_down_all_non_ctrl_proc_mem(stdev->mixer);
                } else if (strstr(msg + i, IAXXX_FW_CRASH_EVENT_STR)) {
                    ALOGD("Firmware has crashed");
                    // Don't allow any op on ST HAL until recovery is complete
                    stdev->is_st_hal_ready = false;
                    stdev->is_streaming = false;
                }

                i += strlen(msg + i) + 1;
            }

            if (ge.event_id == OK_GOOGLE_KW_ID ||
                ge.event_id == AMBIENT_KW_ID ||
                ge.event_id == ENTITY_KW_ID ||
                ge.event_id == WAKEUP_KW_ID) {
                ALOGD("%s: Keyword ID %d", __func__, kwid);

                if (ge.data != 0) {
                    ALOGD("Size of payload is %d", ge.data);
                    payload_size = ge.data;
                    payload = malloc(payload_size);
                    if (payload != NULL) {
                        if (ge.event_id == AMBIENT_KW_ID ||
                            ge.event_id == ENTITY_KW_ID)
                            err = get_entity_param_blk(stdev->odsp_hdl,
                                                    payload,
                                                    payload_size);
                        else if (ge.event_id == OK_GOOGLE_KW_ID ||
                            ge.event_id == WAKEUP_KW_ID)
                            err = get_wakeup_param_blk(stdev->odsp_hdl,
                                                    payload,
                                                    payload_size);
                        if (err != 0) {
                            ALOGE("Failed to get payload data");
                            free(payload);
                            payload = NULL;
                            payload_size = 0;
                        }
                    } else {
                        ALOGE("Failed to allocate memory for payload");
                    }
                }
                int idx = find_handle_for_kw_id(stdev, kwid);
                if (idx < MAX_MODELS && stdev->models[idx].is_active == true) {
                    int recognition_status = RECOGNITION_STATUS_SUCCESS;
                    if (stdev->models[idx].is_state_query == true) {
                        recognition_status =
                            RECOGNITION_STATUS_GET_STATE_RESPONSE;

                        // We need to send this only once, so reset now
                        stdev->models[idx].is_state_query = false;
                    }
                    if (stdev->models[idx].type == SOUND_MODEL_TYPE_KEYPHRASE) {
                        struct sound_trigger_phrase_recognition_event *event;
                        event = (struct sound_trigger_phrase_recognition_event*)
                                    stdev_keyphrase_event_alloc(
                                                stdev->models[idx].model_handle,
                                                stdev->models[idx].config,
                                                recognition_status);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d",
                                kwid);
                            model->recognition_callback(&event->common,
                                                    model->recognition_cookie);
                            // Update the config so that it will be used
                            // during the streaming
                            stdev->last_keyword_detected_config = model->config;

                            free(event);
                        } else {
                            ALOGE("Failed to allocate memory for the event");
                        }
                    } else if (stdev->models[idx].type == SOUND_MODEL_TYPE_GENERIC) {
                        struct sound_trigger_generic_recognition_event *event;
                        event = (struct sound_trigger_generic_recognition_event*)
                                stdev_generic_event_alloc(
                                            stdev->models[idx].model_handle,
                                            payload,
                                            payload_size,
                                            recognition_status);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d",
                                kwid);
                            model->recognition_callback(&event->common,
                                                    model->recognition_cookie);
                            // Update the config so that it will be used
                            // during the streaming
                            stdev->last_keyword_detected_config = model->config;

                            free(event);
                        } else {
                            ALOGE("Failed to allocate memory for the event");
                        }
                    }
                } else {
                    ALOGE("Invalid id or keyword is not active, Subsume the event");
                }
                // Reset all event related data
                ge.event_id = -1;
                ge.data = 0;
                kwid = -1;
            }
            // Free the payload data
            if (payload) {
                free(payload);
                payload = NULL;
                payload_size = 0;
            }
        } else if (fds[1].revents & POLLIN) {
            read(fds[1].fd, &n, sizeof(n)); /* clear the socket */
            ALOGD("%s: Termination message", __func__);
            break;
        }
        else {
            ALOGI("%s: Message ignored", __func__);
        }
        pthread_mutex_unlock(&stdev->lock);
    }

exit:
    stdev_close_term_sock(stdev);
    pthread_mutex_unlock(&stdev->lock);

    return (void *)(long)err;
}

static int stdev_get_properties(
                            const struct sound_trigger_hw_device *dev __unused,
                            struct sound_trigger_properties *properties)
{
    ALOGV("+%s+", __func__);
    if (properties == NULL)
        return -EINVAL;
    memcpy(properties, &hw_properties, sizeof(struct sound_trigger_properties));
    ALOGV("-%s-", __func__);
    return 0;
}

static int stop_recognition(struct knowles_sound_trigger_device *stdev,
                            sound_model_handle_t handle)
{
    int status = 0;
    struct model_info *model = &stdev->models[handle];

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        status = -EAGAIN;
        goto exit;
    }

    if (model->config != NULL) {
        dereg_hal_event_session(model->config, handle);
        free(model->config);
        model->config = NULL;
    }

    model->recognition_callback = NULL;
    model->recognition_cookie = NULL;
    if (check_uuid_equality(model->uuid, stdev->chre_model_uuid) ||
        check_uuid_equality(model->uuid, stdev->sensor_model_uuid)) {
        // This avoids any processing of chre/oslo.
        goto exit;
    }

    if (model->is_active == false) {
        goto exit;
    }

    model->is_active = false;

    tear_package_route(stdev, model->uuid, stdev->is_bargein_route_enabled);

    destroy_package(stdev, model);


    if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid) ||
        (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid))) {
        if ((stdev->hotword_buffer_enable) &&
            !(stdev->current_enable & PLUGIN1_MASK)) {
            tear_hotword_buffer_route(stdev->route_hdl,
                                    stdev->is_bargein_route_enabled);
        }
    }

    if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid) ||
        (check_uuid_equality(model->uuid, stdev->entity_model_uuid))) {
        if ((stdev->music_buffer_enable) &&
            !(stdev->current_enable & PLUGIN2_MASK)) {
            tear_music_buffer_route(stdev->route_hdl,
                                stdev->is_bargein_route_enabled);
        }
    }

    setup_buffer(stdev, model, false);

    handle_input_source(stdev, false);

exit:
    return status;
}

static int stdev_load_sound_model(const struct sound_trigger_hw_device *dev,
                                struct sound_trigger_sound_model *sound_model,
                                sound_model_callback_t callback,
                                void *cookie,
                                sound_model_handle_t *handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int ret = 0;
    int kw_model_sz = 0;
    int i = 0;

    unsigned char *kw_buffer = NULL;

    ALOGD("+%s+", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        ret = -EAGAIN;
        goto exit;
    }

    if (handle == NULL || sound_model == NULL) {
        ALOGE("%s: handle/sound_model is NULL", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (sound_model->data_size == 0 ||
        sound_model->data_offset < sizeof(struct sound_trigger_sound_model)) {
        ALOGE("%s: Invalid sound model data", __func__);
        ret = -EINVAL;
        goto exit;
    }

    kw_buffer = (unsigned char *) sound_model + sound_model->data_offset;
    kw_model_sz = sound_model->data_size;
    ALOGV("%s: kw_model_sz %d", __func__, kw_model_sz);

    // Load the keyword model file
    // Find an empty slot to load the model
    i = find_empty_model_slot(stdev);
    if (i == -1) {
        ALOGE("%s: Can't load model no free slots available", __func__);
        ret = -ENOSYS;
        goto exit;
    }

    *handle = i;
    ALOGV("%s: Loading keyword model handle(%d) type(%d)", __func__,
        *handle, sound_model->type);
    // This will need to be replaced with UUID once they are fixed
    stdev->models[i].model_handle = *handle;
    stdev->models[i].type = sound_model->type;
    stdev->models[i].uuid = sound_model->vendor_uuid;
    stdev->models[i].sound_model_callback = callback;
    stdev->models[i].sound_model_cookie = cookie;
    stdev->models[i].recognition_callback = NULL;
    stdev->models[i].recognition_cookie = NULL;

    stdev->models[i].data = malloc(kw_model_sz);
    if (stdev->models[i].data == NULL) {
        ALOGE("%s: Warning, could not allocate memory for keyword model data,"
            " cannot redownload on crash", __func__);
        stdev->models[i].data_sz = 0;
    } else {
        memcpy(stdev->models[i].data, kw_buffer, kw_model_sz);
        stdev->models[i].data_sz = kw_model_sz;
    }

    if (stdev->is_hmd_proc_on == false) {
        power_on_proc_mem(stdev->route_hdl, true, IAXXX_HMD_ID);
        stdev->is_hmd_proc_on = true;
    }

    if (stdev->is_buffer_package_loaded == false) {
        ret = setup_buffer_package(stdev->odsp_hdl);
        if (ret != 0) {
            ALOGE("%s: ERROR: Failed to load the buffer package", __func__);
            goto exit;
        }
        stdev->is_buffer_package_loaded = true;
    }

    // Send the keyword model to the chip only for hotword and ambient audio
    if (check_uuid_equality(stdev->models[i].uuid,
                            stdev->hotword_model_uuid)) {
        stdev->models[i].kw_id = OK_GOOGLE_KW_ID;
    } else if (check_uuid_equality(stdev->models[i].uuid,
                                stdev->wakeup_model_uuid)) {
        stdev->models[i].kw_id = WAKEUP_KW_ID;
    } else if (check_uuid_equality(stdev->models[i].uuid,
                                stdev->ambient_model_uuid)) {
        stdev->models[i].kw_id = AMBIENT_KW_ID;
    } else if (check_uuid_equality(stdev->models[i].uuid,
                                stdev->entity_model_uuid)) {
        stdev->models[i].kw_id = ENTITY_KW_ID;
    } else if (check_uuid_equality(stdev->models[i].uuid,
                                stdev->sensor_model_uuid)) {
        // setup the sensor route
        stdev->current_enable = stdev->current_enable | OSLO_MASK;
        ret = setup_sensor_package(stdev->odsp_hdl);
        if (ret != 0) {
            ALOGE("%s: setup Sensor package failed", __func__);
            goto exit;
        }

        ret = set_sensor_route(stdev->route_hdl, true);
        if (ret != 0) {
            ALOGE("%s: Sensor route fail", __func__);
            goto exit;
        }
        stdev->models[i].kw_id = USELESS_KW_ID;
    } else if (check_uuid_equality(stdev->models[i].uuid,
                                stdev->chre_model_uuid)) {
        if (hw_properties.concurrent_capture == false &&
            stdev->is_media_recording == true) {
            ALOGW("%s: device is recording, can't enable chre now", __func__);
        } else {
            if (stdev->models[i].is_active == false) {
                stdev->models[i].is_active = true;
                handle_input_source(stdev, true);
                setup_package(stdev, &stdev->models[i]);
                set_package_route(stdev, stdev->models[i].uuid,
                                stdev->is_bargein_route_enabled);
            }
        }
        stdev->models[i].kw_id = USELESS_KW_ID;
        stdev->is_chre_enable = true;
    } else {
        ALOGE("%s: ERROR: unknown keyword model file", __func__);
        ret = -EINVAL;
        goto exit;
    }

    stdev->models[i].is_loaded = true;

exit:
    if (ret != 0) {
        if (stdev->models[i].data) {
            free(stdev->models[i].data);
            stdev->models[i].data = NULL;
            stdev->models[i].data_sz = 0;
        }
        if (!is_any_model_loaded(stdev) && stdev->is_buffer_package_loaded) {
            destroy_buffer_package(stdev->odsp_hdl);
            stdev->is_buffer_package_loaded = false;
        }

        if (!is_any_model_loaded(stdev) && stdev->is_hmd_proc_on) {
            power_on_proc_mem(stdev->route_hdl, false, IAXXX_HMD_ID);
            stdev->is_hmd_proc_on = false;
        }
    }
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s handle %d-", __func__, *handle);
    return ret;
}

static int stdev_unload_sound_model(const struct sound_trigger_hw_device *dev,
                                    sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int ret = 0;
    ALOGD("+%s handle %d+", __func__, handle);
    pthread_mutex_lock(&stdev->lock);

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        ret = -EAGAIN;
        goto exit;
    }

    // Just confirm the model was previously loaded
    if (stdev->models[handle].is_loaded == false) {
        ALOGE("%s: Invalid model(%d) being called for unload",
                __func__, handle);
        ret = -EINVAL;
        goto exit;
    }

    if (stdev->models[handle].is_active == true) {
        ret = stop_recognition(stdev, handle);
        if (ret)
            goto exit;
    }


    if (check_uuid_equality(stdev->models[handle].uuid,
                                stdev->sensor_model_uuid)) {
        // Disable the sensor route
        ret = set_sensor_route(stdev->route_hdl, false);
        if (ret != 0) {
            ALOGE("%s: disable Sensor route failed", __func__);
        }

        ret = destroy_sensor_package(stdev->odsp_hdl);
        if (ret != 0) {
            ALOGE("%s: destroy Sensor package failed", __func__);
        }
        stdev->current_enable = stdev->current_enable & ~OSLO_MASK;
    } else if (check_uuid_equality(stdev->models[handle].uuid,
                                stdev->chre_model_uuid)) {
        if (stdev->models[handle].is_active == true) {
            stdev->models[handle].is_active = false;
            tear_package_route(stdev, stdev->models[handle].uuid,
                            stdev->is_bargein_route_enabled);
            destroy_package(stdev, &stdev->models[handle]);
            handle_input_source(stdev, false);
        }
        stdev->is_chre_enable = false;
    }

    stdev->models[handle].sound_model_callback = NULL;
    stdev->models[handle].sound_model_cookie = NULL;
    stdev->models[handle].is_loaded = false;
    if (stdev->models[handle].data) {
        free(stdev->models[handle].data);
        stdev->models[handle].data = NULL;
        stdev->models[handle].data_sz = 0;
    }

    if (!is_any_model_loaded(stdev) && stdev->is_buffer_package_loaded) {
        destroy_buffer_package(stdev->odsp_hdl);
        stdev->is_buffer_package_loaded = false;

    }

    if (!is_any_model_loaded(stdev) && stdev->is_hmd_proc_on) {
            power_on_proc_mem(stdev->route_hdl, false, IAXXX_HMD_ID);
            stdev->is_hmd_proc_on = false;
    }

    ALOGD("%s: Successfully unloaded the model, handle - %d",
        __func__, handle);
exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s handle %d-", __func__, handle);
    return ret;
}

static int stdev_start_recognition(
                        const struct sound_trigger_hw_device *dev,
                        sound_model_handle_t handle,
                        const struct sound_trigger_recognition_config *config,
                        recognition_callback_t callback,
                        void *cookie)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0;
    struct model_info *model = &stdev->models[handle];

    ALOGD("%s stdev %p, sound model %d", __func__, stdev, handle);

    pthread_mutex_lock(&stdev->lock);

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        status = -EAGAIN;
        goto exit;
    }

    if (callback == NULL) {
        ALOGE("%s: recognition_callback is null", __func__);
        status = -EINVAL;
        goto exit;
    }

    if (model->config != NULL) {
        dereg_hal_event_session(model->config, handle);
        free(model->config);
        model->config = NULL;
    }

    if (config != NULL) {
        model->config = (struct sound_trigger_recognition_config *)
                            malloc(sizeof(*config));
        if (model->config == NULL) {
            ALOGE("%s: Failed to allocate memory for model config", __func__);
            status = -ENOMEM;
            goto exit;
        }

        memcpy(model->config, config, sizeof(*config));
        reg_hal_event_session(model->config, handle);

        ALOGD("%s: Is capture requested %d",
            __func__, config->capture_requested);
    } else {
        ALOGD("%s: config is null", __func__);
        model->config = NULL;
    }

    model->recognition_callback = callback;
    model->recognition_cookie = cookie;
    if (check_uuid_equality(model->uuid, stdev->chre_model_uuid) ||
        check_uuid_equality(model->uuid, stdev->sensor_model_uuid)) {
        // This avoids any processing of chre/oslo.
        goto exit;
    }
    if (model->is_active == true) {
        // This model is already active, do nothing except updating callbacks,
        // configs and cookie
        goto exit;
    }
    model->is_active = true;

    handle_input_source(stdev, true);

    if (stdev->is_buffer_package_loaded == true) {
        setup_buffer(stdev, model, true);
    }

    if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid) ||
        (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid))) {
        if ((stdev->hotword_buffer_enable) &&
            (!(stdev->current_enable & HOTWORD_MASK) ||
              (stdev->current_enable & WAKEUP_MASK))) {
            set_hotword_buffer_route(stdev->route_hdl,
                                    stdev->is_bargein_route_enabled);
        }
    }

    if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid) ||
        (check_uuid_equality(model->uuid, stdev->entity_model_uuid))) {
        if ((stdev->music_buffer_enable) &&
            (!(stdev->current_enable & AMBIENT_MASK) ||
              (stdev->current_enable & ENTITY_MASK))) {
            set_music_buffer_route(stdev->route_hdl,
                                stdev->is_bargein_route_enabled);
        }
    }

    setup_package(stdev, model);

    set_package_route(stdev, model->uuid, stdev->is_bargein_route_enabled);

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s sound model %d-", __func__, handle);
    return status;
}

static int stdev_stop_recognition(
                        const struct sound_trigger_hw_device *dev,
                        sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0;
    pthread_mutex_lock(&stdev->lock);
    ALOGD("+%s sound model %d+", __func__, handle);

    status = stop_recognition(stdev, handle);

    if (status != 0)
        goto exit;

    ALOGD("-%s sound model %d-", __func__, handle);
exit:
    pthread_mutex_unlock(&stdev->lock);

    return status;
}

/**
 * Get the state of a given model.
 * The model state is returned asynchronously as a RecognitionEvent via
 * the callback that was registered in StartRecognition().
 * @param modelHandle The handle of the sound model whose state is being
 *                    queried.
 * @return retval Operation completion status: 0 in case of success,
 *                -ENOSYS in case of invalid model handle,
 *                -ENOMEM in case of memory allocation failure,
 *                -ENODEV in case of initialization error,
 *                -EINVAL in case where a recognition event is already
 *                        being processed.
 */
static int stdev_get_model_state(const struct sound_trigger_hw_device *dev,
                               sound_model_handle_t sound_model_handle) {
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    struct model_info *model = &stdev->models[sound_model_handle];
    int ret = 0;
    ALOGD("+%s+", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (!stdev->opened) {
        ALOGE("%s: stdev isn't initialized", __func__);
        ret = -ENODEV;
        goto exit;
    }

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        ret = -ENODEV;
        goto exit;
    }

    if (model->is_active == false) {
        ALOGE("%s: ERROR: %d model is not active",
            __func__, sound_model_handle);
        ret = -ENOSYS;
        goto exit;
    }

    if (model->is_state_query == true) {
        ALOGE("%s: ERROR: model %d is already processing",
            __func__, sound_model_handle);
        ret = -EINVAL;
        goto exit;
    }

    model->is_state_query = true;

    if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid))
        ret = get_model_state(stdev->odsp_hdl, HOTWORD_INSTANCE_ID,
                            HOTWORD_SLOT_ID);
    else if (check_uuid_equality(model->uuid, stdev->wakeup_model_uuid))
        ret = get_model_state(stdev->odsp_hdl, HOTWORD_INSTANCE_ID,
                            WAKEUP_SLOT_ID);
    else if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid))
        ret = get_model_state(stdev->odsp_hdl, AMBIENT_INSTANCE_ID,
                            AMBIENT_SLOT_ID);
    else if (check_uuid_equality(model->uuid, stdev->entity_model_uuid)) {
        ret = get_model_state(stdev->odsp_hdl, AMBIENT_INSTANCE_ID,
                            ENTITY_SLOT_ID);
    } else {
        ALOGE("%s: ERROR: %d model is not supported",
            __func__, sound_model_handle);
        ret = -ENOSYS;
    }

    if (ret != 0) {
        model->is_state_query = false;
        ALOGE("%s: ERROR: Failed to get the model state", __func__);
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s-", __func__);
    return ret;
}

static int stdev_close(hw_device_t *device)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)device;
    int ret = 0;
    ALOGD("+%s+", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (!stdev->opened) {
        ALOGE("%s: device already closed", __func__);
        ret = -EFAULT;
        goto exit;
    }

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        ret = -EAGAIN;
        goto exit;
    }

    stdev->opened = false;

    if (stdev->send_sock >= 0)
        write(stdev->send_sock, "T", 1);
    pthread_join(stdev->callback_thread, (void **)NULL);

    if (stdev->route_hdl)
        audio_route_free(stdev->route_hdl);
    if (stdev->odsp_hdl)
        iaxxx_odsp_deinit(stdev->odsp_hdl);

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s-", __func__);
    return ret;
}

__attribute__ ((visibility ("default")))
audio_io_handle_t stdev_get_audio_handle()
{
    if (g_stdev.last_keyword_detected_config == NULL) {
        ALOGI("%s: Config is NULL so returning audio handle as 0", __func__);
        return 0;
    }

    ALOGI("%s: Audio Handle is %d",
        __func__, g_stdev.last_keyword_detected_config->capture_handle);

    return g_stdev.last_keyword_detected_config->capture_handle;
}

static int open_streaming_lib(struct knowles_sound_trigger_device *stdev) {
    int ret = 0;

    if (access(ADNC_STRM_LIBRARY_PATH, R_OK) == 0) {
        stdev->adnc_cvq_strm_lib = dlopen(ADNC_STRM_LIBRARY_PATH, RTLD_NOW);
        if (stdev->adnc_cvq_strm_lib == NULL) {
            char const *err_str = dlerror();
            ALOGE("%s: module = %s error = %s", __func__,
                ADNC_STRM_LIBRARY_PATH, err_str ? err_str : "unknown");
            ALOGE("%s: DLOPEN failed for %s", __func__, ADNC_STRM_LIBRARY_PATH);
        } else {
            ALOGV("%s: DLOPEN successful for %s",
                __func__, ADNC_STRM_LIBRARY_PATH);
            stdev->adnc_strm_handle = 0;
            stdev->adnc_strm_open =
                (int (*)(bool, int, int))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_open");
            stdev->adnc_strm_read =
               (size_t (*)(long, void *, size_t))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_read");
            stdev->adnc_strm_close =
                (int (*)(long))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_close");
            if (!stdev->adnc_strm_open || !stdev->adnc_strm_read ||
                !stdev->adnc_strm_close) {
                ALOGE("%s: Error grabbing functions in %s", __func__,
                    ADNC_STRM_LIBRARY_PATH);
                stdev->adnc_strm_open = 0;
                stdev->adnc_strm_read = 0;
                stdev->adnc_strm_close = 0;
            }
        }
    }

    return ret;
}

static struct mixer* find_stdev_mixer_path(int card_num, char *mixer_path_xml)
{
    struct mixer *mixer = NULL;
    const char *in_snd_card_name;
    char *snd_card_name = NULL;
    char *tmp = NULL;
    char *platform = NULL;
    char *snd_card = NULL;
    char *device = NULL;

    mixer = mixer_open(card_num);

    if (!mixer) {
        ALOGE("%s: Unable to open the mixer: %d", __func__,
            card_num);
        return NULL;
    }

    in_snd_card_name = mixer_get_name(mixer);
    snd_card_name = strdup(in_snd_card_name);

    if (snd_card_name == NULL) {
        ALOGE("%s: snd_card_name is NULL", __func__);
        goto on_error;
    }

    platform = strtok_r(snd_card_name, "-", &tmp);
    if (platform == NULL) {
        ALOGE("%s: snd card is invalid", __func__);
        goto on_error;
    }

    snd_card = strtok_r(NULL, "-", &tmp);
    if (snd_card == NULL) {
        ALOGE("%s: snd card is invalid", __func__);
        goto on_error;
    }

    device = strtok_r(NULL, "-", &tmp);
    if (device != NULL) {
        snprintf(mixer_path_xml, NAME_MAX_SIZE, "%s_%s.xml",
                SOUND_TRIGGER_MIXER_PATH_BASE, device);
    } else {
        ALOGE("%s: Unknown device, try to use default xml", __func__);
        snprintf(mixer_path_xml, NAME_MAX_SIZE, "%s",
                SOUND_TRIGGER_MIXER_PATH_XML);
    }

    ALOGD("%s: using %s", __func__, mixer_path_xml);

on_error:
    if (snd_card_name)
        free(snd_card_name);
    return mixer;
}

static int find_sound_card() {
    int retry_num = 0, snd_card_num = 0, ret = -1;
    const char *snd_card_name;
    struct mixer *mixer = NULL;
    bool card_verifed[MAX_SND_CARD] = {false};
    const int retry_limit = property_get_int32("audio.snd_card.open.retries",
                                            RETRY_NUMBER);
    ALOGD("+%s+", __func__);

    for (;;) {
        if (snd_card_num >= MAX_SND_CARD) {
            if (retry_num++ >= retry_limit) {
                ALOGE("%s: iaxxx sound card not found", __func__);
                goto exit;
            }
            snd_card_num = 0;
            usleep(RETRY_US);
            continue;
        }
        if (card_verifed[snd_card_num]) {
            snd_card_num++;
            continue;
        }

        mixer = mixer_open(snd_card_num);
        if (!mixer) {
            snd_card_num++;
            continue;
        }

        snd_card_name = mixer_get_name(mixer);
        if (strstr(snd_card_name, CARD_NAME)) {
            ALOGD("%s: find card %d has iaxxx - %s",
                __func__, snd_card_num, snd_card_name);
            ret = snd_card_num;
            break;
        }

        ALOGD("%s: sound card %s does NOT have iaxxx", __func__, snd_card_name);
        mixer_close(mixer);
        mixer = NULL;
        card_verifed[snd_card_num] = true;
        snd_card_num++;
    }

exit:
    if (mixer)
        mixer_close(mixer);

    ALOGD("-%s-", __func__);
    return ret;
}

static int load_audio_hal()
{
    char audio_hal_lib[100];
    void *sthal_prop_api_version = NULL;
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int ret = 0;

    snprintf(audio_hal_lib, sizeof(audio_hal_lib), "%s/%s.%s.so",
            AUDIO_HAL_LIBRARY_PATH, AUDIO_HAL_NAME_PREFIX,
            SOUND_TRIGGER_PLATFORM);
    if (access(audio_hal_lib, R_OK)) {
        ALOGE("%s: ERROR. %s not found", __func__, audio_hal_lib);
        return -ENOENT;
    }

    stdev->audio_hal_handle = dlopen(audio_hal_lib, RTLD_NOW);
    if (stdev->audio_hal_handle == NULL) {
        ALOGE("%s: ERROR. %s", __func__, dlerror());
        return -ENODEV;
    }

    stdev->audio_hal_cb = dlsym(stdev->audio_hal_handle, "audio_hw_call_back");
    if (stdev->audio_hal_cb == NULL) {
        ALOGE("%s: ERROR. %s", __func__, dlerror());
        ret = -ENODEV;
        goto error;
    }

    sthal_prop_api_version = dlsym(stdev->audio_hal_handle,
                                "sthal_prop_api_version");
    if (sthal_prop_api_version == NULL) {
        stdev->sthal_prop_api_version = 0;
        ret = 0; /* passthru for backward compability */
    } else {
        stdev->sthal_prop_api_version = *(int *)sthal_prop_api_version;
        if (MAJOR_VERSION(stdev->sthal_prop_api_version) !=
            MAJOR_VERSION(STHAL_PROP_API_CURRENT_VERSION)) {
            ALOGE("%s: Incompatible API versions sthal:0x%x != ahal:0x%x",
                __func__, STHAL_PROP_API_CURRENT_VERSION,
                stdev->sthal_prop_api_version);
            goto error;
        }
        ALOGD("%s: ahal is using proprietary API version 0x%04x", __func__,
            stdev->sthal_prop_api_version);
    }

    ALOGD("%s: load AHAL successfully.", __func__);
    return ret;

error:
    dlclose(stdev->audio_hal_handle);
    stdev->audio_hal_handle = NULL;
    return ret;
}

static int stdev_open(const hw_module_t *module, const char *name,
        hw_device_t **device)
{
    struct knowles_sound_trigger_device *stdev;
    int ret = 0, i = 0;
    int snd_card_num = 0;

    ALOGE("!! Knowles SoundTrigger v1!!");

    if (strcmp(name, SOUND_TRIGGER_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    if (device == NULL)
        return -EINVAL;

    stdev = &g_stdev;
    pthread_mutex_lock(&stdev->lock);

    snd_card_num = find_sound_card();
    if (snd_card_num == -1) {
        ALOGE("%s: Unable to find the sound card %s", __func__, CARD_NAME);
        ret = -EAGAIN;
        goto exit;
    }

    if (stdev->opened) {
        ALOGE("%s: Only one sountrigger can be opened at a time", __func__);
        ret = -EBUSY;
        goto exit;
    }

    ret = open_streaming_lib(stdev);
    if (ret != 0) {
        ALOGE("%s: Couldnot open the streaming library", __func__);
        goto error;
    }

    ret = load_audio_hal();
    if (ret != 0) {
        ALOGE("%s: Couldn't load AHAL", __func__);
        goto error;
    }

    stdev->device.common.tag = HARDWARE_DEVICE_TAG;
    stdev->device.common.version = SOUND_TRIGGER_DEVICE_API_VERSION_1_2;
    stdev->device.common.module = (struct hw_module_t *)module;
    stdev->device.common.close = stdev_close;
    stdev->device.get_properties = stdev_get_properties;
    stdev->device.load_sound_model = stdev_load_sound_model;
    stdev->device.unload_sound_model = stdev_unload_sound_model;
    stdev->device.start_recognition = stdev_start_recognition;
    stdev->device.stop_recognition = stdev_stop_recognition;
    stdev->device.get_model_state = stdev_get_model_state;

    stdev->opened = true;
    /* Initialize all member variable */
    for (i = 0; i < MAX_MODELS; i++) {
        stdev->models[i].config = NULL;
        stdev->models[i].data = NULL;
        stdev->models[i].data_sz = 0;
        stdev->models[i].is_loaded = false;
        stdev->models[i].is_active = false;
        stdev->last_keyword_detected_config = NULL;
        stdev->models[i].is_state_query = false;
    }

    stdev->is_mic_route_enabled = false;
    stdev->is_music_playing = false;
    stdev->is_bargein_route_enabled = false;
    stdev->is_buffer_package_loaded = false;
    stdev->hotword_buffer_enable = 0;
    stdev->music_buffer_enable = 0;
    stdev->current_enable = 0;
    stdev->is_hmd_proc_on = false;
    stdev->is_dmx_proc_on = false;
    stdev->is_chre_enable = false;
    stdev->is_media_recording = false;

    stdev->snd_crd_num = snd_card_num;
    stdev->fw_reset_done_by_hal = false;

    str_to_uuid(HOTWORD_AUDIO_MODEL, &stdev->hotword_model_uuid);
    str_to_uuid(WAKEUP_MODEL, &stdev->wakeup_model_uuid);
    str_to_uuid(SENSOR_MANAGER_MODEL, &stdev->sensor_model_uuid);
    str_to_uuid(AMBIENT_AUDIO_MODEL, &stdev->ambient_model_uuid);
    str_to_uuid(CHRE_AUDIO_MODEL, &stdev->chre_model_uuid);
    str_to_uuid(ENTITY_AUDIO_MODEL, &stdev->entity_model_uuid);

    stdev->odsp_hdl = iaxxx_odsp_init();
    if (stdev->odsp_hdl == NULL) {
        ALOGE("%s: Failed to get handle to ODSP HAL", __func__);
        ret = -EIO;
        goto error;
    }
    stdev->mixer = find_stdev_mixer_path(stdev->snd_crd_num, stdev->mixer_path_xml);
    if (stdev->mixer == NULL) {
        ALOGE("Failed to init the mixer");
        ret = -EAGAIN;
        goto error;
    }

    ALOGD("stdev before pthread_create %p", stdev);
    // Create a thread to handle all events from kernel
    pthread_create(&stdev->callback_thread, (const pthread_attr_t *) NULL,
                callback_thread_loop, stdev);

    *device = &stdev->device.common; /* same address as stdev */
exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;

error:
    if (stdev->adnc_cvq_strm_lib)
        dlclose(stdev->adnc_cvq_strm_lib);
    if (stdev->audio_hal_handle)
        dlclose(stdev->audio_hal_handle);
    if (stdev->route_hdl)
        audio_route_free(stdev->route_hdl);
    if (stdev->odsp_hdl)
        iaxxx_odsp_deinit(stdev->odsp_hdl);
    if (stdev->mixer)
        mixer_close(stdev->mixer);

    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

/* AHAL calls this callback to communicate with STHAL */
int sound_trigger_hw_call_back(audio_event_type_t event,
                            struct audio_event_info *config)
{
    int ret = 0;
    int i = 0;
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    if (!stdev)
        return -ENODEV;

    if (!stdev->opened) {
        ALOGE("%s: Error SoundTrigger has not been opened", __func__);
        return -EINVAL;
    }

    pthread_mutex_lock(&stdev->lock);

    if (stdev->is_st_hal_ready == false) {
        ALOGE("%s: ST HAL is not ready yet", __func__);
        ret = -EINVAL;
        goto exit;
    }

    switch (event) {
    case AUDIO_EVENT_CAPTURE_DEVICE_INACTIVE:
        if (stdev->is_chre_enable == true) {
            for (i = 0; i < MAX_MODELS; i++) {
                if (check_uuid_equality(stdev->models[i].uuid,
                                        stdev->chre_model_uuid)) {
                    if (stdev->models[i].is_active == false) {
                        stdev->models[i].is_active = true;
                        handle_input_source(stdev, true);
                        setup_package(stdev, &stdev->models[i]);
                        set_package_route(stdev, stdev->models[i].uuid,
                                        stdev->is_bargein_route_enabled);
                    }
                }
            }
        }
        stdev->is_media_recording = false;
    case AUDIO_EVENT_CAPTURE_STREAM_INACTIVE:
        /*
         * [TODO] handle capture device on/off event
         * There might be concurrency devices usecase.
         *
         */
        ALOGD("%s: handle capture inactive event %d", __func__, event);
        break;
    case AUDIO_EVENT_CAPTURE_DEVICE_ACTIVE:
        stdev->is_media_recording = true;
    case AUDIO_EVENT_CAPTURE_STREAM_ACTIVE:
        /*
         * Handle capture active event
         * That stop all recognition in codec
         */
        ALOGD("%s: handle capture active event %d", __func__, event);

        for (i = 0; i < MAX_MODELS; i++) {
            if (stdev->models[i].is_active == true) {
                tear_package_route(stdev, stdev->models[i].uuid,
                                stdev->is_bargein_route_enabled);
                stdev->models[i].is_active = false;
                destroy_package(stdev, &stdev->models[i]);
                if ((stdev->hotword_buffer_enable) &&
                    !(stdev->current_enable & PLUGIN1_MASK))
                    tear_hotword_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);

                if ((stdev->music_buffer_enable) &&
                    !(stdev->current_enable & PLUGIN2_MASK))
                    tear_music_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);

                setup_buffer(stdev, &stdev->models[i], false);
            }
        }

        handle_input_source(stdev, false);
        break;
    case AUDIO_EVENT_PLAYBACK_STREAM_INACTIVE:
        ALOGD("%s: handle playback stream inactive", __func__);

        if (stdev->is_music_playing != false) {
            stdev->is_music_playing = false;
            if (stdev->is_mic_route_enabled != false) {
                // Atleast one keyword model is active so update the routes
                // Check if the bargein route is enabled if not enable bargein route
                // Check each model, if it is active then update it's route
                if (stdev->is_bargein_route_enabled != false) {
                    ALOGD("Bargein disable");
                    stdev->is_bargein_route_enabled = false;
                    // Check each model, if it is active then update it's route
                    // Disable the bargein route
                    for (i = 0; i < MAX_MODELS; i++) {
                        if (stdev->models[i].is_active == true) {
                            // teardown the package route with bargein
                            ret = tear_package_route(stdev,
                                                    stdev->models[i].uuid,
                                                    !stdev->is_bargein_route_enabled);
                            if (ret != 0) {
                                ALOGE("Failed to tear old package route");
                                goto exit;
                            }
                            // resetup the package route with out bargein
                            ret = set_package_route(stdev,
                                                    stdev->models[i].uuid,
                                                    stdev->is_bargein_route_enabled);
                            if (ret != 0) {
                                ALOGE("Failed to enable package route");
                                goto exit;
                            }
                        }
                    }

                    //Switch buffer input source
                    if (stdev->hotword_buffer_enable) {
                        ret = tear_hotword_buffer_route(stdev->route_hdl,
                                            !stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to tear old buffer route");
                            goto exit;
                        }
                        ret = set_hotword_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to enable buffer route");
                            goto exit;
                        }
                    }

                    if (stdev->music_buffer_enable) {
                        ret = tear_music_buffer_route(stdev->route_hdl,
                                            !stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to tear old music buffer route");
                            goto exit;
                        }
                        ret = set_music_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to enable buffer route");
                            goto exit;
                        }
                    }

                    ret = enable_bargein_route(stdev->route_hdl, false);
                    if (ret != 0) {
                        ALOGE("Failed to enable buffer route");
                        goto exit;
                    }

                    ret = destroy_aec_package(stdev->odsp_hdl);
                    if (ret != 0) {
                        ALOGE("Failed to unload AEC package");
                        goto exit;
                    }

                    ret = enable_mic_route(stdev->route_hdl, false,
                                        EXTERNAL_OSCILLATOR);
                    if (ret != 0) {
                        ALOGE("Failed to disable mic route with INT OSC");
                        goto exit;
                    }
                    ret = enable_mic_route(stdev->route_hdl, true,
                                        INTERNAL_OSCILLATOR);
                    if (ret != 0) {
                        ALOGE("Failed to enable mic route with EXT OSC");
                        goto exit;
                    }
                }
            }
        } else {
            ALOGD("%s: STHAL setup playback Inactive alrealy", __func__);
        }
        break;
    case AUDIO_EVENT_PLAYBACK_STREAM_ACTIVE:
        ALOGD("%s: handle playback stream active", __func__);
        if (!(config->device_info.device & AUDIO_DEVICE_OUT_SPEAKER)) {
            ALOGD("%s: Playback device doesn't include SPEAKER.",
                __func__);
            goto exit;
        }
        if (stdev->is_music_playing != true) {
            stdev->is_music_playing = true;
            if (stdev->is_mic_route_enabled != false) {
                // Atleast one keyword model is active so update the routes
                // Check if the bargein route is enabled if not enable bargein route
                // Check each model, if it is active then update it's route
                if (stdev->is_bargein_route_enabled != true) {
                    ALOGD("Bargein enable");
                    ret = enable_mic_route(stdev->route_hdl, false,
                                        INTERNAL_OSCILLATOR);
                    if (ret != 0) {
                        ALOGE("Failed to disable mic route with INT OSC");
                        goto exit;
                    }
                    ret = enable_mic_route(stdev->route_hdl, true,
                                        EXTERNAL_OSCILLATOR);
                    if (ret != 0) {
                        ALOGE("Failed to enable mic route with EXT OSC");
                        goto exit;
                    }
                    ret = setup_aec_package(stdev->odsp_hdl);
                    if (ret != 0) {
                        ALOGE("Failed to load AEC package");
                        goto exit;
                    }

                    ret = enable_bargein_route(stdev->route_hdl, true);
                    if (ret != 0) {
                        ALOGE("Failed to enable buffer route");
                        goto exit;
                    }
                    stdev->is_bargein_route_enabled = true;

                    if (stdev->hotword_buffer_enable) {
                        ret = tear_hotword_buffer_route(stdev->route_hdl,
                                            !stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to tear old buffer route");
                            goto exit;
                        }
                        ret = set_hotword_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to enable buffer route");
                            goto exit;
                        }
                    }

                    if (stdev->music_buffer_enable) {
                        ret = tear_music_buffer_route(stdev->route_hdl,
                                            !stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to tear old music buffer route");
                            goto exit;
                        }
                        ret = set_music_buffer_route(stdev->route_hdl,
                                            stdev->is_bargein_route_enabled);
                        if (ret != 0) {
                            ALOGE("Failed to enable buffer route");
                            goto exit;
                        }
                    }

                    // Check each model, if it is active then update it's route
                    for (i = 0; i < MAX_MODELS; i++) {
                        if (stdev->models[i].is_active == true) {
                            // teardown the package route without bargein
                            ret = tear_package_route(stdev,
                                                    stdev->models[i].uuid,
                                                    !stdev->is_bargein_route_enabled);
                            if (ret != 0) {
                                ALOGE("Failed to tear old package route");
                                goto exit;
                            }
                            // resetup the package route with bargein
                            ret = set_package_route(stdev,
                                                    stdev->models[i].uuid,
                                                    stdev->is_bargein_route_enabled);
                            if (ret != 0) {
                                ALOGE("Failed to enable package route");
                                goto exit;
                            }
                        }
                    }
                }
            }
        } else {
            ALOGD("%s: STHAL setup playback active alrealy", __func__);
        }
        break;
    case AUDIO_EVENT_STOP_LAB:
        /* Close Stream Driver */
        ALOGD("%s: close streaming %d", __func__, event);
        if (stdev->adnc_strm_handle) {
            stdev->adnc_strm_close(stdev->adnc_strm_handle);
            stdev->adnc_strm_handle = 0;
            stdev->is_streaming = 0;
        }

        break;

    case AUDIO_EVENT_SSR:
        /*[TODO] Do we need to handle adsp SSR event ? */
        ALOGD("%s: handle audio subsystem restart %d", __func__, event);
        break;

    case AUDIO_EVENT_READ_SAMPLES:
        /* Open Stream Driver */
        if (stdev->is_streaming == false) {
            if (stdev->adnc_strm_open == NULL) {
                ALOGE("%s: Error adnc streaming not supported", __func__);
            } else {
                bool keyword_stripping_enabled = false;
                int stream_end_point;
                switch (stdev->last_detected_model_type) {
                    case OK_GOOGLE_KW_ID:
                        stream_end_point = CVQ_ENDPOINT;
                        break;
                    case AMBIENT_KW_ID:
                    case ENTITY_KW_ID:
                        stream_end_point = MUSIC_BUF_ENDPOINT;
                        break;
                    default:
                        stream_end_point = CVQ_ENDPOINT;
                        break;
                };
                stdev->adnc_strm_handle = stdev->adnc_strm_open(
                                            keyword_stripping_enabled, 0,
                                            stream_end_point);
                if (stdev->adnc_strm_handle) {
                    ALOGD("Successfully opened adnc streaming");
                    stdev->is_streaming = true;
                } else {
                    ALOGE("%s: DSP is currently not streaming", __func__);
                }
            }
        }
        /* Read pcm data from tunnel */
        if (stdev->is_streaming == true) {
            //ALOGD("%s: soundtrigger HAL adnc_strm_read", __func__);
            stdev->adnc_strm_read(stdev->adnc_strm_handle,
                                config->u.aud_info.buf,
                                config->u.aud_info.num_bytes);
        } else {
            ALOGE("%s: soundtrigger is not streaming", __func__);
        }

        break;

    case AUDIO_EVENT_NUM_ST_SESSIONS:
    case AUDIO_EVENT_DEVICE_CONNECT:
    case AUDIO_EVENT_DEVICE_DISCONNECT:
    case AUDIO_EVENT_SVA_EXEC_MODE:
    case AUDIO_EVENT_SVA_EXEC_MODE_STATUS:
        ALOGV("%s: useless event %d", __func__, event);
        break;

    default:
        ALOGW("%s: Unknown event %d", __func__, event);
        break;
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = stdev_open,
};

struct sound_trigger_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = SOUND_TRIGGER_MODULE_API_VERSION_1_0,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = SOUND_TRIGGER_HARDWARE_MODULE_ID,
        .name = "Knowles Sound Trigger HAL",
        .author = "Knowles Electronics",
        .methods = &hal_module_methods,
    },
};
