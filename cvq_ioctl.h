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

#ifndef _CVQ_IOCTL_H
#define _CVQ_IOCTL_H

#include "iaxxx_odsp_hw.h"
#include <audio_route/audio_route.h>
#include <tinyalsa/asoundlib.h>

#define HOTWORD_MASK 0x1
#define AMBIENT_MASK 0x2
#define ENTITY_MASK  0x4
#define PLUGIN1_MASK (HOTWORD_MASK | WAKEUP_MASK)
#define PLUGIN2_MASK (AMBIENT_MASK | ENTITY_MASK)
#define WAKEUP_MASK  0x8
#define OSLO_MASK  0x10
#define CHRE_MASK  0x20

#define HOTWORD_EVT_SRC_ID            IAXXX_SYSID_PLUGIN_INSTANCE_0
#define AMBIENT_EVT_SRC_ID            IAXXX_SYSID_PLUGIN_INSTANCE_2
#define CHRE_EVT_SRC_ID               IAXXX_SYSID_PLUGIN_INSTANCE_5
// HOST1 Plugin instances start from 8 to 15
// plugin 8 is assigned for Oslo buffer and 9 for Oslo plugin
#define OSLO_EVT_SRC_ID               IAXXX_SYSID_PLUGIN_INSTANCE_9

#define HOTWORD_PKG_ID      11
#define HOTWORD_PLUGIN_IDX  0
#define HOTWORD_INSTANCE_ID 0
#define HOTWORD_PRIORITY    1

#define AMBIENT_PKG_ID          12
#define AMBIENT_PLUGIN_IDX      0
#define AMBIENT_INSTANCE_ID     2
#define AMBIENT_PRIORITY        1

#define SENSOR_PKG_ID           0
#define SENSOR_PLUGIN_IDX       0
#define SENSOR_INSTANCE_ID      9
#define SENSOR_PRIORITY         1
#define SENSOR_PRESENCE_MODE    0
#define SENSOR_DETECTED_MODE    1
#define SENSOR_MAX_MODE         2
#define OSLO_CONFIGURED         (0x201)
#define OSLO_DESTROYED          (0x202)
#define OSLO_BUF_INSTANCE_ID    8

#define AEC_PKG_ID       7
#define AEC_PLUGIN_IDX   0
#define AEC_INSTANCE_ID  4
#define AEC_PRIORITY     1

#define CHRE_PLUGIN_IDX      0
#define CHRE_INSTANCE_ID     5
#define CHRE_EVT_ID          3
#define CHRE_EVT_PARAM_ID    8
#define CHRE_BUF_SIZE        60480
#define CHRE_CONFIGURED      (0x203)
#define CHRE_DESTROYED       (0x204)
#define CHRE_EVT_MASK        (0x1f)

#define MIXER_PKG_ID       5
#define MIXER_PLUGIN_IDX   0
#define MIXER_INSTANCE_ID  7
#define MIXER_PRIORITY     1

#define BUF_PKG_ID          4
#define BUF_PLUGIN_IDX      0
#define BUF_INSTANCE_ID     1
#define BUF_PRIORITY        1

#define DA_BUF_INSTANCE_ID    3

#define HOTWORD_DETECTION   0
#define AMBIENT_DETECTION   1
#define ENTITY_DETECTION    2
#define WAKEUP_DETECTION    3

#define HOTWORD_SLOT_ID     1
#define AMBIENT_SLOT_ID     3
#define ENTITY_SLOT_ID      5
#define WAKEUP_SLOT_ID      6

#define HOTWORD_UNLOAD_PARAM_ID          1
#define AMBIENT_UNLOAD_PARAM_ID          1
#define AMBIENT_RESET_PARAM_ID           2
#define AMBIENT_GET_MODEL_STATE_PARAM_ID 7

#define BUFFER_PACKAGE              "BufferPackage.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_VAL_MULTI_SEC "BufferConfigVal.bin"
#define BUFFER_CONFIG_VAL_2_SEC     "BufferConfigVal2Sec.bin"
#define OK_GOOGLE_PACKAGE           "OkGooglePackage.bin"
#define AMBIENT_EC_PACKAGE          "AmbientECPackage.bin"
#define AMBIENT_DA_PACKAGE          "AmbientDAPackage.bin"
#define SENSOR_PACKAGE              "OsloSensorPackage.bin"
#define SENSOR_CONFIG_VAL           "OsloSensorConfig.bin"
#define ECHOCANCELLER_PACKAGE       "EchoCancellerPackage.bin"
#define MIXER_PACKAGE               "AScalarSimpleMixerPackage.bin"

#define MIC_ROUTE                            "mic1-route"
#define MIC_ROUTE_EXT_CLK                    "mic-route-external-clock"
#define MIC_ROUTE_INT_CLK                    "mic-route-internal-clock"
#define BARGEIN_ROUTE                        "bargein-route"
#define DOWNLINK_AUDIO_ROUTE                 "downlink-audio-route"
#define MUSIC_AUDIO_ROUTE                    "music-audio-route"
#define SENSOR_ROTUE                         "oslo-route"
#define HOTWORD_WITH_BARGEIN_ROUTE           "hotword-route-with-bargein"
#define HOTWORD_WITHOUT_BARGEIN_ROUTE        "hotword-route-without-bargein"
#define CHRE_WITH_BARGEIN_ROUTE              "chre-route-with-bargein"
#define CHRE_WITHOUT_BARGEIN_ROUTE           "chre-route-without-bargein"
#define AMBIENT_WITH_BARGEIN_ROUTE           "ambient-route-with-bargein"
#define AMBIENT_WITHOUT_BARGEIN_ROUTE        "ambient-route-without-bargein"
#define BUFFER_WITH_BARGEIN_ROUTE            "buffer-route-with-bargein"
#define BUFFER_WITHOUT_BARGEIN_ROUTE         "buffer-route-without-bargein"
#define POWER_DOWN_ROUTE                     "pwr-dwn-all-non-ctrl-proc-mem"
#define POWER_ON_HMD_PROC_MEM                "pwr-up-hmd-proc-mem"
#define POWER_ON_SSP_PROC_MEM                "pwr-up-ssp-proc-mem"
#define POWER_ON_DMX_PROC_MEM                "pwr-up-dmx-proc-mem"
#define POWER_OFF_HMD_PROC_MEM               "pwr-dwn-hmd-proc-mem"
#define POWER_OFF_SSP_PROC_MEM               "pwr-dwn-ssp-proc-mem"
#define POWER_OFF_DMX_PROC_MEM               "pwr-dwn-dmx-proc-mem"
#define POWER_OFF_SSP_PROC_MEM_IN_RETN       "pwr-dwn-ssp-proc-mem-in-retn"
#define POWER_OFF_DMX_PROC_MEM_IN_RETN       "pwr-dwn-dmx-proc-mem-in-retn"
#define POWER_OFF_HMD_PROC_MEM_IN_RETN       "pwr-dwn-hmd-proc-mem-in-retn"
#define POWER_ON_HMD_PROC_MEM_OUT_OFF_RETN   "pwr-on-hmd-proc-mem-out-off-retn"
#define POWER_ON_DMX_PROC_MEM_OUT_OFF_RETN   "pwr-on-dmx-proc-mem-out-off-retn"
#define POWER_ON_SSP_PROC_MEM_OUT_OFF_RETN   "pwr-on-ssp-proc-mem-out-off-retn"

enum clock_type {
    INTERNAL_OSCILLATOR,
    EXTERNAL_OSCILLATOR
};

#define PLUGIN_DEF_CONFIG_ID    0

int write_model(struct iaxxx_odsp_hw *odsp_hdl, unsigned char *data,
                int length, int kw_type);
int get_model_state(struct iaxxx_odsp_hw *odsp_hdl, const uint32_t inst_id,
                    const uint32_t param_val);
int get_event(struct iaxxx_odsp_hw *odsp_hdl,
            struct iaxxx_get_event_info *ge);
int setup_chip(struct iaxxx_odsp_hw *odsp_hdl);
int setup_buffer_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_buffer_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_hotword_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_hotword_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_ambient_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_ambient_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_aec_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_aec_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_chre_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_chre_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_sensor_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_sensor_package(struct iaxxx_odsp_hw *odsp_hdl);
int setup_mixer_package(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_mixer_package(struct iaxxx_odsp_hw *odsp_hdl);

int setup_music_buffer(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_music_buffer(struct iaxxx_odsp_hw *odsp_hdl);
int setup_howord_buffer(struct iaxxx_odsp_hw *odsp_hdl);
int destroy_howord_buffer(struct iaxxx_odsp_hw *odsp_hdl);
int set_hotword_buffer_route(struct audio_route *route_hdl, bool bargein);
int tear_hotword_buffer_route(struct audio_route *route_hdl, bool bargein);
int enable_mic_route(struct audio_route *route_hdl, bool enable,
                    enum clock_type ct);
int set_sensor_route(struct audio_route *route_hdl, bool enable);
int set_ambient_state(struct iaxxx_odsp_hw *odsp_hdl, unsigned int current);
int tear_ambient_state(struct iaxxx_odsp_hw *odsp_hdl, unsigned int current);
int set_ambient_route(struct audio_route *route_hdl, bool bargein);
int tear_ambient_route(struct audio_route *route_hdl, bool bargein);
int set_hotword_state(struct iaxxx_odsp_hw *odsp_hdl, unsigned int current);
int tear_hotword_state(struct iaxxx_odsp_hw *odsp_hdl, unsigned int current);
int set_hotword_route(struct audio_route *route_hdl, bool bargein);
int tear_hotword_route(struct audio_route *route_hdl, bool bargein);
int set_chre_audio_route(struct audio_route *route_hdl, bool bargein);
int tear_chre_audio_route(struct audio_route *route_hdl, bool bargein);
int reset_ambient_plugin(struct iaxxx_odsp_hw *odsp_hdl);
int enable_bargein_route(struct audio_route *route_hdl, bool enable);
int set_music_buffer_route(struct audio_route *route_hdl, bool downlink);
int tear_music_buffer_route(struct audio_route *route_hdl, bool downlink);

int flush_model(struct iaxxx_odsp_hw *odsp_hdl, int kw_type);
int get_entity_param_blk(struct iaxxx_odsp_hw *odsp_hdl, void *payload, unsigned int payload_size);
int get_wakeup_param_blk(struct iaxxx_odsp_hw *odsp_hdl, void *payload, unsigned int payload_size);
int power_down_all_non_ctrl_proc_mem(struct mixer *mixer);
int power_on_proc_mem(struct audio_route *route_hdl, int enable, int core);
int power_off_proc_mem(struct audio_route *route_hdl, int enable, int core);
int power_off_proc_mem_in_retn(struct audio_route *route_hdl, int enable, int core);
int power_on_proc_mem_out_off_retn(struct audio_route *route_hdl, int enable, int core);
int get_fw_status(struct iaxxx_odsp_hw *odsp_hdl, unsigned int *status);
int reset_fw(struct iaxxx_odsp_hw *odsp_hdl);
#endif /* _CVQ_IOCTL_H */
