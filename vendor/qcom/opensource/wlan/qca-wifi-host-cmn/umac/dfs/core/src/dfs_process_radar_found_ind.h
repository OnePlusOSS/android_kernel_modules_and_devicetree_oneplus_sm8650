/*
 * Copyright (c) 2017-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: dfs_process_radar_found_ind.h
 * This file provides prototypes of the routines needed for the
 * external components to utilize the services provided by the
 * DFS component.
 */

#ifndef _DFS_PROCESS_RADAR_FOUND_IND_H_
#define _DFS_PROCESS_RADAR_FOUND_IND_H_
#include "dfs_partial_offload_radar.h"

#define BW_INVALID    0
#define BW_10        10
#define BW_20        20
#define BW_40        40
#define BW_80        80
#define BW_160      160
#define BW_320      320
/**
 * dfs_flush_additional_pulses() - Reset dfs radar detection related
 * variables and queues after processing radar and disabling phyerror reception.
 *
 * @dfs: Pointer to wlan_dfs structure.
 */
static inline void
dfs_flush_additional_pulses(struct wlan_dfs *dfs)
{
	dfs_false_radarfound_reset_vars(dfs);
}

/* Number of channel marking offsets */
#define DFS_NUM_FREQ_OFFSET   3

/* Lower channel from 20 Mhz center channel */
#define DFS_20MHZ_LOWER_CHANNEL(_f)    ((_f) - 20)
/* Upper channel from 20 Mhz center channel */
#define DFS_20MHZ_UPPER_CHANNEL(_f)    ((_f) + 20)
/* 1st lower channel from center channel of bandwidth 40/80/160Mhz */
#define DFS_FIRST_LOWER_CHANNEL(_f)    ((_f) - 10)
/* 2nd lower channel from center channel of bandwidth 40/80/160Mhz */
#define DFS_SECOND_LOWER_CHANNEL(_f)   ((_f) - 30)
/* 3rd lower channel from center channel of bandwidth 80/160Mhz */
#define DFS_THIRD_LOWER_CHANNEL(_f)    ((_f) - 50)
/* 1st upper channel from center channel of bandwidth 40/80/160Mhz */
#define DFS_FIRST_UPPER_CHANNEL(_f)    ((_f) + 10)
/* 2nd upper channel from center channel of bandwidth 40/80/160Mhz */
#define DFS_SECOND_UPPER_CHANNEL(_f)   ((_f) + 30)
/* 3rd upper channel from center channel of bandwidth 80/160Mhz */
#define DFS_THIRD_UPPER_CHANNEL(_f)    ((_f) + 50)

/* 20 Mhz freq_offset lower */
#define DFS_20MZ_OFFSET_LOWER    (-10)
/* 20 Mhz freq_offset upper */
#define DFS_20MZ_OFFSET_UPPER     (10)
/* 40/80 Mhz freq_offset first lower */
#define DFS_OFFSET_FIRST_LOWER    (-20)
/* 40/80 Mhz freq_offset second lower */
#define DFS_OFFSET_SECOND_LOWER   (-40)
/* 40/80 Mhz freq_offset first upper */
#define DFS_OFFSET_FIRST_UPPER     (20)
/* 40/80 Mhz freq_offset second upper */
#define DFS_OFFSET_SECOND_UPPER    (40)

/* Frequency offset to sidx */
#define DFS_FREQ_OFFSET_TO_SIDX(_f)  ((32 * (_f)) / 10)
/* Sidx to frequency offset */
#define DFS_SIDX_TO_FREQ_OFFSET(_s)  ((10 * (_s)) / 32)
/* sidx offset boundary */
#define DFS_BOUNDARY_SIDX  32
/* freq offset for chirp */
#define DFS_CHIRP_OFFSET  10
/* second segment freq offset */
#define DFS_160MHZ_SECOND_SEG_OFFSET  40
#define DFS_165MHZ_SECOND_SEG_OFFSET_LEFT 40 /* in MHz */
#define DFS_165MHZ_SECOND_SEG_OFFSET_RIGHT 45 /* in MHz  */
/*Primary segment id is 0 */
#define PRIMARY_SEG 0
/* Secondary segment id is 1 */
#define SECONDARY_SEG 1

/* Frequency offset indices */
#define CENTER_CH 0
#define LEFT_CH   1
#define RIGHT_CH  2

#ifdef CONFIG_CHAN_FREQ_API
/* Next channel frequency offsets from center channel frequency */
#define DFS_5GHZ_NEXT_CHAN_FREQ_OFFSET   10
#define DFS_5GHZ_2ND_CHAN_FREQ_OFFSET    30
#define DFS_5GHZ_3RD_CHAN_FREQ_OFFSET    50
#define DFS_5GHZ_4TH_CHAN_FREQ_OFFSET    70
#define DFS_5GHZ_5TH_CHAN_FREQ_OFFSET    90
#define DFS_5GHZ_6TH_CHAN_FREQ_OFFSET   110
#define DFS_5GHZ_7TH_CHAN_FREQ_OFFSET   130
#define DFS_5GHZ_8TH_CHAN_FREQ_OFFSET   150
#endif
/* Number of 20MHz sub-channels in 160 MHz segment */
#define NUM_CHANNELS_160MHZ  8
/* Number of 20MHz sub-channels in 320 MHz segment */
#define NUM_CHANNELS_320MHZ 16
/* Number of 20MHz sub-channels in 240 MHz (320-80) segment */
#define NUM_CHANNELS_240MHZ 12

#ifdef WLAN_FEATURE_11BE
#define MAX_20MHZ_SUBCHANS NUM_CHANNELS_320MHZ
#else
#define MAX_20MHZ_SUBCHANS NUM_CHANNELS_160MHZ
#endif

#if defined(QCA_DFS_RCSA_SUPPORT)
/**
 * dfs_send_nol_ie_and_rcsa()- Send NOL IE and RCSA action frames.
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: Pointer to radar found structure.
 * @nol_freq_list: List of 20MHz frequencies on which radar has been detected.
 * @num_channels: number of radar affected channels.
 * @wait_for_csa: indicates if the repeater AP should take DFS action or wait
 * for CSA
 *
 * Return: void.
 */
void dfs_send_nol_ie_and_rcsa(struct wlan_dfs *dfs,
			      struct radar_found_info *radar_found,
			      uint16_t *nol_freq_list,
			      uint8_t num_channels,
			      bool *wait_for_csa);
#else
static inline
void dfs_send_nol_ie_and_rcsa(struct wlan_dfs *dfs,
			      struct radar_found_info *radar_found,
			      uint16_t *nol_freq_list,
			      uint8_t num_channels,
			      bool *wait_for_csa)
{
}
#endif /* QCA_DFS_RCSA_SUPPORT */

/**
 * struct freqs_offsets - frequency and offset information
 * @freq: channel frequency in mhz.
 * @offset: offset from center frequency.
 *
 * Index 0 - Center channel affected by RADAR.
 * Index 1 - Left of Center channel affected by RADAR.
 * Index 2 - Right of Center channel affected by RADAR.
 *
 * This information is needed to find and mark radar infected
 * channels in NOL and regulatory database.
 */
struct freqs_offsets {
	uint32_t freq[DFS_NUM_FREQ_OFFSET];
	int32_t offset[DFS_NUM_FREQ_OFFSET];
};

/**
 * dfs_process_radar_found_indication() - Process radar found indication
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: radar found info.
 *
 * Process radar found indication and update radar effected channel in NOL
 * and regulatory.
 *
 * Return: None
 */
void dfs_process_radar_found_indication(struct wlan_dfs *dfs,
		struct radar_found_info *radar_found);

#if defined(QCA_DFS_BW_PUNCTURE)
/**
 * dfs_generate_radar_bitmap() - Generate radar bitmap for DFS channel
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_freq_list: Channel list affected by radar.
 * @num_radar_channels: Number of channels affected by radar.
 *
 * Return: Bitmap of radar punctured channels.
 */
uint16_t dfs_generate_radar_bitmap(struct wlan_dfs *dfs,
				   uint16_t *radar_freq_list,
				   uint8_t num_radar_channels);
#else
static inline
uint16_t dfs_generate_radar_bitmap(struct wlan_dfs *dfs,
				   uint16_t *radar_freq_list,
				   uint8_t num_radar_channels)
{
	return 0;
}
#endif

/**
 * dfs_handle_radar_puncturing() - Check if the puncture bitmap is valid
 *                                 and initialize puncture SM for the
 *                                 punctured channels.
 * @dfs:                      Pointer to wlan_dfs structure.
 * @dfs_radar_bitmap:         Puncture bitmap.
 * @freq_list:                Channel list affected by radar.
 * @num_channels:             Number of channels affected by radar.
 * @is_ignore_radar_puncture: Bool to check if radar should be ignored.
 *
 * Return: Nothing.
 */
#if defined(QCA_DFS_BW_PUNCTURE) && !defined(CONFIG_REG_CLIENT)
void
dfs_handle_radar_puncturing(struct wlan_dfs *dfs,
			    uint16_t *dfs_radar_bitmap,
			    uint16_t *freq_list,
			    uint8_t num_channels,
			    bool *is_ignore_radar_puncture);
#else
static inline
void dfs_handle_radar_puncturing(struct wlan_dfs *dfs,
				 uint16_t *dfs_radar_bitmap,
				 uint16_t *freq_list,
				 uint8_t num_channels,
				 bool *is_ignore_radar_puncture)
{
}
#endif /* QCA_DFS_BW_PUNCTURE */

/**
 * dfs_process_radar_ind() - Process radar indication event
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: Pointer to radar_found_info structure.
 *
 * Wrapper function of dfs_process_radar_found_indication().
 *
 * Return: QDF_STATUS
 */
QDF_STATUS dfs_process_radar_ind(struct wlan_dfs *dfs,
		struct radar_found_info *radar_found);

/**
 * dfs_process_radar_ind_on_home_chan() - Process radar indication event on
 * home channel.
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: Pointer to radar_found_info structure.
 *
 * Return: QDF_STATUS
 */
QDF_STATUS
dfs_process_radar_ind_on_home_chan(struct wlan_dfs *dfs,
				   struct radar_found_info *radar_found);

/**
 * dfs_radarfound_action_generic() - The dfs action on radar detection by host
 * for domains other than FCC.
 * @dfs: Pointer to wlan_dfs structure.
 * @seg_id: segment id.
 *
 * Return: None
 */
void dfs_radarfound_action_generic(struct wlan_dfs *dfs, uint8_t seg_id);

/**
 * dfs_get_bonding_channels_for_freq() - Get bonding channels.
 * @dfs:         Pointer to wlan_dfs structure.
 * @curchan:     Pointer to dfs_channels to know width and primary channel.
 * @segment_id:  Segment id, useful for 80+80/160 MHz operating band.
 * @detector_id: Detector id, used to find if radar is detected on
 *               Agile detector.
 * @freq_list:   Pointer to save radar affected channel's frequency.
 *
 * Return: Number of channels.
 */
#ifdef CONFIG_CHAN_FREQ_API
uint8_t dfs_get_bonding_channels_for_freq(struct wlan_dfs *dfs,
					  struct dfs_channel *curchan,
					  uint32_t segment_id,
					  uint8_t detector_id,
					  uint16_t *freq_list);

/**
 * dfs_compute_radar_found_cfreq(): Computes the centre frequency of the
 * radar hit channel.
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: Pointer to radar_found_info.
 * @freq_center: Pointer to retrieve the value of radar found cfreq.
 */
void
dfs_compute_radar_found_cfreq(struct wlan_dfs *dfs,
			      struct radar_found_info *radar_found,
			      uint32_t *freq_center);

/**
 * dfs_find_radar_affected_channels()- Find the radar affected 20MHz channels.
 * @dfs: Pointer to wlan_dfs structure.
 * @radar_found: Pointer to radar found structure.
 * @freq_list: List of 20MHz frequencies on which radar has been detected.
 * @freq_center: Frequency center of the band on which the radar was detected.
 *
 * Return: number of radar affected channels.
 */
uint8_t
dfs_find_radar_affected_channels(struct wlan_dfs *dfs,
				 struct radar_found_info *radar_found,
				 uint16_t *freq_list,
				 uint32_t freq_center);

/**
 * dfs_radar_add_channel_list_to_nol_for_freq()- Add given channels to nol
 * @dfs: Pointer to wlan_dfs structure.
 * @freq_list: Pointer to list of frequency(has both nonDFS and DFS channels).
 * Input frequency list.
 * @nol_freq_list: Pointer to list of NOL frequencies. Output frequency list.
 * @num_channels: Pointer to number of channels in the list. It is both input
 * and output to this function.
 * *Input: Number of subchannels in @freq_list.
 * *Output: Number of subchannels in @nol_freq_list.
 *
 * Add list of channels to nol, only if the channel is dfs.
 *
 * Return: QDF_STATUS
 */
QDF_STATUS
dfs_radar_add_channel_list_to_nol_for_freq(struct wlan_dfs *dfs,
					   uint16_t *freq_list,
					   uint16_t *nol_freq_list,
					   uint8_t *num_channels);
#endif

/**
 * dfs_reset_bangradar() - Rest bangradar parameters.
 * @dfs: Pointer to wlan_dfs structure.
 *
 * Return: void.
 */
void dfs_reset_bangradar(struct wlan_dfs *dfs);

/**
 * dfs_send_csa_to_current_chan() - Send CSA to current channel
 * @dfs: Pointer to wlan_dfs structure.
 *
 * For the test mode(usenol = 0), don't do a CSA; but setup the test timer so
 * we get a CSA _back_ to the current operating channel.
 */
void dfs_send_csa_to_current_chan(struct wlan_dfs *dfs);

/**
 * dfs_get_bonding_channel_without_seg_info_for_freq() - Get bonding channels
 * in chan.
 * @chan: Pointer to dfs_channel structure.
 * @freq_list: channel array holding list of bonded channel's frequency.
 *
 * Return: number of sub channels in the input channel.
 */
#ifdef CONFIG_CHAN_FREQ_API
uint8_t
dfs_get_bonding_channel_without_seg_info_for_freq(struct dfs_channel *chan,
						  uint16_t *freq_list);
#endif

/**
 * dfs_set_nol_subchannel_marking() - Set or unset NOL subchannel marking.
 * @dfs: Pointer to wlan_dfs structure.
 * @nol_subchannel_marking: Configure NOL subchannel marking.
 *
 * Return: Status of the configuration.
 */
int
dfs_set_nol_subchannel_marking(struct wlan_dfs *dfs,
			       bool nol_subchannel_marking);

/**
 * dfs_get_nol_subchannel_marking() - Get the value of NOL subchannel marking.
 * @dfs: Pointer to wlan_dfs structure.
 * @nol_subchannel_marking: Read and store the value of NOL subchannel marking
 * config.
 *
 * Return: Status of the read.
 */
int
dfs_get_nol_subchannel_marking(struct wlan_dfs *dfs,
			       bool *nol_subchannel_marking);

#ifdef WLAN_DFS_FULL_OFFLOAD
/**
 * dfs_inc_num_radar - Increment radar detect stats for FO.
 *
 * @dfs: Pointer to the wlan_dfs object.
 *
 * Return: void.
 */
void dfs_inc_num_radar(struct wlan_dfs *dfs);
#else
static inline void dfs_inc_num_radar(struct wlan_dfs *dfs)
{
}
#endif

#if defined(WLAN_DFS_TRUE_160MHZ_SUPPORT) && defined(WLAN_DFS_FULL_OFFLOAD)
#define DFS_80P80MHZ_SECOND_SEG_OFFSET 85
/**
 * dfs_translate_radar_params() - Translate the radar parameters received in
 *                                true 160MHz supported chipsets.
 * @dfs: Pointer to the wlan_dfs object.
 * @radar_found: Radar found parameters.
 *
 * Radar found parameters in true 160MHz detectors are represented below:
 *
 * Offset received with respect to the center of 160MHz ranging from -80 to +80.
 *          __________________________________________
 *         |                                          |
 *         |             160 MHz Channel              |
 *         |__________________________________________|
 *         |        |           |           |         |
 *         |        |           |           |         |
 *        -80    -ve offset   center    +ve offset   +80
 *
 *
 * Radar found parameters after translation by this API:
 *
 * Offsets with respect to pri/sec 80MHz center ranging from -40 to +40.
 *          __________________________________________
 *         |                    |                     |
 *         |             160 MHz|Channel              |
 *         |____________________|_____________________|
 *         |         |          |           |         |
 *         |         |          |           |         |
 *        -40    pri center  +40/-40     sec center  +40
 *
 * Return: void.
 */
void
dfs_translate_radar_params(struct wlan_dfs *dfs,
			   struct radar_found_info *radar_found);
#else
static inline void
dfs_translate_radar_params(struct wlan_dfs *dfs,
			   struct radar_found_info *radar_found)
{
}
#endif /* WLAN_DFS_TRUE_160MHZ_SUPPORT */
#endif /*_DFS_PROCESS_RADAR_FOUND_IND_H_ */
