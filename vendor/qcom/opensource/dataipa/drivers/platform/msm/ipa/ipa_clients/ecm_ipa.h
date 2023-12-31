/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _ECM_IPA_H_
#define _ECM_IPA_H_

#include "ipa.h"

/*
 * @priv: private data given upon ipa_connect
 * @evt: event enum, should be IPA_WRITE_DONE
 * @data: for tx path the data field is the sent socket buffer.
 */
typedef void (*ecm_ipa_callback)(void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data);

/*
 * struct ecm_ipa_params - parameters for ecm_ipa initialization API
 *
 * @device_ready_notify: callback supplied by USB core driver.
 * This callback shall be called by the Netdev once the device
 * is ready to receive data from tethered PC.
 * @ecm_ipa_rx_dp_notify: ecm_ipa will set this callback (out parameter).
 * this callback shall be supplied for ipa_connect upon pipe
 * connection (USB->IPA), once IPA driver receive data packets
 * from USB pipe destined for Apps this callback will be called.
 * @ecm_ipa_tx_dp_notify: ecm_ipa will set this callback (out parameter).
 * this callback shall be supplied for ipa_connect upon pipe
 * connection (IPA->USB), once IPA driver send packets destined
 * for USB, IPA BAM will notify for Tx-complete.
 * @priv: ecm_ipa will set this pointer (out parameter).
 * This pointer will hold the network device for later interaction
 * with ecm_ipa APIs
 * @host_ethaddr: host Ethernet address in network order
 * @device_ethaddr: device Ethernet address in network order
 * @skip_ep_cfg: boolean field that determines if Apps-processor
 *  should or should not configure this end-point.
 */
struct ecm_ipa_params {
	void (*device_ready_notify)(void);
	ecm_ipa_callback ecm_ipa_rx_dp_notify;
	ecm_ipa_callback ecm_ipa_tx_dp_notify;
	u8 host_ethaddr[ETH_ALEN];
	u8 device_ethaddr[ETH_ALEN];
	void *private;
	bool skip_ep_cfg;
};


#if IS_ENABLED(CONFIG_ECM_IPA)

int ecm_ipa_init(struct ecm_ipa_params *params);

int ecm_ipa_connect(u32 usb_to_ipa_hdl, u32 ipa_to_usb_hdl,
		void *priv);

int ecm_ipa_disconnect(void *priv);

void ecm_ipa_cleanup(void *priv);

#else /* IS_ENABLED(CONFIG_ECM_IPA) */

static inline int ecm_ipa_init(struct ecm_ipa_params *params)
{
	return 0;
}

static inline int ecm_ipa_connect(u32 usb_to_ipa_hdl, u32 ipa_to_usb_hdl,
		void *priv)
{
	return 0;
}

static inline int ecm_ipa_disconnect(void *priv)
{
	return 0;
}

static inline void ecm_ipa_cleanup(void *priv)
{

}
#endif /* IS_ENABLED(CONFIG_ECM_IPA) */

#endif /* _ECM_IPA_H_ */
