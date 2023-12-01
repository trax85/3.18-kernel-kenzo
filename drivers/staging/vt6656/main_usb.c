/*
 * Copyright (c) 1996, 2003 VIA Networking Technologies, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * File: main_usb.c
 *
 * Purpose: driver entry for initial, open, close, tx and rx.
 *
 * Author: Lyndon Chen
 *
 * Date: Dec 8, 2005
 *
 * Functions:
 *
 *   vt6656_probe - module initial (insmod) driver entry
 *   vnt_free_tx_bufs - free tx buffer function
 *   vnt_init_registers- initial MAC & BBP & RF internal registers.
 *
 * Revision History:
 */
#undef __NO_VERSION__

#include <linux/file.h>
#include "device.h"
#include "card.h"
#include "baseband.h"
#include "mac.h"
#include "power.h"
#include "wcmd.h"
#include "rxtx.h"
#include "dpc.h"
#include "rf.h"
#include "firmware.h"
#include "usbpipe.h"
#include "channel.h"
#include "int.h"

/*
 * define module options
 */

/* version information */
#define DRIVER_AUTHOR \
	"VIA Networking Technologies, Inc., <lyndonchen@vntek.com.tw>"
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DEVICE_FULL_DRV_NAM);

#define RX_DESC_DEF0 64
static int vnt_rx_buffers = RX_DESC_DEF0;
module_param_named(rx_buffers, vnt_rx_buffers, int, 0644);
MODULE_PARM_DESC(rx_buffers, "Number of receive usb rx buffers");

#define TX_DESC_DEF0 64
static int vnt_tx_buffers = TX_DESC_DEF0;
module_param_named(tx_buffers, vnt_tx_buffers, int, 0644);
MODULE_PARM_DESC(tx_buffers, "Number of receive usb tx buffers");

#define RTS_THRESH_DEF     2347
#define FRAG_THRESH_DEF     2346
#define SHORT_RETRY_DEF     8
#define LONG_RETRY_DEF     4

/* BasebandType[] baseband type selected
   0: indicate 802.11a type
   1: indicate 802.11b type
   2: indicate 802.11g type
*/

#define BBP_TYPE_DEF     2

/*
 * Static vars definitions
 */

static struct usb_device_id vt6656_table[] = {
	{USB_DEVICE(VNT_USB_VENDOR_ID, VNT_USB_PRODUCT_ID)},
	{}
};

static void vnt_set_options(struct vnt_private *priv)
{
	/* Set number of TX buffers */
	if (vnt_tx_buffers < CB_MIN_TX_DESC || vnt_tx_buffers > CB_MAX_TX_DESC)
		priv->num_tx_context = TX_DESC_DEF0;
	else
		priv->num_tx_context = vnt_tx_buffers;

	/* Set number of RX buffers */
	if (vnt_rx_buffers < CB_MIN_RX_DESC || vnt_rx_buffers > CB_MAX_RX_DESC)
		priv->num_rcb = RX_DESC_DEF0;
	else
		priv->num_rcb = vnt_rx_buffers;

	priv->short_retry_limit = SHORT_RETRY_DEF;
	priv->long_retry_limit = LONG_RETRY_DEF;
	priv->op_mode = NL80211_IFTYPE_UNSPECIFIED;
	priv->bb_type = BBP_TYPE_DEF;
	priv->packet_type = priv->bb_type;
	priv->auto_fb_ctrl = AUTO_FB_0;
	priv->preamble_type = 0;
	priv->exist_sw_net_addr = false;
}

/*
 * initialization of MAC & BBP registers
 */
static int vnt_init_registers(struct vnt_private *priv)
{
	struct vnt_cmd_card_init *init_cmd = &priv->init_command;
	struct vnt_rsp_card_init *init_rsp = &priv->init_response;
	u8 antenna;
	int ii;
	int status = STATUS_SUCCESS;
	u8 tmp;
	u8 calib_tx_iq = 0, calib_tx_dc = 0, calib_rx_iq = 0;

	dev_dbg(&priv->usb->dev, "---->INIbInitAdapter. [%d][%d]\n",
				DEVICE_INIT_COLD, priv->packet_type);

	if (!vnt_check_firmware_version(priv)) {
		if (vnt_download_firmware(priv) == true) {
			if (vnt_firmware_branch_to_sram(priv) == false) {
				dev_dbg(&priv->usb->dev,
					" vnt_firmware_branch_to_sram fail\n");
				return false;
			}
		} else {
			dev_dbg(&priv->usb->dev, "FIRMWAREbDownload fail\n");
			return false;
		}
	}

	if (!vnt_vt3184_init(priv)) {
		dev_dbg(&priv->usb->dev, "vnt_vt3184_init fail\n");
		return false;
	}

	init_cmd->init_class = DEVICE_INIT_COLD;
	init_cmd->exist_sw_net_addr = priv->exist_sw_net_addr;
	for (ii = 0; ii < 6; ii++)
		init_cmd->sw_net_addr[ii] = priv->current_net_addr[ii];
	init_cmd->short_retry_limit = priv->short_retry_limit;
	init_cmd->long_retry_limit = priv->long_retry_limit;

	/* issue card_init command to device */
	status = vnt_control_out(priv,
		MESSAGE_TYPE_CARDINIT, 0, 0,
		sizeof(struct vnt_cmd_card_init), (u8 *)init_cmd);
	if (status != STATUS_SUCCESS) {
		dev_dbg(&priv->usb->dev, "Issue Card init fail\n");
		return false;
	}

	status = vnt_control_in(priv, MESSAGE_TYPE_INIT_RSP, 0, 0,
		sizeof(struct vnt_rsp_card_init), (u8 *)init_rsp);
	if (status != STATUS_SUCCESS) {
		dev_dbg(&priv->usb->dev,
			"Cardinit request in status fail!\n");
		return false;
	}

	/* local ID for AES functions */
	status = vnt_control_in(priv, MESSAGE_TYPE_READ,
		MAC_REG_LOCALID, MESSAGE_REQUEST_MACREG, 1,
			&priv->local_id);
	if (status != STATUS_SUCCESS)
		return false;

	/* do MACbSoftwareReset in MACvInitialize */

	priv->top_ofdm_basic_rate = RATE_24M;
	priv->top_cck_basic_rate = RATE_1M;

	/* target to IF pin while programming to RF chip */
	priv->power = 0xFF;

	priv->cck_pwr = priv->eeprom[EEP_OFS_PWR_CCK];
	priv->ofdm_pwr_g = priv->eeprom[EEP_OFS_PWR_OFDMG];
	/* load power table */
	for (ii = 0; ii < 14; ii++) {
		priv->cck_pwr_tbl[ii] =
			priv->eeprom[ii + EEP_OFS_CCK_PWR_TBL];
		if (priv->cck_pwr_tbl[ii] == 0)
			priv->cck_pwr_tbl[ii] = priv->cck_pwr;

		priv->ofdm_pwr_tbl[ii] =
				priv->eeprom[ii + EEP_OFS_OFDM_PWR_TBL];
		if (priv->ofdm_pwr_tbl[ii] == 0)
			priv->ofdm_pwr_tbl[ii] = priv->ofdm_pwr_g;
	}

	/*
	 * original zonetype is USA, but custom zonetype is Europe,
	 * then need to recover 12, 13, 14 channels with 11 channel
	 */
	for (ii = 11; ii < 14; ii++) {
		priv->cck_pwr_tbl[ii] = priv->cck_pwr_tbl[10];
		priv->ofdm_pwr_tbl[ii] = priv->ofdm_pwr_tbl[10];
	}

	priv->ofdm_pwr_a = 0x34; /* same as RFbMA2829SelectChannel */

	/* load OFDM A power table */
	for (ii = 0; ii < CB_MAX_CHANNEL_5G; ii++) {
		priv->ofdm_a_pwr_tbl[ii] =
			priv->eeprom[ii + EEP_OFS_OFDMA_PWR_TBL];

		if (priv->ofdm_a_pwr_tbl[ii] == 0)
			priv->ofdm_a_pwr_tbl[ii] = priv->ofdm_pwr_a;
	}

	antenna = priv->eeprom[EEP_OFS_ANTENNA];

	if (antenna & EEP_ANTINV)
		priv->tx_rx_ant_inv = true;
	else
		priv->tx_rx_ant_inv = false;

	antenna &= (EEP_ANTENNA_AUX | EEP_ANTENNA_MAIN);

	if (antenna == 0) /* if not set default is both */
		antenna = (EEP_ANTENNA_AUX | EEP_ANTENNA_MAIN);

	if (antenna == (EEP_ANTENNA_AUX | EEP_ANTENNA_MAIN)) {
		priv->tx_antenna_mode = ANT_B;
		priv->rx_antenna_sel = 1;

		if (priv->tx_rx_ant_inv == true)
			priv->rx_antenna_mode = ANT_A;
		else
			priv->rx_antenna_mode = ANT_B;
	} else  {
		priv->rx_antenna_sel = 0;

		if (antenna & EEP_ANTENNA_AUX) {
			priv->tx_antenna_mode = ANT_A;

			if (priv->tx_rx_ant_inv == true)
				priv->rx_antenna_mode = ANT_B;
			else
				priv->rx_antenna_mode = ANT_A;
		} else {
			priv->tx_antenna_mode = ANT_B;

		if (priv->tx_rx_ant_inv == true)
			priv->rx_antenna_mode = ANT_A;
		else
			priv->rx_antenna_mode = ANT_B;
		}
	}

	/* Set initial antenna mode */
	vnt_set_antenna_mode(priv, priv->rx_antenna_mode);

	/* get Auto Fall Back type */
	priv->auto_fb_ctrl = AUTO_FB_0;

	/* default Auto Mode */
	priv->bb_type = BB_TYPE_11G;

	/* get RFType */
	priv->rf_type = init_rsp->rf_type;

	/* load vt3266 calibration parameters in EEPROM */
	if (priv->rf_type == RF_VT3226D0) {
		if ((priv->eeprom[EEP_OFS_MAJOR_VER] == 0x1) &&
		    (priv->eeprom[EEP_OFS_MINOR_VER] >= 0x4)) {

			calib_tx_iq = priv->eeprom[EEP_OFS_CALIB_TX_IQ];
			calib_tx_dc = priv->eeprom[EEP_OFS_CALIB_TX_DC];
			calib_rx_iq = priv->eeprom[EEP_OFS_CALIB_RX_IQ];
			if (calib_tx_iq || calib_tx_dc || calib_rx_iq) {
				/* CR255, enable TX/RX IQ and
				   DC compensation mode */
				vnt_control_out_u8(priv,
						   MESSAGE_REQUEST_BBREG,
						   0xff,
						   0x03);
				/* CR251, TX I/Q Imbalance Calibration */
				vnt_control_out_u8(priv,
						   MESSAGE_REQUEST_BBREG,
						   0xfb,
						   calib_tx_iq);
				/* CR252, TX DC-Offset Calibration */
				vnt_control_out_u8(priv,
						   MESSAGE_REQUEST_BBREG,
						   0xfC,
						   calib_tx_dc);
				/* CR253, RX I/Q Imbalance Calibration */
				vnt_control_out_u8(priv,
						   MESSAGE_REQUEST_BBREG,
						   0xfd,
						   calib_rx_iq);
			} else {
				/* CR255, turn off
				   BB Calibration compensation */
				vnt_control_out_u8(priv,
						   MESSAGE_REQUEST_BBREG,
						   0xff,
						   0x0);
			}
		}
	}

	/* get permanent network address */
	memcpy(priv->permanent_net_addr, init_rsp->net_addr, 6);
	memcpy(priv->current_net_addr, priv->permanent_net_addr, ETH_ALEN);

	/* if exist SW network address, use it */
	dev_dbg(&priv->usb->dev, "Network address = %pM\n",
		priv->current_net_addr);

	/*
	* set BB and packet type at the same time
	* set Short Slot Time, xIFS, and RSPINF
	*/
	if (priv->bb_type == BB_TYPE_11A)
		priv->short_slot_time = true;
	else
		priv->short_slot_time = false;

	vnt_set_short_slot_time(priv);

	priv->radio_ctl = priv->eeprom[EEP_OFS_RADIOCTL];

	if ((priv->radio_ctl & EEP_RADIOCTL_ENABLE) != 0) {
		status = vnt_control_in(priv, MESSAGE_TYPE_READ,
			MAC_REG_GPIOCTL1, MESSAGE_REQUEST_MACREG, 1, &tmp);

		if (status != STATUS_SUCCESS)
			return false;

		if ((tmp & GPIO3_DATA) == 0)
			vnt_mac_reg_bits_on(priv, MAC_REG_GPIOCTL1,
								GPIO3_INTMD);
		else
			vnt_mac_reg_bits_off(priv, MAC_REG_GPIOCTL1,
								GPIO3_INTMD);
	}

	vnt_mac_set_led(priv, LEDSTS_TMLEN, 0x38);

	vnt_mac_set_led(priv, LEDSTS_STS, LEDSTS_SLOW);

	vnt_mac_reg_bits_on(priv, MAC_REG_GPIOCTL0, 0x01);

	vnt_radio_power_on(priv);

	dev_dbg(&priv->usb->dev, "<----INIbInitAdapter Exit\n");

	return true;
}

static void vnt_free_tx_bufs(struct vnt_private *priv)
{
	struct vnt_usb_send_context *tx_context;
	int ii;

	for (ii = 0; ii < priv->num_tx_context; ii++) {
		tx_context = priv->tx_context[ii];
		/* deallocate URBs */
		if (tx_context->urb) {
			usb_kill_urb(tx_context->urb);
			usb_free_urb(tx_context->urb);
		}

		kfree(tx_context);
	}
}

static void vnt_free_rx_bufs(struct vnt_private *priv)
{
	struct vnt_rcb *rcb;
	int ii;

	for (ii = 0; ii < priv->num_rcb; ii++) {
		rcb = priv->rcb[ii];
		if (!rcb)
			continue;

		/* deallocate URBs */
		if (rcb->urb) {
			usb_kill_urb(rcb->urb);
			usb_free_urb(rcb->urb);
		}

		/* deallocate skb */
		if (rcb->skb)
			dev_kfree_skb(rcb->skb);

		kfree(rcb);
	}
}

static void usb_device_reset(struct vnt_private *priv)
{
	int status;

	status = usb_reset_device(priv->usb);
	if (status)
		dev_warn(&priv->usb->dev,
			 "usb_device_reset fail status=%d\n", status);
}

static void vnt_free_int_bufs(struct vnt_private *priv)
{
	kfree(priv->int_buf.data_buf);
}

static bool vnt_alloc_bufs(struct vnt_private *priv)
{
	struct vnt_usb_send_context *tx_context;
	struct vnt_rcb *rcb;
	int ii;

	for (ii = 0; ii < priv->num_tx_context; ii++) {
		tx_context = kmalloc(sizeof(struct vnt_usb_send_context),
								GFP_KERNEL);
		if (tx_context == NULL) {
			dev_err(&priv->usb->dev,
					"allocate tx usb context failed\n");
			goto free_tx;
		}

		priv->tx_context[ii] = tx_context;
		tx_context->priv = priv;
		tx_context->pkt_no = ii;

		/* allocate URBs */
		tx_context->urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!tx_context->urb) {
			dev_err(&priv->usb->dev, "alloc tx urb failed\n");
			goto free_tx;
		}

		tx_context->in_use = false;
	}

	for (ii = 0; ii < priv->num_rcb; ii++) {
		priv->rcb[ii] = kzalloc(sizeof(struct vnt_rcb), GFP_KERNEL);
		if (!priv->rcb[ii]) {
			dev_err(&priv->usb->dev,
					"failed to allocate rcb no %d\n", ii);
			goto free_rx_tx;
		}

		rcb = priv->rcb[ii];

		rcb->priv = priv;

		/* allocate URBs */
		rcb->urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (rcb->urb == NULL) {
			dev_err(&priv->usb->dev, "Failed to alloc rx urb\n");
			goto free_rx_tx;
		}

		rcb->skb = dev_alloc_skb(priv->rx_buf_sz);
		if (rcb->skb == NULL) {
			dev_err(&priv->usb->dev, "Failed to alloc rx skb\n");
			goto free_rx_tx;
		}

		rcb->in_use = false;

		/* submit rx urb */
		if (vnt_submit_rx_urb(priv, rcb))
			goto free_rx_tx;
	}

	priv->interrupt_urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (priv->interrupt_urb == NULL) {
		dev_err(&priv->usb->dev, "Failed to alloc int urb\n");
		goto free_rx_tx;
	}

	priv->int_buf.data_buf = kmalloc(MAX_INTERRUPT_SIZE, GFP_KERNEL);
	if (priv->int_buf.data_buf == NULL) {
		dev_err(&priv->usb->dev, "Failed to alloc int buf\n");
		usb_free_urb(priv->interrupt_urb);
		goto free_rx_tx;
	}

	return true;

free_rx_tx:
	vnt_free_rx_bufs(priv);

free_tx:
	vnt_free_tx_bufs(priv);

	return false;
}

static void vnt_tx_80211(struct ieee80211_hw *hw,
	struct ieee80211_tx_control *control, struct sk_buff *skb)
{
	struct vnt_private *priv = hw->priv;

	ieee80211_stop_queues(hw);

	if (vnt_tx_packet(priv, skb)) {
		ieee80211_free_txskb(hw, skb);

		ieee80211_wake_queues(hw);
	}
}

static int vnt_start(struct ieee80211_hw *hw)
{
	struct vnt_private *priv = hw->priv;

	priv->rx_buf_sz = MAX_TOTAL_SIZE_WITH_ALL_HEADERS;

	if (vnt_alloc_bufs(priv) == false) {
		dev_dbg(&priv->usb->dev, "vnt_alloc_bufs fail...\n");
		return -ENOMEM;
	}

	clear_bit(DEVICE_FLAGS_DISCONNECTED, &priv->flags);

	if (vnt_init_registers(priv) == false) {
		dev_dbg(&priv->usb->dev, " init register fail\n");
		goto free_all;
	}

	if (vnt_key_init_table(priv))
		goto free_all;

	priv->int_interval = 1;  /* bInterval is set to 1 */

	vnt_int_start_interrupt(priv);

	ieee80211_wake_queues(hw);

	return 0;

free_all:
	vnt_free_rx_bufs(priv);
	vnt_free_tx_bufs(priv);
	vnt_free_int_bufs(priv);

	usb_kill_urb(priv->interrupt_urb);
	usb_free_urb(priv->interrupt_urb);

	return -ENOMEM;
}

static void vnt_stop(struct ieee80211_hw *hw)
{
	struct vnt_private *priv = hw->priv;
	int i;

	if (!priv)
		return;

	for (i = 0; i < MAX_KEY_TABLE; i++)
		vnt_mac_disable_keyentry(priv, i);

	/* clear all keys */
	priv->key_entry_inuse = 0;

	if (!test_bit(DEVICE_FLAGS_UNPLUG, &priv->flags))
		vnt_mac_shutdown(priv);

	ieee80211_stop_queues(hw);

	set_bit(DEVICE_FLAGS_DISCONNECTED, &priv->flags);

	cancel_delayed_work_sync(&priv->run_command_work);

	priv->cmd_running = false;

	vnt_free_tx_bufs(priv);
	vnt_free_rx_bufs(priv);
	vnt_free_int_bufs(priv);

	usb_kill_urb(priv->interrupt_urb);
	usb_free_urb(priv->interrupt_urb);
}

static int vnt_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct vnt_private *priv = hw->priv;

	priv->vif = vif;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
		break;
	case NL80211_IFTYPE_ADHOC:
		vnt_mac_reg_bits_off(priv, MAC_REG_RCR, RCR_UNICAST);

		vnt_mac_reg_bits_on(priv, MAC_REG_HOSTCR, HOSTCR_ADHOC);

		break;
	case NL80211_IFTYPE_AP:
		vnt_mac_reg_bits_off(priv, MAC_REG_RCR, RCR_UNICAST);

		vnt_mac_reg_bits_on(priv, MAC_REG_HOSTCR, HOSTCR_AP);

		break;
	default:
		return -EOPNOTSUPP;
	}

	priv->op_mode = vif->type;

	vnt_set_bss_mode(priv);

	/* LED blink on TX */
	vnt_mac_set_led(priv, LEDSTS_STS, LEDSTS_INTER);

	return 0;
}

static void vnt_remove_interface(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif)
{
	struct vnt_private *priv = hw->priv;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
		break;
	case NL80211_IFTYPE_ADHOC:
		vnt_mac_reg_bits_off(priv, MAC_REG_TCR, TCR_AUTOBCNTX);
		vnt_mac_reg_bits_off(priv, MAC_REG_TFTCTL, TFTCTL_TSFCNTREN);
		vnt_mac_reg_bits_off(priv, MAC_REG_HOSTCR, HOSTCR_ADHOC);
		break;
	case NL80211_IFTYPE_AP:
		vnt_mac_reg_bits_off(priv, MAC_REG_TCR, TCR_AUTOBCNTX);
		vnt_mac_reg_bits_off(priv, MAC_REG_TFTCTL, TFTCTL_TSFCNTREN);
		vnt_mac_reg_bits_off(priv, MAC_REG_HOSTCR, HOSTCR_AP);
		break;
	default:
		break;
	}

	vnt_radio_power_off(priv);

	priv->op_mode = NL80211_IFTYPE_UNSPECIFIED;

	/* LED slow blink */
	vnt_mac_set_led(priv, LEDSTS_STS, LEDSTS_SLOW);
}

static int vnt_config(struct ieee80211_hw *hw, u32 changed)
{
	struct vnt_private *priv = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;
	u8 bb_type;

	if (changed & IEEE80211_CONF_CHANGE_PS) {
		if (conf->flags & IEEE80211_CONF_PS)
			vnt_enable_power_saving(priv, conf->listen_interval);
		else
			vnt_disable_power_saving(priv);
	}

	if ((changed & IEEE80211_CONF_CHANGE_CHANNEL) ||
			(conf->flags & IEEE80211_CONF_OFFCHANNEL)) {
		vnt_set_channel(priv, conf->chandef.chan->hw_value);

		if (conf->chandef.chan->band == IEEE80211_BAND_5GHZ)
			bb_type = BB_TYPE_11A;
		else
			bb_type = BB_TYPE_11G;

		if (priv->bb_type != bb_type) {
			priv->bb_type = bb_type;

			vnt_set_bss_mode(priv);
		}
	}

	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		if (priv->bb_type == BB_TYPE_11B)
			priv->current_rate = RATE_1M;
		else
			priv->current_rate = RATE_54M;

		vnt_rf_setpower(priv, priv->current_rate,
				conf->chandef.chan->hw_value);
	}

	return 0;
}

static void vnt_bss_info_changed(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif, struct ieee80211_bss_conf *conf,
		u32 changed)
{
	struct vnt_private *priv = hw->priv;

	priv->current_aid = conf->aid;

	if (changed & BSS_CHANGED_BSSID)
		vnt_mac_set_bssid_addr(priv, (u8 *)conf->bssid);


	if (changed & BSS_CHANGED_BASIC_RATES) {
		priv->basic_rates = conf->basic_rates;

		vnt_update_top_rates(priv);

		dev_dbg(&priv->usb->dev, "basic rates %x\n", conf->basic_rates);
	}

	if (changed & BSS_CHANGED_ERP_PREAMBLE) {
		if (conf->use_short_preamble) {
			vnt_mac_enable_barker_preamble_mode(priv);
			priv->preamble_type = true;
		} else {
			vnt_mac_disable_barker_preamble_mode(priv);
			priv->preamble_type = false;
		}
	}

	if (changed & BSS_CHANGED_ERP_CTS_PROT) {
		if (conf->use_cts_prot)
			vnt_mac_enable_protect_mode(priv);
		else
			vnt_mac_disable_protect_mode(priv);
	}

	if (changed & BSS_CHANGED_ERP_SLOT) {
		if (conf->use_short_slot)
			priv->short_slot_time = true;
		else
			priv->short_slot_time = false;

		vnt_set_short_slot_time(priv);
		vnt_set_vga_gain_offset(priv, priv->bb_vga[0]);
		vnt_update_pre_ed_threshold(priv, false);
	}

	if (changed & BSS_CHANGED_TXPOWER)
		vnt_rf_setpower(priv, priv->current_rate,
					conf->chandef.chan->hw_value);

	if (changed & BSS_CHANGED_BEACON_ENABLED) {
		dev_dbg(&priv->usb->dev,
				"Beacon enable %d\n", conf->enable_beacon);

		if (conf->enable_beacon) {
			vnt_beacon_enable(priv, vif, conf);

			vnt_mac_reg_bits_on(priv, MAC_REG_TCR, TCR_AUTOBCNTX);
		} else {
			vnt_mac_reg_bits_off(priv, MAC_REG_TCR, TCR_AUTOBCNTX);
		}
	}
}

static u64 vnt_prepare_multicast(struct ieee80211_hw *hw,
	struct netdev_hw_addr_list *mc_list)
{
	struct vnt_private *priv = hw->priv;
	struct netdev_hw_addr *ha;
	u64 mc_filter = 0;
	u32 bit_nr = 0;

	netdev_hw_addr_list_for_each(ha, mc_list) {
		bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;

		mc_filter |= 1ULL << (bit_nr & 0x3f);
	}

	priv->mc_list_count = mc_list->count;

	return mc_filter;
}

static void vnt_configure(struct ieee80211_hw *hw,
	unsigned int changed_flags, unsigned int *total_flags, u64 multicast)
{
	struct vnt_private *priv = hw->priv;
	u8 rx_mode = 0;
	int rc;

	*total_flags &= FIF_ALLMULTI | FIF_OTHER_BSS | FIF_PROMISC_IN_BSS |
		FIF_BCN_PRBRESP_PROMISC;

	rc = vnt_control_in(priv, MESSAGE_TYPE_READ, MAC_REG_RCR,
		MESSAGE_REQUEST_MACREG, sizeof(u8), &rx_mode);

	if (!rc)
		rx_mode = RCR_MULTICAST | RCR_BROADCAST;

	dev_dbg(&priv->usb->dev, "rx mode in = %x\n", rx_mode);

	if (changed_flags & FIF_PROMISC_IN_BSS) {
		/* unconditionally log net taps */
		if (*total_flags & FIF_PROMISC_IN_BSS)
			rx_mode |= RCR_UNICAST;
		else
			rx_mode &= ~RCR_UNICAST;
	}

	if (changed_flags & FIF_ALLMULTI) {
		if (*total_flags & FIF_ALLMULTI) {
			if (priv->mc_list_count > 2)
				vnt_mac_set_filter(priv, ~0);
			else
				vnt_mac_set_filter(priv, multicast);

			rx_mode |= RCR_MULTICAST | RCR_BROADCAST;
		} else {
			rx_mode &= ~(RCR_MULTICAST | RCR_BROADCAST);
		}

	}

	if (changed_flags & (FIF_OTHER_BSS | FIF_BCN_PRBRESP_PROMISC)) {
		if (*total_flags & (FIF_OTHER_BSS | FIF_BCN_PRBRESP_PROMISC))
			rx_mode &= ~RCR_BSSID;
		else
			rx_mode |= RCR_BSSID;
	}

	vnt_control_out_u8(priv, MESSAGE_REQUEST_MACREG, MAC_REG_RCR, rx_mode);

	dev_dbg(&priv->usb->dev, "rx mode out= %x\n", rx_mode);
}

static int vnt_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
	struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		struct ieee80211_key_conf *key)
{
	struct vnt_private *priv = hw->priv;

	switch (cmd) {
	case SET_KEY:
		if (vnt_set_keys(hw, sta, vif, key))
			return -EOPNOTSUPP;
		break;
	case DISABLE_KEY:
		if (test_bit(key->hw_key_idx, &priv->key_entry_inuse))
			clear_bit(key->hw_key_idx, &priv->key_entry_inuse);
	default:
		break;
	}

	return 0;
}

static void vnt_sw_scan_start(struct ieee80211_hw *hw)
{
	struct vnt_private *priv = hw->priv;

	vnt_set_bss_mode(priv);
	/* Set max sensitivity*/
	vnt_update_pre_ed_threshold(priv, true);
}

static void vnt_sw_scan_complete(struct ieee80211_hw *hw)
{
	struct vnt_private *priv = hw->priv;

	/* Return sensitivity to channel level*/
	vnt_update_pre_ed_threshold(priv, false);
}

static int vnt_get_stats(struct ieee80211_hw *hw,
				struct ieee80211_low_level_stats *stats)
{
	struct vnt_private *priv = hw->priv;

	memcpy(stats, &priv->low_stats, sizeof(*stats));

	return 0;
}

static u64 vnt_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct vnt_private *priv = hw->priv;

	return priv->current_tsf;
}

static void vnt_set_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			u64 tsf)
{
	struct vnt_private *priv = hw->priv;

	vnt_update_next_tbtt(priv, tsf, vif->bss_conf.beacon_int);
}

static void vnt_reset_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct vnt_private *priv = hw->priv;

	vnt_mac_reg_bits_off(priv, MAC_REG_TFTCTL, TFTCTL_TSFCNTREN);

	vnt_clear_current_tsf(priv);
}

static const struct ieee80211_ops vnt_mac_ops = {
	.tx			= vnt_tx_80211,
	.start			= vnt_start,
	.stop			= vnt_stop,
	.add_interface		= vnt_add_interface,
	.remove_interface	= vnt_remove_interface,
	.config			= vnt_config,
	.bss_info_changed	= vnt_bss_info_changed,
	.prepare_multicast	= vnt_prepare_multicast,
	.configure_filter	= vnt_configure,
	.set_key		= vnt_set_key,
	.sw_scan_start		= vnt_sw_scan_start,
	.sw_scan_complete	= vnt_sw_scan_complete,
	.get_stats		= vnt_get_stats,
	.get_tsf		= vnt_get_tsf,
	.set_tsf		= vnt_set_tsf,
	.reset_tsf		= vnt_reset_tsf,
};

int vnt_init(struct vnt_private *priv)
{

	if (!(vnt_init_registers(priv)))
		return -EAGAIN;

	SET_IEEE80211_PERM_ADDR(priv->hw, priv->permanent_net_addr);

	vnt_init_bands(priv);

	if (ieee80211_register_hw(priv->hw))
		return -ENODEV;

	priv->mac_hw = true;

	vnt_radio_power_off(priv);

	return 0;
}

static int
vt6656_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev;
	struct vnt_private *priv;
	struct ieee80211_hw *hw;
	struct wiphy *wiphy;
	int rc = 0;

	udev = usb_get_dev(interface_to_usbdev(intf));

	dev_notice(&udev->dev, "%s Ver. %s\n",
					DEVICE_FULL_DRV_NAM, DEVICE_VERSION);
	dev_notice(&udev->dev,
		"Copyright (c) 2004 VIA Networking Technologies, Inc.\n");

	hw = ieee80211_alloc_hw(sizeof(struct vnt_private), &vnt_mac_ops);
	if (!hw) {
		dev_err(&udev->dev, "could not register ieee80211_hw\n");
		goto err_nomem;
	}

	priv = hw->priv;
	priv->hw = hw;
	priv->usb = udev;

	vnt_set_options(priv);

	spin_lock_init(&priv->lock);
	mutex_init(&priv->usb_lock);

	INIT_DELAYED_WORK(&priv->run_command_work, vnt_run_command);

	usb_set_intfdata(intf, priv);

	wiphy = priv->hw->wiphy;

	wiphy->frag_threshold = FRAG_THRESH_DEF;
	wiphy->rts_threshold = RTS_THRESH_DEF;
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
		BIT(NL80211_IFTYPE_ADHOC) | BIT(NL80211_IFTYPE_AP);

	priv->hw->flags = IEEE80211_HW_RX_INCLUDES_FCS |
		IEEE80211_HW_REPORTS_TX_ACK_STATUS |
		IEEE80211_HW_SIGNAL_DBM |
		IEEE80211_HW_TIMING_BEACON_ONLY;

	priv->hw->max_signal = 100;

	SET_IEEE80211_DEV(priv->hw, &intf->dev);

	usb_device_reset(priv);

	clear_bit(DEVICE_FLAGS_DISCONNECTED, &priv->flags);
	vnt_reset_command_timer(priv);

	vnt_schedule_command(priv, WLAN_CMD_INIT_MAC80211);

	return 0;

err_nomem:
	usb_put_dev(udev);

	return rc;
}

<<<<<<< HEAD
=======
static void device_free_tx_bufs(struct vnt_private *pDevice)
{
    PUSB_SEND_CONTEXT pTxContext;
    int ii;

    for (ii = 0; ii < pDevice->cbTD; ii++) {

        pTxContext = pDevice->apTD[ii];
	/* deallocate URBs */
        if (pTxContext->pUrb) {
            usb_kill_urb(pTxContext->pUrb);
            usb_free_urb(pTxContext->pUrb);
        }
        kfree(pTxContext);
    }
    return;
}

static void device_free_rx_bufs(struct vnt_private *pDevice)
{
    PRCB pRCB;
    int ii;

    for (ii = 0; ii < pDevice->cbRD; ii++) {

        pRCB = pDevice->apRCB[ii];
	/* deallocate URBs */
        if (pRCB->pUrb) {
            usb_kill_urb(pRCB->pUrb);
            usb_free_urb(pRCB->pUrb);
        }
	/* deallocate skb */
        if (pRCB->skb)
            dev_kfree_skb(pRCB->skb);
    }
    kfree(pDevice->pRCBMem);

    return;
}

static void usb_device_reset(struct vnt_private *pDevice)
{
 int status;
 status = usb_reset_device(pDevice->usb);
	if (status)
            printk("usb_device_reset fail status=%d\n",status);
	return ;
}

static void device_free_int_bufs(struct vnt_private *pDevice)
{
    kfree(pDevice->intBuf.pDataBuf);
    return;
}

static bool device_alloc_bufs(struct vnt_private *pDevice)
{

    PUSB_SEND_CONTEXT pTxContext;
    PRCB pRCB;
    int ii;

    for (ii = 0; ii < pDevice->cbTD; ii++) {

        pTxContext = kmalloc(sizeof(USB_SEND_CONTEXT), GFP_KERNEL);
        if (pTxContext == NULL) {
            DBG_PRT(MSG_LEVEL_ERR,KERN_ERR "%s : allocate tx usb context failed\n", pDevice->dev->name);
            goto free_tx;
        }
        pDevice->apTD[ii] = pTxContext;
	pTxContext->pDevice = (void *) pDevice;
	/* allocate URBs */
        pTxContext->pUrb = usb_alloc_urb(0, GFP_ATOMIC);
        if (pTxContext->pUrb == NULL) {
            DBG_PRT(MSG_LEVEL_ERR,KERN_ERR "alloc tx urb failed\n");
            goto free_tx;
        }
        pTxContext->bBoolInUse = false;
    }

    /* allocate RCB mem */
	pDevice->pRCBMem = kzalloc((sizeof(RCB) * pDevice->cbRD), GFP_KERNEL);
    if (pDevice->pRCBMem == NULL) {
        DBG_PRT(MSG_LEVEL_ERR,KERN_ERR "%s : alloc rx usb context failed\n", pDevice->dev->name);
        goto free_tx;
    }

    pDevice->FirstRecvFreeList = NULL;
    pDevice->LastRecvFreeList = NULL;
    pDevice->FirstRecvMngList = NULL;
    pDevice->LastRecvMngList = NULL;
    pDevice->NumRecvFreeList = 0;
    pRCB = (PRCB) pDevice->pRCBMem;

    for (ii = 0; ii < pDevice->cbRD; ii++) {

        pDevice->apRCB[ii] = pRCB;
	pRCB->pDevice = (void *) pDevice;
	/* allocate URBs */
        pRCB->pUrb = usb_alloc_urb(0, GFP_ATOMIC);

        if (pRCB->pUrb == NULL) {
            DBG_PRT(MSG_LEVEL_ERR,KERN_ERR" Failed to alloc rx urb\n");
            goto free_rx_tx;
        }
        pRCB->skb = dev_alloc_skb((int)pDevice->rx_buf_sz);
        if (pRCB->skb == NULL) {
            DBG_PRT(MSG_LEVEL_ERR,KERN_ERR" Failed to alloc rx skb\n");
            goto free_rx_tx;
        }
        pRCB->skb->dev = pDevice->dev;
        pRCB->bBoolInUse = false;
        EnqueueRCB(pDevice->FirstRecvFreeList, pDevice->LastRecvFreeList, pRCB);
        pDevice->NumRecvFreeList++;
        pRCB++;
    }

	pDevice->pControlURB = usb_alloc_urb(0, GFP_ATOMIC);
	if (pDevice->pControlURB == NULL) {
	    DBG_PRT(MSG_LEVEL_ERR,KERN_ERR"Failed to alloc control urb\n");
	    goto free_rx_tx;
	}

	pDevice->pInterruptURB = usb_alloc_urb(0, GFP_ATOMIC);
	if (pDevice->pInterruptURB == NULL) {
	    DBG_PRT(MSG_LEVEL_ERR,KERN_ERR"Failed to alloc int urb\n");
	    usb_free_urb(pDevice->pControlURB);
	    goto free_rx_tx;
	}

    pDevice->intBuf.pDataBuf = kmalloc(MAX_INTERRUPT_SIZE, GFP_KERNEL);
	if (pDevice->intBuf.pDataBuf == NULL) {
	    DBG_PRT(MSG_LEVEL_ERR,KERN_ERR"Failed to alloc int buf\n");
	    usb_free_urb(pDevice->pControlURB);
	    usb_free_urb(pDevice->pInterruptURB);
	    goto free_rx_tx;
	}

    return true;

free_rx_tx:
    device_free_rx_bufs(pDevice);

free_tx:
    device_free_tx_bufs(pDevice);

	return false;
}

static bool device_init_defrag_cb(struct vnt_private *pDevice)
{
	int i;
	PSDeFragControlBlock pDeF;

    /* Init the fragment ctl entries */
    for (i = 0; i < CB_MAX_RX_FRAG; i++) {
        pDeF = &(pDevice->sRxDFCB[i]);
        if (!device_alloc_frag_buf(pDevice, pDeF)) {
            DBG_PRT(MSG_LEVEL_ERR,KERN_ERR "%s: can not alloc frag bufs\n",
                pDevice->dev->name);
            goto free_frag;
        }
    }
    pDevice->cbDFCB = CB_MAX_RX_FRAG;
    pDevice->cbFreeDFCB = pDevice->cbDFCB;
    return true;

free_frag:
    device_free_frag_bufs(pDevice);
    return false;
}

static void device_free_frag_bufs(struct vnt_private *pDevice)
{
	PSDeFragControlBlock pDeF;
	int i;

    for (i = 0; i < CB_MAX_RX_FRAG; i++) {

        pDeF = &(pDevice->sRxDFCB[i]);

        if (pDeF->skb)
            dev_kfree_skb(pDeF->skb);
    }
}

int device_alloc_frag_buf(struct vnt_private *pDevice,
		PSDeFragControlBlock pDeF)
{

    pDeF->skb = dev_alloc_skb((int)pDevice->rx_buf_sz);
    if (pDeF->skb == NULL)
        return false;
    ASSERT(pDeF->skb);
    pDeF->skb->dev = pDevice->dev;

    return true;
}

static int  device_open(struct net_device *dev)
{
	struct vnt_private *pDevice = netdev_priv(dev);

     pDevice->fWPA_Authened = false;

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO " device_open...\n");

    pDevice->rx_buf_sz = MAX_TOTAL_SIZE_WITH_ALL_HEADERS;

    if (device_alloc_bufs(pDevice) == false) {
        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO " device_alloc_bufs fail... \n");
        return -ENOMEM;
    }

    if (device_init_defrag_cb(pDevice)== false) {
        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO " Initial defragment cb fail \n");
        goto free_rx_tx;
    }

    MP_CLEAR_FLAG(pDevice, fMP_DISCONNECTED);
    MP_CLEAR_FLAG(pDevice, fMP_CONTROL_READS);
    MP_CLEAR_FLAG(pDevice, fMP_CONTROL_WRITES);
    MP_SET_FLAG(pDevice, fMP_POST_READS);
    MP_SET_FLAG(pDevice, fMP_POST_WRITES);

    /* read config file */
    Read_config_file(pDevice);

    if (device_init_registers(pDevice, DEVICE_INIT_COLD) == false) {
        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO " init register fail\n");
        goto free_all;
    }

    device_set_multi(pDevice->dev);

    /* init for key management */
    KeyvInitTable(pDevice,&pDevice->sKey);
	memcpy(pDevice->vnt_mgmt.abyMACAddr,
		pDevice->abyCurrentNetAddr, ETH_ALEN);
    memcpy(pDevice->dev->dev_addr, pDevice->abyCurrentNetAddr, ETH_ALEN);
    pDevice->bStopTx0Pkt = false;
    pDevice->bStopDataPkt = false;
    pDevice->bRoaming = false;
    pDevice->bIsRoaming = false;
    pDevice->bEnableRoaming = false;
    if (pDevice->bDiversityRegCtlON) {
        device_init_diversity_timer(pDevice);
    }

    vMgrObjectInit(pDevice);
    tasklet_init(&pDevice->RxMngWorkItem, (void *)RXvMngWorkItem, (unsigned long)pDevice);
    tasklet_init(&pDevice->ReadWorkItem, (void *)RXvWorkItem, (unsigned long)pDevice);
    tasklet_init(&pDevice->EventWorkItem, (void *)INTvWorkItem, (unsigned long)pDevice);
	add_timer(&pDevice->vnt_mgmt.sTimerSecondCallback);
	pDevice->int_interval = 100;  /* max 100 microframes */
    pDevice->eEncryptionStatus = Ndis802_11EncryptionDisabled;

    pDevice->bIsRxWorkItemQueued = true;
    pDevice->fKillEventPollingThread = false;
    pDevice->bEventAvailable = false;

   pDevice->bWPADEVUp = false;
     pDevice->bwextstep0 = false;
     pDevice->bwextstep1 = false;
     pDevice->bwextstep2 = false;
     pDevice->bwextstep3 = false;
     pDevice->bWPASuppWextEnabled = false;
    pDevice->byReAssocCount = 0;

    RXvWorkItem(pDevice);
    INTvWorkItem(pDevice);

    /* if WEP key already set by iwconfig but device not yet open */
    if ((pDevice->bEncryptionEnable == true) && (pDevice->bTransmitKey == true)) {
         spin_lock_irq(&pDevice->lock);
         KeybSetDefaultKey( pDevice,
                            &(pDevice->sKey),
                            pDevice->byKeyIndex | (1 << 31),
                            pDevice->uKeyLength,
                            NULL,
                            pDevice->abyKey,
                            KEY_CTL_WEP
                          );
         spin_unlock_irq(&pDevice->lock);
         pDevice->eEncryptionStatus = Ndis802_11Encryption1Enabled;
    }

	if (pDevice->vnt_mgmt.eConfigMode == WMAC_CONFIG_AP)
		bScheduleCommand((void *) pDevice, WLAN_CMD_RUN_AP, NULL);
	else
		bScheduleCommand((void *) pDevice, WLAN_CMD_BSSID_SCAN, NULL);

    netif_stop_queue(pDevice->dev);
    pDevice->flags |= DEVICE_FLAGS_OPENED;

	DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "device_open success..\n");
	return 0;

free_all:
    device_free_frag_bufs(pDevice);
free_rx_tx:
    device_free_rx_bufs(pDevice);
    device_free_tx_bufs(pDevice);
    device_free_int_bufs(pDevice);
	usb_kill_urb(pDevice->pControlURB);
	usb_kill_urb(pDevice->pInterruptURB);
    usb_free_urb(pDevice->pControlURB);
    usb_free_urb(pDevice->pInterruptURB);

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "device_open fail.. \n");
    return -ENOMEM;
}

static int device_close(struct net_device *dev)
{
	struct vnt_private *pDevice = netdev_priv(dev);
	struct vnt_manager *pMgmt = &pDevice->vnt_mgmt;
	int uu;

	DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "device_close1\n");
    if (pDevice == NULL)
        return -ENODEV;

    if (pDevice->bLinkPass) {
	bScheduleCommand((void *) pDevice, WLAN_CMD_DISASSOCIATE, NULL);
        mdelay(30);
    }

        memset(pMgmt->abyDesireSSID, 0, WLAN_IEHDR_LEN + WLAN_SSID_MAXLEN + 1);
        pMgmt->bShareKeyAlgorithm = false;
        pDevice->bEncryptionEnable = false;
        pDevice->eEncryptionStatus = Ndis802_11EncryptionDisabled;
	spin_lock_irq(&pDevice->lock);
	for (uu = 0; uu < MAX_KEY_TABLE; uu++)
                MACvDisableKeyEntry(pDevice,uu);
	spin_unlock_irq(&pDevice->lock);

    if ((pDevice->flags & DEVICE_FLAGS_UNPLUG) == false) {
        MACbShutdown(pDevice);
    }
    netif_stop_queue(pDevice->dev);
    MP_SET_FLAG(pDevice, fMP_DISCONNECTED);
    MP_CLEAR_FLAG(pDevice, fMP_POST_WRITES);
    MP_CLEAR_FLAG(pDevice, fMP_POST_READS);
    pDevice->fKillEventPollingThread = true;
    del_timer(&pDevice->sTimerCommand);
    del_timer(&pMgmt->sTimerSecondCallback);

    del_timer(&pDevice->sTimerTxData);

    if (pDevice->bDiversityRegCtlON) {
        del_timer(&pDevice->TimerSQ3Tmax1);
        del_timer(&pDevice->TimerSQ3Tmax2);
        del_timer(&pDevice->TimerSQ3Tmax3);
    }
    tasklet_kill(&pDevice->RxMngWorkItem);
    tasklet_kill(&pDevice->ReadWorkItem);
    tasklet_kill(&pDevice->EventWorkItem);

   pDevice->bRoaming = false;
   pDevice->bIsRoaming = false;
   pDevice->bEnableRoaming = false;
    pDevice->bCmdRunning = false;
    pDevice->bLinkPass = false;
    memset(pMgmt->abyCurrBSSID, 0, 6);
    pMgmt->eCurrState = WMAC_STATE_IDLE;

	pDevice->flags &= ~DEVICE_FLAGS_OPENED;

    device_free_tx_bufs(pDevice);
    device_free_rx_bufs(pDevice);
    device_free_int_bufs(pDevice);
    device_free_frag_bufs(pDevice);

	usb_kill_urb(pDevice->pControlURB);
	usb_kill_urb(pDevice->pInterruptURB);
    usb_free_urb(pDevice->pControlURB);
    usb_free_urb(pDevice->pInterruptURB);

    BSSvClearNodeDBTable(pDevice, 0);

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "device_close2 \n");

    return 0;
}

>>>>>>> p9x
static void vt6656_disconnect(struct usb_interface *intf)
{
	struct vnt_private *priv = usb_get_intfdata(intf);

	if (!priv)
		return;

	if (priv->mac_hw)
		ieee80211_unregister_hw(priv->hw);

	usb_set_intfdata(intf, NULL);
	usb_put_dev(interface_to_usbdev(intf));

	set_bit(DEVICE_FLAGS_UNPLUG, &priv->flags);

	ieee80211_free_hw(priv->hw);
}

#ifdef CONFIG_PM

static int vt6656_suspend(struct usb_interface *intf, pm_message_t message)
{
	return 0;
}

static int vt6656_resume(struct usb_interface *intf)
{
	return 0;
}

#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(usb, vt6656_table);

static struct usb_driver vt6656_driver = {
	.name =		DEVICE_NAME,
	.probe =	vt6656_probe,
	.disconnect =	vt6656_disconnect,
	.id_table =	vt6656_table,
#ifdef CONFIG_PM
	.suspend = vt6656_suspend,
	.resume = vt6656_resume,
#endif /* CONFIG_PM */
};

module_usb_driver(vt6656_driver);
