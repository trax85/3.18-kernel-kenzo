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
 *
 * File: baseband.c
 *
 * Purpose: Implement functions to access baseband
 *
 * Author: Jerry Chen
 *
 * Date: Jun. 5, 2002
 *
 * Functions:
 *	vnt_get_frame_time	- Calculate data frame transmitting time
 *	vnt_get_phy_field	- Calculate PhyLength, PhyService and Phy
 *				  Signal parameter for baseband Tx
 *	vnt_vt3184_init		- VIA VT3184 baseband chip init code
 *
 * Revision History:
 *
 *
 */

#include "mac.h"
#include "baseband.h"
#include "rf.h"
#include "usbpipe.h"

static u8 vnt_vt3184_agc[] = {
	0x00, 0x00, 0x02, 0x02, 0x04, 0x04, 0x06, 0x06,
	0x08, 0x08, 0x0a, 0x0a, 0x0c, 0x0c, 0x0e, 0x0e, /* 0x0f */
	0x10, 0x10, 0x12, 0x12, 0x14, 0x14, 0x16, 0x16,
	0x18, 0x18, 0x1a, 0x1a, 0x1c, 0x1c, 0x1e, 0x1e, /* 0x1f */
	0x20, 0x20, 0x22, 0x22, 0x24, 0x24, 0x26, 0x26,
	0x28, 0x28, 0x2a, 0x2a, 0x2c, 0x2c, 0x2e, 0x2e, /* 0x2f */
	0x30, 0x30, 0x32, 0x32, 0x34, 0x34, 0x36, 0x36,
	0x38, 0x38, 0x3a, 0x3a, 0x3c, 0x3c, 0x3e, 0x3e  /* 0x3f */
};

static u8 vnt_vt3184_al2230[] = {
	0x31, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
	0x70, 0x45, 0x2a, 0x76, 0x00, 0x00, 0x80, 0x00, /* 0x0f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x8e, 0x0a, 0x00, 0x00, 0x00, /* 0x1f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x0c, /* 0x2f */
	0x26, 0x5b, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa,
	0xff, 0xff, 0x79, 0x00, 0x00, 0x0b, 0x48, 0x04, /* 0x3f */
	0x00, 0x08, 0x00, 0x08, 0x08, 0x14, 0x05, 0x09,
	0x00, 0x00, 0x00, 0x00, 0x09, 0x73, 0x00, 0xc5, /* 0x4f */
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x5f */
	0xe4, 0x80, 0x00, 0x00, 0x00, 0x00, 0x98, 0x0a,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, /* 0x6f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x7f */
	0x8c, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x00, 0x1f, 0xb7, 0x88, 0x47, 0xaa, 0x00, /* 0x8f */
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xeb,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* 0x9f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
	0x18, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00, 0x18, /* 0xaf */
	0x38, 0x30, 0x00, 0x00, 0xff, 0x0f, 0xe4, 0xe2,
	0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, /* 0xbf */
	0x18, 0x20, 0x07, 0x18, 0xff, 0xff, 0x0e, 0x0a,
	0x0e, 0x00, 0x82, 0xa7, 0x3c, 0x10, 0x30, 0x05, /* 0xcf */
	0x40, 0x12, 0x00, 0x00, 0x10, 0x28, 0x80, 0x2a,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xdf */
	0x00, 0xf3, 0x00, 0x00, 0x00, 0x10, 0x00, 0x12,
	0x00, 0xf4, 0x00, 0xff, 0x79, 0x20, 0x30, 0x05, /* 0xef */
	0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 0xff */
};

/* {{RobertYu:20060515, new BB setting for VT3226D0 */
static u8 vnt_vt3184_vt3226d0[] = {
	0x31, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
	0x70, 0x45, 0x2a, 0x76, 0x00, 0x00, 0x80, 0x00, /* 0x0f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x8e, 0x0a, 0x00, 0x00, 0x00, /* 0x1f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x0c, /* 0x2f */
	0x26, 0x5b, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa,
	0xff, 0xff, 0x79, 0x00, 0x00, 0x0b, 0x48, 0x04, /* 0x3f */
	0x00, 0x08, 0x00, 0x08, 0x08, 0x14, 0x05, 0x09,
	0x00, 0x00, 0x00, 0x00, 0x09, 0x73, 0x00, 0xc5, /* 0x4f */
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x5f */
	0xe4, 0x80, 0x00, 0x00, 0x00, 0x00, 0x98, 0x0a,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, /* 0x6f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x7f */
	0x8c, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x00, 0x1f, 0xb7, 0x88, 0x47, 0xaa, 0x00, /* 0x8f */
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xeb,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* 0x9f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
	0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, /* 0xaf */
	0x38, 0x30, 0x00, 0x00, 0xff, 0x0f, 0xe4, 0xe2,
	0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, /* 0xbf */
	0x18, 0x20, 0x07, 0x18, 0xff, 0xff, 0x10, 0x0a,
	0x0e, 0x00, 0x84, 0xa7, 0x3c, 0x10, 0x24, 0x05, /* 0xcf */
	0x40, 0x12, 0x00, 0x00, 0x10, 0x28, 0x80, 0x2a,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xdf */
	0x00, 0xf3, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
	0x00, 0xf4, 0x00, 0xff, 0x79, 0x20, 0x30, 0x08, /* 0xef */
	0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 0xff */
};

static const u16 vnt_frame_time[MAX_RATE] = {
	10, 20, 55, 110, 24, 36, 48, 72, 96, 144, 192, 216
};

/*
 * Description: Calculate data frame transmitting time
 *
 * Parameters:
 *  In:
 *	preamble_type	- Preamble Type
 *	pkt_type	- PK_TYPE_11A, PK_TYPE_11B, PK_TYPE_11GB, PK_TYPE_11GA
 *	frame_length	- Baseband Type
 *	tx_rate		- Tx Rate
 *  Out:
 *
 * Return Value: FrameTime
 *
 */
unsigned int vnt_get_frame_time(u8 preamble_type, u8 pkt_type,
	unsigned int frame_length, u16 tx_rate)
{
	unsigned int frame_time;
	unsigned int preamble;
	unsigned int tmp;
	unsigned int rate = 0;

	if (tx_rate > RATE_54M)
		return 0;

	rate = (unsigned int)vnt_frame_time[tx_rate];

	if (tx_rate <= 3) {
		if (preamble_type == 1)
			preamble = 96;
		else
			preamble = 192;

		frame_time = (frame_length * 80) / rate;
		tmp = (frame_time * rate) / 80;

		if (frame_length != tmp)
			frame_time++;

		return preamble + frame_time;
	}
	frame_time = (frame_length * 8 + 22) / rate;
	tmp = ((frame_time * rate) - 22) / 8;

	if (frame_length != tmp)
		frame_time++;

	frame_time = frame_time * 4;

	if (pkt_type != PK_TYPE_11A)
		frame_time += 6;
	return 20 + frame_time;
}

/*
 * Description: Calculate Length, Service, and Signal fields of Phy for Tx
 *
 * Parameters:
 *  In:
 *      priv         - Device Structure
 *      frame_length   - Tx Frame Length
 *      tx_rate           - Tx Rate
 *  Out:
 *	struct vnt_phy_field *phy
 *		- pointer to Phy Length field
 *		- pointer to Phy Service field
 *		- pointer to Phy Signal field
 *
 * Return Value: none
 *
 */
void vnt_get_phy_field(struct vnt_private *priv, u32 frame_length,
	u16 tx_rate, u8 pkt_type, struct vnt_phy_field *phy)
{
	u32 bit_count;
	u32 count = 0;
	u32 tmp;
	int ext_bit;
	u8 preamble_type = priv->preamble_type;

	bit_count = frame_length * 8;
	ext_bit = false;

	switch (tx_rate) {
	case RATE_1M:
		count = bit_count;

		phy->signal = 0x00;

		break;
	case RATE_2M:
		count = bit_count / 2;

		if (preamble_type == 1)
			phy->signal = 0x09;
		else
			phy->signal = 0x01;

		break;
	case RATE_5M:
		count = (bit_count * 10) / 55;
		tmp = (count * 55) / 10;

		if (tmp != bit_count)
			count++;

		if (preamble_type == 1)
			phy->signal = 0x0a;
		else
			phy->signal = 0x02;

		break;
	case RATE_11M:
		count = bit_count / 11;
		tmp = count * 11;

		if (tmp != bit_count) {
			count++;

			if ((bit_count - tmp) <= 3)
				ext_bit = true;
		}

		if (preamble_type == 1)
			phy->signal = 0x0b;
		else
			phy->signal = 0x03;

		break;
	case RATE_6M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9b;
		else
			phy->signal = 0x8b;

		break;
	case RATE_9M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9f;
		else
			phy->signal = 0x8f;

		break;
	case RATE_12M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9a;
		else
			phy->signal = 0x8a;

		break;
	case RATE_18M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9e;
		else
			phy->signal = 0x8e;

		break;
	case RATE_24M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x99;
		else
			phy->signal = 0x89;

		break;
	case RATE_36M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9d;
		else
			phy->signal = 0x8d;

		break;
	case RATE_48M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x98;
		else
			phy->signal = 0x88;

		break;
	case RATE_54M:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9c;
		else
			phy->signal = 0x8c;
		break;
	default:
		if (pkt_type == PK_TYPE_11A)
			phy->signal = 0x9c;
		else
			phy->signal = 0x8c;
		break;
	}

	if (pkt_type == PK_TYPE_11B) {
		phy->service = 0x00;
		if (ext_bit)
			phy->service |= 0x80;
		phy->len = cpu_to_le16((u16)count);
	} else {
		phy->service = 0x00;
		phy->len = cpu_to_le16((u16)frame_length);
	}
}

/*
 * Description: Set Antenna mode
 *
 * Parameters:
 *  In:
 *	priv		- Device Structure
 *	antenna_mode	- Antenna Mode
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */
void vnt_set_antenna_mode(struct vnt_private *priv, u8 antenna_mode)
{
	switch (antenna_mode) {
	case ANT_TXA:
	case ANT_TXB:
		break;
	case ANT_RXA:
		priv->bb_rx_conf &= 0xFC;
		break;
	case ANT_RXB:
		priv->bb_rx_conf &= 0xFE;
		priv->bb_rx_conf |= 0x02;
		break;
	}

	vnt_control_out(priv, MESSAGE_TYPE_SET_ANTMD,
		(u16)antenna_mode, 0, 0, NULL);
}

/*
 * Description: Set Antenna mode
 *
 * Parameters:
 *  In:
 *      pDevice          - Device Structure
 *      byAntennaMode    - Antenna Mode
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */

int vnt_vt3184_init(struct vnt_private *priv)
{
<<<<<<< HEAD
	int status;
	u16 length;
	u8 *addr;
	u8 *agc;
	u16 length_agc;
	u8 array[256];
=======
	int ntStatus;
    u16                    wLength;
    u8 *                   pbyAddr;
    u8 *                   pbyAgc;
    u16                    wLengthAgc;
    u8                    abyArray[256];
>>>>>>> p9x
	u8 data;

	status = vnt_control_in(priv, MESSAGE_TYPE_READ, 0,
		MESSAGE_REQUEST_EEPROM, EEP_MAX_CONTEXT_SIZE,
						priv->eeprom);
	if (status != STATUS_SUCCESS)
		return false;

	priv->rf_type = priv->eeprom[EEP_OFS_RFTYPE];

	dev_dbg(&priv->usb->dev, "RF Type %d\n", priv->rf_type);

	if ((priv->rf_type == RF_AL2230) ||
				(priv->rf_type == RF_AL2230S)) {
		priv->bb_rx_conf = vnt_vt3184_al2230[10];
		length = sizeof(vnt_vt3184_al2230);
		addr = vnt_vt3184_al2230;
		agc = vnt_vt3184_agc;
		length_agc = sizeof(vnt_vt3184_agc);

		priv->bb_vga[0] = 0x1C;
		priv->bb_vga[1] = 0x10;
		priv->bb_vga[2] = 0x0;
		priv->bb_vga[3] = 0x0;

	} else if (priv->rf_type == RF_AIROHA7230) {
		priv->bb_rx_conf = vnt_vt3184_al2230[10];
		length = sizeof(vnt_vt3184_al2230);
		addr = vnt_vt3184_al2230;
		agc = vnt_vt3184_agc;
		length_agc = sizeof(vnt_vt3184_agc);

		addr[0xd7] = 0x06;

		priv->bb_vga[0] = 0x1c;
		priv->bb_vga[1] = 0x10;
		priv->bb_vga[2] = 0x0;
		priv->bb_vga[3] = 0x0;

	} else if ((priv->rf_type == RF_VT3226) ||
			(priv->rf_type == RF_VT3226D0)) {
		priv->bb_rx_conf = vnt_vt3184_vt3226d0[10];
		length = sizeof(vnt_vt3184_vt3226d0);
		addr = vnt_vt3184_vt3226d0;
		agc = vnt_vt3184_agc;
		length_agc = sizeof(vnt_vt3184_agc);

		priv->bb_vga[0] = 0x20;
		priv->bb_vga[1] = 0x10;
		priv->bb_vga[2] = 0x0;
		priv->bb_vga[3] = 0x0;

		/* Fix VT3226 DFC system timing issue */
		vnt_mac_reg_bits_on(priv, MAC_REG_SOFTPWRCTL2,
				    SOFTPWRCTL_RFLEOPT);
	} else if (priv->rf_type == RF_VT3342A0) {
		priv->bb_rx_conf = vnt_vt3184_vt3226d0[10];
		length = sizeof(vnt_vt3184_vt3226d0);
		addr = vnt_vt3184_vt3226d0;
		agc = vnt_vt3184_agc;
		length_agc = sizeof(vnt_vt3184_agc);

		priv->bb_vga[0] = 0x20;
		priv->bb_vga[1] = 0x10;
		priv->bb_vga[2] = 0x0;
		priv->bb_vga[3] = 0x0;

		/* Fix VT3226 DFC system timing issue */
		vnt_mac_reg_bits_on(priv, MAC_REG_SOFTPWRCTL2,
				    SOFTPWRCTL_RFLEOPT);
	} else {
		return true;
	}

	memcpy(array, addr, length);

	vnt_control_out(priv, MESSAGE_TYPE_WRITE, 0,
		MESSAGE_REQUEST_BBREG, length, array);

<<<<<<< HEAD
	memcpy(array, agc, length_agc);

	vnt_control_out(priv, MESSAGE_TYPE_WRITE, 0,
		MESSAGE_REQUEST_BBAGC, length_agc, array);

	if ((priv->rf_type == RF_VT3226) ||
		(priv->rf_type == RF_VT3342A0)) {
		vnt_control_out_u8(priv, MESSAGE_REQUEST_MACREG,
						MAC_REG_ITRTMSET, 0x23);
		vnt_mac_reg_bits_on(priv, MAC_REG_PAPEDELAY, 0x01);
	} else if (priv->rf_type == RF_VT3226D0) {
		vnt_control_out_u8(priv, MESSAGE_REQUEST_MACREG,
						MAC_REG_ITRTMSET, 0x11);
		vnt_mac_reg_bits_on(priv, MAC_REG_PAPEDELAY, 0x01);
	}

	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x04, 0x7f);
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0d, 0x01);

	vnt_rf_table_download(priv);

	/* Fix for TX USB resets from vendors driver */
	vnt_control_in(priv, MESSAGE_TYPE_READ, USB_REG4,
=======
    RFbRFTableDownload(pDevice);

	/* Fix for TX USB resets from vendors driver */
	CONTROLnsRequestIn(pDevice, MESSAGE_TYPE_READ, USB_REG4,
>>>>>>> p9x
		MESSAGE_REQUEST_MEM, sizeof(data), &data);

	data |= 0x2;

<<<<<<< HEAD
	vnt_control_out(priv, MESSAGE_TYPE_WRITE, USB_REG4,
		MESSAGE_REQUEST_MEM, sizeof(data), &data);

	return true;
=======
	CONTROLnsRequestOut(pDevice, MESSAGE_TYPE_WRITE, USB_REG4,
		MESSAGE_REQUEST_MEM, sizeof(data), &data);

    return true;//ntStatus;
>>>>>>> p9x
}

/*
 * Description: Set ShortSlotTime mode
 *
 * Parameters:
 *  In:
 *	priv	- Device Structure
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */
void vnt_set_short_slot_time(struct vnt_private *priv)
{
	u8 bb_vga = 0;

	if (priv->short_slot_time)
		priv->bb_rx_conf &= 0xdf;
	else
		priv->bb_rx_conf |= 0x20;

	vnt_control_in_u8(priv, MESSAGE_REQUEST_BBREG, 0xe7, &bb_vga);

	if (bb_vga == priv->bb_vga[0])
		priv->bb_rx_conf |= 0x20;

	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0a, priv->bb_rx_conf);
}

void vnt_set_vga_gain_offset(struct vnt_private *priv, u8 data)
{

	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0xE7, data);

	/* patch for 3253B0 Baseband with Cardbus module */
	if (priv->short_slot_time)
		priv->bb_rx_conf &= 0xdf; /* 1101 1111 */
	else
		priv->bb_rx_conf |= 0x20; /* 0010 0000 */

	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0a, priv->bb_rx_conf);
}

/*
 * Description: vnt_set_deep_sleep
 *
 * Parameters:
 *  In:
 *	priv	- Device Structure
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */
void vnt_set_deep_sleep(struct vnt_private *priv)
{
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0c, 0x17);/* CR12 */
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0d, 0xB9);/* CR13 */
}

void vnt_exit_deep_sleep(struct vnt_private *priv)
{
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0c, 0x00);/* CR12 */
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0x0d, 0x01);/* CR13 */
}

void vnt_update_pre_ed_threshold(struct vnt_private *priv, int scanning)
{
	u8 cr_201 = 0x0, cr_206 = 0x0;
	u8 ed_inx = priv->bb_pre_ed_index;

	switch (priv->rf_type) {
	case RF_AL2230:
	case RF_AL2230S:
	case RF_AIROHA7230:
		if (scanning) { /* Max sensitivity */
			ed_inx = 0;
			cr_206 = 0x30;
			break;
		}

<<<<<<< HEAD
		if (priv->bb_pre_ed_rssi <= 45) {
			ed_inx = 20;
			cr_201 = 0xff;
		} else if (priv->bb_pre_ed_rssi <= 46) {
			ed_inx = 19;
			cr_201 = 0x1a;
		} else if (priv->bb_pre_ed_rssi <= 47) {
			ed_inx = 18;
			cr_201 = 0x15;
		} else if (priv->bb_pre_ed_rssi <= 49) {
			ed_inx = 17;
			cr_201 = 0xe;
		} else if (priv->bb_pre_ed_rssi <= 51) {
			ed_inx = 16;
			cr_201 = 0x9;
		} else if (priv->bb_pre_ed_rssi <= 53) {
			ed_inx = 15;
			cr_201 = 0x6;
		} else if (priv->bb_pre_ed_rssi <= 55) {
			ed_inx = 14;
			cr_201 = 0x3;
		} else if (priv->bb_pre_ed_rssi <= 56) {
			ed_inx = 13;
			cr_201 = 0x2;
			cr_206 = 0xa0;
		} else if (priv->bb_pre_ed_rssi <= 57) {
			ed_inx = 12;
			cr_201 = 0x2;
			cr_206 = 0x20;
		} else if (priv->bb_pre_ed_rssi <= 58) {
			ed_inx = 11;
			cr_201 = 0x1;
			cr_206 = 0xa0;
		} else if (priv->bb_pre_ed_rssi <= 59) {
			ed_inx = 10;
			cr_201 = 0x1;
			cr_206 = 0x54;
		} else if (priv->bb_pre_ed_rssi <= 60) {
			ed_inx = 9;
			cr_201 = 0x1;
			cr_206 = 0x18;
		} else if (priv->bb_pre_ed_rssi <= 61) {
			ed_inx = 8;
			cr_206 = 0xe3;
		} else if (priv->bb_pre_ed_rssi <= 62) {
			ed_inx = 7;
			cr_206 = 0xb9;
		} else if (priv->bb_pre_ed_rssi <= 63) {
			ed_inx = 6;
			cr_206 = 0x93;
		} else if (priv->bb_pre_ed_rssi <= 64) {
			ed_inx = 5;
			cr_206 = 0x79;
		} else if (priv->bb_pre_ed_rssi <= 65) {
			ed_inx = 4;
			cr_206 = 0x62;
		} else if (priv->bb_pre_ed_rssi <= 66) {
			ed_inx = 3;
			cr_206 = 0x51;
		} else if (priv->bb_pre_ed_rssi <= 67) {
			ed_inx = 2;
			cr_206 = 0x43;
		} else if (priv->bb_pre_ed_rssi <= 68) {
			ed_inx = 1;
			cr_206 = 0x36;
		} else {
			ed_inx = 0;
			cr_206 = 0x30;
		}
		break;
=======
	for (ii = RATE_48M; ii >= RATE_6M; ii--)
		if (pDevice->aulPktNum[ii] > ulMaxPacket) {
            ulMaxPacket = pDevice->aulPktNum[ii];
            ulSQ3 = pDevice->aulSQ3Val[ii] / pDevice->aulPktNum[ii];
        }

    return ulSQ3;
}

static unsigned long s_ulGetRatio(struct vnt_private *pDevice)
{
	int ii, jj;
	unsigned long ulRatio = 0;
	unsigned long ulMaxPacket;
	unsigned long ulPacketNum;

    //This is a thousand-ratio
    ulMaxPacket = pDevice->aulPktNum[RATE_54M];
    if ( pDevice->aulPktNum[RATE_54M] != 0 ) {
        ulPacketNum = pDevice->aulPktNum[RATE_54M];
        ulRatio = (ulPacketNum * 1000 / pDevice->uDiversityCnt);
        ulRatio += TOP_RATE_54M;
    }
	for (ii = RATE_48M; ii >= RATE_1M; ii--)
        if ( pDevice->aulPktNum[ii] > ulMaxPacket ) {
            ulPacketNum = 0;
            for ( jj=RATE_54M;jj>=ii;jj--)
                ulPacketNum += pDevice->aulPktNum[jj];
            ulRatio = (ulPacketNum * 1000 / pDevice->uDiversityCnt);
            ulRatio += TOP_RATE_48M;
            ulMaxPacket = pDevice->aulPktNum[ii];
        }

    return ulRatio;
}

static void s_vClearSQ3Value(struct vnt_private *pDevice)
{
    int ii;
    pDevice->uDiversityCnt = 0;

    for ( ii=RATE_1M;ii<MAX_RATE;ii++) {
        pDevice->aulPktNum[ii] = 0;
        pDevice->aulSQ3Val[ii] = 0;
    }
}

/*
 * Description: Antenna Diversity
 *
 * Parameters:
 *  In:
 *      pDevice          - Device Structure
 *      byRSR            - RSR from received packet
 *      bySQ3            - SQ3 value from received packet
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */

void BBvAntennaDiversity(struct vnt_private *pDevice,
	u8 byRxRate, u8 bySQ3)
{

    pDevice->uDiversityCnt++;
    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"pDevice->uDiversityCnt = %d\n", (int)pDevice->uDiversityCnt);

    if (byRxRate == 2) {
        pDevice->aulPktNum[RATE_1M]++;
    }
    else if (byRxRate==4) {
        pDevice->aulPktNum[RATE_2M]++;
    }
    else if (byRxRate==11) {
        pDevice->aulPktNum[RATE_5M]++;
    }
    else if (byRxRate==22) {
        pDevice->aulPktNum[RATE_11M]++;
    }
    else if(byRxRate==12){
        pDevice->aulPktNum[RATE_6M]++;
        pDevice->aulSQ3Val[RATE_6M] += bySQ3;
    }
    else if(byRxRate==18){
        pDevice->aulPktNum[RATE_9M]++;
        pDevice->aulSQ3Val[RATE_9M] += bySQ3;
    }
    else if(byRxRate==24){
        pDevice->aulPktNum[RATE_12M]++;
        pDevice->aulSQ3Val[RATE_12M] += bySQ3;
    }
    else if(byRxRate==36){
        pDevice->aulPktNum[RATE_18M]++;
        pDevice->aulSQ3Val[RATE_18M] += bySQ3;
    }
    else if(byRxRate==48){
        pDevice->aulPktNum[RATE_24M]++;
        pDevice->aulSQ3Val[RATE_24M] += bySQ3;
    }
    else if(byRxRate==72){
        pDevice->aulPktNum[RATE_36M]++;
        pDevice->aulSQ3Val[RATE_36M] += bySQ3;
    }
    else if(byRxRate==96){
        pDevice->aulPktNum[RATE_48M]++;
        pDevice->aulSQ3Val[RATE_48M] += bySQ3;
    }
    else if(byRxRate==108){
        pDevice->aulPktNum[RATE_54M]++;
        pDevice->aulSQ3Val[RATE_54M] += bySQ3;
    }

    if (pDevice->byAntennaState == 0) {

        if (pDevice->uDiversityCnt > pDevice->ulDiversityNValue) {
            DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"ulDiversityNValue=[%d],54M-[%d]\n",(int)pDevice->ulDiversityNValue, (int)pDevice->aulPktNum[RATE_54M]);

            pDevice->ulSQ3_State0 = s_ulGetLowSQ3(pDevice);
            pDevice->ulRatio_State0 = s_ulGetRatio(pDevice);
            DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"SQ3_State0, SQ3= [%08x] rate = [%08x]\n",(int)pDevice->ulSQ3_State0,(int)pDevice->ulRatio_State0);

            if ( ((pDevice->aulPktNum[RATE_54M] < pDevice->ulDiversityNValue/2) &&
                  (pDevice->ulSQ3_State0 > pDevice->ulSQ3TH) ) ||
                 (pDevice->ulSQ3_State0 == 0 ) )  {

                if ( pDevice->byTMax == 0 )
                    return;

		bScheduleCommand((void *) pDevice,
				 WLAN_CMD_CHANGE_ANTENNA,
				 NULL);

                pDevice->byAntennaState = 1;

                del_timer(&pDevice->TimerSQ3Tmax3);
                del_timer(&pDevice->TimerSQ3Tmax2);
                pDevice->TimerSQ3Tmax1.expires =  RUN_AT(pDevice->byTMax * HZ);
                add_timer(&pDevice->TimerSQ3Tmax1);

            } else {
                pDevice->TimerSQ3Tmax3.expires =  RUN_AT(pDevice->byTMax3 * HZ);
                add_timer(&pDevice->TimerSQ3Tmax3);
            }
            s_vClearSQ3Value(pDevice);

        }
    } else { //byAntennaState == 1

        if (pDevice->uDiversityCnt > pDevice->ulDiversityMValue) {

            del_timer(&pDevice->TimerSQ3Tmax1);
            pDevice->ulSQ3_State1 = s_ulGetLowSQ3(pDevice);
            pDevice->ulRatio_State1 = s_ulGetRatio(pDevice);
            DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"SQ3_State1, rate0 = %08x,rate1 = %08x\n",(int)pDevice->ulRatio_State0,(int)pDevice->ulRatio_State1);

            if ( ((pDevice->ulSQ3_State1 == 0) && (pDevice->ulSQ3_State0 != 0)) ||
                 ((pDevice->ulSQ3_State1 == 0) && (pDevice->ulSQ3_State0 == 0) && (pDevice->ulRatio_State1 < pDevice->ulRatio_State0)) ||
                 ((pDevice->ulSQ3_State1 != 0) && (pDevice->ulSQ3_State0 != 0) && (pDevice->ulSQ3_State0 < pDevice->ulSQ3_State1))
               ) {

		bScheduleCommand((void *) pDevice,
				 WLAN_CMD_CHANGE_ANTENNA,
				 NULL);

                pDevice->TimerSQ3Tmax3.expires =  RUN_AT(pDevice->byTMax3 * HZ);
                pDevice->TimerSQ3Tmax2.expires =  RUN_AT(pDevice->byTMax2 * HZ);
                add_timer(&pDevice->TimerSQ3Tmax3);
                add_timer(&pDevice->TimerSQ3Tmax2);

            }
            pDevice->byAntennaState = 0;
            s_vClearSQ3Value(pDevice);
        }
    } //byAntennaState
}

/*+
 *
 * Description:
 *  Timer for SQ3 antenna diversity
 *
 * Parameters:
 *  In:
 *      pvSysSpec1
 *      hDeviceContext - Pointer to the adapter
 *      pvSysSpec2
 *      pvSysSpec3
 *  Out:
 *      none
 *
 * Return Value: none
 *
-*/

void TimerSQ3CallBack(struct vnt_private *pDevice)
{

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"TimerSQ3CallBack...");
    spin_lock_irq(&pDevice->lock);

    bScheduleCommand((void *) pDevice, WLAN_CMD_CHANGE_ANTENNA, NULL);
    pDevice->byAntennaState = 0;
    s_vClearSQ3Value(pDevice);
    pDevice->TimerSQ3Tmax3.expires =  RUN_AT(pDevice->byTMax3 * HZ);
    pDevice->TimerSQ3Tmax2.expires =  RUN_AT(pDevice->byTMax2 * HZ);
    add_timer(&pDevice->TimerSQ3Tmax3);
    add_timer(&pDevice->TimerSQ3Tmax2);

    spin_unlock_irq(&pDevice->lock);
}

/*+
 *
 * Description:
 *  Timer for SQ3 antenna diversity
 *
 * Parameters:
 *  In:
 *      pvSysSpec1
 *      hDeviceContext - Pointer to the adapter
 *      pvSysSpec2
 *      pvSysSpec3
 *  Out:
 *      none
 *
 * Return Value: none
 *
-*/

void TimerSQ3Tmax3CallBack(struct vnt_private *pDevice)
{

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"TimerSQ3Tmax3CallBack...");
    spin_lock_irq(&pDevice->lock);

    pDevice->ulRatio_State0 = s_ulGetRatio(pDevice);
    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"SQ3_State0 = [%08x]\n",(int)pDevice->ulRatio_State0);

    s_vClearSQ3Value(pDevice);
    if ( pDevice->byTMax == 0 ) {
        pDevice->TimerSQ3Tmax3.expires =  RUN_AT(pDevice->byTMax3 * HZ);
        add_timer(&pDevice->TimerSQ3Tmax3);
        spin_unlock_irq(&pDevice->lock);
        return;
    }

    bScheduleCommand((void *) pDevice, WLAN_CMD_CHANGE_ANTENNA, NULL);
    pDevice->byAntennaState = 1;
    del_timer(&pDevice->TimerSQ3Tmax3);
    del_timer(&pDevice->TimerSQ3Tmax2);
    pDevice->TimerSQ3Tmax1.expires =  RUN_AT(pDevice->byTMax * HZ);
    add_timer(&pDevice->TimerSQ3Tmax1);

    spin_unlock_irq(&pDevice->lock);
}

void BBvUpdatePreEDThreshold(struct vnt_private *pDevice, int bScanning)
{

    switch(pDevice->byRFType)
    {
        case RF_AL2230:
        case RF_AL2230S:
        case RF_AIROHA7230:
            //RobertYu:20060627, update new table

            if( bScanning )
            {   // need Max sensitivity //RSSI -69, -70,....
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -69, -70, -71,...\n");
                break;
            }

            if(pDevice->byBBPreEDRSSI <= 45) { // RSSI 0, -1,-2,....-45
                if(pDevice->byBBPreEDIndex == 20) break;
                pDevice->byBBPreEDIndex = 20;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0xFF); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI 0, -1,-2,..-45\n");
            } else if(pDevice->byBBPreEDRSSI <= 46)  { //RSSI -46
                if(pDevice->byBBPreEDIndex == 19) break;
                pDevice->byBBPreEDIndex = 19;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x1A); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -46\n");
            } else if(pDevice->byBBPreEDRSSI <= 47)  { //RSSI -47
                if(pDevice->byBBPreEDIndex == 18) break;
                pDevice->byBBPreEDIndex = 18;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x15); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -47\n");
            } else if(pDevice->byBBPreEDRSSI <= 49)  { //RSSI -48, -49
                if(pDevice->byBBPreEDIndex == 17) break;
                pDevice->byBBPreEDIndex = 17;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x0E); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -48,-49\n");
            } else if(pDevice->byBBPreEDRSSI <= 51)  { //RSSI -50, -51
                if(pDevice->byBBPreEDIndex == 16) break;
                pDevice->byBBPreEDIndex = 16;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x09); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -50,-51\n");
            } else if(pDevice->byBBPreEDRSSI <= 53)  { //RSSI -52, -53
                if(pDevice->byBBPreEDIndex == 15) break;
                pDevice->byBBPreEDIndex = 15;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x06); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -52,-53\n");
            } else if(pDevice->byBBPreEDRSSI <= 55)  { //RSSI -54, -55
                if(pDevice->byBBPreEDIndex == 14) break;
                pDevice->byBBPreEDIndex = 14;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x03); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -54,-55\n");
            } else if(pDevice->byBBPreEDRSSI <= 56)  { //RSSI -56
                if(pDevice->byBBPreEDIndex == 13) break;
                pDevice->byBBPreEDIndex = 13;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xA0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -56\n");
            } else if(pDevice->byBBPreEDRSSI <= 57)  { //RSSI -57
                if(pDevice->byBBPreEDIndex == 12) break;
                pDevice->byBBPreEDIndex = 12;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x20); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -57\n");
            } else if(pDevice->byBBPreEDRSSI <= 58)  { //RSSI -58
                if(pDevice->byBBPreEDIndex == 11) break;
                pDevice->byBBPreEDIndex = 11;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xA0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -58\n");
            } else if(pDevice->byBBPreEDRSSI <= 59)  { //RSSI -59
                if(pDevice->byBBPreEDIndex == 10) break;
                pDevice->byBBPreEDIndex = 10;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x54); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -59\n");
            } else if(pDevice->byBBPreEDRSSI <= 60)  { //RSSI -60
                if(pDevice->byBBPreEDIndex == 9) break;
                pDevice->byBBPreEDIndex = 9;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x18); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -60\n");
            } else if(pDevice->byBBPreEDRSSI <= 61)  { //RSSI -61
                if(pDevice->byBBPreEDIndex == 8) break;
                pDevice->byBBPreEDIndex = 8;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xE3); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -61\n");
            } else if(pDevice->byBBPreEDRSSI <= 62)  { //RSSI -62
                if(pDevice->byBBPreEDIndex == 7) break;
                pDevice->byBBPreEDIndex = 7;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xB9); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -62\n");
            } else if(pDevice->byBBPreEDRSSI <= 63)  { //RSSI -63
                if(pDevice->byBBPreEDIndex == 6) break;
                pDevice->byBBPreEDIndex = 6;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x93); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -63\n");
            } else if(pDevice->byBBPreEDRSSI <= 64)  { //RSSI -64
                if(pDevice->byBBPreEDIndex == 5) break;
                pDevice->byBBPreEDIndex = 5;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x79); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -64\n");
            } else if(pDevice->byBBPreEDRSSI <= 65)  { //RSSI -65
                if(pDevice->byBBPreEDIndex == 4) break;
                pDevice->byBBPreEDIndex = 4;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x62); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -65\n");
            } else if(pDevice->byBBPreEDRSSI <= 66)  { //RSSI -66
                if(pDevice->byBBPreEDIndex == 3) break;
                pDevice->byBBPreEDIndex = 3;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x51); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -66\n");
            } else if(pDevice->byBBPreEDRSSI <= 67)  { //RSSI -67
                if(pDevice->byBBPreEDIndex == 2) break;
                pDevice->byBBPreEDIndex = 2;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x43); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -67\n");
            } else if(pDevice->byBBPreEDRSSI <= 68)  { //RSSI -68
                if(pDevice->byBBPreEDIndex == 1) break;
                pDevice->byBBPreEDIndex = 1;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x36); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -68\n");
            } else { //RSSI -69, -70,....
                if(pDevice->byBBPreEDIndex == 0) break;
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -69, -70,...\n");
            }
            break;

        case RF_VT3226:
        case RF_VT3226D0:
            //RobertYu:20060627, update new table

            if( bScanning )
            {   // need Max sensitivity  //RSSI -69, -70, ...
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x24); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -69, -70,..\n");
                break;
            }

            if(pDevice->byBBPreEDRSSI <= 41) { // RSSI 0, -1,-2,....-41
                if(pDevice->byBBPreEDIndex == 22) break;
                pDevice->byBBPreEDIndex = 22;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0xFF); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI 0, -1,-2,..-41\n");
            } else if(pDevice->byBBPreEDRSSI <= 42)  { //RSSI -42
                if(pDevice->byBBPreEDIndex == 21) break;
                pDevice->byBBPreEDIndex = 21;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x36); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -42\n");
            } else if(pDevice->byBBPreEDRSSI <= 43)  { //RSSI -43
                if(pDevice->byBBPreEDIndex == 20) break;
                pDevice->byBBPreEDIndex = 20;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x26); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -43\n");
            } else if(pDevice->byBBPreEDRSSI <= 45)  { //RSSI -44, -45
                if(pDevice->byBBPreEDIndex == 19) break;
                pDevice->byBBPreEDIndex = 19;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x18); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -44,-45\n");
            } else if(pDevice->byBBPreEDRSSI <= 47)  { //RSSI -46, -47
                if(pDevice->byBBPreEDIndex == 18) break;
                pDevice->byBBPreEDIndex = 18;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x11); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -46,-47\n");
            } else if(pDevice->byBBPreEDRSSI <= 49)  { //RSSI -48, -49
                if(pDevice->byBBPreEDIndex == 17) break;
                pDevice->byBBPreEDIndex = 17;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x0a); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -48,-49\n");
            } else if(pDevice->byBBPreEDRSSI <= 51)  { //RSSI -50, -51
                if(pDevice->byBBPreEDIndex == 16) break;
                pDevice->byBBPreEDIndex = 16;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x07); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -50,-51\n");
            } else if(pDevice->byBBPreEDRSSI <= 53)  { //RSSI -52, -53
                if(pDevice->byBBPreEDIndex == 15) break;
                pDevice->byBBPreEDIndex = 15;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x04); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -52,-53\n");
            } else if(pDevice->byBBPreEDRSSI <= 55)  { //RSSI -54, -55
                if(pDevice->byBBPreEDIndex == 14) break;
                pDevice->byBBPreEDIndex = 14;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xC0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -54,-55\n");
            } else if(pDevice->byBBPreEDRSSI <= 56)  { //RSSI -56
                if(pDevice->byBBPreEDIndex == 13) break;
                pDevice->byBBPreEDIndex = 13;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -56\n");
            } else if(pDevice->byBBPreEDRSSI <= 57)  { //RSSI -57
                if(pDevice->byBBPreEDIndex == 12) break;
                pDevice->byBBPreEDIndex = 12;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xB0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -57\n");
            } else if(pDevice->byBBPreEDRSSI <= 58)  { //RSSI -58
                if(pDevice->byBBPreEDIndex == 11) break;
                pDevice->byBBPreEDIndex = 11;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x70); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -58\n");
            } else if(pDevice->byBBPreEDRSSI <= 59)  { //RSSI -59
                if(pDevice->byBBPreEDIndex == 10) break;
                pDevice->byBBPreEDIndex = 10;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -59\n");
            } else if(pDevice->byBBPreEDRSSI <= 60)  { //RSSI -60
                if(pDevice->byBBPreEDIndex == 9) break;
                pDevice->byBBPreEDIndex = 9;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xEA); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -60\n");
            } else if(pDevice->byBBPreEDRSSI <= 61)  { //RSSI -61
                if(pDevice->byBBPreEDIndex == 8) break;
                pDevice->byBBPreEDIndex = 8;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xC0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -61\n");
            } else if(pDevice->byBBPreEDRSSI <= 62)  { //RSSI -62
                if(pDevice->byBBPreEDIndex == 7) break;
                pDevice->byBBPreEDIndex = 7;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x9C); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -62\n");
            } else if(pDevice->byBBPreEDRSSI <= 63)  { //RSSI -63
                if(pDevice->byBBPreEDIndex == 6) break;
                pDevice->byBBPreEDIndex = 6;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x80); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -63\n");
            } else if(pDevice->byBBPreEDRSSI <= 64)  { //RSSI -64
                if(pDevice->byBBPreEDIndex == 5) break;
                pDevice->byBBPreEDIndex = 5;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x68); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -64\n");
            } else if(pDevice->byBBPreEDRSSI <= 65)  { //RSSI -65
                if(pDevice->byBBPreEDIndex == 4) break;
                pDevice->byBBPreEDIndex = 4;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x52); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -65\n");
            } else if(pDevice->byBBPreEDRSSI <= 66)  { //RSSI -66
                if(pDevice->byBBPreEDIndex == 3) break;
                pDevice->byBBPreEDIndex = 3;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x43); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -66\n");
            } else if(pDevice->byBBPreEDRSSI <= 67)  { //RSSI -67
                if(pDevice->byBBPreEDIndex == 2) break;
                pDevice->byBBPreEDIndex = 2;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x36); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -67\n");
            } else if(pDevice->byBBPreEDRSSI <= 68)  { //RSSI -68
                if(pDevice->byBBPreEDIndex == 1) break;
                pDevice->byBBPreEDIndex = 1;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x2D); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -68\n");
            } else { //RSSI -69, -70, ...
                if(pDevice->byBBPreEDIndex == 0) break;
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x24); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -69, -70,..\n");
            }
            break;

        case RF_VT3342A0: //RobertYu:20060627, testing table
            if( bScanning )
            {   // need Max sensitivity  //RSSI -67, -68, ...
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x38); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -67, -68,..\n");
                break;
            }

            if(pDevice->byBBPreEDRSSI <= 41) { // RSSI 0, -1,-2,....-41
                if(pDevice->byBBPreEDIndex == 20) break;
                pDevice->byBBPreEDIndex = 20;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0xFF); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI 0, -1,-2,..-41\n");
            } else if(pDevice->byBBPreEDRSSI <= 42)  { //RSSI -42
                if(pDevice->byBBPreEDIndex == 19) break;
                pDevice->byBBPreEDIndex = 19;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x36); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -42\n");
            } else if(pDevice->byBBPreEDRSSI <= 43)  { //RSSI -43
                if(pDevice->byBBPreEDIndex == 18) break;
                pDevice->byBBPreEDIndex = 18;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x26); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -43\n");
            } else if(pDevice->byBBPreEDRSSI <= 45)  { //RSSI -44, -45
                if(pDevice->byBBPreEDIndex == 17) break;
                pDevice->byBBPreEDIndex = 17;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x18); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -44,-45\n");
            } else if(pDevice->byBBPreEDRSSI <= 47)  { //RSSI -46, -47
                if(pDevice->byBBPreEDIndex == 16) break;
                pDevice->byBBPreEDIndex = 16;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x11); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -46,-47\n");
            } else if(pDevice->byBBPreEDRSSI <= 49)  { //RSSI -48, -49
                if(pDevice->byBBPreEDIndex == 15) break;
                pDevice->byBBPreEDIndex = 15;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x0a); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -48,-49\n");
            } else if(pDevice->byBBPreEDRSSI <= 51)  { //RSSI -50, -51
                if(pDevice->byBBPreEDIndex == 14) break;
                pDevice->byBBPreEDIndex = 14;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x07); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -50,-51\n");
            } else if(pDevice->byBBPreEDRSSI <= 53)  { //RSSI -52, -53
                if(pDevice->byBBPreEDIndex == 13) break;
                pDevice->byBBPreEDIndex = 13;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x04); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x00); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -52,-53\n");
            } else if(pDevice->byBBPreEDRSSI <= 55)  { //RSSI -54, -55
                if(pDevice->byBBPreEDIndex == 12) break;
                pDevice->byBBPreEDIndex = 12;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xC0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -54,-55\n");
            } else if(pDevice->byBBPreEDRSSI <= 56)  { //RSSI -56
                if(pDevice->byBBPreEDIndex == 11) break;
                pDevice->byBBPreEDIndex = 11;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x02); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -56\n");
            } else if(pDevice->byBBPreEDRSSI <= 57)  { //RSSI -57
                if(pDevice->byBBPreEDIndex == 10) break;
                pDevice->byBBPreEDIndex = 10;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xB0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -57\n");
            } else if(pDevice->byBBPreEDRSSI <= 58)  { //RSSI -58
                if(pDevice->byBBPreEDIndex == 9) break;
                pDevice->byBBPreEDIndex = 9;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x70); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -58\n");
            } else if(pDevice->byBBPreEDRSSI <= 59)  { //RSSI -59
                if(pDevice->byBBPreEDIndex == 8) break;
                pDevice->byBBPreEDIndex = 8;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x01); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x30); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -59\n");
            } else if(pDevice->byBBPreEDRSSI <= 60)  { //RSSI -60
                if(pDevice->byBBPreEDIndex == 7) break;
                pDevice->byBBPreEDIndex = 7;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xEA); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -60\n");
            } else if(pDevice->byBBPreEDRSSI <= 61)  { //RSSI -61
                if(pDevice->byBBPreEDIndex == 6) break;
                pDevice->byBBPreEDIndex = 6;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0xC0); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -61\n");
            } else if(pDevice->byBBPreEDRSSI <= 62)  { //RSSI -62
                if(pDevice->byBBPreEDIndex == 5) break;
                pDevice->byBBPreEDIndex = 5;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x9C); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -62\n");
            } else if(pDevice->byBBPreEDRSSI <= 63)  { //RSSI -63
                if(pDevice->byBBPreEDIndex == 4) break;
                pDevice->byBBPreEDIndex = 4;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x80); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -63\n");
            } else if(pDevice->byBBPreEDRSSI <= 64)  { //RSSI -64
                if(pDevice->byBBPreEDIndex == 3) break;
                pDevice->byBBPreEDIndex = 3;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x68); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -64\n");
            } else if(pDevice->byBBPreEDRSSI <= 65)  { //RSSI -65
                if(pDevice->byBBPreEDIndex == 2) break;
                pDevice->byBBPreEDIndex = 2;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x52); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -65\n");
            } else if(pDevice->byBBPreEDRSSI <= 66)  { //RSSI -66
                if(pDevice->byBBPreEDIndex == 1) break;
                pDevice->byBBPreEDIndex = 1;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x43); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -66\n");
            } else { //RSSI -67, -68, ...
                if(pDevice->byBBPreEDIndex == 0) break;
                pDevice->byBBPreEDIndex = 0;
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xC9, 0x00); //CR201(0xC9)
                ControlvWriteByte(pDevice, MESSAGE_REQUEST_BBREG, 0xCE, 0x38); //CR206(0xCE)
                DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO"            pDevice->byBBPreEDRSSI -67, -68,..\n");
            }
            break;

    }
>>>>>>> p9x

	case RF_VT3226:
	case RF_VT3226D0:
		if (scanning)	{ /* Max sensitivity */
			ed_inx = 0;
			cr_206 = 0x24;
			break;
		}

		if (priv->bb_pre_ed_rssi <= 41) {
			ed_inx = 22;
			cr_201 = 0xff;
		} else if (priv->bb_pre_ed_rssi <= 42) {
			ed_inx = 21;
			cr_201 = 0x36;
		} else if (priv->bb_pre_ed_rssi <= 43) {
			ed_inx = 20;
			cr_201 = 0x26;
		} else if (priv->bb_pre_ed_rssi <= 45) {
			ed_inx = 19;
			cr_201 = 0x18;
		} else if (priv->bb_pre_ed_rssi <= 47) {
			ed_inx = 18;
			cr_201 = 0x11;
		} else if (priv->bb_pre_ed_rssi <= 49) {
			ed_inx = 17;
			cr_201 = 0xa;
		} else if (priv->bb_pre_ed_rssi <= 51) {
			ed_inx = 16;
			cr_201 = 0x7;
		} else if (priv->bb_pre_ed_rssi <= 53) {
			ed_inx = 15;
			cr_201 = 0x4;
		} else if (priv->bb_pre_ed_rssi <= 55) {
			ed_inx = 14;
			cr_201 = 0x2;
			cr_206 = 0xc0;
		} else if (priv->bb_pre_ed_rssi <= 56) {
			ed_inx = 13;
			cr_201 = 0x2;
			cr_206 = 0x30;
		} else if (priv->bb_pre_ed_rssi <= 57) {
			ed_inx = 12;
			cr_201 = 0x1;
			cr_206 = 0xb0;
		} else if (priv->bb_pre_ed_rssi <= 58) {
			ed_inx = 11;
			cr_201 = 0x1;
			cr_206 = 0x70;
		} else if (priv->bb_pre_ed_rssi <= 59) {
			ed_inx = 10;
			cr_201 = 0x1;
			cr_206 = 0x30;
		} else if (priv->bb_pre_ed_rssi <= 60) {
			ed_inx = 9;
			cr_206 = 0xea;
		} else if (priv->bb_pre_ed_rssi <= 61) {
			ed_inx = 8;
			cr_206 = 0xc0;
		} else if (priv->bb_pre_ed_rssi <= 62) {
			ed_inx = 7;
			cr_206 = 0x9c;
		} else if (priv->bb_pre_ed_rssi <= 63) {
			ed_inx = 6;
			cr_206 = 0x80;
		} else if (priv->bb_pre_ed_rssi <= 64) {
			ed_inx = 5;
			cr_206 = 0x68;
		} else if (priv->bb_pre_ed_rssi <= 65) {
			ed_inx = 4;
			cr_206 = 0x52;
		} else if (priv->bb_pre_ed_rssi <= 66) {
			ed_inx = 3;
			cr_206 = 0x43;
		} else if (priv->bb_pre_ed_rssi <= 67) {
			ed_inx = 2;
			cr_206 = 0x36;
		} else if (priv->bb_pre_ed_rssi <= 68) {
			ed_inx = 1;
			cr_206 = 0x2d;
		} else {
			ed_inx = 0;
			cr_206 = 0x24;
		}
		break;

	case RF_VT3342A0:
		if (scanning) { /* need Max sensitivity */
			ed_inx = 0;
			cr_206 = 0x38;
			break;
		}

		if (priv->bb_pre_ed_rssi <= 41) {
			ed_inx = 20;
			cr_201 = 0xff;
		} else if (priv->bb_pre_ed_rssi <= 42) {
			ed_inx = 19;
			cr_201 = 0x36;
		} else if (priv->bb_pre_ed_rssi <= 43) {
			ed_inx = 18;
			cr_201 = 0x26;
		} else if (priv->bb_pre_ed_rssi <= 45) {
			ed_inx = 17;
			cr_201 = 0x18;
		} else if (priv->bb_pre_ed_rssi <= 47) {
			ed_inx = 16;
			cr_201 = 0x11;
		} else if (priv->bb_pre_ed_rssi <= 49) {
			ed_inx = 15;
			cr_201 = 0xa;
		} else if (priv->bb_pre_ed_rssi <= 51) {
			ed_inx = 14;
			cr_201 = 0x7;
		} else if (priv->bb_pre_ed_rssi <= 53) {
			ed_inx = 13;
			cr_201 = 0x4;
		} else if (priv->bb_pre_ed_rssi <= 55) {
			ed_inx = 12;
			cr_201 = 0x2;
			cr_206 = 0xc0;
		} else if (priv->bb_pre_ed_rssi <= 56) {
			ed_inx = 11;
			cr_201 = 0x2;
			cr_206 = 0x30;
		} else if (priv->bb_pre_ed_rssi <= 57) {
			ed_inx = 10;
			cr_201 = 0x1;
			cr_206 = 0xb0;
		} else if (priv->bb_pre_ed_rssi <= 58) {
			ed_inx = 9;
			cr_201 = 0x1;
			cr_206 = 0x70;
		} else if (priv->bb_pre_ed_rssi <= 59) {
			ed_inx = 8;
			cr_201 = 0x1;
			cr_206 = 0x30;
		} else if (priv->bb_pre_ed_rssi <= 60) {
			ed_inx = 7;
			cr_206 = 0xea;
		} else if (priv->bb_pre_ed_rssi <= 61) {
			ed_inx = 6;
			cr_206 = 0xc0;
		} else if (priv->bb_pre_ed_rssi <= 62) {
			ed_inx = 5;
			cr_206 = 0x9c;
		} else if (priv->bb_pre_ed_rssi <= 63) {
			ed_inx = 4;
			cr_206 = 0x80;
		} else if (priv->bb_pre_ed_rssi <= 64) {
			ed_inx = 3;
			cr_206 = 0x68;
		} else if (priv->bb_pre_ed_rssi <= 65) {
			ed_inx = 2;
			cr_206 = 0x52;
		} else if (priv->bb_pre_ed_rssi <= 66) {
			ed_inx = 1;
			cr_206 = 0x43;
		} else {
			ed_inx = 0;
			cr_206 = 0x38;
		}
		break;

	}

	if (ed_inx == priv->bb_pre_ed_index && !scanning)
		return;

	priv->bb_pre_ed_index = ed_inx;

	dev_dbg(&priv->usb->dev, "%s bb_pre_ed_rssi %d\n",
					__func__, priv->bb_pre_ed_rssi);

	if (!cr_201 && !cr_206)
		return;

	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0xc9, cr_201);
	vnt_control_out_u8(priv, MESSAGE_REQUEST_BBREG, 0xce, cr_206);
}

