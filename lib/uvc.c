/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * UVC protocol handling
 *
 * Copyright (C) 2010-2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <errno.h>
#include <limits.h>
#include <linux/usb/ch9.h>
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include "configfs.h"
#include "events.h"
#include "stream.h"
#include "tools.h"
#include "uvc.h"
#include "v4l2.h"

struct uvc_device
{
	struct v4l2_device *vdev;

	struct uvc_stream *stream;
	struct uvc_function_config *fc;

	struct uvc_streaming_control probe;
	struct uvc_streaming_control commit;

	int control;

	unsigned int fcc;
	unsigned int width;
	unsigned int height;

	/* Processing Unit control tracking across SETUP → DATA phases */
	unsigned int control_cs;     /* PU control selector, 0 = none pending */
	unsigned int control_entity; /* entity ID from wIndex >> 8           */
	short brightness_val;        /* current value tracked for GET_CUR    */
	short contrast_val;
	short saturation_val;
	short sharpness_val;         /* forwarded as libcamera Sharpness     */
	short wb_temp_val;           /* Kelvin; forwarded as ColourTemperature */
	uint8_t wb_auto_val;         /* 0=manual, 1=auto; forwarded as AwbEnable */
	uint8_t plf_val;             /* 0=off, 1=50 Hz, 2=60 Hz              */

	/* Camera Terminal AE Mode and Exposure tracking */
	short    ae_mode_val;        /* 1=manual, 2=auto (UVC AE Mode bitmask) */
	uint32_t exposure_abs_val;   /* CT Exposure Time Absolute, 100µs units */
};

/*
 * Virtual control ID for Camera Terminal AE Mode.  The UVC CT AE Mode
 * control selector (0x02) conflicts with UVC PU Brightness (also 0x02);
 * disambiguation is by entity_id in uvc.c.  A synthetic ID outside the
 * normal 0x00–0x1F CS range is used when forwarding to the video source.
 */
#define UVC_CT_AE_MODE_CS          0x02
#define UVC_CT_AE_MODE_VIRTUAL     0x100

#define UVC_CT_EXPOSURE_ABS_CS     0x04
#define UVC_CT_EXPOSURE_ABS_VIRTUAL 0x101

static const char *uvc_request_names[] = {
	[UVC_RC_UNDEFINED] = "UNDEFINED",
	[UVC_SET_CUR] = "SET_CUR",
	[UVC_GET_CUR] = "GET_CUR",
	[UVC_GET_MIN] = "GET_MIN",
	[UVC_GET_MAX] = "GET_MAX",
	[UVC_GET_RES] = "GET_RES",
	[UVC_GET_LEN] = "GET_LEN",
	[UVC_GET_INFO] = "GET_INFO",
	[UVC_GET_DEF] = "GET_DEF",
};

static const char *uvc_request_name(uint8_t req)
{
    if (req < ARRAY_SIZE(uvc_request_names))
        return uvc_request_names[req];
    else
        return "UNKNOWN";
}

static const char *uvc_pu_control_names[] = {
	[UVC_PU_CONTROL_UNDEFINED] = "UNDEFINED",
	[UVC_PU_BACKLIGHT_COMPENSATION_CONTROL] = "BACKLIGHT_COMPENSATION",
	[UVC_PU_BRIGHTNESS_CONTROL] = "BRIGHTNESS",
	[UVC_PU_CONTRAST_CONTROL] = "CONTRAST",
	[UVC_PU_GAIN_CONTROL] = "GAIN",
	[UVC_PU_POWER_LINE_FREQUENCY_CONTROL] = "POWER_LINE_FREQUENCY",
	[UVC_PU_HUE_CONTROL] = "HUE",
	[UVC_PU_SATURATION_CONTROL] = "SATURATION",
	[UVC_PU_SHARPNESS_CONTROL] = "SHARPNESS",
	[UVC_PU_GAMMA_CONTROL] = "GAMMA",
	[UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL] = "WHITE_BALANCE_TEMPERATURE",
	[UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL] = "WHITE_BALANCE_TEMPERATURE_AUTO",
	[UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL] = "WHITE_BALANCE_COMPONENT",
	[UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL] = "WHITE_BALANCE_COMPONENT_AUTO",
	[UVC_PU_DIGITAL_MULTIPLIER_CONTROL] = "DIGITAL_MULTIPLIER",
	[UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL] = "DIGITAL_MULTIPLIER_LIMIT",
	[UVC_PU_HUE_AUTO_CONTROL] = "HUE_AUTO",
	[UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL] = "ANALOG_VIDEO_STANDARD",
	[UVC_PU_ANALOG_LOCK_STATUS_CONTROL] = "ANALOG_LOCK_STATUS",
};

static const char *pu_control_name(uint8_t cs)
{
    if (cs < ARRAY_SIZE(uvc_pu_control_names))
        return uvc_pu_control_names[cs];
    else
        return "UNKNOWN";
}

static const char *ct_control_name(uint8_t cs)
{
	switch (cs) {
	case UVC_CT_AE_MODE_CS:        return "AE_MODE";
	case UVC_CT_EXPOSURE_ABS_CS:   return "EXPOSURE_TIME_ABSOLUTE";
	default:                        return "UNKNOWN";
	}
}

struct uvc_device *uvc_open(const char *devname, struct uvc_stream *stream)
{
	struct uvc_device *dev;

	dev = malloc(sizeof *dev);
	if (dev == NULL)
		return NULL;

	memset(dev, 0, sizeof *dev);
	dev->stream = stream;

	dev->vdev = v4l2_open(devname);
	if (dev->vdev == NULL) {
		free(dev);
		return NULL;
	}

	return dev;
}

void uvc_close(struct uvc_device *dev)
{
	v4l2_close(dev->vdev);
	dev->vdev = NULL;

	free(dev);
}

/* ---------------------------------------------------------------------------
 * Request processing
 */

static void
uvc_fill_streaming_control(struct uvc_device *dev,
			   struct uvc_streaming_control *ctrl,
			   int iformat, int iframe, unsigned int ival)
{
	const struct uvc_function_config_format *format;
	const struct uvc_function_config_frame *frame;
	unsigned int i;

	/*
	 * Restrict the iformat, iframe and ival to valid values. Negative
	 * values for iformat or iframe will result in the maximum valid value
	 * being selected.
	 */
        iformat = clamp((unsigned int)iformat, 1U,
                        dev->fc->streaming.num_formats);
	format = &dev->fc->streaming.formats[iformat-1];

	iframe = clamp((unsigned int)iframe, 1U, format->num_frames);
	frame = &format->frames[iframe-1];

	for (i = 0; i < frame->num_intervals; ++i) {
		if (ival <= frame->intervals[i]) {
			ival = frame->intervals[i];
			break;
		}
	}

	if (i == frame->num_intervals)
		ival = frame->intervals[frame->num_intervals-1];

	memset(ctrl, 0, sizeof *ctrl);

	ctrl->bmHint = 1;
	ctrl->bFormatIndex = iformat;
	ctrl->bFrameIndex = iframe ;
	ctrl->dwFrameInterval = ival;

	/*
	 * The maximum size in bytes for a single frame depends on the format.
	 * This switch will need extending for any new formats that are added
	 * to ensure the buffer size calculations are done correctly.
	 */
	switch (format->fcc) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_MJPEG:
		ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
		break;
	}

	ctrl->dwMaxPayloadTransferSize = dev->fc->streaming.ep.wMaxPacketSize;
	ctrl->bmFramingInfo = 3;
	ctrl->bPreferedVersion = 1;
	ctrl->bMaxVersion = 1;
}

static void
uvc_events_process_standard(struct uvc_device *dev,
			    const struct usb_ctrlrequest *ctrl,
			    struct uvc_request_data *resp)
{
	printf("standard request\n");
	(void)dev;
	(void)ctrl;
	(void)resp;
}

static void
uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
			   uint8_t entity_id, uint8_t len,
			   struct uvc_request_data *resp)
{
	short *cur;
	short min_val, max_val, def_val;

	printf("control request (req %s cs %s entity %u)\n",
	       uvc_request_name(req),
	       entity_id == 1 ? ct_control_name(cs) : pu_control_name(cs),
	       entity_id);

	/*
	 * Camera Terminal AE Mode (entity=1, CS=0x02) has the same CS value as
	 * PU Brightness (entity=2, CS=0x02).  Handle CT first to avoid the
	 * conflict, then fall through to the PU switch for everything else.
	 */
	if (entity_id == 1 && cs == UVC_CT_AE_MODE_CS) {
		switch (req) {
		case UVC_SET_CUR:
			dev->control_cs     = cs;
			dev->control_entity = entity_id;
			resp->data[0] = 0;
			resp->length  = len;
			break;
		case UVC_GET_CUR:
			resp->data[0] = (uint8_t)dev->ae_mode_val;
			resp->length  = 1;
			break;
		case UVC_GET_DEF:
			resp->data[0] = 2; /* Auto mode */
			resp->length  = 1;
			break;
		case UVC_GET_MIN:
			resp->data[0] = 1; /* Manual */
			resp->length  = 1;
			break;
		case UVC_GET_MAX:
			/* For AE Mode (bitmask), GET_MAX == GET_RES: the set of
			 * supported modes.  Update both together if adding relay
			 * support for shutter/aperture priority (bits 2/3). */
			resp->data[0] = 3; /* Manual (bit0) + Auto (bit1) */
			resp->length  = 1;
			break;
		case UVC_GET_RES:
			resp->data[0] = 3; /* Manual (bit0) + Auto (bit1) */
			resp->length  = 1;
			break;
		case UVC_GET_INFO:
			resp->data[0] = 0x03;
			resp->length  = 1;
			break;
		default:
			resp->length = -EL2HLT;
			break;
		}
		return;
	}

	/* CT Exposure Time Absolute (entity=1, CS=0x04): 4-byte unsigned DWORD. */
	if (entity_id == 1 && cs == UVC_CT_EXPOSURE_ABS_CS) {
		uint32_t min_u = 1, max_u = 10000, def_u = 166, res_u = 1;
		switch (req) {
		case UVC_SET_CUR:
			dev->control_cs     = cs;
			dev->control_entity = entity_id;
			resp->data[0] = 0;
			resp->length  = len;
			break;
		case UVC_GET_CUR:
			memcpy(resp->data, &dev->exposure_abs_val, 4);
			resp->length = 4;
			break;
		case UVC_GET_MIN:
			memcpy(resp->data, &min_u, 4);
			resp->length = 4;
			break;
		case UVC_GET_MAX:
			memcpy(resp->data, &max_u, 4);
			resp->length = 4;
			break;
		case UVC_GET_DEF:
			memcpy(resp->data, &def_u, 4);
			resp->length = 4;
			break;
		case UVC_GET_RES:
			memcpy(resp->data, &res_u, 4);
			resp->length = 4;
			break;
		case UVC_GET_LEN: {
			uint16_t l = 4;
			memcpy(resp->data, &l, 2);
			resp->length = 2;
			break;
		}
		case UVC_GET_INFO:
			resp->data[0] = 0x03;
			resp->length  = 1;
			break;
		default:
			resp->length = -EL2HLT;
			break;
		}
		return;
	}

	/*
	 * PU Power Line Frequency (CS=UVC_PU_POWER_LINE_FREQUENCY_CONTROL):
	 * 1-byte menu; 0=disabled, 1=50 Hz, 2=60 Hz.
	 * Handle before the generic 2-byte PU switch.
	 */
	if (entity_id == 2 && cs == UVC_PU_POWER_LINE_FREQUENCY_CONTROL) {
		switch (req) {
		case UVC_SET_CUR:
			dev->control_cs     = cs;
			dev->control_entity = entity_id;
			resp->data[0] = 0;
			resp->length  = len;
			break;
		case UVC_GET_CUR:
			resp->data[0] = dev->plf_val;
			resp->length  = 1;
			break;
		case UVC_GET_DEF:
			resp->data[0] = 0; /* default: disabled */
			resp->length  = 1;
			break;
		case UVC_GET_MIN:
			resp->data[0] = 0;
			resp->length  = 1;
			break;
		case UVC_GET_MAX:
			resp->data[0] = 2;
			resp->length  = 1;
			break;
		case UVC_GET_RES:
			resp->data[0] = 1;
			resp->length  = 1;
			break;
		case UVC_GET_LEN: {
			uint16_t l = 1;
			memcpy(resp->data, &l, 2);
			resp->length = 2;
			break;
		}
		case UVC_GET_INFO:
			resp->data[0] = 0x03;
			resp->length  = 1;
			break;
		default:
			resp->length = -EL2HLT;
			break;
		}
		return;
	}

	/*
	 * PU White Balance Temperature Auto (CS=0x0B) is a 1-byte boolean.
	 * Handle it before the 2-byte PU switch.
	 */
	if (entity_id == 2 && cs == UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL) {
		switch (req) {
		case UVC_SET_CUR:
			dev->control_cs     = cs;
			dev->control_entity = entity_id;
			resp->data[0] = 0;
			resp->length  = len;
			break;
		case UVC_GET_CUR:
			resp->data[0] = dev->wb_auto_val;
			resp->length  = 1;
			break;
		case UVC_GET_DEF:
			resp->data[0] = 1; /* default: auto */
			resp->length  = 1;
			break;
		case UVC_GET_MIN:
			resp->data[0] = 0;
			resp->length  = 1;
			break;
		case UVC_GET_MAX:
			resp->data[0] = 1;
			resp->length  = 1;
			break;
		case UVC_GET_RES:
			resp->data[0] = 1;
			resp->length  = 1;
			break;
		case UVC_GET_LEN: {
			uint16_t l = 1;
			memcpy(resp->data, &l, 2);
			resp->length = 2;
			break;
		}
		case UVC_GET_INFO:
			resp->data[0] = 0x03;
			resp->length  = 1;
			break;
		default:
			resp->length = -EL2HLT;
			break;
		}
		return;
	}

	switch (cs) {
	case UVC_PU_BRIGHTNESS_CONTROL:
		cur = &dev->brightness_val;
		min_val = 0; max_val = 255; def_val = 127;
		break;
	case UVC_PU_CONTRAST_CONTROL:
		cur = &dev->contrast_val;
		min_val = 0; max_val = 255; def_val = 127;
		break;
	case UVC_PU_SATURATION_CONTROL:
		cur = &dev->saturation_val;
		min_val = 0; max_val = 255; def_val = 127;
		break;
	case UVC_PU_SHARPNESS_CONTROL:
		cur = &dev->sharpness_val;
		/* UVC 16 maps to libcamera 1.0 via value * 16.0 / 255.0 */
		min_val = 0; max_val = 255; def_val = 16;
		break;
	case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
		cur = &dev->wb_temp_val;
		min_val = 2800; max_val = 6500; def_val = 4000;
		break;
	default:
		/* Unknown control: acknowledge as GET+SET capable. */
		resp->data[0] = (req == UVC_GET_INFO) ? 0x03 : 0;
		resp->length  = (req == UVC_GET_INFO) ? 1    : len;
		return;
	}

	switch (req) {
	case UVC_SET_CUR:
		dev->control_cs     = cs;
		dev->control_entity = entity_id;
		resp->data[0] = 0;
		resp->length  = len;
		break;
	case UVC_GET_CUR:
		memcpy(resp->data, cur, sizeof(*cur));
		resp->length = sizeof(*cur);
		break;
	case UVC_GET_MIN:
		memcpy(resp->data, &min_val, sizeof(min_val));
		resp->length = sizeof(min_val);
		break;
	case UVC_GET_MAX:
		memcpy(resp->data, &max_val, sizeof(max_val));
		resp->length = sizeof(max_val);
		break;
	case UVC_GET_DEF:
		memcpy(resp->data, &def_val, sizeof(def_val));
		resp->length = sizeof(def_val);
		break;
	case UVC_GET_RES: {
		short res = 1;
		memcpy(resp->data, &res, sizeof(res));
		resp->length = sizeof(res);
		break;
	}
	case UVC_GET_LEN: {
		uint16_t l = sizeof(*cur); /* 2 bytes for all generic PU controls */
		memcpy(resp->data, &l, 2);
		resp->length = 2;
		break;
	}
	case UVC_GET_INFO:
		resp->data[0] = 0x03; /* GET + SET supported */
		resp->length  = 1;
		break;
	default:
		resp->length = -EL2HLT;
		break;
	}
}

static void
uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs,
			     struct uvc_request_data *resp)
{
	struct uvc_streaming_control *ctrl;

	printf("streaming request (req %s cs %02x)\n", uvc_request_name(req), cs);

	if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
		return;

	ctrl = (struct uvc_streaming_control *)&resp->data;
	resp->length = sizeof *ctrl;

	switch (req) {
	case UVC_SET_CUR:
		dev->control = cs;
		resp->length = 34;
		break;

	case UVC_GET_CUR:
		if (cs == UVC_VS_PROBE_CONTROL)
			memcpy(ctrl, &dev->probe, sizeof *ctrl);
		else
			memcpy(ctrl, &dev->commit, sizeof *ctrl);
		break;

	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_DEF:
		if (req == UVC_GET_MAX)
			uvc_fill_streaming_control(dev, ctrl, -1, -1, UINT_MAX);
		else
			uvc_fill_streaming_control(dev, ctrl, 1, 1, 0);
		break;

	case UVC_GET_RES:
		memset(ctrl, 0, sizeof *ctrl);
		break;

	case UVC_GET_LEN:
		resp->data[0] = 0x00;
		resp->data[1] = 0x22;
		resp->length = 2;
		break;

	case UVC_GET_INFO:
		resp->data[0] = 0x03;
		resp->length = 1;
		break;
	}
}

static void
uvc_events_process_class(struct uvc_device *dev,
			 const struct usb_ctrlrequest *ctrl,
			 struct uvc_request_data *resp)
{
	unsigned int interface = ctrl->wIndex & 0xff;

	if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
		return;

	if (interface == dev->fc->control.intf.bInterfaceNumber)
		uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8,
					   ctrl->wIndex >> 8, ctrl->wLength, resp);
	else if (interface == dev->fc->streaming.intf.bInterfaceNumber)
		uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8, resp);
}

static void
uvc_events_process_setup(struct uvc_device *dev,
			 const struct usb_ctrlrequest *ctrl,
			 struct uvc_request_data *resp)
{
	dev->control = 0;
	dev->control_cs = 0;

	printf("bRequestType %02x bRequest %02x wValue %04x wIndex %04x "
		"wLength %04x\n", ctrl->bRequestType, ctrl->bRequest,
		ctrl->wValue, ctrl->wIndex, ctrl->wLength);

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		uvc_events_process_standard(dev, ctrl, resp);
		break;

	case USB_TYPE_CLASS:
		uvc_events_process_class(dev, ctrl, resp);
		break;

	default:
		break;
	}
}

static void
uvc_events_process_data(struct uvc_device *dev,
			const struct uvc_request_data *data)
{
	const struct uvc_streaming_control *ctrl =
		(const struct uvc_streaming_control *)&data->data;
	struct uvc_streaming_control *target;

	switch (dev->control) {
	case UVC_VS_PROBE_CONTROL:
		printf("setting probe control, length = %d\n", data->length);
		target = &dev->probe;
		break;

	case UVC_VS_COMMIT_CONTROL:
		printf("setting commit control, length = %d\n", data->length);
		target = &dev->commit;
		break;

	default:
		if (dev->control_cs) {
			/*
			 * Read up to 4 bytes from the host.  Controls have
			 * different widths: 1-byte (WBauto), 2-byte (most PU),
			 * 4-byte (CT exposure time).  We zero-extend and narrow
			 * per-control below.
			 */
			uint32_t raw = 0;
			size_t copy_len = (size_t)data->length < sizeof(raw)
					  ? (size_t)data->length : sizeof(raw);
			memcpy(&raw, data->data, copy_len);

			if (dev->control_entity == 2) {
				/* Processing Unit controls */
				switch (dev->control_cs) {
				case UVC_PU_BRIGHTNESS_CONTROL:
					dev->brightness_val = (short)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_BRIGHTNESS_CONTROL,
								      (int)(short)raw);
					break;
				case UVC_PU_CONTRAST_CONTROL:
					dev->contrast_val = (short)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_CONTRAST_CONTROL,
								      (int)(uint16_t)raw);
					break;
				case UVC_PU_SATURATION_CONTROL:
					dev->saturation_val = (short)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_SATURATION_CONTROL,
								      (int)(uint16_t)raw);
					break;
				case UVC_PU_SHARPNESS_CONTROL:
					dev->sharpness_val = (short)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_SHARPNESS_CONTROL,
								      (int)(uint16_t)raw);
					break;
				case UVC_PU_POWER_LINE_FREQUENCY_CONTROL:
					dev->plf_val = (uint8_t)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_POWER_LINE_FREQUENCY_CONTROL,
								      (int)(uint8_t)raw);
					break;
				case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
					dev->wb_temp_val = (short)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL,
								      (int)(uint16_t)raw);
					break;
				case UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL:
					dev->wb_auto_val = (uint8_t)raw;
					uvc_stream_set_camera_control(dev->stream,
								      UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL,
								      (int)(uint8_t)raw);
					break;
				default:
					printf("unknown PU control cs=%u\n",
					       dev->control_cs);
					break;
				}
			} else if (dev->control_entity == 1 &&
				   dev->control_cs == UVC_CT_AE_MODE_CS) {
				/* Camera Terminal AE Mode */
				dev->ae_mode_val = (short)(uint8_t)raw;
				uvc_stream_set_camera_control(dev->stream,
							      UVC_CT_AE_MODE_VIRTUAL,
							      (int)(uint8_t)raw);
			} else if (dev->control_entity == 1 &&
				   dev->control_cs == UVC_CT_EXPOSURE_ABS_CS) {
				/* Camera Terminal Exposure Time Absolute (4-byte) */
				dev->exposure_abs_val = raw;
				/* Convert 100µs UVC units to µs for libcamera */
				uvc_stream_set_camera_control(dev->stream,
							      UVC_CT_EXPOSURE_ABS_VIRTUAL,
							      (int)(raw * 100u));
			} else {
				printf("unknown control entity=%u cs=%u\n",
				       dev->control_entity, dev->control_cs);
			}
			dev->control_cs = 0;
		} else {
			printf("setting unknown control, length = %d\n",
			       data->length);
		}
		return;
	}

	uvc_fill_streaming_control(dev, target, ctrl->bFormatIndex,
				   ctrl->bFrameIndex, ctrl->dwFrameInterval);

	if (dev->control == UVC_VS_COMMIT_CONTROL) {
		const struct uvc_function_config_format *format;
		const struct uvc_function_config_frame *frame;
		struct v4l2_pix_format pixfmt;
		unsigned int fps;

		format = &dev->fc->streaming.formats[target->bFormatIndex-1];
		frame = &format->frames[target->bFrameIndex-1];

		dev->fcc = format->fcc;
		dev->width = frame->width;
		dev->height = frame->height;

		memset(&pixfmt, 0, sizeof pixfmt);
		pixfmt.width = frame->width;
		pixfmt.height = frame->height;
		pixfmt.pixelformat = format->fcc;
		pixfmt.field = V4L2_FIELD_NONE;
		if (format->fcc == V4L2_PIX_FMT_MJPEG)
			pixfmt.sizeimage = target->dwMaxVideoFrameSize;

		uvc_stream_set_format(dev->stream, &pixfmt);

		/* fps is guaranteed to be non-zero and thus valid. */
		fps = 1.0 / (target->dwFrameInterval / 10000000.0);
		uvc_stream_set_frame_rate(dev->stream, fps);
	}
}

static void uvc_events_process(void *d)
{
	struct uvc_device *dev = d;
	struct v4l2_event v4l2_event;
	const struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	struct uvc_request_data resp;
	int ret;

	ret = ioctl(dev->vdev->fd, VIDIOC_DQEVENT, &v4l2_event);
	if (ret < 0) {
		printf("VIDIOC_DQEVENT failed: %s (%d)\n", strerror(errno),
			errno);
		return;
	}

	memset(&resp, 0, sizeof resp);
	resp.length = -EL2HLT;

	switch (v4l2_event.type) {
	case UVC_EVENT_CONNECT:
	case UVC_EVENT_DISCONNECT:
		return;

	case UVC_EVENT_SETUP:
		uvc_events_process_setup(dev, &uvc_event->req, &resp);
		break;

	case UVC_EVENT_DATA:
		uvc_events_process_data(dev, &uvc_event->data);
		return;

	case UVC_EVENT_STREAMON:
		uvc_stream_enable(dev->stream, 1);
		return;

	case UVC_EVENT_STREAMOFF:
		uvc_stream_enable(dev->stream, 0);
		return;
	}

	ret = ioctl(dev->vdev->fd, UVCIOC_SEND_RESPONSE, &resp);
	if (ret < 0) {
		printf("UVCIOC_SEND_RESPONSE failed: %s (%d)\n",
		       strerror(errno), errno);
		return;
	}
}

/* ---------------------------------------------------------------------------
 * Initialization and setup
 */

void uvc_events_init(struct uvc_device *dev, struct events *events)
{
	struct v4l2_event_subscription sub;

	/* Default to the minimum values. */
	uvc_fill_streaming_control(dev, &dev->probe, 1, 1, 0);
	uvc_fill_streaming_control(dev, &dev->commit, 1, 1, 0);

	/* Default PU control values — match libcamera's neutral starting point. */
	dev->brightness_val = 127;   /* UVC 127 → libcamera ~0.0 (neutral) */
	dev->contrast_val   = 127;   /* UVC 127 → libcamera ~1.0 (neutral) */
	dev->saturation_val = 127;   /* UVC 127 → libcamera ~1.0 (neutral) */
	dev->sharpness_val  = 16;    /* UVC 16  → libcamera  1.0 (neutral) */
	dev->wb_temp_val    = 4000;  /* 4000 K (neutral daylight) */
	dev->wb_auto_val    = 1;    /* auto white balance */
	dev->plf_val        = 0;    /* power line frequency: disabled */

	/* Default CT values. */
	dev->ae_mode_val     = 2;   /* Auto mode */
	dev->exposure_abs_val = 166; /* ~16.6ms in 100µs units */

	memset(&sub, 0, sizeof sub);
	sub.type = UVC_EVENT_SETUP;
	ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
	sub.type = UVC_EVENT_DATA;
	ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
	sub.type = UVC_EVENT_STREAMON;
	ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
	sub.type = UVC_EVENT_STREAMOFF;
	ioctl(dev->vdev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);

	events_watch_fd(events, dev->vdev->fd, EVENT_EXCEPTION,
			uvc_events_process, dev);
}

void uvc_set_config(struct uvc_device *dev, struct uvc_function_config *fc)
{
	dev->fc = fc;
}

int uvc_set_format(struct uvc_device *dev, struct v4l2_pix_format *format)
{
	return v4l2_set_format(dev->vdev, format);
}

struct v4l2_device *uvc_v4l2_device(struct uvc_device *dev)
{
	/*
	 * TODO: The V4L2 device shouldn't be exposed. We should replace this
	 * with an abstract video sink class when one will be avaiilable.
	 */
	return dev->vdev;
}
