/* usb_desc_fix.c -- USB configuration descriptor with a notification
 * endpoint big enough for a CDC SERIAL_STATE notification (2026-11-05).
 *
 * The Pico SDK's stdio_usb descriptor (stdio_usb_descriptors.c) declares
 * the CDC notification endpoint (EP 0x81) with wMaxPacketSize = 8. A
 * SERIAL_STATE notification is 10 bytes (8-byte header + 2-byte
 * bmUartState), so the one main.c sends to raise DSR/DCD for legacy hosts
 * (usb_cdc_assert_modem_lines) was split 8 + 2 on the wire. Linux's cdc-acm
 * then read the state bits past the end of its 8-byte buffer (it reported
 * RI+DCD instead of DSR+DCD, measured with TIOCMGET) and Windows usbser.sys
 * most likely dropped it -- UI-View32 kept its hourglass on Windows while
 * the same firmware was fine under Wine, where cdc-acm forces CTS on.
 *
 * The SDK compiles its descriptor file into every stdio_usb target and
 * offers no hook for the endpoint size, so we take over the configuration
 * descriptor callback with the linker's --wrap (see CMakeLists.txt): TinyUSB
 * calls tud_descriptor_configuration_cb() from usbd.c, which resolves to
 * __wrap_tud_descriptor_configuration_cb() below. Everything but the
 * notification packet size is kept identical to the SDK's layout (CDC on
 * interfaces 0/1, the picotool reset interface on 2, same endpoint
 * addresses, same string indexes), so picotool and the SDK's own stdio_usb
 * code are unaffected. */
#include "tusb.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_usb/reset_interface.h"

#define USBD_ITF_CDC       (0)
#if !PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
#define USBD_ITF_MAX       (2)
#else
#define USBD_ITF_RPI_RESET (2)
#define USBD_ITF_MAX       (3)
#endif

#define USBD_CDC_EP_CMD          (0x81)
#define USBD_CDC_EP_OUT          (0x02)
#define USBD_CDC_EP_IN           (0x82)
#define USBD_CDC_CMD_MAX_SIZE    (16)   /* SDK: 8 -- too small for SERIAL_STATE */
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0         (0x00)
#define USBD_STR_CDC       (0x04)
#define USBD_STR_RPI_RESET (0x05)

#if !PICO_STDIO_USB_DEVICE_SELF_POWERED
#define USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE (0)
#define USBD_MAX_POWER_MA (250)
#else
#define USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE TUSB_DESC_CONFIG_ATT_SELF_POWERED
#define USBD_MAX_POWER_MA (1)
#endif

#define TUD_RPI_RESET_DESC_LEN 9
#if !PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#else
#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_RPI_RESET_DESC_LEN)
#endif

#define TUD_RPI_RESET_DESCRIPTOR(_itfnum, _stridx) \
  /* Interface */ \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_VENDOR_SPECIFIC, \
  RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, _stridx,

static const uint8_t usbd_desc_cfg_fixed[USBD_DESC_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, USBD_STR_0, USBD_DESC_LEN,
        USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE, USBD_MAX_POWER_MA),

    TUD_CDC_DESCRIPTOR(USBD_ITF_CDC, USBD_STR_CDC, USBD_CDC_EP_CMD,
        USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),

#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
    TUD_RPI_RESET_DESCRIPTOR(USBD_ITF_RPI_RESET, USBD_STR_RPI_RESET)
#endif
};

const uint8_t *__real_tud_descriptor_configuration_cb(uint8_t index);
const uint8_t *__wrap_tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return usbd_desc_cfg_fixed;
}
