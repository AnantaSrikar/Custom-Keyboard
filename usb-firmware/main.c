/*
 * Copyright (c) 2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd10.h"
#include "hal_gpio.h"

// For Software based USB 
#include "grainuum.h"

#define PERIOD_FAST     100
#define PERIOD_SLOW     500
#define BAUD_RATE 115200

#define BUFFER_SIZE 8
#define NUM_BUFFERS 4
#define EP_INTERVAL_MS 6


// GPIO pin setup
HAL_GPIO_PIN(UART_TX,  A, 14)
HAL_GPIO_PIN(UART_RX,  A, 15)

// UART setup
static void uart_init(uint32_t baud)
{
  uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_C_Val);
  HAL_GPIO_UART_RX_in();
  HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_C_Val);

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  SERCOM0->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_RXPO(1/*PAD1*/) | SERCOM_USART_CTRLA_TXPO(0/*PAD0*/);

  SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

  SERCOM0->USART.BAUD.reg = (uint16_t)br+1;

  SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

static void uart_putc(char c)
{
  while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
  SERCOM0->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
  while (*s)
    uart_putc(*s++);
}

// Very important, otherwise timings will be messed up
static void sys_init(void)
{
  // Switch to 8MHz clock (disable prescaler)
  SYSCTRL->OSC8M.bit.PRESC = 0;
}

// USB setup
// USB port setup, PORT_PA04 and PORT_PA02
static struct GrainuumUSB defaultUsbPhy = {

    // PA04 as D+
    .usbdpIAddr = (uint32_t)&PORT->Group[0].IN,
    .usbdpSAddr = (uint32_t)&PORT->Group[0].OUTSET,
    .usbdpCAddr = (uint32_t)&PORT->Group[0].OUTCLR,
    .usbdpDAddr = (uint32_t)&PORT->Group[0].DIR,
    .usbdpMask = (1 << 4),
    .usbdpShift = 4,

    // PA02 as D-
    .usbdnIAddr = (uint32_t)&PORT->Group[0].IN,
    .usbdnSAddr = (uint32_t)&PORT->Group[0].OUTSET,
    .usbdnCAddr = (uint32_t)&PORT->Group[0].OUTCLR,
    .usbdnDAddr = (uint32_t)&PORT->Group[0].DIR,
    .usbdnMask = (1 << 2),
    .usbdnShift = 2,
};


static void set_usb_config_num(struct GrainuumUSB *usb, int configNum)
{
  uart_puts("\r\nIn set_usb_config_num\r\n");
  (void)usb;
  (void)configNum;
  ;
}

static const uint8_t hid_report_descriptor[] = {
    0x06, 0x00, 0xFF, // (GLOBAL) USAGE_PAGE         0xFF00 Vendor-defined
    0x09, 0x00,       // (LOCAL)  USAGE              0xFF000000
    0xA1, 0x01,       // (MAIN)   COLLECTION         0x01 Application (Usage=0xFF000000: Page=Vendor-defined, Usage=, Type=)
    0x26, 0xFF, 0x00, //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)
    0x75, 0x08,       //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x08,       //   (GLOBAL) REPORT_COUNT       0x08 (8) Number of fields
    0x06, 0xFF, 0xFF, //   (GLOBAL) USAGE_PAGE         0xFFFF Vendor-defined
    0x09, 0x01,       //   (LOCAL)  USAGE              0xFFFF0001
    0x81, 0x02,       //   (MAIN)   INPUT              0x00000002 (8 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x09, 0x01,       //   (LOCAL)  USAGE              0xFFFF0001
    0x91, 0x02,       //   (MAIN)   OUTPUT             0x00000002 (8 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0xC0,             // (MAIN)   END_COLLECTION     Application
};

static const struct usb_device_descriptor device_descriptor = {
    .bLength = 18,                //sizeof(struct usb_device_descriptor),
    .bDescriptorType = DT_DEVICE, /* DEVICE */
    .bcdUSB = 0x0200,             /* USB 2.0 */
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 0x08, /* 8-byte packets max */
    .idVendor = 0x1209,
    .idProduct = 0x9317,
    .bcdDevice = 0x0114,   /* Device release 1.14 */
    .iManufacturer = 0x02, /* No manufacturer string */
    .iProduct = 0x01,      /* Product name in string #2 */
    .iSerialNumber = 0x03, /* No serial number */
    .bNumConfigurations = 0x01,
};

static const struct usb_configuration_descriptor configuration_descriptor = {
    .bLength = 9, //sizeof(struct usb_configuration_descriptor),
    .bDescriptorType = DT_CONFIGURATION,
    .wTotalLength = (9 + /*9 + 9 + 7  +*/ 9 + 9 + 7 + 7) /*
                  (sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor)*/,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 5,
    .bmAttributes = 0x80, /* Remote wakeup not supported */
    .bMaxPower = 100 / 2, /* 100 mA (in 2-mA units) */
    .data = {
        /* struct usb_interface_descriptor { */
        /*  uint8_t bLength;            */ 9,
        /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
        /*  uint8_t bInterfaceNumber;   */ 0,
        /*  uint8_t bAlternateSetting;  */ 0,
        /*  uint8_t bNumEndpoints;      */ 2, /* Two extra EPs */
        /*  uint8_t bInterfaceClass;    */ 3, /* HID class */
        /*  uint8_t bInterfaceSubclass; */ 0, /* Boot Device subclass */
        /*  uint8_t bInterfaceProtocol; */ 0, /* 1 == keyboard, 2 == mouse */
        /*  uint8_t iInterface;         */ 4, /* String index #4 */
                                              /* }*/

        /* struct usb_hid_descriptor {        */
        /*  uint8_t  bLength;                 */ 9,
        /*  uint8_t  bDescriptorType;         */ DT_HID,
        /*  uint16_t bcdHID;                  */ 0x11, 0x01,
        /*  uint8_t  bCountryCode;            */ 0,
        /*  uint8_t  bNumDescriptors;         */ 1, /* We have only one REPORT */
        /*  uint8_t  bReportDescriptorType;   */ DT_HID_REPORT,
        /*  uint16_t wReportDescriptorLength; */ sizeof(hid_report_descriptor),
        sizeof(hid_report_descriptor) >> 8,
        /* }                                  */

        /* struct usb_endpoint_descriptor { */
        /*  uint8_t  bLength;             */ 7,
        /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
        /*  uint8_t  bEndpointAddress;    */ 0x81, /* EP1 (IN) */
        /*  uint8_t  bmAttributes;        */ 3,    /* Interrupt */
        /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
        /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
                                                             /* }                              */

        /* struct usb_endpoint_descriptor { */
        /*  uint8_t  bLength;             */ 7,
        /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
        /*  uint8_t  bEndpointAddress;    */ 0x01, /* EP1 (OUT) */
        /*  uint8_t  bmAttributes;        */ 3,    /* Interrupt */
        /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
        /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
                                                             /* }                              */
    },
};

#define USB_STR_BUF_LEN 64

static uint32_t str_buf_storage[USB_STR_BUF_LEN / sizeof(uint32_t)];
static int send_string_descriptor(const char *str, const void **data)
{

  uart_puts("\r\nIn send_string_descriptor\r\n");
  int len;
  int max_len;
  uint8_t *str_buf = (uint8_t *)str_buf_storage;
  uint8_t *str_offset = str_buf;

  len = strlen(str);
  max_len = (USB_STR_BUF_LEN / 2) - 2;

  if (len > max_len)
    len = max_len;

  *str_offset++ = (len * 2) + 2; // Two bytes for length count
  *str_offset++ = DT_STRING;     // Sending a string descriptor

  while (len--)
  {
    *str_offset++ = *str++;
    *str_offset++ = 0;
  }

  *data = str_buf;

  // Return the size, which is stored in the first byte of the output data.
  return str_buf[0];
}

static int get_string_descriptor(struct GrainuumUSB *usb,
                                 uint32_t num,
                                 const void **data)
{

  uart_puts("\r\nIn get_string_descriptor\r\n");

  static const uint8_t en_us[] = {0x04, DT_STRING, 0x09, 0x04};

  (void)usb;

  if (num == 0)
  {
    *data = en_us;
    return sizeof(en_us);
  }

  // Product
  if (num == 1)
    return send_string_descriptor("Palawan Bootloader", data);

  if (num == 2)
    return send_string_descriptor("21", data);

  if (num == 3)
    return send_string_descriptor("1236", data);

  if (num == 4)
    return send_string_descriptor("12345", data);

  if (num == 5)
    return send_string_descriptor("54", data);

  if (num == 6)
    return send_string_descriptor("12345678901234", data);

  return 0;
}

static int get_device_descriptor(struct GrainuumUSB *usb,
                                 uint32_t num,
                                 const void **data)
{

  uart_puts("\r\nIn get_device_descriptor\r\n");

  (void)usb;

  if (num == 0)
  {
    *data = &device_descriptor;
    return sizeof(device_descriptor);
  }
  return 0;
}

static int get_hid_report_descriptor(struct GrainuumUSB *usb,
                                     uint32_t num,
                                     const void **data)
{

  uart_puts("\r\nIn get_hid_report_descriptor\r\n");

  (void)usb;

  if (num == 0)
  {
    *data = &hid_report_descriptor;
    return sizeof(hid_report_descriptor);
  }

  return 0;
}

static int get_configuration_descriptor(struct GrainuumUSB *usb,
                                        uint32_t num,
                                        const void **data)
{

  uart_puts("\r\nIn get_configuration_descriptor\r\n");

  (void)usb;

  if (num == 0)
  {
    *data = &configuration_descriptor;
    return configuration_descriptor.wTotalLength;
  }
  return 0;
}

static int get_descriptor(struct GrainuumUSB *usb,
                          const void *packet,
                          const void **response)
{

  uart_puts("\r\nIn get_descriptor\r\n");

  const struct usb_setup_packet *setup = packet;

  switch (setup->wValueH)
  {
  case DT_DEVICE:
    return get_device_descriptor(usb, setup->wValueL, response);

  case DT_STRING:
    return get_string_descriptor(usb, setup->wValueL, response);

  case DT_CONFIGURATION:
    return get_configuration_descriptor(usb, setup->wValueL, response);

  case DT_HID_REPORT:
    return get_hid_report_descriptor(usb, setup->wValueL, response);
  }

  return 0;
}

static uint32_t rx_buffer[NUM_BUFFERS][BUFFER_SIZE / sizeof(uint32_t)];
static uint8_t rx_buffer_head;
static uint8_t rx_buffer_tail;

static uint32_t rx_buffer_queries = 0;
static void *get_usb_rx_buffer(struct GrainuumUSB *usb,
                               uint8_t epNum,
                               int32_t *size)
{

  uart_puts("\r\nIn get_usb_rx_buffer\r\n");
  (void)usb;
  (void)epNum;

  if (size)
    *size = sizeof(rx_buffer[0]);
  rx_buffer_queries++;
  return rx_buffer[rx_buffer_head];
}

static int received_data(struct GrainuumUSB *usb,
                         uint8_t epNum,
                         uint32_t bytes,
                         const void *data)
{
  (void)usb;
  (void)epNum;
  (void)bytes;
  (void)data;

  if (epNum == 1)
  {
    rx_buffer_head = (rx_buffer_head + 1) & (NUM_BUFFERS - 1);
  }

  /* Return 0, indicating this packet is complete. */
  return 0;
}

static int send_data_finished(struct GrainuumUSB *usb, int result)
{
  uart_puts("\r\nIn send_data_finished\r\n");
  (void)usb;
  (void)result;

  return 0;
}

static struct GrainuumConfig hid_link = {
    .getDescriptor = get_descriptor,
    .getReceiveBuffer = get_usb_rx_buffer,
    .receiveData = received_data,
    .sendDataFinished = send_data_finished,
    .setConfigNum = set_usb_config_num,
};

static GRAINUUM_BUFFER(phy_queue, 8);

static void process_all_usb_events(struct GrainuumUSB *usb)
{
  while (!GRAINUUM_BUFFER_IS_EMPTY(phy_queue))
  {
    uart_puts("\r\nSomething is happening ig\r\n");
    uint8_t *in_ptr = (uint8_t *)GRAINUUM_BUFFER_TOP(phy_queue);

    // Advance to the next packet (allowing us to be reentrant)
    GRAINUUM_BUFFER_REMOVE(phy_queue);

    // Process the current packet
    grainuumProcess(usb, in_ptr);
  }
}

int main(void)
{
  sys_init();
  uart_init(BAUD_RATE);

  uart_puts("\r\nFirmware init!\n\r");

  grainuumInit(&defaultUsbPhy, &hid_link);
  uart_puts("\r\nFirmware init complete, attempting disconnet!\r\n");

  grainuumDisconnect(&defaultUsbPhy);
  grainuumConnect(&defaultUsbPhy);

  uart_puts("\r\nFirmware must be online now!\r\n");

  while (1)
  {
    process_all_usb_events(&defaultUsbPhy);
    if (rx_buffer_head != rx_buffer_tail)
    {
        uart_puts("\r\nSomething is happening in main ig\r\n");
    }
  }

  return 0;
}