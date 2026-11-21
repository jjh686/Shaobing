#include "bsp_usb.h"
#include "usb_tx_queue.h"
#include <assert.h>
#include <stdio.h>
USBD_HandleTypeDef hUsbDeviceFS={.dev_state=USBD_STATE_CONFIGURED,.pClassData=(void *)1};
extern UsbTxQueue usb_tx_queue;
static uint32_t tick=100;
static unsigned busy=1,sent,last_kind;
uint32_t HAL_GetTick(void){return tick;}
uint8_t *CDCInitRxbufferNcallback(USBCallback a,USBCallback b){(void)a;(void)b;return NULL;}
uint8_t CDC_Transmit_FS(uint8_t *b,uint16_t n)
{if(busy)return USBD_BUSY;last_kind=(unsigned)usb_tx_kind(b,n);sent++;return USBD_OK;}
int main(void)
{
    uint8_t ack[34]={0x5a,28,0x21},sp[43]={'S','P'},joint[46]={0x5a,40,0x22};
    assert(USBTransmit(sp,43)==USBD_OK&&USBTransmit(joint,46)==USBD_OK);
    assert(USBTransmitStamped(ack,34,95)==USBD_OK&&sent==0);
    busy=0;tick=101;USBTransmitPump();assert(sent==1&&last_kind==0);
    USBTransmitPump();assert(last_kind==1);USBTransmitPump();assert(last_kind==2);
    busy=1;assert(USBTransmitStamped(ack,34,95)==USBD_OK);
    busy=0;tick=106;unsigned before=sent;USBTransmitPump();
    assert(sent==before&&usb_tx_queue.expired[0]==1); /* Original age, not enqueue age. */
    busy=1;USBTransmit(ack,34);USBTransmitReset();assert(usb_tx_queue.disconnect_drops==1);
    busy=0;USBTransmitPump();assert(sent==before);
    hUsbDeviceFS.dev_state=0;assert(USBTransmit(ack,34)==USBD_FAIL);
    assert(usb_tx_queue.busy>=3&&usb_tx_queue.sent[0]==1);
    puts("PASS actual USB BSP: CDC BUSY retry, ACK priority, original timestamp deadline and disconnect reset");
}
