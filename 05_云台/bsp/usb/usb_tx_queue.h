#ifndef USB_TX_QUEUE_H
#define USB_TX_QUEUE_H
#include <stdint.h>
#include <string.h>

/* Latest frame per telemetry type; ACK > SP > joint. Never replay a backlog.
 * Caller serializes enqueue/select/completion with CDC and USB disconnect. */
#define USB_TX_MAX_AGE_MS 10u
typedef struct {uint8_t data[46],len;uint32_t received_ms;} UsbTxSlot;
typedef struct {
    UsbTxSlot slots[3];
    volatile uint32_t queued[3],sent[3],replaced[3],expired[3];
    volatile uint32_t busy,failed,disconnect_drops,max_wait_ms;
} UsbTxQueue;
static inline int usb_tx_kind(const uint8_t *b,unsigned n)
{
    if(n==34&&b[0]==0x5a&&b[2]==0x21)return 0;
    if(n==43&&b[0]=='S'&&b[1]=='P')return 1;
    if(n==46&&b[0]==0x5a&&b[2]==0x22)return 2;
    return -1;
}
static inline void usb_tx_clear(UsbTxQueue *q)
{
    for(unsigned i=0;i<3;i++)if(q->slots[i].len){q->disconnect_drops++;q->slots[i].len=0;}
}
static inline void usb_tx_enqueue(UsbTxQueue *q,unsigned kind,const uint8_t *b,unsigned n,uint32_t now)
{
    UsbTxSlot *s=&q->slots[kind];if(s->len)q->replaced[kind]++;
    memcpy(s->data,b,n);s->len=n;s->received_ms=now;q->queued[kind]++;
}
static inline int usb_tx_select(UsbTxQueue *q,uint32_t now)
{
    int selected=-1;
    for(unsigned i=0;i<3;i++)if(q->slots[i].len){
        uint32_t age=now-q->slots[i].received_ms;
        if(age>q->max_wait_ms)q->max_wait_ms=age;
        if(age>USB_TX_MAX_AGE_MS){q->expired[i]++;q->slots[i].len=0;}
        else if(selected<0)selected=(int)i;
    }
    return selected;
}
#endif
