#include "usb_tx_queue.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    UsbTxQueue q={0};uint8_t ack[34]={0x5a,28,0x21},sp[43]={'S','P'},joint[46]={0x5a,40,0x22};
    assert(usb_tx_kind(ack,34)==0&&usb_tx_kind(sp,43)==1&&usb_tx_kind(joint,46)==2);
    usb_tx_enqueue(&q,2,joint,46,100);usb_tx_enqueue(&q,1,sp,43,100);usb_tx_enqueue(&q,0,ack,34,101);
    assert(usb_tx_select(&q,101)==0);q.slots[0].len=0;assert(usb_tx_select(&q,102)==1);
    q.slots[1].len=0;assert(usb_tx_select(&q,102)==2);
    ack[10]=42;usb_tx_enqueue(&q,0,ack,34,105);ack[10]=43;usb_tx_enqueue(&q,0,ack,34,106);
    assert(q.replaced[0]==1&&q.slots[0].data[10]==43);
    assert(usb_tx_select(&q,116)==0&&q.expired[2]==1);
    assert(usb_tx_select(&q,117)==-1&&q.expired[0]==1);
    usb_tx_enqueue(&q,0,ack,34,0xfffffffeu);assert(usb_tx_select(&q,2)==0);
    usb_tx_clear(&q);assert(q.disconnect_drops==1&&usb_tx_select(&q,3)==-1);
    puts("PASS USB telemetry: ACK priority, bounded latest frames, true expiry, disconnect reset and clock wrap");
}
