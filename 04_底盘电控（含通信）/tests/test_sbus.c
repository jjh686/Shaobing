#include "remote_control.h"
#include "bsp_usart.h"
#include "daemon.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static USARTInstance uart;
static DaemonInstance daemon;
static uint32_t tick=1;
static unsigned uart_registrations,daemon_registrations;
uint32_t HAL_GetTick(void){return tick;}
USARTInstance *USARTRegister(USART_Init_Config_s *c){uart_registrations++;uart.module_callback=c->module_callback;return &uart;}
void USARTServiceInit(USARTInstance *p){(void)p;}
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *c){daemon_registrations++;(void)c;return &daemon;}
void DaemonReload(DaemonInstance *p){(void)p;}
int DaemonIsOnline(DaemonInstance *p){(void)p;return 1;}
static void push(const uint8_t *b,unsigned n){memcpy(uart.recv_buff,b,n);uart.recv_len=n;uart.module_callback();}
static FS_RC_ctrl_t snapshot(void){FS_RC_ctrl_t rc;FS_RemoteControlGetSnapshot(&rc);return rc;}
static void frame(uint8_t b[25],unsigned ch8)
{
    memset(b,0,25);b[0]=0x0f;
    unsigned channels[10]={992,992,992,992,240,240,1024,ch8,240,240};
    for(unsigned i=0;i<10;i++)for(unsigned bit=0;bit<11;bit++)
        if(channels[i]&(1u<<bit))b[1+(i*11+bit)/8]|=1u<<((i*11+bit)%8);
}
int main(void)
{
    UART_HandleTypeDef handle={0};FS_RC_ctrl_t *first=FS_RemoteControlInit(&handle);
    assert(first==FS_RemoteControlInit(&handle));
    assert(uart_registrations==1&&daemon_registrations==1);
    uint8_t b[25],two[50];frame(b,1024);push(b,25);assert(FS_RemoteControlIsOnline());
    uint32_t generation=snapshot().fault_generation;
    for(unsigned cut=1;cut<25;cut++){
        tick+=14;push(b,cut);tick++;push(b+cut,25-cut);
        assert(FS_RemoteControlIsOnline()&&snapshot().fault_generation==generation);
    }
    memcpy(two,b,25);memcpy(two+25,b,25);push(two,50);
    assert(fs_sbus_diag.valid_frames==27&&fs_sbus_diag.raw_channels[7]==1024);
    const uint8_t noise[]={0xaa,0x00,0xff};push(noise,3);push(b,25);assert(fs_sbus_diag.discarded_bytes==3);
    /* Lose bytes, then recover the next frame without a DMA-aligned header. */
    push(b,9);push(b,25);push(b,25);assert(FS_RemoteControlIsOnline());
    b[24]=0xff;push(b,25);b[24]=0;push(b,25);push(b,25);
    assert(fs_sbus_diag.bad_tail>0);
    uint32_t lost=fs_sbus_diag.frame_lost,failed=fs_sbus_diag.failsafe,invalid=fs_sbus_diag.invalid_switches;
    b[23]=4;push(b,25);assert(!FS_RemoteControlIsOnline()&&fs_sbus_diag.frame_lost==lost+1);
    b[23]=8;push(b,25);assert(!FS_RemoteControlIsOnline()&&fs_sbus_diag.failsafe==failed+1);
    frame(b,700);push(b,25);assert(!FS_RemoteControlIsOnline()&&fs_sbus_diag.invalid_switches>=invalid+1);
    frame(b,1024);push(b,25);assert(FS_RemoteControlIsOnline());
    generation=snapshot().fault_generation;uart.error_events++;assert(!FS_RemoteControlIsOnline());
    assert(snapshot().fault_generation==generation+1&&fs_sbus_diag.uart_errors==1);
    push(b,25);assert(FS_RemoteControlIsOnline());
    push(b,8);tick+=21;push(b,25);assert(FS_RemoteControlIsOnline()&&fs_sbus_diag.partial_timeouts==1);
    tick=0xfffffffeu;push(b,25);push(b,8);generation=snapshot().fault_generation;
    tick=2;push(b+8,17);assert(FS_RemoteControlIsOnline()&&snapshot().fault_generation==generation);
    tick+=101;assert(!FS_RemoteControlIsOnline());
    puts("PASS actual SBUS source: all 24 split points, concatenation, resync after missing bytes, raw channels, lost/failsafe, invalid CH8, UART error, partial timeout, clock wrap, real timeout");
}
