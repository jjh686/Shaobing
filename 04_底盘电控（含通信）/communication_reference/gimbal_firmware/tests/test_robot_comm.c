/* [USB_SP 通信整理] 同一组收发行为测试同时验证旧接口和统一通信入口。 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usb.h"
#include "gimbal_config.h"
#include "ins_task.h"
#include "master_process.h"
/* This regression suite tests the original SP path. The actual navigation port
 * is exercised separately by the two-board chain test. */
void NavGatewayInit(void) {}
void NavGatewayReset(void) {}
void NavGatewayReceive(const uint8_t *p,unsigned n,uint32_t t) {(void)p;(void)n;(void)t;}

#ifdef ROBOT_COMM_TEST_DIRECT
#include "robot_comm.h"
#define InitLink() RobotCommInit()
#define ReadCommand(out) RobotCommGetVisionCommand(out)
#define LinkOnline() RobotCommVisionIsOnline()
#define UpdateLink() RobotCommUpdate()
#define SetMode(mode) RobotCommSetVisionMode(mode)
#define SetSpeed(speed) RobotCommSetVisionBulletSpeed(speed)
#else
#define InitLink() VisionInit(NULL)
#define ReadCommand(out) VisionGetCommand(out)
#define LinkOnline() VisionIsOnline()
#define UpdateLink() VisionSend()
#define SetMode(mode) VisionSetMode(mode)
#define SetSpeed(speed) VisionSetFlag(COLOR_BLUE, VISION_MODE_AIM, speed)
#endif

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

static uint32_t now, primask;
static uint8_t configured, transmit_result, snapshot_valid;
static unsigned int transmit_calls;
static USBCallback receive_callback;
static uint8_t receive_buffer[2048], transmitted[SP_STATUS_SIZE];
static INS_Snapshot_t snapshot;

uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; }

uint8_t *USBInit(USB_Init_Config_s config)
{
    CHECK(config.rx_cbk != NULL && config.tx_cbk == NULL);
    receive_callback = config.rx_cbk;
    return receive_buffer;
}

uint8_t USBIsConfigured(void) { return configured; }

uint8_t USBTransmit(uint8_t *buffer, uint16_t length)
{
    CHECK(length == SP_STATUS_SIZE);
    memcpy(transmitted, buffer, length);
    ++transmit_calls;
    return transmit_result;
}

uint8_t INS_GetSnapshot(INS_Snapshot_t *out)
{
    *out = snapshot;
    return snapshot_valid;
}

static void Near(float actual, float expected)
{
    CHECK(fabsf(actual - expected) < 1e-6f);
}

static void Prepare(void)
{
    now += 1000;
    primask = 0;
    configured = snapshot_valid = 1;
    transmit_result = USBD_OK;
    transmit_calls = 0;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.q[0] = 1.0f;
    snapshot.tick = now;
    snapshot.valid = 1;
    CHECK(InitLink() != NULL);
    CHECK(receive_callback != NULL);
}

static void Advance(uint32_t milliseconds)
{
    now += milliseconds;
    snapshot.tick = now;
}

static SP_Command Command(uint8_t mode)
{
    SP_Command command = {
        .head = {'S', 'P'}, .mode = mode,
        .yaw = 0.25f, .yaw_vel = -0.4f, .yaw_acc = 0.6f,
        .pitch = -0.15f, .pitch_vel = 0.3f, .pitch_acc = -0.5f,
    };
    command.crc16 = SP_CRC16((const uint8_t *)&command, sizeof(command) - 2);
    return command;
}

static void Inject(const void *bytes, size_t length)
{
    CHECK(length <= sizeof(receive_buffer));
    memcpy(receive_buffer, bytes, length);
    receive_callback((uint16_t)length);
}

static SP_Status Status(void)
{
    SP_Status status;
    memcpy(&status, transmitted, sizeof(status));
    CHECK(status.head[0] == 'S' && status.head[1] == 'P');
    CHECK(status.crc16 == SP_CRC16(transmitted, SP_STATUS_SIZE - 2));
    return status;
}

static void InitializationAndFragmentedCommand(void)
{
    Prepare();
    Vision_Recv_s out;
    CHECK(!ReadCommand(NULL));
    CHECK(!ReadCommand(&out) && !LinkOnline());
    SP_Command command = Command(2);
    Inject(&command, 7);
    CHECK(!ReadCommand(&out));
    Inject((const uint8_t *)&command + 7, sizeof(command) - 7);
    CHECK(ReadCommand(&out) && LinkOnline());
    CHECK(out.mode == 2 && out.fire_mode == NO_FIRE);
    CHECK(out.sequence == 1 && out.session == 0 && out.received_tick == now);
    CHECK(out.target_state == TARGET_CONVERGING);
    Near(out.yaw, command.yaw); Near(out.yaw_vel, command.yaw_vel);
    Near(out.yaw_acc, command.yaw_acc); Near(out.pitch, command.pitch);
    Near(out.pitch_vel, command.pitch_vel); Near(out.pitch_acc, command.pitch_acc);
    primask = 1;
    CHECK(ReadCommand(&out) && primask == 1);
    primask = 0;
    CHECK(ReadCommand(&out) && primask == 0);
}

static void NoiseCorruptionAndBurst(void)
{
    Prepare();
    Vision_Recv_s out;
    SP_Command command = Command(1), corrupt = command;
    corrupt.crc16 ^= 1u;
    Inject(&corrupt, sizeof(corrupt));
    CHECK(!ReadCommand(&out));
    const uint8_t noise[] = {0xa5, 'S', 'x', 0, 'S'};
    Inject(noise, sizeof(noise));
    Inject(&command, sizeof(command));
    CHECK(ReadCommand(&out) && out.sequence == 1);
    SP_Command burst[] = {Command(1), Command(0)};
    Inject(burst, sizeof(burst));
    CHECK(ReadCommand(&out) && out.sequence == 3 && out.mode == 0);
    CHECK(out.target_state == NO_TARGET && out.fire_mode == NO_FIRE);
    command.yaw = NAN;
    command.crc16 = SP_CRC16((const uint8_t *)&command, sizeof(command) - 2);
    Inject(&command, sizeof(command));
    CHECK(ReadCommand(&out) && out.sequence == 3);
    command = Command(3);
    Inject(&command, sizeof(command));
    CHECK(ReadCommand(&out) && out.sequence == 3);
    command = Command(1);
    Inject(&command, sizeof(command));
    CHECK(ReadCommand(&out) && out.sequence == 4);
}

static void TimeoutAndSessionChange(void)
{
    Prepare();
    Vision_Recv_s out;
    SP_Command command = Command(1);
    Inject(&command, sizeof(command));
    Advance(VISION_RX_TIMEOUT_MS);
    CHECK(LinkOnline());
    Advance(1);
    CHECK(!LinkOnline());
    CHECK(ReadCommand(&out)); /* 过期由在线/控制逻辑判断，保留旧getter的行为。 */
    Inject(&command, sizeof(command));
    configured = 0;
    CHECK(!LinkOnline());
    configured = 1;
    Inject(&command, 10);
    receive_callback(0); /* 模拟USB重新枚举，残帧不得跨会话拼接。 */
    CHECK(!ReadCommand(&out) && out.session == 1);
    Inject(&command, sizeof(command));
    CHECK(ReadCommand(&out) && out.session == 1 && out.sequence == 3);
}

static void FeedbackFieldsAndRate(void)
{
    Prepare();
    snapshot.q[0] = cosf(0.1f);
    snapshot.q[3] = sinf(0.1f);
    snapshot.gyro[1] = 0.2f;
    snapshot.gyro[2] = -0.3f;
    SetMode(1);
    UpdateLink();
    CHECK(transmit_calls == 1);
    SP_Status status = Status();
    CHECK(status.mode == 1 && status.bullet_count == 0);
    CHECK(memcmp(status.q, snapshot.q, sizeof(status.q)) == 0);
    Near(status.yaw, 0.2f); Near(status.pitch, 0.0f);
    Near(status.yaw_vel, -0.3f); Near(status.pitch_vel, 0.2f);
    Near(status.bullet_speed, VISION_NOMINAL_SPEED_MPS);
    UpdateLink();
    Advance(VISION_TX_PERIOD_MS - 1);
    UpdateLink();
    CHECK(transmit_calls == 1);
    Advance(1);
    UpdateLink();
    CHECK(transmit_calls == 2);
}

static void InvalidFeedbackAndBusyTransport(void)
{
    Prepare();
    configured = 0;
    UpdateLink();
    CHECK(transmit_calls == 0);
    configured = 1;
    UpdateLink();
    CHECK(transmit_calls == 0); /* 失败也保留原来的5ms节流时机。 */
    Advance(VISION_TX_PERIOD_MS);
    snapshot_valid = 0;
    UpdateLink();
    CHECK(transmit_calls == 0);
    snapshot_valid = 1;
    Advance(VISION_TX_PERIOD_MS);
    snapshot.tick = now - GIMBAL_INS_TIMEOUT_MS - 1;
    UpdateLink();
    CHECK(transmit_calls == 0);
    Advance(VISION_TX_PERIOD_MS);
    snapshot.q[0] = NAN;
    UpdateLink();
    CHECK(transmit_calls == 0);
    snapshot.q[0] = 1.0f;
    Advance(VISION_TX_PERIOD_MS);
    transmit_result = USBD_BUSY;
    UpdateLink();
    CHECK(transmit_calls == 1);
    transmit_result = USBD_OK;
    UpdateLink();
    CHECK(transmit_calls == 1);
    Advance(VISION_TX_PERIOD_MS);
    UpdateLink();
    CHECK(transmit_calls == 2);
    transmit_result = USBD_FAIL;
    Advance(VISION_TX_PERIOD_MS);
    UpdateLink();
    CHECK(transmit_calls == 3);
}

static void ModeAndLegacySpeedSetting(void)
{
    Prepare();
    SetMode(3);
    UpdateLink();
    CHECK(Status().mode == 3);
    SetMode(4);
    SetSpeed(BIG_AMU_10);
    Advance(VISION_TX_PERIOD_MS);
    UpdateLink();
    SP_Status status = Status();
    CHECK(status.mode == 0);
    Near(status.bullet_speed, 10.0f);
    SetSpeed(BULLET_SPEED_NONE);
    Advance(VISION_TX_PERIOD_MS);
    UpdateLink();
    Near(Status().bullet_speed, 10.0f);
    VisionSetAltitude(123.0f, 456.0f, 789.0f);
    Advance(VISION_TX_PERIOD_MS);
    UpdateLink();
    status = Status();
    Near(status.yaw, 0.0f); Near(status.pitch, 0.0f);
}

static void TickWraparound(void)
{
    Prepare();
    now = UINT32_MAX - 2u;
    snapshot.tick = now;
    SP_Command command = Command(1);
    Inject(&command, sizeof(command));
    UpdateLink();
    unsigned int calls = transmit_calls;
    CHECK(calls == 1);
    Advance(VISION_TX_PERIOD_MS);
    CHECK(LinkOnline());
    UpdateLink();
    CHECK(transmit_calls == calls + 1);
    Advance(VISION_RX_TIMEOUT_MS);
    CHECK(!LinkOnline());
}

int main(void)
{
    InitializationAndFragmentedCommand();
    NoiseCorruptionAndBurst();
    TimeoutAndSessionChange();
    FeedbackFieldsAndRate();
    InvalidFeedbackAndBusyTransport();
    ModeAndLegacySpeedSetting();
    TickWraparound();
#ifdef ROBOT_COMM_TEST_DIRECT
    puts("robot communication: 7 test groups passed (unified API)");
#else
    puts("robot communication: 7 test groups passed (legacy API)");
#endif
    return 0;
}
