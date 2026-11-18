/* [USB_SP 新增 01 开始] 新增SP CRC与流式拆包、粘包/噪声恢复、有限数值检查及回传打包。 */
#include "sp_protocol.h"
#include <math.h>
#include <string.h>

uint16_t SP_CRC16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xffffu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0x8408u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

void SP_ParserInit(SP_Parser *parser)
{
    memset(parser, 0, sizeof(*parser));
}

static void DropByte(SP_Parser *parser)
{
    --parser->used;
    memmove(parser->data, parser->data + 1, parser->used);
}

static int CommandValid(const SP_Command *c)
{
    return c->mode <= 2u && isfinite(c->yaw) && isfinite(c->pitch) &&
           isfinite(c->yaw_vel) && isfinite(c->pitch_vel) &&
           isfinite(c->yaw_acc) && isfinite(c->pitch_acc) &&
           fabsf(c->yaw) <= 3.141593f && fabsf(c->pitch) <= 1.570797f;
}

void SP_ParserFeed(SP_Parser *parser, const uint8_t *data, size_t size,
                   SP_CommandCallback callback, void *context)
{
    if (!parser || (!data && size)) return;
    for (size_t i = 0; i < size; ++i) {
        parser->data[parser->used++] = data[i];
        /* Retain a trailing S. Recover one byte at a time after a corrupt frame. */
        while (parser->used &&
               (parser->data[0] != 'S' || (parser->used > 1 && parser->data[1] != 'P')))
            DropByte(parser);
        if (parser->used != SP_COMMAND_SIZE) continue;
        uint16_t received = (uint16_t)parser->data[27] | ((uint16_t)parser->data[28] << 8);
        if (SP_CRC16(parser->data, SP_COMMAND_SIZE - 2) != received) {
            ++parser->crc_errors;
            DropByte(parser);
            continue;
        }
        SP_Command command;
        memcpy(&command, parser->data, sizeof(command));
        if (!CommandValid(&command)) {
            ++parser->invalid_frames;
            DropByte(parser);
            continue;
        }
        parser->used = 0;
        ++parser->accepted;
        if (callback) callback(&command, context);
    }
}

void SP_PackStatus(const SP_Status *status, uint8_t out[SP_STATUS_SIZE])
{
    memcpy(out, status, SP_STATUS_SIZE);
    out[0] = 'S';
    out[1] = 'P';
    uint16_t crc = SP_CRC16(out, SP_STATUS_SIZE - 2);
    out[41] = (uint8_t)crc;
    out[42] = (uint8_t)(crc >> 8);
}
/* [USB_SP 新增 01 结束] */
