#ifndef IST_SERIAL_DRIVER__CMD_HPP_
#define IST_SERIAL_DRIVER__CMD_HPP_

#include <stdint.h>

namespace ist_serial_driver
{
#define CMD_HEADING_BYTES ("IR")
#define CMD_HEADING_BYTES_LENGTH (2)
#define CMD_TAILING_BYTES ("ON")
#define CMD_TAILING_BYTES_LENGTH (2)
#define CMD_TO_CV_CONTENT_LENGTH (14)
#define CMD_TO_CV_SIZE \
  (CMD_HEADING_BYTES_LENGTH + CMD_TO_CV_CONTENT_LENGTH + CMD_TAILING_BYTES_LENGTH)
#define CMD_TO_EC_CONTENT_LENGTH (9)
#define CMD_TO_EC_SIZE \
  (CMD_HEADING_BYTES_LENGTH + CMD_TO_EC_CONTENT_LENGTH + CMD_TAILING_BYTES_LENGTH)

struct CmdToCv
{
    uint8_t enemy_color;
    uint8_t task_mode;
    float bullet_speed;
    float pitch;
    float yaw;
};

struct CmdToEc
{
    float pitch;
    float yaw;
    uint8_t fire;
};

void CmdToCv_make(const CmdToCv *cmd, uint8_t *data);

int32_t CmdToCv_parse(CmdToCv *cmd, uint8_t *data, uint32_t data_len);

void CmdToEc_make(const CmdToEc *cmd, uint8_t *data);

int32_t CmdToEc_parse(const CmdToEc *cmd, uint8_t *data, uint32_t data_len);

} // namespace ist_serial_driver

#endif  // IST_SERIAL_DRIVER__CMD_HPP_