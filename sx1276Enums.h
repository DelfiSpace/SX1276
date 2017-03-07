/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#ifndef __SX1276ENUMS_H__
#define __SX1276ENUMS_H__

/*!
 * Radio driver internal state machine states definition
 */
typedef enum RadioState
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

/*!
 * Shaping filter values
 */
typedef enum ShapingFilter
{
    NONE = 0,
    BT_1_0,
    BT_0_5,
    BT_0_3
}ShapingFilter_t;

/*!
 *    Type of the modem. [LORA / FSK]
 */
typedef enum ModemType
{
    MODEM_FSK = 0,
    MODEM_LORA
}RadioModems_t;

typedef struct
{
    RadioModems_t modem;
    unsigned char power;
    ShapingFilter_t filtertype;
    unsigned int fdev;
    unsigned int bandwidth;
    unsigned int datarate;
    unsigned char coderate;
    unsigned short preambleLen;
    unsigned short payloadLen;
    bool fixLen;
    bool crcOn;
    bool freqHopOn;
    unsigned char hopPeriod;
    bool iqInverted;
    unsigned int timeout;
}TxConfig_t;

typedef struct
{
    RadioModems_t modem;
    unsigned char power;
    ShapingFilter_t filtertype;
    unsigned int fdev;
    unsigned int bandwidth;
    unsigned int bandwidthAfc;
    unsigned int datarate;
    unsigned char coderate;
    unsigned short preambleLen;
    unsigned short payloadLen;
    bool fixLen;
    bool crcOn;
    bool freqHopOn;
    unsigned char hopPeriod;
    bool iqInverted;
    unsigned int timeout;
}RxConfig_t;

#endif //__SX1276ENUMS_H__
