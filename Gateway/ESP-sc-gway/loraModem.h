// 1-channel LoRa Gateway for ESP8266 and ESP32
// Copyright (c) 2016-2021 Maarten Westenberg version for ESP8266
//
// based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
// and many other contributors.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)

// ----------------------------------------
#define PAYLOAD_LENGTH              0x40
#define MAX_PAYLOAD_LENGTH          0x40

#define NUM_HOPS 3

#define RSSI_LIMIT	35
#define RSSI_WAIT	6

#define EVENT_WAIT	15000
#define DONE_WAIT	1950

#define WAIT_CORRECTION	50000

#define SPISPEED 8000000

struct vector {
	uint32_t upFreq;
	uint16_t upBW;
	uint8_t  upLo;
	uint8_t  upHi;
	uint32_t dwnFreq;
	uint16_t dwnBW;
	uint8_t  dwnLo;
	uint8_t  dwnHi;
};

// Define all the relevant LoRa Regions
//=====================================
#ifdef EU863_870
vector freqs [] = {
    { 868100000, 125, 7, 12, 868100000, 125, 7, 12},
    { 868300000, 125, 7, 12, 868300000, 125, 7, 12},
    { 868500000, 125, 7, 12, 868500000, 125, 7, 12},
    { 867100000, 125, 7, 12, 867100000, 125, 7, 12},
    { 867300000, 125, 7, 12, 867300000, 125, 7, 12},
    { 867500000, 125, 7, 12, 867500000, 125, 7, 12},
    { 867700000, 125, 7, 12, 867700000, 125, 7, 12},
    { 867900000, 125, 7, 12, 867900000, 125, 7, 12},
    { 868800000, 125, 7, 12, 868800000, 125, 7, 12},
    { 0,         0  , 0,  0, 869525000, 125, 9, 9}
};

#elif defined(EU433)
vector freqs [] = {
	{ 433175000, 125, 7, 12, 433175000, 125, 7, 12},
	{ 433375000, 125, 7, 12, 433375000, 125, 7, 12},
	{ 433575000, 125, 7, 12, 433575000, 125, 7, 12},
	{ 433775000, 125, 7, 12, 433775000, 125, 7, 12},
	{ 433975000, 125, 7, 12, 433975000, 125, 7, 12},
	{ 434175000, 125, 7, 12, 434175000, 125, 7, 12},
	{ 434375000, 125, 7, 12, 434375000, 125, 7, 12},
	{ 434575000, 125, 7, 12, 434575000, 125, 7, 12},
	{ 434775000, 125, 7, 12, 434775000, 125, 7, 12}
};

#elif defined(US902_928)
vector freqs [] = {
	{ 903900000, 125, 7, 10, 923300000, 500, 7, 12},
	{ 904100000, 125, 7, 10, 923900000, 500, 7, 12},
	{ 904300000, 125, 7, 10, 924500000, 500, 7, 12},
	{ 904500000, 125, 7, 10, 925100000, 500, 7, 12},
	{ 904700000, 125, 7, 10, 925700000, 500, 7, 12},
	{ 904900000, 125, 7, 10, 926300000, 500, 7, 12},
	{ 905100000, 125, 7, 10, 926900000, 500, 7, 12},
	{ 905300000, 125, 7, 10, 927500000, 500, 7, 12},
	{ 904600000, 500, 8,  8, 0        , 0,   0, 00},
};

#elif defined(AU925_928)
vector freqs [] = {
	{ 916800000, 125, 7, 10, 916800000, 125, 7, 12},
	{ 917000000, 125, 7, 10, 917000000, 125, 7, 12},
	{ 917200000, 125, 7, 10, 917200000, 125, 7, 12},
	{ 917400000, 125, 7, 10, 917400000, 125, 7, 12},
	{ 917600000, 125, 7, 10, 917600000, 125, 7, 12},
	{ 917800000, 125, 7, 10, 917800000, 125, 7, 12},
	{ 918000000, 125, 7, 10, 918000000, 125, 7, 12},
	{ 918200000, 125, 7, 10, 918200000, 125, 7, 12},
	{ 917500000, 500, 8,  8,         0,   0, 0,  0}
};

#elif defined(CN470_510)
vector freqs [] = {
	{ 486300000, 125, 7, 12, 486300000, 125, 7, 12},
	{ 486500000, 125, 7, 12, 486500000, 125, 7, 12},
	{ 486700000, 125, 7, 12, 486700000, 125, 7, 12},
	{ 486900000, 125, 7, 12, 486900000, 125, 7, 12},
	{ 487100000, 125, 7, 12, 487100000, 125, 7, 12},
	{ 487300000, 125, 7, 12, 487300000, 125, 7, 12},
	{ 487500000, 125, 7, 12, 487500000, 125, 7, 12},
	{ 487700000, 125, 7, 12, 487700000, 125, 7, 12}
};

#elif defined(KR920)
vector freqs [] = {
	{ 922100000, 125, 7, 12, 922100000, 125, 7, 12},
	{ 922300000, 125, 7, 12, 922300000, 125, 7, 12},
	{ 922500000, 125, 7, 12, 922500000, 125, 7, 12},
	{ 922700000, 125, 7, 12, 922700000, 125, 7, 12},
	{ 922900000, 125, 7, 12, 922900000, 125, 7, 12},
	{ 923100000, 125, 7, 12, 923100000, 125, 7, 12},
	{ 923300000, 125, 7, 12, 923300000, 125, 7, 12},
};

// ===== AS923 Philippines =====
#elif defined(AS923)
vector freqs [] = {
	{ 916800000, 125, 7, 12, 916800000, 125, 7, 12},  // Channel 0, 916.8 MHz primary
	{ 917000000, 125, 7, 12, 917000000, 125, 7, 12},  // Channel 1, 917.0 MHz
	{ 917200000, 125, 7, 12, 917200000, 125, 7, 12},  // Channel 2, 917.2 MHz
	{ 917400000, 125, 7, 12, 917400000, 125, 7, 12},  // Channel 3, 917.4 MHz
	{ 917600000, 125, 7, 12, 917600000, 125, 7, 12},  // Channel 4, 917.6 MHz
	{ 917800000, 125, 7, 12, 917800000, 125, 7, 12},  // Channel 5, 917.8 MHz
	{ 918000000, 125, 7, 12, 918000000, 125, 7, 12},  // Channel 6, 918.0 MHz
	{ 918200000, 125, 7, 12, 918200000, 125, 7, 12},  // Channel 7, 918.2 MHz
	{ 917500000, 500, 8,  8,         0,   0, 0,  0}   // Channel 8, 917.5 SF8BW500
};
// =============================

#elif defined(IN865_867)
vector freqs [] = {
	{ 865062500, 125, 7, 12, 865062500, 125, 7, 12},
	{ 865402500, 125, 7, 12, 865402500, 125, 7, 12},
	{ 865985000, 125, 7, 12, 865985000, 125, 7, 12},
	{         0,   0, 0,  0, 866550000, 125, 10, 10}
};

#else
vector freqs [] = {
#	error "Sorry, but your frequency plan is not supported"
};
#endif


// Set the structure for spreading factor
enum sf_t { SF6=6, SF7, SF8, SF9, SF10, SF11, SF12 };

// The state of the receiver
enum state_t { S_INIT=0, S_SCAN, S_CAD, S_RX, S_TX, S_TXDONE};

volatile state_t _state=S_INIT;
volatile uint8_t _event=0;

uint8_t _rssi;

uint32_t msgTime=0;
uint32_t hopTime=0;
uint32_t detTime=0;

#if _PIN_OUT==1
struct pins {
	uint8_t dio0=15;
	uint8_t dio1=15;
	uint8_t dio2=15;
	uint8_t ss=16;
	uint8_t rst=0;
} pins;

#elif _PIN_OUT==2
struct pins {
	uint8_t dio0=5;
	uint8_t dio1=4;
	uint8_t dio2=0;
	uint8_t ss=15;
	uint8_t rst=0;
} pins;

#elif _PIN_OUT==3
struct pins {
	uint8_t dio0=26;
	uint8_t dio1=26;
	uint8_t dio2=26;
	uint8_t ss=18;
	uint8_t rst=14;
} pins;

#elif _PIN_OUT==4
// ===== T-Beam AXP2101 =====
struct pins {
	uint8_t dio0=26;
	uint8_t dio1=33;
	uint8_t dio2=32;
	uint8_t ss=18;
	uint8_t rst=14;    // ← T-Beam RST pin
} pins;
#define SCK  5
#define MISO 19
#define MOSI 27
#define RST  14
#define SS   18

#if _GPS==1
#define GPS_RX 15
#define GPS_TX 12
#endif

#elif _PIN_OUT==5
struct pins {
	uint8_t dio0=26;
	uint8_t dio1=35;
	uint8_t dio2=34;
	uint8_t ss=18;
	uint8_t rst=14;
} pins;
#define SCK  5
#define MISO 19
#define MOSI 27
#define RST  14
#define SS   18

#else
#error "Pin Definitions _PIN_OUT must be defined in loraModem.h"
#endif

struct stat_t {
	uint32_t time;
	uint32_t node;
	uint8_t ch;
	uint8_t sf;
#if RSSI==1
	int8_t rssi;
#endif
	int8_t prssi;
	uint8_t upDown;
#if _LOCALSERVER>=1
	uint8_t data[24];
	uint8_t datal;
#endif
} stat_t;


#if _STATISTICS >= 1
struct stat_c {
	uint32_t msg_ok;
	uint32_t msg_ttl;
	uint32_t msg_down;
	uint32_t msg_sens;

#if _STATISTICS >= 2
	uint32_t sf7, sf8, sf9;
	uint32_t sf10, sf11, sf12;

#if _STATISTICS >=3
	uint32_t msg_ok_0,   msg_ok_1,   msg_ok_2;
	uint32_t msg_ttl_0,  msg_ttl_1,  msg_ttl_2;
	uint32_t msg_down_0, msg_down_1, msg_down_2;
	uint32_t msg_sens_0, msg_sens_1, msg_sens_2;

	uint32_t  sf7_0,  sf7_1,  sf7_2;
	uint32_t  sf8_0,  sf8_1,  sf8_2;
	uint32_t  sf9_0,  sf9_1,  sf9_2;
	uint32_t sf10_0, sf10_1, sf10_2;
	uint32_t sf11_0, sf11_1, sf11_2;
	uint32_t sf12_0, sf12_1, sf12_2;
#endif

	uint16_t boots;
	uint16_t resets;
#endif

} stat_c;
struct stat_c statc;

struct stat_t * statr;

#else
struct stat_t statr[1];
#endif


uint8_t payLoad[128];


struct LoraDown {
	uint32_t tmst;
	uint32_t tmms;
	uint32_t time;
	uint32_t freq;
	uint32_t usec;
	uint16_t fcnt;
	uint8_t  size;
	uint8_t  chan;
	uint8_t  sf;
	uint8_t  bw;
	bool     ipol;
	uint8_t  powe;
	uint8_t  crc;
	uint8_t  iiq;
	uint8_t  imme;
	uint8_t  ncrc;
	uint8_t  prea;
	uint8_t  rfch;
	char *   modu;
	char *   datr;
	char *   codr;
	uint8_t* payLoad;
} LoraDown;


struct LoraUp {
	uint32_t tmst;
	uint32_t tmms;
	uint32_t time;
	uint32_t freq;
	uint8_t  size;
	uint8_t  chan;
	uint8_t  sf;
	uint16_t fcnt;
	int32_t  snr;
	int16_t  prssi;
	int16_t  rssicorr;
	char *   modu;
	uint8_t  payLoad[128];
} LoraUp;


// ============================================================================
// Register definitions
// ============================================================================
#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PAC                     0x09
#define REG_PARAMP                  0x0A
#define REG_OCP                     0x0B
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_BYTES_NB             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI                0x1A
#define REG_RSSI                    0x1B
#define REG_HOP_CHANNEL             0x1C
#define REG_MODEM_CONFIG1           0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_SYMB_TIMEOUT_LSB        0x1F
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MAX_PAYLOAD_LENGTH      0x23
#define REG_HOP_PERIOD              0x24
#define REG_FIFO_RX_BYTE_ADDR_PTR   0x25
#define REG_MODEM_CONFIG3           0x26
#define REG_PPM_CORRECTION          0x27
#define REG_FREQ_ERROR_MSB          0x28
#define REG_FREQ_ERROR_MID          0x29
#define REG_FREQ_ERROR_LSB          0x2A
#define REG_RSSI_WIDEBAND           0x2C
#define REG_DETECT_OPTIMIZE         0x31
#define REG_INVERTIQ                0x33
#define REG_DET_TRESH               0x37
#define REG_SYNC_WORD               0x39
#define REG_INVERTIQ2               0x3B
#define REG_TEMP                    0x3C
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42
#define REG_PADAC                   0x4D
#define REG_PADAC_SX1272            0x5A
#define REG_PADAC_SX1276            0x4D

// opModes
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81
#define SX72_MODE_FSTX              0x82
#define SX72_MODE_TX                0x83
#define SX72_MODE_RX_CONTINUOS      0x85

#define OPMODE_LORA                 0x80
#define OPMODE_MASK                 0x0F
#define OPMODE_LOWFREQ              0x08
#define OPMODE_SLEEP                0x00
#define OPMODE_STANDBY              0x01
#define OPMODE_FSTX                 0x02
#define OPMODE_TX                   0x03
#define OPMODE_FSRX                 0x04
#define OPMODE_RX                   0x05
#define OPMODE_RX_SINGLE            0x06
#define OPMODE_CAD                  0x07

#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0x20

#define REG1                        0x0A
#define REG2                        0x84

#define SX1276_MC1_BW_125           0x70
#define SX1276_MC1_BW_250           0x80
#define SX1276_MC1_BW_500           0x90
#define SX1276_MC1_CR_4_5           0x02
#define SX1276_MC1_CR_4_6           0x04
#define SX1276_MC1_CR_4_7           0x06
#define SX1276_MC1_CR_4_8           0x08
#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON  0x01
#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE     0x01

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

#define FRF_MSB                     0xD9
#define FRF_MID                     0x06
#define FRF_LSB                     0x66

#define MAP_DIO0_LORA_RXDONE        0x00
#define MAP_DIO0_LORA_TXDONE        0x40
#define MAP_DIO0_LORA_CADDONE       0x80
#define MAP_DIO0_LORA_NOP           0xC0
#define MAP_DIO1_LORA_RXTOUT        0x00
#define MAP_DIO1_LORA_FCC           0x10
#define MAP_DIO1_LORA_CADDETECT     0x20
#define MAP_DIO1_LORA_NOP           0x30
#define MAP_DIO2_LORA_FCC0          0x00
#define MAP_DIO2_LORA_FCC1          0x04
#define MAP_DIO2_LORA_FCC2          0x08
#define MAP_DIO2_LORA_NOP           0x0C
#define MAP_DIO3_LORA_CADDONE       0x00
#define MAP_DIO3_LORA_HEADER        0x01
#define MAP_DIO3_LORA_CRC           0x02
#define MAP_DIO3_LORA_NOP           0x03
#define MAP_DIO0_FSK_READY          0x00
#define MAP_DIO1_FSK_NOP            0x30
#define MAP_DIO2_FSK_TXNOP          0x04
#define MAP_DIO2_FSK_TIMEOUT        0x08

#define IRQ_LORA_RXTOUT_MASK        0x80
#define IRQ_LORA_RXDONE_MASK        0x40
#define IRQ_LORA_CRCERR_MASK        0x20
#define IRQ_LORA_HEADER_MASK        0x10
#define IRQ_LORA_TXDONE_MASK        0x08
#define IRQ_LORA_CDDONE_MASK        0x04
#define IRQ_LORA_FHSSCH_MASK        0x02
#define IRQ_LORA_CDDETD_MASK        0x01

#define _PROTOCOL                   0x02
#define PUSH_DATA                   0x00
#define PUSH_ACK                    0x01
#define PULL_DATA                   0x02
#define PULL_RESP                   0x03
#define PULL_ACK                    0x04
#define TX_ACK                      0x05
#define MGT_RESET                   0x15
#define MGT_SET_SF                  0x16
#define MGT_SET_FREQ                0x17

struct regs {
	const char * name;
	uint8_t regid;
	uint8_t reghex;
	uint8_t regvalue;
};

#define _REG_AMOUNT 42
regs registers [] = {
	{ "REG_FIFO",           REG_FIFO,                   1, 0 },
	{ "REG_OPMODE",         REG_OPMODE,                 1, 0 },
	{ "FRF_MSB",            REG_FRF_MSB,                1, 0 },
	{ "FRF_MID",            REG_FRF_MID,                1, 0 },
	{ "FRF_LSB",            REG_FRF_LSB,                1, 0 },
	{ "PAC",                REG_PAC,                    1, 0 },
	{ "PARAMP",             REG_PARAMP,                 1, 0 },
	{ "OCP",                REG_OCP,                    1, 0 },
	{ "LNA",                REG_LNA,                    1, 0 },
	{ "ADDR_PTR",           REG_FIFO_ADDR_PTR,          1, 0 },
	{ "TX_BASE_AD",         REG_FIFO_TX_BASE_AD,        1, 0 },
	{ "RX_BASE_AD",         REG_FIFO_RX_BASE_AD,        1, 0 },
	{ "RX_CURRENT_ADDR",    REG_FIFO_RX_CURRENT_ADDR,   1, 0 },
	{ "IRQ_FLAGS_MASK",     REG_IRQ_FLAGS_MASK,         1, 0 },
	{ "IRQ_FLAGS",          REG_IRQ_FLAGS,              1, 0 },
	{ "RX_BYTES_NB",        REG_RX_BYTES_NB,            1, 0 },
	{ "PKT_SNR_VALUE",      REG_PKT_SNR_VALUE,          1, 0 },
	{ "REG_PKT_RSSI",       REG_PKT_RSSI,               1, 0 },
	{ "REG_RSSI",           REG_RSSI,                   1, 0 },
	{ "HOP_CHANNEL",        REG_HOP_CHANNEL,            1, 0 },
	{ "MODEM_CONFIG1",      REG_MODEM_CONFIG1,          1, 0 },
	{ "MODEM_CONFIG2",      REG_MODEM_CONFIG2,          1, 0 },
	{ "SYMB_TIMEOUT_LSB",   REG_SYMB_TIMEOUT_LSB,       1, 0 },
	{ "PREAMBLE_MSB",       REG_PREAMBLE_MSB,           1, 0 },
	{ "PREAMBLE_LSB",       REG_PREAMBLE_LSB,           1, 0 },
	{ "PAYLOAD_LENGTH",     REG_PAYLOAD_LENGTH,         1, 0 },
	{ "MAX_PAYLOAD_LENGTH", REG_MAX_PAYLOAD_LENGTH,     1, 0 },
	{ "HOP_PERIOD",         REG_HOP_PERIOD,             1, 0 },
	{ "FIFO_RX_BYTE",       REG_FIFO_RX_BYTE_ADDR_PTR,  1, 0 },
	{ "MODEM_CONFIG3",      REG_MODEM_CONFIG3,          1, 0 },
	{ "PPM_CORRECTION",     REG_PPM_CORRECTION,         1, 0 },
	{ "FREQ_ERROR_MSB",     REG_FREQ_ERROR_MSB,         1, 0 },
	{ "FREQ_ERROR_MID",     REG_FREQ_ERROR_MID,         1, 0 },
	{ "FREQ_ERROR_LSB",     REG_FREQ_ERROR_LSB,         1, 0 },
	{ "RSSI_WIDEBAND",      REG_RSSI_WIDEBAND,          1, 0 }, 
	{ "DETECT_OPT",         REG_DETECT_OPTIMIZE,        1, 0 },
	{ "INVERTIQ",           REG_INVERTIQ,               1, 0 },
	{ "DET_TRESH",          REG_DET_TRESH,              1, 0 },
	{ "SYNC_WORD",          REG_SYNC_WORD,              1, 0 },
	{ "INVERTIQ2",          REG_INVERTIQ2,              1, 0 },
	{ "REG_TEMP",           REG_TEMP,                   1, 0 },
	{ "PADAC_1276",         REG_PADAC_SX1276,           1, 0 },
	{ "PADAC",              REG_PADAC,                  1, 0 }
};