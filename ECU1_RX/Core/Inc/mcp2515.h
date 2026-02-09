#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f4xx_hal.h"

/* ================= MCP2515 COMMANDS ================= */
#define MCP_RESET          0xC0
#define MCP_READ           0x03
#define MCP_WRITE          0x02
#define MCP_BITMOD         0x05
#define MCP_READ_STATUS    0xA0
#define MCP_RTS_TX0        0x81
#define MCP_LOAD_TX0       0x40
#define MCP_READ_RX0       0x90

/* ================= MCP2515 REGISTERS ================= */
#define CANCTRL            0x0F
#define CANSTAT            0x0E
#define CNF1               0x2A
#define CNF2               0x29
#define CNF3               0x28
#define CANINTE            0x2B
#define CANINTF            0x2C
#define RXB0CTRL           0x60

/* ================= MODES ================= */
#define MODE_NORMAL        0x00
#define MODE_CONFIG        0x80

/* ================= CS MACROS ================= */
#define MCP_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MCP_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* ================= FUNCTIONS ================= */
void MCP2515_Init(void);
void MCP2515_Reset(void);

void MCP2515_SendMessage(uint16_t id, uint8_t *data, uint8_t len);
uint8_t MCP2515_ReceiveMessage(uint16_t *id, uint8_t *data);

#endif
