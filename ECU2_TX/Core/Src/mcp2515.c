#include "mcp2515.h"

extern SPI_HandleTypeDef hspi1;

/* ================= SPI LOW LEVEL ================= */

/* Write one byte */
static void SPI_Write(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

/* Read one byte (Transmit dummy + Receive) */
static uint8_t SPI_Read(void)
{
    uint8_t tx = 0xFF;
    uint8_t rx = 0;

    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);

    return rx;
}

/* ================= MCP2515 CORE ================= */

void MCP2515_Reset(void)
{
    MCP_CS_LOW();
    SPI_Write(MCP_RESET);
    MCP_CS_HIGH();

    HAL_Delay(10);
}

/* Write MCP register */
static void MCP2515_WriteRegister(uint8_t reg, uint8_t value)
{
    MCP_CS_LOW();

    SPI_Write(MCP_WRITE);
    SPI_Write(reg);
    SPI_Write(value);

    MCP_CS_HIGH();
}

/* Read MCP register */
static uint8_t MCP2515_ReadRegister(uint8_t reg)
{
    uint8_t value;

    MCP_CS_LOW();

    SPI_Write(MCP_READ);
    SPI_Write(reg);
    value = SPI_Read();

    MCP_CS_HIGH();

    return value;
}

/* Modify MCP register bits */
static void MCP2515_BitModify(uint8_t reg, uint8_t mask, uint8_t data)
{
    MCP_CS_LOW();

    SPI_Write(MCP_BITMOD);
    SPI_Write(reg);
    SPI_Write(mask);
    SPI_Write(data);

    MCP_CS_HIGH();
}

/* ================= INIT ================= */

void MCP2515_Init(void)
{
    MCP2515_Reset();

    /* Enter Config Mode */
    MCP2515_WriteRegister(CANCTRL, MODE_CONFIG);
    HAL_Delay(10);

    /* Baudrate: 500kbps @ 8MHz crystal */
    MCP2515_WriteRegister(CNF1, 0x00);
    MCP2515_WriteRegister(CNF2, 0xD1);
    MCP2515_WriteRegister(CNF3, 0x81);

    /* Enable RX0 Interrupt */
    MCP2515_WriteRegister(CANINTE, 0x01);

    /* Accept all messages */
    MCP2515_WriteRegister(RXB0CTRL, 0x60);

    /* Normal Mode */
    MCP2515_WriteRegister(CANCTRL, MODE_NORMAL);
}

/* ================= TRANSMIT ================= */

void MCP2515_SendMessage(uint16_t id, uint8_t *data, uint8_t len)
{
    if(len > 8) len = 8;

    /* Load TX Buffer 0 */
    MCP_CS_LOW();

    SPI_Write(MCP_LOAD_TX0);

    /* Standard ID */
    SPI_Write(id >> 3);     // SIDH
    SPI_Write(id << 5);     // SIDL
    SPI_Write(0x00);        // EID8
    SPI_Write(0x00);        // EID0

    /* DLC */
    SPI_Write(len & 0x0F);

    /* Data */
    for(uint8_t i = 0; i < len; i++)
    {
        SPI_Write(data[i]);
    }

    MCP_CS_HIGH();

    /* Request to Send */
    MCP_CS_LOW();
    SPI_Write(MCP_RTS_TX0);
    MCP_CS_HIGH();
}

/* ================= RECEIVE ================= */

uint8_t MCP2515_ReceiveMessage(uint16_t *id, uint8_t *data)
{
    uint8_t len;

    /* Read RX Buffer 0 */
    MCP_CS_LOW();

    SPI_Write(MCP_READ_RX0);

    /* Read ID */
    *id  = SPI_Read() << 3;
    *id |= SPI_Read() >> 5;

    /* Skip EID */
    SPI_Read();
    SPI_Read();

    /* Read DLC */
    len = SPI_Read() & 0x0F;

    if(len > 8) len = 8;

    /* Read Data */
    for(uint8_t i = 0; i < len; i++)
    {
        data[i] = SPI_Read();
    }

    MCP_CS_HIGH();

    /* Clear RX0 & RX1 Interrupt Flags */
    MCP2515_BitModify(CANINTF, 0x03, 0x00);

    return len;
}
