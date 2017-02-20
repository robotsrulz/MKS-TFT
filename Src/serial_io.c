
#include <stdint.h>
#include "stm32f1xx_hal.h"

static uint16_t numChars = 0;
static uint8_t checksum = 0;
static unsigned int lineNumber = 0;

static void rawSendChar(char c);
static void sendChar(char c);
static void sendCharAndChecksum(char c);

extern UART_HandleTypeDef huart2;

static void rawSendChar(char c)
{
    HAL_UART_Transmit(&huart2, &c, 1, 1000);
}

static void sendCharAndChecksum(char c)
{
    checksum ^= c;
    rawSendChar(c);
    ++numChars;
}

void sendInt(int i)
{
    if (i < 0)
    {
        sendChar('-');
        i = -i;
    }
    if (i >= 10)
    {
        sendInt(i/10);
        i %= 10;
    }
    sendChar((char)((char)i + '0'));
}

static void sendChar(char c)
{
    if (c == '\n')
    {
        if (numChars != 0)
        {
            // Send the checksum
            rawSendChar('*');
            char digit0 = checksum % 10 + '0';
            checksum /= 10;
            char digit1 = checksum % 10 + '0';
            checksum /= 10;
            if (checksum != 0)
            {
                rawSendChar(checksum + '0');
            }
            rawSendChar(digit1);
            rawSendChar(digit0);
        }
        rawSendChar(c);
        numChars = 0;
    }
    else
    {
        if (numChars == 0)
        {
            checksum = 0;
            // Send a dummy line number
            sendCharAndChecksum('N');
            sendInt(lineNumber++);			// numChars is no longer zero, so only recurses once
            sendCharAndChecksum(' ');
        }
        sendCharAndChecksum(c);
    }
}
