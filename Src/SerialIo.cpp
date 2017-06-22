/*
 * SerialIo.cpp
 *
 * Created: 09/11/2014 09:20:26
 *  Author: David
 */


#include "ecv.h"
#include "stm32f1xx_hal.h"
#include "SerialIo.h"
#include "Vector.h"
#include "PanelDue.h"

extern UART_HandleTypeDef huart2;

extern "C" void Error_Handler(void);

// Receive data processing
#define MAXCOMM1SIZE    0x0800u             // Biggest string the user will type

static uint8_t       comm1RxString[MAXCOMM1SIZE]; // where we build our string from characters coming in
static String<0xffu> comm1TxString;
static uint32_t      comm1RxStringPtr = 0;

namespace SerialIo
{
	static unsigned int lineNumber = 0;

	const char* array trGrave =			"A\xC0" "E\xC8" "I\xCC"         "O\xD2" "U\xD9" "a\xE0" "e\xE8" "i\xEC" "o\xF2" "u\xF9"        ;
	const char* array trAcute =			"A\xC1" "E\xC9" "I\xCD"         "O\xD3" "U\xDA" "a\xE1" "e\xE9" "i\xED" "o\xF3" "u\xFA" "y\xFD";
	const char* array trCircumflex =	"A\xC2" "E\xCA" "I\xCE"         "O\xD4" "U\xDB" "a\xE2" "e\xEA" "i\xEE" "o\xF4" "u\xFB"        ;
	const char* array trTilde =			"A\xC3"                 "N\xD1" "O\xD5"         "a\xE3"                 "o\xF5"                ;
	const char* array trUmlaut =		"A\xC4" "E\xCB" "I\xCF"         "O\xD6" "U\xDC" "a\xE4" "e\xEB" "i\xEF" "o\xF6" "u\xFC" "y\xFF";
	const char* array trCircle =		"A\xC5"                                         "a\xE5"                                        ;
	const char* array trCedilla =		"C\xC7" "c\xE7";

	// Initialize the serial I/O subsystem, or re-initialize it with a new baud rate
	uint8_t checksum = 0;

	// Send a character to the 3D printer.
	// A typical command string is only about 12 characters long, which at 115200 baud takes just over 1ms to send.
	// So there is no particular reason to use interrupts, and by so doing so we avoid having to handle buffer full situations.

	void Init(uint32_t baudRate)
	{
	    /* USART2 init function */

        huart2.Instance = USART2;
        huart2.Init.BaudRate = baudRate;
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        huart2.Init.StopBits = UART_STOPBITS_1;
        huart2.Init.Parity = UART_PARITY_NONE;
        huart2.Init.Mode = UART_MODE_TX_RX;
        huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart2.Init.OverSampling = UART_OVERSAMPLING_16;

        if (HAL_UART_Init(&huart2) != HAL_OK)
            Error_Handler();

        SendChar('\n');

        __HAL_UART_FLUSH_DRREGISTER(&huart2);
        HAL_UART_Receive_DMA(&huart2, comm1RxString, MAXCOMM1SIZE);
	}

	void FlushTxBuffer()
	{
        HAL_UART_Transmit(&huart2, (uint8_t *)comm1TxString.c_ptr(), comm1TxString.size(), 1000);
        comm1TxString.clear();
	}

	void SendCharAndChecksum(char c)
	{
		checksum ^= c;
		comm1TxString.add(c);

		if (comm1TxString.full())
		    FlushTxBuffer();
	}

	void SendChar(char c)
	{
		if (c == '\n')
		{
			if (comm1TxString.size())
			{
				// Send the checksum
				comm1TxString.add('*');
                if (comm1TxString.full())
                    FlushTxBuffer();

				char digit0 = checksum % 10 + '0';
				checksum /= 10;
				char digit1 = checksum % 10 + '0';
				checksum /= 10;
				if (checksum != 0)
				{
					comm1TxString.add(checksum + '0');
                    if (comm1TxString.full())
                        FlushTxBuffer();
				}
				comm1TxString.add(digit1);
                if (comm1TxString.full())
                    FlushTxBuffer();

				comm1TxString.add(digit0);
                if (comm1TxString.full())
                    FlushTxBuffer();
			}
			comm1TxString.add(c);
			FlushTxBuffer();
		}
		else
		{
			if (!comm1TxString.size())
			{
				checksum = 0;
				// Send a dummy line number
				SendCharAndChecksum('N');
				SendInt(lineNumber++);
				SendCharAndChecksum(' ');
			}
			SendCharAndChecksum(c);
		}
	}

	void SendString(const char * array s)
	{
		while (*s != 0)
		{
			SendChar(*s++);
		}
	}

	void SendFilename(const char * array dir, const char * array name)
	{
		if (*dir != 0)
		{
			// We have a directory, so send it followed by '/' if necessary
			char c;
			while ((c = *dir) != 0)
			{
				SendChar(c);
				++dir;
			}
			if (c != '/')
			{
				SendChar('/');
			}

		}
		SendString(name);
	}

	void SendInt(int i)
	decrease(i < 0; i)
	{
		if (i < 0)
		{
			SendChar('-');
			i = -i;
		}
		if (i >= 10)
		{
			SendInt(i/10);
			i %= 10;
		}
		SendChar((char)((char)i + '0'));
	}

	// Enumeration to represent the json parsing state.
	// We don't allow nested objects or nested arrays, so we don't need a state stack.
	// An additional variable elementCount is 0 if we are not in an array, else the number of elements we have found (including the current one)
	enum JsonState
	{
		jsBegin,			// initial state, expecting '{'
		jsExpectId,			// just had '{' so expecting a quoted ID
		jsId,				// expecting an identifier, or in the middle of one
		jsHadId,			// had a quoted identifier, expecting ':'
		jsVal,				// had ':', expecting value
		jsStringVal,		// had '"' and expecting or in a string value
		jsStringEscape,		// just had backslash in a string
		jsIntVal,			// receiving an integer value
		jsNegIntVal,		// had '-' so expecting a integer value
		jsFracVal,			// receiving a fractional value
		jsEndVal,			// had the end of a string or array value, expecting comma or ] or }
		jsError				// something went wrong
	};

	JsonState state = jsBegin;

	String<20> fieldId;
	String<100> fieldVal;
	int arrayElems = -1;

	static void ProcessField()
	{
		ProcessReceivedValue(fieldId.c_str(), fieldVal.c_str(), arrayElems);
		fieldVal.clear();
	}

	static void EndArray()
	{
		ProcessArrayLength(fieldId.c_str(), arrayElems);
		arrayElems = -1;
	}

	// Look for combining characters in the string value and convert them if possible
	static void ConvertUnicode()
	{
		unsigned int numContinuationBytesLeft = 0;
		uint32_t charVal;
		for (size_t i = 0; i < fieldVal.size(); )
		{
			const unsigned char c = fieldVal[i++];
			if (numContinuationBytesLeft == 0)
			{
				if (c >= 0x80)
				{
					if ((c & 0xE0) == 0xC0)
					{
						charVal = (uint32_t)(c & 0x1F);
						numContinuationBytesLeft = 1;
					}
					else if ((c & 0xF0) == 0xE0)
					{
						charVal = (uint32_t)(c & 0x0F);
						numContinuationBytesLeft = 2;
					}
					else if ((c & 0xF8) == 0xF0)
					{
						charVal = (uint32_t)(c & 0x07);
						numContinuationBytesLeft = 3;
					}
					else if ((c & 0xFC) == 0xF8)
					{
						charVal = (uint32_t)(c & 0x03);
						numContinuationBytesLeft = 4;
					}
					else if ((c & 0xFE) == 0xFC)
					{
						charVal = (uint32_t)(c & 0x01);
						numContinuationBytesLeft = 5;
					}
				}
			}
			else if ((c & 0xC0) == 0x80)
			{
				charVal = (charVal << 6) | (c & 0x3F);
				--numContinuationBytesLeft;
				if (numContinuationBytesLeft == 0)
				{
					const char* array trtab;
					switch(charVal)
					{
					case 0x0300:	// grave accent
						trtab = trGrave;
						break;
					case 0x0301:	// acute accent
						trtab = trAcute;
						break;
					case 0x0302:	// circumflex
						trtab = trCircumflex;
						break;
					case 0x0303:	// tilde
						trtab = trTilde;
						break;
					case 0x0308:	// umlaut
						trtab = trUmlaut;
						break;
					case 0x030A:	// small circle
						trtab = trCircle;
						break;
					case 0x327:		// cedilla
						trtab = trCedilla;
						break;
					default:
						trtab = nullptr;
						break;
					}

					// If it is a diacritical mark that we handle, try to combine it with the previous character.
					// The diacritical marks are in the range 03xx so they are encoded as 2 UTF8 bytes.
					if (trtab != nullptr && i > 2)
					{
						const char c2 = fieldVal[i - 3];
						while (*trtab != 0 && *trtab != c2)
						{
							trtab += 2;
						}
						if (*trtab != 0)
						{
							// Get he translated character and encode it as 2 ITF8 bytes
							const unsigned char c3 = trtab[1];
							fieldVal[i - 3] = (c3 >> 6) | 0xC0;
							fieldVal[i - 2] = (c3 & 0x3F) | 0x80;
							fieldVal.erase(i - 1);
							--i;
						}
					}
				}
			}
			else
			{
				// Bad UTF8 state
				numContinuationBytesLeft = 0;
			}
		}
	}

	void CheckInput()
	{
        if (huart2.hdmarx && huart2.hdmarx->Instance) {
            while (comm1RxStringPtr != ((MAXCOMM1SIZE - huart2.hdmarx->Instance->CNDTR) & (MAXCOMM1SIZE - 1))) {

                char c = comm1RxString[comm1RxStringPtr++];
                comm1RxStringPtr &= (MAXCOMM1SIZE - 1);

                if (c == '\n')
                {
                    state = jsBegin;		// abandon current parse (if any) and start again
                }
                else
                {
                    switch(state)
                    {
                    case jsBegin:			// initial state, expecting '{'
                        if (c == '{')
                        {
                            StartReceivedMessage();
                            state = jsExpectId;
                            fieldVal.clear();
                        }
                        break;

                    case jsExpectId:		// expecting a quoted ID
                        switch (c)
                        {
                        case ' ':
                            break;
                        case '"':
                            fieldId.clear();
                            state = jsId;
                            break;
                        case '}':
                            EndReceivedMessage();
                            state = jsBegin;
                            break;
                        default:
                            state = jsError;
                            break;
                        }
                        break;

                    case jsId:				// expecting an identifier, or in the middle of one
                        switch (c)
                        {
                        case '"':
                            state = jsHadId;
                            break;
                        default:
                            if (c >= ' ' && !fieldId.full())
                            {
                                fieldId.add(c);
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        }
                        break;

                    case jsHadId:			// had a quoted identifier, expecting ':'
                        switch(c)
                        {
                        case ':':
                            arrayElems = -1;
                            state = jsVal;
                            break;
                        case ' ':
                            break;
                        default:
                            state = jsError;
                            break;
                        }
                        break;

                    case jsVal:				// had ':' or ':[', expecting value
                        switch(c)
                        {
                        case ' ':
                            break;
                        case '"':
                            fieldVal.clear();
                            state = jsStringVal;
                            break;
                        case '[':
                            if (arrayElems == -1)	// if not already readuing an array
                            {
                                arrayElems = 0;		// start an array
                            }
                            else
                            {
                                state = jsError;	// we don't support nested arrays
                            }
                            break;
                        case ']':
                            if (arrayElems == 0)
                            {
                                EndArray();			// empty array
                                state = jsEndVal;
                            }
                            else
                            {
                                state = jsError;	// ']' received without a matching '[' first
                            }
                            break;
                        case '-':
                            fieldVal.clear();
                            fieldVal.add(c);
                            state = jsNegIntVal;
                            break;
                        default:
                            if (c >= '0' && c <= '9')
                            {
                                fieldVal.clear();
                                fieldVal.add(c);
                                state = jsIntVal;
                                break;
                            }
                            else
                            {
                                state = jsError;
                            }
                        }
                        break;

                    case jsStringVal:		// just had '"' and expecting a string value
                        switch (c)
                        {
                        case '"':
                            ConvertUnicode();
                            ProcessField();
                            state = jsEndVal;
                            break;
                        case '\\':
                            state = jsStringEscape;
                            break;
                        default:
                            if (c < ' ')
                            {
                                state = jsError;
                            }
                            else if (!fieldVal.full())
                            {
                                fieldVal.add(c);
                            }
                            break;
                        }
                        break;

                    case jsStringEscape:	// just had backslash in a string
                        if (!fieldVal.full())
                        {
                            switch (c)
                            {
                            case '"':
                            case '\\':
                            case '/':
                                fieldVal.add(c);
                                break;
                            case 'n':
                            case 't':
                                fieldVal.add(' ');		// replace newline and tab by space
                                break;
                            case 'b':
                            case 'f':
                            case 'r':
                            default:
                                break;
                            }
                        }
                        state = jsStringVal;
                        break;

                    case jsNegIntVal:		// had '-' so expecting a integer value
                        if (c >= '0' && c <= '9')
                        {
                            fieldVal.add(c);
                            state = jsIntVal;
                        }
                        else
                        {
                            state = jsError;
                        }
                        break;

                    case jsIntVal:			// receiving an integer value
                        switch(c)
                        {
                        case '.':
                            if (fieldVal.full())
                            {
                                state = jsError;
                            }
                            else
                            {
                                fieldVal.add(c);
                                state = jsFracVal;
                            }
                            break;
                        case ',':
                            ProcessField();
                            if (arrayElems >= 0)
                            {
                                ++arrayElems;
                                state = jsVal;
                            }
                            else
                            {
                                state = jsExpectId;
                            }
                            break;
                        case ']':
                            if (arrayElems >= 0)
                            {
                                ProcessField();
                                ++arrayElems;
                                EndArray();
                                state = jsEndVal;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        case '}':
                            if (arrayElems == -1)
                            {
                                ProcessField();
                                EndReceivedMessage();
                                state = jsBegin;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        default:
                            if (c >= '0' && c <= '9' && !fieldVal.full())
                            {
                                fieldVal.add(c);
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        }
                        break;

                    case jsFracVal:			// receiving a fractional value
                        switch(c)
                        {
                        case ',':
                            ProcessField();
                            if (arrayElems >= 0)
                            {
                                ++arrayElems;
                                state = jsVal;
                            }
                            else
                            {
                                state = jsExpectId;
                            }
                            break;
                        case ']':
                            if (arrayElems >= 0)
                            {
                                ProcessField();
                                ++arrayElems;
                                EndArray();
                                state = jsEndVal;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        case '}':
                            if (arrayElems == -1)
                            {
                                ProcessField();
                                EndReceivedMessage();
                                state = jsBegin;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        default:
                            if (c >= '0' && c <= '9' && !fieldVal.full())
                            {
                                fieldVal.add(c);
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        }
                        break;

                    case jsEndVal:			// had the end of a string or array value, expecting comma or ] or }
                        switch (c)
                        {
                        case ',':
                            if (arrayElems >= 0)
                            {
                                ++arrayElems;
                                fieldVal.clear();
                                state = jsVal;
                            }
                            else
                            {
                                state = jsExpectId;
                            }
                            break;
                        case ']':
                            if (arrayElems >= 0)
                            {
                                ++arrayElems;
                                EndArray();
                                state = jsEndVal;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        case '}':
                            if (arrayElems == -1)
                            {
                                EndReceivedMessage();
                                state = jsBegin;
                            }
                            else
                            {
                                state = jsError;
                            }
                            break;
                        default:
                            break;
                        }
                        break;

                    case jsError:
                        // Ignore all characters. State will be reset to jsBegin at the start of this function when we receive a newline.
                        break;
                    }
                }
            }
        }
	}
};

// End
