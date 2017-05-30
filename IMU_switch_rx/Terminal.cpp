#include <Arduino.h>
#include <stdlib.h>
#include "Terminal.h"
#include "Trace.h"

#define CMD_BUF_SIZE 64

//defined in app specific code
void terminalCmdHandler(char *cmd);

//
void processTerminalCmd(char *cmdStr)
{
	trace("\r\n"); //go to next line
	
	char *cmd = strtok(cmdStr, " \r\n");
	if (cmd)
		terminalCmdHandler(cmd);
		
	trace(":"); //show next prompt
}

//
void processSerial()
{
	static char cmdBuf[CMD_BUF_SIZE];
	static char cmdBufIdx = 0;
	
	if (!Serial.available())
		return;
	
	char c = Serial.read();
	//Serial.println((uint8_t)c);
	
	if (!(
		(c >= 'a' && c <= 'z') ||
		(c >= 'A' && c <= 'Z') ||
		(c >= '0' && c <= '9') ||
		strchr(" _.-\r\n\x08\x7F", c)
	))
	{
		//tracef("<%d>", (uint8_t)c);
		return;
	}
		
	if (c == '\x08' || c == '\x7F')
	{
		if (cmdBufIdx > 0)
		{
			Serial.print("\x08 \x08");
			cmdBufIdx--;
		}
		return;
	}

	if (cmdBufIdx == CMD_BUF_SIZE - 1)
		return;
	
	Serial.write(c); //local echo
	
	if (c == '\r' || c == '\n')
	{
		cmdBuf[cmdBufIdx++] = 0; //null-terminate
		processTerminalCmd(cmdBuf);
		cmdBufIdx = 0;
		return;
	}
	cmdBuf[cmdBufIdx++] = c;
}
