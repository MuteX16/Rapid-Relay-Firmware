// 1-channel LoRa Gateway for ESP8266 and ESP32
// Copyright (c) 2016-2021 Maarten Westenberg version for ESP8266
//
// 	based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
//	and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the utilities for time and other functions
// ========================================================================================


// ======================= STRING STRING STRING ===========================================

void printInt (uint32_t i, String & response)
{
	response +=(String(i/1000000) + "." + String(i%1000000));
}

#define printReg(x) {int i=readRegister(x); if(i<=0x0F) Serial.print('0'); Serial.print(i,HEX);}

void printRegs(struct LoraDown *LoraDown, String & response)
{
	response += "v FIFO(0x00)=0x" + String(readRegister(REG_FIFO),HEX);
	response += "v OPMODE(0x01)=0x" + String(readRegister(REG_OPMODE),HEX);
	response += "v FRF(0x06)=0x" + String(readRegister(REG_FRF_MSB),HEX);  
	response += "     (0x07)=0x" + String(readRegister(REG_FRF_MID),HEX);
	response += "     (0x08)=0x" + String(readRegister(REG_FRF_LSB),HEX);
	
	response += "v PREAMBLE_MSB (0x20)=0x" + String(readRegister(REG_PREAMBLE_MSB),HEX);
	response += "v PREAMBLE_LSB (0x21)=0x" + String(readRegister(REG_PREAMBLE_LSB),HEX);
	
#	if _DUSB>=1
	if (debug>=1) {
		Serial.print("v PAC                  (0x09)\t=0x"); printReg(REG_PAC); Serial.println();
		Serial.print("v PARAMP               (0x0A)\t=0x"); printReg(REG_PARAMP); Serial.println();
		Serial.print("v REG_OCP              (0x0B)\t=0x"); printReg(REG_OCP); Serial.println();
		Serial.print("v LNA                  (0x0C)\t=0x"); printReg(REG_LNA); Serial.println();
		Serial.print("v FIFO_ADDR_PTR        (0x0D)\t=0x"); printReg(REG_FIFO_ADDR_PTR); Serial.println();
		Serial.print("v FIFO_TX/RX_BASE_AD   (0x0E/0x0F)\t=0x"); printReg(REG_FIFO_TX_BASE_AD); 
			Serial.print("/"); printReg(REG_FIFO_RX_BASE_AD); Serial.println();
		Serial.print("v FIFO_RX_CURRENT_ADDR (0x10)\t=0x"); printReg(REG_FIFO_RX_CURRENT_ADDR); Serial.println();
		Serial.print("v PREAMBLE_MSB         (0x20)\t=0x"); printReg(REG_PREAMBLE_MSB); Serial.println();
		Serial.print("v PREAMBLE_LSB         (0x21)\t=0x"); printReg(REG_PREAMBLE_LSB); Serial.println();
		Serial.print("v REG_PAYLOAD_LENGTH   (0x22)\t="); Serial.println(readRegister(REG_PAYLOAD_LENGTH));
		Serial.print("v MAX_PAYLOAD_LENGTH   (0x23)\t="); Serial.println(readRegister(REG_MAX_PAYLOAD_LENGTH));
		Serial.print("v REG_HOP_PERIOD       (0x24)\t="); Serial.println(readRegister(REG_HOP_PERIOD));
		Serial.print("v FIFO_RX_BYTE_ADDR_PTR(0x25)\t=0x"); printReg(REG_FIFO_RX_BYTE_ADDR_PTR); Serial.println();
		Serial.println("");
	}
#	endif // _DUSB

	return;
}


void printDwn(struct LoraDown *LoraDown, String & response)
{
	uint32_t i= LoraDown->tmst;
	uint32_t m= LoraDown->usec;
	
	response += "micr=";	printInt(m, response);
	response += ", tmst=";	printInt(i, response);

	response += ", wait=";
	if (i>m) {
		response += String(i-m);
	}
	else {
		response += "(";
		response += String(m-i);
		response += ")";
	}

	response += ", freq="	+String(LoraDown->freq);
	response += ", sf="		+String(LoraDown->sf);
	response += ", bw="		+String(LoraDown->bw);
	response += ", datr="	+String(LoraDown->datr);
	response += ", powe="	+String(LoraDown->powe);
	response += ", crc="	+String(LoraDown->crc);
	response += ", imme="	+String(LoraDown->imme);
	response += ", iiq="	+String(LoraDown->iiq, HEX);
	response += ", prea="	+String(LoraDown->prea);
	response += ", rfch="	+String(LoraDown->rfch);
	response += ", ncrc="	+String(LoraDown->ncrc);
	response += ", size="	+String(LoraDown->size);
	response += ", strict="	+String(_STRICT_1CH);

	response += ", a=";
	uint8_t DevAddr [4];
		DevAddr[0] = LoraDown->payLoad[4];
		DevAddr[1] = LoraDown->payLoad[3];
		DevAddr[2] = LoraDown->payLoad[2];
		DevAddr[3] = LoraDown->payLoad[1];
	printHex((IPAddress)DevAddr, ':', response);
	
	yield();
	return;
}


void printIP(IPAddress ipa, const char sep, String & response)
{
	response += (String)ipa[0] + sep;
	response += (String)ipa[1] + sep;
	response += (String)ipa[2] + sep;
	response += (String)ipa[3];
}

void printHex(uint32_t hexa, const char sep, String & response) 
{
#	if _MONITOR>=1
	if ((debug>=0) && (hexa==0)) {
		mPrint("printHex:: hexa amount to convert is 0");
	}
#	endif

	uint8_t * h = (uint8_t *)(& hexa);

	if (h[0]<016) response += '0'; response += String(h[0], HEX) + sep;
	if (h[1]<016) response += '0'; response += String(h[1], HEX) + sep;
	if (h[2]<016) response += '0'; response += String(h[2], HEX) + sep;
	if (h[3]<016) response += '0'; response += String(h[3], HEX) + sep;
}

void printHexDigit(uint8_t digit, String & response)
{
    if(digit < 0x10)
        response += '0';
    response += String(digit,HEX);
}

void mPrint(String txt) 
{
#	if _DUSB>=1
	if (gwayConfig.dusbStat>=1) {
		Serial.println(txt);
	}
#	endif //_DUSB

#if _MONITOR>=1
	time_t tt = now();
	
	monitor[iMoni].txt = "";
	stringTime(tt, monitor[iMoni].txt);
	
	monitor[iMoni].txt += "- " + String(txt);
	
	iMoni = (iMoni+1) % gwayConfig.maxMoni;

#endif //_MONITOR

	return;
}


int mStat(uint8_t intr, String & response) 
{
#	if _MONITOR>=1

	if (debug>=0) {
	
		response += "I=";

		if (intr & IRQ_LORA_RXTOUT_MASK) response += "RXTOUT ";
		if (intr & IRQ_LORA_RXDONE_MASK) response += "RXDONE ";
		if (intr & IRQ_LORA_CRCERR_MASK) response += "CRCERR ";
		if (intr & IRQ_LORA_HEADER_MASK) response += "HEADER ";
		if (intr & IRQ_LORA_TXDONE_MASK) response += "TXDONE ";
		if (intr & IRQ_LORA_CDDONE_MASK) response += "CDDONE ";
		if (intr & IRQ_LORA_FHSSCH_MASK) response += "FHSSCH ";
		if (intr & IRQ_LORA_CDDETD_MASK) response += "CDDETD ";

		if (intr == 0x00) response += "  --  ";
			
		response += ", CH=" + String(gwayConfig.ch);
		response += ", SF=" + String(sf);
		response += ", E=" + String(_event);
			
		response += ", S=";
		switch (_state) {
			case S_TXDONE: response += "S_TXDONE"; break;
			case S_INIT:   response += "S_INIT "; break;
			case S_SCAN:   response += "S_SCAN "; break;
			case S_CAD:    response += "S_CAD  "; break;
			case S_RX:     response += "S_RX   "; break;
			case S_TX:     response += "S_TX   "; break;
			default:       response += " -- ";
		}
		response += ", eT=";
		response += String( micros() - eventTime );
		
		response += ", dT=";
		response += String( micros() - doneTime );
	}
#	endif //_MONITOR
	return(1);
}


void ftoa(float f, char *val, int p) 
{
	int j=1;
	int ival, fval;
	char b[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	
	for (int i=0; i< p; i++) { j= j*10; }

	ival = (int) f;
	fval = (int) ((f- ival)*j);
	if (fval<0) fval = -fval;
	
	if ((f<0) && (ival == 0)) strcat(val, "-");
	strcat(val,itoa(ival,b,10));
	strcat(val,".");
	
	itoa(fval,b,10);
	for (unsigned int i=0; i<(p-strlen(b)); i++) {
		strcat(val,"0");
	}
	strcat(val,b);
}


void printDigits(uint32_t digits)
{
    if(digits < 10)
        Serial.print(F("0"));
    Serial.print(digits);
}


static void stringTime(time_t t, String & response) 
{
	if (t==0) { response += "--"; return; }
	
	time_t eTime = t;
	
	uint8_t _hour   = hour(eTime);
	uint8_t _minute = minute(eTime);
	uint8_t _second = second(eTime);
	uint8_t _month	= month(eTime);
	uint8_t _day 	= day(eTime);
	
	switch(weekday(eTime)) {
		case 1: response += "Sun "; break;
		case 2: response += "Mon "; break;
		case 3: response += "Tue "; break;
		case 4: response += "Wed "; break;
		case 5: response += "Thu "; break;
		case 6: response += "Fri "; break;
		case 7: response += "Sat "; break;
	}
	if (_day < 10) response += "0"; response += String(_day) + "-";
	if (_month < 10) response += "0"; response += String(_month) + "-";
	response += String(year(eTime)) + " ";	
	
	if (_hour < 10) response += "0";   response += String(_hour) + ":";
	if (_minute < 10) response += "0"; response += String(_minute) + ":";
	if (_second < 10) response += "0"; response += String(_second);
}

int sendNtpRequest(IPAddress timeServerIP) 
{
	const int NTP_PACKET_SIZE = 48;
	uint8_t packetBuffer[NTP_PACKET_SIZE];

	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	
	packetBuffer[0]  = 0b11100011;
	packetBuffer[1]  = 0;
	packetBuffer[2]  = 6;
	packetBuffer[3]  = 0xEC;
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;	

	if (!sendUdp( (IPAddress) timeServerIP, (int) 123, packetBuffer, NTP_PACKET_SIZE)) {
		gwayConfig.ntpErr++;
		gwayConfig.ntpErrTime = now();
		return(0);	
	}
	return(1);
}


int getNtpTime(time_t *t)
{
	gwayConfig.ntps++;
	
    if (!sendNtpRequest(ntpServer))
	{
#		if _MONITOR>=1
		if (debug>=0) {
			mPrint("utils:: ERROR getNtpTime: sendNtpRequest failed");
		}
#		endif //_MONITOR
		return(0);
	}
	
	const int NTP_PACKET_SIZE = 48;
	// ===== FIXED: changed byte to uint8_t to avoid ambiguity =====
	uint8_t packetBuffer[NTP_PACKET_SIZE];
	// =============================================================
	memset(packetBuffer, 0, NTP_PACKET_SIZE);

    uint32_t beginWait = millis();
	delay(10);
    while (millis() - beginWait < 1500)
	{
		int size = Udp.parsePacket();
		if ( size >= NTP_PACKET_SIZE ) {
		
			if (Udp.read(packetBuffer, NTP_PACKET_SIZE) < NTP_PACKET_SIZE) {
#				if _MONITOR>=1
				if (debug>=0) {
					mPrint("getNtpTime:: ERROR packetsize too low");
				}
#				endif //_MONITOR
				break;
			}
			else {
				uint32_t secs;
				secs  = packetBuffer[40] << 24;
				secs |= packetBuffer[41] << 16;
				secs |= packetBuffer[42] <<  8;
				secs |= packetBuffer[43];
				
				*t = (time_t)(secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
				Udp.flush();
				return(1);
			}
			Udp.flush();	
		}
		delay(100);
    }

	Udp.flush();

	gwayConfig.ntpErr++;
	gwayConfig.ntpErrTime = now();

#	if _MONITOR>=1
	if ((debug>=3) && (pdebug & P_MAIN)) {
		mPrint("getNtpTime:: WARNING read time failed");
	}
#	endif //_MONITOR

	return(0);
}


#if _NTP_INTR==1
void setupTime() 
{
  time_t t;
  getNtpTime(&t);
  setSyncProvider(t);
  setSyncInterval(_NTP_INTERVAL);
}
#endif //_NTP_INTR


int SerialName(uint32_t a, String & response)
{
#if _TRUSTED_NODES>=1
	uint8_t * in = (uint8_t *)(& a);
	uint32_t id = ((in[0]<<24) | (in[1]<<16) | (in[2]<<8) | in[3]);

	for (unsigned int i=0; i< (sizeof(nodes)/sizeof(nodex)); i++) {
		if (id == nodes[i].id) {
#			if _MONITOR>=1
			if ((debug>=3) && (pdebug & P_MAIN )) {
				mPrint("SerialName:: i="+String(i)+", Name="+String(nodes[i].nm)+". for node=0x"+String(nodes[i].id,HEX));
			}
#			endif //_MONITOR
			response += nodes[i].nm;
			return(i);
		}
	}
#endif //_TRUSTED_NODES
	return(-1);
}


#if _LOCALSERVER>=1
int inDecodes(char * id) {
	uint32_t ident = ((id[3]<<24) | (id[2]<<16) | (id[1]<<8) | id[0]);
	for (unsigned int i=0; i< (sizeof(decodes)/sizeof(codex)); i++) {
		if (ident == decodes[i].id) {
			return(i);
		}
	}
	return(-1);
}
#endif //_LOCALSERVER


void die(String s)
{
#	if _MONITOR>=1
	mPrint(s);
#	endif //_MONITOR

#	if _DUSB>=1
	Serial.println(s);
	if (debug>=2) Serial.flush();
#	endif //_DUSB

	delay(50);
	abort();
}


void gway_failed(const char *file, uint16_t line) {
#if  _MONITOR>=1
	String response = "Program failed in file: ";
	response += String(file);
	response += ", line: ";
	response += String(line);
	mPrint(response);
#endif //_MONITOR
}