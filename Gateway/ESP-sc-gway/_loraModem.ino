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
// This file contains the LoRa modem specific code enabling to receive
// and transmit packages/messages.
// The functions implemented work in user-space so not with interrupt.
// ========================================================================================


// ===== T-Beam AXP2101 Power Management =====
// The T-Beam board uses the AXP2101 PMIC to control power to the SX1276.
// Without enabling the LoRa power rail, the radio chip is completely off
// and SPI reads return garbage (wrong version register).
// Call enableLoRaPower() ONCE from setup() only — not from initLoraModem()
// ===========================================
#include <Wire.h>

void enableLoRaPower() {
  Wire.begin(21, 22);                         // T-Beam I2C SDA=21, SCL=22

  // Set ALDO3 voltage to 3.3V (register 0x94 on AXP2101)
  Wire.beginTransmission(0x34);               // AXP2101 I2C address
  Wire.write(0x94);                           // ALDO3 voltage register
  Wire.write(0x1F);                           // 3.3V = 0x1F
  Wire.endTransmission();
  delay(10);

  // Enable ALDO3 output (register 0x90 bit 2)
  Wire.beginTransmission(0x34);
  Wire.write(0x90);
  Wire.endTransmission();

  Wire.requestFrom(0x34, 1);
  uint8_t val = 0;
  if (Wire.available()) val = Wire.read();

  Wire.beginTransmission(0x34);
  Wire.write(0x90);
  Wire.write(val | 0x04);                     // Set bit 2 to enable ALDO3
  Wire.endTransmission();
  delay(200);                                 // Wait for power rail to stabilise

  Serial.println("AXP2101: LoRa power rail enabled");
}
// ============================================


// ----------------------------------------------------------------------------------------
// Mutex definitions
// ----------------------------------------------------------------------------------------
#if _MUTEX==1
	void CreateMutux(int *mutex) {
		*mutex=1;
	}

#define LIB__MUTEX 1

#if LIB__MUTEX==1
	bool GetMutex(int *mutex) {
		if (*mutex==1) { 
			*mutex=0; 
			return(true); 
		}
		return(false);
	}
#else	
	bool GetMutex(int *mutex) {
	int iOld = 1, iNew = 0;
	asm volatile (
		"rsil a15, 1\n"
		"l32i %0, %1, 0\n"
		"bne %0, %2, 1f\n"
		"s32i %3, %1, 0\n"
		"1:\n"
		"wsr.ps a15\n"
		"rsync\n"
		: "=&r" (iOld)
		: "r" (mutex), "r" (iOld), "r" (iNew)
		: "a15", "memory"
	);
	return (bool)iOld;
}
#endif

	void ReleaseMutex(int *mutex) {
		*mutex=1;
	}
	
#endif //_MUTEX==1


// ----------------------------------------------------------------------------------------
// Read one byte value, par addr is address
// ----------------------------------------------------------------------------------------
uint8_t readRegister(uint8_t addr)
{
    digitalWrite(pins.ss, LOW);
	SPI.transfer(addr & 0x7F);
	uint8_t res = (uint8_t) SPI.transfer(0x00);
    digitalWrite(pins.ss, HIGH);
    return((uint8_t) res);
}


// ----------------------------------------------------------------------------------------
// Write value to a register with address addr. 
// ----------------------------------------------------------------------------------------
void writeRegister(uint8_t addr, uint8_t value)
{
	noInterrupts();
	digitalWrite(pins.ss, LOW);
	SPI.transfer((int8_t)((addr | 0x80) & 0xFF));
	SPI.transfer(value & 0xFF);
    digitalWrite(pins.ss, HIGH);
	interrupts();
}


// ----------------------------------------------------------------------------------------
// Write a buffer to a register with address addr. 
// ----------------------------------------------------------------------------------------
void writeBuffer(uint8_t addr, uint8_t *buf, uint8_t len)
{
	noInterrupts();
	digitalWrite(pins.ss, LOW);

	if (len >= 24) {
		len=24;
		mPrint("WARNING writeBuffer:: len > 24");
	}

	SPI.transfer((int8_t)(addr | 0x80) );

#	if _BUF_WRITE==1
		SPI.transfer((uint8_t *) buf, len);
#	else
		for (int i=0; i< len; i++) {
			SPI.transfer(buf[i]);
		}
#	endif

    digitalWrite(pins.ss, HIGH);
	interrupts();	
	
#	if _MONITOR>=1
	if ((debug>=1) && (pdebug & P_TX)) {
		mPrint("v writeBuffer:: payLength=0x"+String(len,HEX)+", len FIFO=0x"+String((int8_t)(readRegister(REG_FIFO_ADDR_PTR)-readRegister(REG_FIFO_TX_BASE_AD)),HEX) );
	}
	if ((debug>=1) && (pdebug & P_MAIN)) {
		String response = "v writeBuffer:: after  : buf=";
		for (int j=0; j<len; j++) {
			response += String(buf[j],HEX)+" ";
		}
		mPrint(response);
	}
#	endif
}


// ----------------------------------------------------------------------------------------
// setRate
// ----------------------------------------------------------------------------------------
void setRate(uint8_t sf, uint8_t crc) 
{
	uint8_t mc1=0, mc2=0, mc3=0;

	if (sf<SF7) sf=7;
	else if (sf>SF12) sf=12;

    if (sx1276) {
		mc1= 0x72;
		mc2= (sf<<4) | crc;
		mc3= 0x00;
        if (sf == SF11 || sf == SF12) { 
			mc3|= 0x01;
		}
    }
	else {
		mc1= 0x0A;
		mc2= ((sf<<4) | crc) % 0xFF;
        if (sf == SF11 || sf == SF12) { 
			mc1= 0x0B; 
		}
#		if _MONITOR>=1
		if ((debug>=1) &&(pdebug & P_MAIN)) {
			mPrint("WARNING, sx1272 selected");
		}
#		endif
    }
	
	writeRegister(REG_MODEM_CONFIG1, (uint8_t) mc1);
	writeRegister(REG_MODEM_CONFIG2, (uint8_t) mc2);
	writeRegister(REG_MODEM_CONFIG3, (uint8_t) mc3);
	
    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t) 0x05);
    } 
	else {
        writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t) 0x08);
    }
	return;
}


// ----------------------------------------------------------------------------------------
// setFreq
// ----------------------------------------------------------------------------------------
void setFreq(uint32_t freq)
{
    uint32_t temp_bytes = (((uint64_t)freq << 19) / 32000000) & 0x00FFFFFF;	
    writeRegister(REG_FRF_MSB, ((uint8_t)(temp_bytes>>16)) & 0xFF );
    writeRegister(REG_FRF_MID, ((uint8_t)(temp_bytes>> 8)) & 0xFF );
    writeRegister(REG_FRF_LSB, ((uint8_t)(temp_bytes>> 0)) & 0xFF );
	return;
}


// ----------------------------------------------------------------------------------------
// setPow
// ----------------------------------------------------------------------------------------
void setPow(uint8_t pow)
{
	uint8_t pac = 0x00;
	
	if (pow>17) {	
		pac = 0xFF;
	}
	else if (pow<2) {
		pac = 0x42;
	}
	else if (pow<=12) {
		pac=0x40+pow;
	}
	else {
		pac=0x70+pow;
	}

#	if _MONITOR>=1
	if ((debug>=2) && ( pdebug & P_MAIN)) {
		mPrint("v setPow:: pow=0x"+String(pow,HEX)+", pac=0x"+String(pac,HEX) );
	}
#	endif

	// ASSERT removed: pac=0x70+pow gives lower nibble 0x00 for pow=16, false fail on ESP32
	// ASSERT(((pac&0x0F)>=2) &&((pac&0x0F)<=20));
	
	writeRegister(REG_PAC, (uint8_t) pac);
	return;
}


// ----------------------------------------------------------------------------------------
// opmode
// ----------------------------------------------------------------------------------------
void opmode(uint8_t mode)
{
	if (mode == OPMODE_LORA) {
#		ifdef CFG_sx1276_radio
#		endif
		writeRegister(REG_OPMODE, 0xFF & (uint8_t) mode );
	}
	else {
		writeRegister(REG_OPMODE, 0xFF & (uint8_t)((readRegister(REG_OPMODE) & 0x80) | mode));
	}
}


// ----------------------------------------------------------------------------------------
// hop
// ----------------------------------------------------------------------------------------
void hop() 
{
	opmode(OPMODE_STANDBY);
	gwayConfig.ch = (gwayConfig.ch + 1) % NUM_HOPS;
	setFreq(freqs[gwayConfig.ch].upFreq);
	sf = SF7;
	setRate(sf, 0x04);
	writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);
	writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);
	writeRegister(REG_INVERTIQ,0x27);
	writeRegister(REG_MAX_PAYLOAD_LENGTH,MAX_PAYLOAD_LENGTH);
	writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
	writeRegister(REG_FIFO_ADDR_PTR,(uint8_t)readRegister(REG_FIFO_RX_BASE_AD));
	writeRegister(REG_HOP_PERIOD,0x00);
	writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08);
	writeRegister(REG_IRQ_FLAGS_MASK, 0x00);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
	
#	if _MONITOR>=1
	if ((debug>=2) && (pdebug & P_RADIO)){
			String response = "hop:: hopTime:: " + String(micros() - hopTime);
			mStat(0, response);
			mPrint(response);
	}
#	endif
	hopTime = micros();
} //hop


// ----------------------------------------------------------------------------------------
// receivePkt
// ----------------------------------------------------------------------------------------
uint8_t receivePkt(uint8_t *payload)
{
    statc.msg_ttl++;

    uint8_t irqflags = readRegister(REG_IRQ_FLAGS);
	uint8_t crcUsed  = readRegister(REG_HOP_CHANNEL);
	if (crcUsed & 0x40) {
#		if _MONITOR>=1
		if (( debug>=2) && (pdebug & P_RX )) {
			mPrint("R rxPkt:: CRC used");
		}
#		endif
	}

    if (irqflags & IRQ_LORA_CRCERR_MASK)
    {
#		if _MONITOR>=1
        if ((debug>=0) && (pdebug & P_RADIO)) {
			String response=("rxPkt:: Err CRC, t=");
			stringTime(now(), response);
			mPrint(response);
		}
#		endif
		return 0;
    }
	else if ((irqflags & IRQ_LORA_HEADER_MASK) == false)
    {
#		if _MONITOR>=1
			if ((debug>=0) && (pdebug & P_RADIO)) {
				mPrint("rxPkt:: Err HEADER");
			}
#		endif
        writeRegister(REG_IRQ_FLAGS, (uint8_t)(IRQ_LORA_HEADER_MASK | IRQ_LORA_RXDONE_MASK));
        return 0;
    }
	else {
        statc.msg_ok++;
		switch(statr[0].ch) {
			case 0: statc.msg_ok_0++; break;
			case 1: statc.msg_ok_1++; break;
			case 2: statc.msg_ok_2++; break;
		}

		if (readRegister(REG_FIFO_RX_CURRENT_ADDR) != readRegister(REG_FIFO_RX_BASE_AD)) {
#			if _MONITOR>=1
			if ((debug>=1) && (pdebug & P_RADIO)) {
				mPrint("RX BASE <" + String(readRegister(REG_FIFO_RX_BASE_AD)) + "> != RX CURRENT <" + String(readRegister(REG_FIFO_RX_CURRENT_ADDR)) + ">"	);
			}
#			endif
		}

		uint8_t currentAddr   = readRegister(REG_FIFO_RX_BASE_AD);
        uint8_t receivedCount = readRegister(REG_RX_BYTES_NB);
#		if _MONITOR>=1
			if ((debug>=1) && (currentAddr > 64)) {
				mPrint("rxPkt:: ERROR Rx addr>64"+String(currentAddr));
			}
#		endif
        writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) currentAddr);

		if (receivedCount > PAYLOAD_LENGTH) {
#			if _MONITOR>=1
				if ((debug>=0) & (pdebug & P_RADIO)) {
					mPrint("rxPkt:: ERROR Payload receivedCount="+String(receivedCount));
				}
#			endif
			receivedCount=PAYLOAD_LENGTH;
		}

        for(int i=0; i < receivedCount; i++) {
            payload[i] = readRegister(REG_FIFO);
        }
		
#		if _MONITOR>=1
		if ((debug>=1) && (pdebug & P_RX)) {
			String response="^ (" + String(receivedCount) + "): " ;
			for (int i=0; i<receivedCount; i++) { 
				if (payload[i]<0x10) response += '0';
				response += String(payload[i],HEX) + " "; 
			}
			mPrint(response);					
		}
#		endif

#	if _MONITOR>=1
		if ((debug>=1) && (pdebug & P_RX)) {

			String response = "^ receivePkt:: rxPkt: t=";
			stringTime(now(), response);
			response += ", f=" + String(gwayConfig.ch) + ", sf=" + String(sf);

			response += ", a=";
			uint8_t DevAddr [4];
					DevAddr[0] = payload[4];
					DevAddr[1] = payload[3];
					DevAddr[2] = payload[2];
					DevAddr[3] = payload[1];
			printHex((IPAddress)DevAddr, ':', response);

			response += ", flags=" + String(irqflags,HEX);
			response += ", addr=" + String(currentAddr);
			response += ", len=" + String(receivedCount);

#			if _LOCALSERVER>=1
			if (debug>=1) {
				int index;
				uint8_t data[receivedCount];
			
				if ((index = inDecodes((char *)(payload+1))) >=0 ) {	
					response += (", inDecodes="+String(index));
				}
				else {	
					response += (", ERROR No Index in inDecodes");
					mPrint(response);
					return(receivedCount);
				}	

				Serial.print(F(", data="));

				for (int i=0; i<receivedCount; i++) {
					data[i]= payload[i]; 
				}

				LoraUp.fcnt= payload[6] | (payload[7] << 8);

				uint8_t CodeLength= encodePacket(
					(uint8_t *)(data + 9),
					receivedCount-9-4,
					(uint16_t)LoraUp.fcnt,
					DevAddr,
					decodes[index].appKey,
					0
				);

				Serial.print(F("- NEW fc="));
				Serial.print(LoraUp.fcnt);
				Serial.print(F(", addr="));

				for (int i=0; i<4; i++) {
					if (DevAddr[i]<=0xF) Serial.print('0');
					Serial.print(DevAddr[i], HEX);
					Serial.print(' ');
				}
				Serial.print(F(", len="));
				Serial.print(CodeLength);
			}
#			endif

			mPrint(response);

			Serial.print(F(", paylength="));
			Serial.print(receivedCount);
			Serial.print(F(", payload="));
			for (int i=0; i<receivedCount; i++) {
					if (payload[i]<=0xF) Serial.print('0');
					Serial.print(payload[i], HEX);
					Serial.print(' ');
			}
			Serial.println();
		}
#	endif

		return(receivedCount);
    }

	writeRegister(REG_IRQ_FLAGS, (uint8_t) (
		IRQ_LORA_RXDONE_MASK | 
		IRQ_LORA_RXTOUT_MASK |
		IRQ_LORA_HEADER_MASK | 
		IRQ_LORA_CRCERR_MASK));

	return 0;
} //receivePkt


// ----------------------------------------------------------------------------------------
// initDown
// ----------------------------------------------------------------------------------------
void initDown(struct LoraDown *LoraDown)
{
	LoraDown->fcnt = 0x00;
}


// ----------------------------------------------------------------------------------------
// sendPkt
// ----------------------------------------------------------------------------------------
bool sendPkt(uint8_t *payLoad, uint8_t payLength)
{
#	if _MONITOR>=1
	if (payLength>=128) {
		if ((debug>=1) && (pdebug & P_TX)) {
			mPrint("v sendPkt:: ERROR len="+String(payLength));
		}
		return false;
	}
#	endif

	payLoad[payLength] = 0x00;
	
	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_TX_BASE_AD));
	writeRegister(REG_PAYLOAD_LENGTH, (uint8_t) payLength);

	delayMicroseconds(100);
	// ASSERT removed: ESP32 SPI timing causes false failures on readback comparison
	// ASSERT( (uint8_t)readRegister(REG_FIFO_ADDR_PTR)==(uint8_t)readRegister(REG_FIFO_TX_BASE_AD) );

#	if _BUF_WRITE>=1
		writeBuffer(REG_FIFO, (uint8_t *) payLoad, (uint8_t) payLength);
#	else
		SPI.transfer((int8_t)(REG_FIFO | 0x80) );
		for (int i=0; i<(uint8_t) payLength; i++) {
			writeRegister(REG_FIFO, (uint8_t) payLoad[i]);
		}
#	endif

#	if _MONITOR>=1
		if ((debug>=2) && (pdebug & P_TX)) {
			Serial.print("v sendPkt:: <");
			for (int i=0; i<(uint8_t) payLength; i++) {
				Serial.print(" ");
				Serial.print((uint8_t) payLoad[i],HEX);
			}
			Serial.println(">");
		}
#	endif

	return true;
}


// ----------------------------------------------------------------------------------------
// loraWait
// ----------------------------------------------------------------------------------------
int loraWait(struct LoraDown *LoraDown)
{
	if (LoraDown->imme == 1) {
		if ((debug>=3) && (pdebug & P_TX)) {
			mPrint("loraWait:: imme is 1");
		}
		return(1);
	}
	LoraDown->usec    = micros();
	int32_t delayTmst = (int32_t)(LoraDown->tmst - LoraDown->usec) + gwayConfig.txDelay - WAIT_CORRECTION;
	
	if ((delayTmst > 8000000) || (delayTmst < -1000)) {
#		if _MONITOR>=1
		if (delayTmst > 8000000) {
			String response= "v loraWait:: ERROR: ";
			printDwn(LoraDown, response);
			mPrint(response);
		}
		else {
			String response= "v loraWait:: return 0: ";
			printDwn(LoraDown, response);
			mPrint(response);
		}
#		endif
		gwayConfig.waitErr++;
		return(0);
	}

	while (delayTmst > 15000) {
		delay(15);
		delayTmst -= 15000;
	}
	
	delayMicroseconds(delayTmst);
	gwayConfig.waitOk++;
	return (1);
}


// ----------------------------------------------------------------------------------------
// txLoraModem
// ----------------------------------------------------------------------------------------
void txLoraModem(struct LoraDown *LoraDown)
{
	_state = S_TX;
	
	opmode(OPMODE_SLEEP);
	delayMicroseconds(100);
	opmode(OPMODE_LORA | 0x03);
	
	ASSERT((readRegister(REG_OPMODE) & OPMODE_LORA) != 0);
	
	opmode(OPMODE_STANDBY);
	setRate(LoraDown->sf, LoraDown->crc);
	setFreq(LoraDown->freq);
	writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);
	writeRegister(REG_PARAMP,(readRegister(REG_PARAMP) & 0xF0) | 0x08);
	writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);
	setPow(LoraDown->powe);
	writeRegister(REG_INVERTIQ, readRegister(REG_INVERTIQ) | 0x40);
	writeRegister(REG_INVERTIQ2, 0x19);
	
    writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
		MAP_DIO0_LORA_TXDONE | 
		MAP_DIO1_LORA_NOP | 
		MAP_DIO2_LORA_NOP |
		MAP_DIO3_LORA_NOP));

    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~IRQ_LORA_TXDONE_MASK);
    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);

	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_TX_BASE_AD));
	writeRegister(REG_PAYLOAD_LENGTH, (uint8_t) LoraDown->size);
	writeRegister(REG_MAX_PAYLOAD_LENGTH, (uint8_t) MAX_PAYLOAD_LENGTH);

#	if _MONITOR >= 1
		if ((debug>=2) && (pdebug & P_TX)) {
			String response ="v txLoraModem:: Before sendPkt: Addr=";
			response+=
						String((uint8_t)LoraDown->payLoad[4],HEX) + " " +
						String((uint8_t)LoraDown->payLoad[3],HEX) + " " +
						String((uint8_t)LoraDown->payLoad[2],HEX) + " " +
						String((uint8_t)LoraDown->payLoad[1],HEX);
			response += ", FCtrl=" + String(LoraDown->payLoad[5],HEX);
			response += ", FPort=" + String(LoraDown->payLoad[8],HEX);
			mPrint(response);
		}
#	endif

	sendPkt(LoraDown->payLoad, LoraDown->size);

#	if _MONITOR >= 1
		if ((debug>=2) && (pdebug & P_TX)) {
			String response ="v txLoraModem::  After sendPkt: Addr=";
			response+=
						String(LoraDown->payLoad[4],HEX) + " " +
						String(LoraDown->payLoad[3],HEX) + " " +
						String(LoraDown->payLoad[2],HEX) + " " +
						String(LoraDown->payLoad[1],HEX);
			response += ", FCtrl=" + String(LoraDown->payLoad[5],HEX);
			response += ", FPort=" + String(LoraDown->payLoad[8],HEX);
			response += ", Fcnt="  + String(LoraDown->payLoad[6] | LoraDown->payLoad[7] << 8);
			mPrint(response);
		}
#	endif

	for (int i=0; i< _REG_AMOUNT; i++) {
		registers[i].regvalue= readRegister(registers[i].regid);
	}

	delayMicroseconds(WAIT_CORRECTION);
	opmode(OPMODE_TX);
	yield();
}


// ----------------------------------------------------------------------------------------
// rxLoraModem
// ----------------------------------------------------------------------------------------
void rxLoraModem()
{
	opmode(OPMODE_STANDBY);
	setFreq(freqs[gwayConfig.ch].upFreq);
    setRate(sf, 0x04);
	
#	if _GWAYSCAN!=1
	writeRegister(REG_INVERTIQ, (uint8_t) 0x27);
#	else
	writeRegister(REG_INVERTIQ, (uint8_t) 0x40);
#	endif

	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_RX_BASE_AD));
	writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);
	
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
		IRQ_LORA_RXDONE_MASK |
		IRQ_LORA_RXTOUT_MASK |
		IRQ_LORA_HEADER_MASK |
		IRQ_LORA_CRCERR_MASK));

	if (gwayConfig.hop) {
		writeRegister(REG_HOP_PERIOD,0x00);
	}
	else {
		writeRegister(REG_HOP_PERIOD,0xFF);
	}

	writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
			MAP_DIO0_LORA_RXDONE | 
			MAP_DIO1_LORA_RXTOUT |
			MAP_DIO2_LORA_NOP |			
			MAP_DIO3_LORA_CRC));

	if (gwayConfig.cad) {
		_state= S_RX;
		opmode(OPMODE_RX_SINGLE);
	}
	else {
		_state= S_RX;
#		if _MONITOR>=1
		if (gwayConfig.hop) {
			mPrint("rxLoraModem:: ERROR continuous receive in hop mode");
		}
#		endif
		opmode(OPMODE_RX);
	}
	
    writeRegister(REG_IRQ_FLAGS, 0xFF);
	return;
}


// ----------------------------------------------------------------------------------------
// cadScanner
// ----------------------------------------------------------------------------------------
void cadScanner()
{
	opmode(OPMODE_STANDBY);
	setFreq(freqs[gwayConfig.ch].upFreq);
	setRate(sf, 0x04);
	writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);
	
	writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
		MAP_DIO0_LORA_CADDONE | 
		MAP_DIO1_LORA_CADDETECT | 
		MAP_DIO2_LORA_NOP | 
		MAP_DIO3_LORA_CRC ));
	
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
		IRQ_LORA_CDDONE_MASK | 
		IRQ_LORA_CDDETD_MASK | 
		IRQ_LORA_CRCERR_MASK | 
		IRQ_LORA_HEADER_MASK));
	
	opmode(OPMODE_CAD);
	return;
}


// ----------------------------------------------------------------------------------------
// initLoraModem
// NOTE: enableLoRaPower() is NOT called here — it must be called ONCE from setup()
// ----------------------------------------------------------------------------------------
void initLoraModem()
{
	_state = S_INIT;										// enableLoRaPower() removed — called once from setup()
#if defined(ESP32_ARCH)
	digitalWrite(pins.rst, LOW);
	delayMicroseconds(10000);
    digitalWrite(pins.rst, HIGH);
	delayMicroseconds(10000);
	digitalWrite(pins.ss, HIGH);
#else
	digitalWrite(pins.rst, HIGH);
	delayMicroseconds(10000);
    digitalWrite(pins.rst, LOW);
	delayMicroseconds(10000);
#endif
	opmode(OPMODE_SLEEP);
	opmode(OPMODE_LORA);
	setFreq(freqs[gwayConfig.ch].upFreq);
    setRate(sf, 0x04);
    writeRegister(REG_LNA, (uint8_t) LNA_MAX_GAIN);
#	if _PIN_OUT==4
		delay(1);
#	endif

    uint8_t version = readRegister(REG_VERSION);
	if (version == 0x12) {
#		if _MONITOR>=1
        if ((debug>=1) && (pdebug & P_MAIN)) {
			mPrint("SX1276 starting");
		}
#		endif
		sx1276= true;
	}
    else if (version == 0x22) {
#		if _MONITOR>=1
        if ((debug>=1) && (pdebug & P_MAIN)) {
			mPrint("WARNING:: SX1272 detected");
		}
#		endif
        sx1276= false;
    } 
	else {
#		if _DUSB>=1
			Serial.print(F("Unknown transceiver="));
			Serial.print(version,HEX);
			Serial.print(F(", pins.rst ="));	Serial.print(pins.rst);
			Serial.print(F(", pins.ss  ="));	Serial.print(pins.ss);
			Serial.print(F(", pins.dio0 ="));	Serial.print(pins.dio0);
			Serial.print(F(", pins.dio1 ="));	Serial.print(pins.dio1);
			Serial.print(F(", pins.dio2 ="));	Serial.print(pins.dio2);
			Serial.println();
			Serial.flush();
#		endif
		die("initLoraModem, unknown transceiver?");
    }

	writeRegister(REG_SYNC_WORD, (uint8_t) 0x34);

#	if _GWAYSCAN==0
		writeRegister(REG_INVERTIQ,0x27);
#	else
		writeRegister(REG_INVERTIQ,0x40);
		mPrint("initLoraModem:: Set REG_INVERTIQ | 0x40");
#	endif

	writeRegister(REG_MAX_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
	writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t) readRegister(REG_FIFO_RX_BASE_AD));
	writeRegister(REG_HOP_PERIOD,0x00);
	writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08);
	writeRegister(REG_PADAC_SX1276, 0x84);
	writeRegister(REG_IRQ_FLAGS_MASK, 0x00);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
}


// ----------------------------------------------------------------------------------------
// startReceiver
// ----------------------------------------------------------------------------------------
void startReceiver() 
{
	initLoraModem();
	if (gwayConfig.cad) {
#		if _MONITOR>=1
		if ((debug>=1) && (pdebug & P_SCAN)) {
			mPrint("S PULL:: _state set to S_SCAN");
			if (debug>=2) Serial.flush();
		}
#		endif
		_state = S_SCAN;
		sf = SF7;
		cadScanner();
	}
	else {
		_state = S_RX;
		rxLoraModem();
	}
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
	writeRegister(REG_IRQ_FLAGS, 0xFF);
}


// ----------------------------------------------------------------------------------------
// Interrupt handlers
// ----------------------------------------------------------------------------------------
void ICACHE_RAM_ATTR Interrupt_0()
{
	_event=1;
}

void ICACHE_RAM_ATTR Interrupt_1()
{
	_event=1;
}

void ICACHE_RAM_ATTR Interrupt_2() 
{
	_event=1;
}