#ifndef BC7215_H
#define BC7215_H

/******************************************************************************
*  bc7215.h
*  BC7215 universal IR encoder/decoder chip library
*  This library is to be used with BC7215, which can decode almost all remote
*  controllers and out its original data. It can also use any IR remote protocol
*  to transmit data, therefore can be used to simulate any remote controller,
*  or used for data communication. See BC7215 datasheets for details.
*
*  Dependencies:
*     This Library relies on the Arduino Serial or Software Serial library.
*
*  Author:
*     Bitcode
*
*  Version:
*     V1.0 March 2024
*
*  License:
*     MIT license.
******************************************************************************/
#include <Arduino.h>
#include <Stream.h>
#include "./config/bc7215_config.h"

// buffer size is data packet size + format packet size
#if ENABLE_FORMAT == 1
#	define BC7215_BUFFER_SIZE ((BC7215_MAX_RX_DATA_SIZE + 3) + (32 + 1))

#else

#	define BC7215_BUFFER_SIZE (BC7215_MAX_RX_DATA_SIZE + 3)

#endif

// **********************************
// *** Data structure definitions ***
// **********************************
struct bc7215DataVarPkt_t
{
    word bitLen;
    byte  data[1];
};        // used for variable sized data packet, usually as pointer

struct bc7215DataMaxPkt_t
{
    word bitLen;
    byte  data[BC7215_MAX_RX_DATA_SIZE];
};        // The maximum data packet the library can process

struct bc7215FormatPkt_t
{
    union
    {
        struct
        {
            byte sig : 6;
            byte c56k : 1;
            byte noCA : 1;
        } bits;
        byte byte;
    } signature;
    byte format[32];
};


class BC7215
{
public:
	enum MODConnect {MOD_HIGH=-1, MOD_LOW=-2};
	enum BUSYConnect {BUSY_NC=-3};
    BC7215(Stream& SerialPort, int ModPin, int BusyPin);        // SerialPort could be hardware serial or software serial
    BC7215(Stream& SerialPort, MODConnect ModStat, BUSYConnect BusyStat);
    BC7215(Stream& SerialPort, int ModPin, BUSYConnect BusyStat);
    BC7215(Stream& SerialPort, MODConnect ModStat, int BusyPin);
    
    void setTx();
    void setRx();
    void setRxMode(byte mode);
    void setShutDown();

#if ENABLE_RECEIVING == 1

    bool dataReady();
    void clrData();
    word getLen();
    word dpketSize();
    byte getData(bc7215DataVarPkt_t* target);
    byte getData(bc7215DataMaxPkt_t& target);
	word getRaw(void* addr, word size);

#	if ENABLE_FORMAT == 1

		bool formatReady();
		void clrFormat();
		byte getFormat(bc7215FormatPkt_t& target);

#endif
#endif

	
#if ENABLE_TRANSMITTING == 1

	void loadFormat(const bc7215FormatPkt_t& source);
	void irTx(const bc7215DataVarPkt_t* source);
	void irTx(const bc7215DataMaxPkt_t& source);
	void sendRaw(const void* source, word size);

#endif
	
    bool cmdCompleted();
    
	static void setC56K(bc7215FormatPkt_t& formatPkt);
	static void clrC56K(bc7215FormatPkt_t& formatPkt);
	static void setNOCA(bc7215FormatPkt_t& formatPkt);
	static void clrNOCA(bc7215FormatPkt_t& formatPkt);

	static byte crc8(byte* data, word len);
	static word calSize(const bc7215DataVarPkt_t* dataPkt);
	static word calSize(const bc7215DataMaxPkt_t& dataPkt);
	static void copyDpkt(void* target, bc7215DataVarPkt_t* source);
	static void copyDpkt(void* target, bc7215DataMaxPkt_t& source);
	static bool compareDpkt(byte sig, const bc7215DataVarPkt_t* pkt1, const bc7215DataVarPkt_t* pkt2);
	static bool compareDpkt(byte sig, const bc7215DataMaxPkt_t& pkt1, const bc7215DataMaxPkt_t& pkt2);
	
private:
	Stream&         uart;
	int	modPin;
	int busyPin;
	
	struct
	{
		byte formatPktReady : 1;
		byte dataPktReady : 1;
		byte pktStarted : 1;
		byte overLap : 1;
		byte cmdComplete : 1;
	} bc7215Status;

#if ENABLE_RECEIVING == 1

	word bitLen;
	byte circularBuffer[BC7215_BUFFER_SIZE];

#if BC7215_BUFFER_SIZE > 255
	word       startPos;
	word       datStartPos;
	word       lastWritingPos;
	word       datEndPos;
	word       byteCount;
	word       datCount;
	byte bufBackRead(word pos, word n);
	byte bufRead(word pos, word n);
#else
	byte        startPos;
	byte        datStartPos;
	byte        lastWritingPos;
	byte        datEndPos;
	byte        byteCount;
	byte        datCount;
	byte bufBackRead(byte pos, byte n);
	byte bufRead(byte pos, byte n);
#endif
	
#endif

	void processData(byte data);
	void byteStuffingSend(byte data);
	void sendOneByte(byte data);
	void statusUpdate();
};


#endif
