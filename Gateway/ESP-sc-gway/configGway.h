#define VERSION "V.6.2.8.EU868-Philippines"

// Force debug settings to prevent CAD/SCAN serial spam
#if !defined _DEBUG_LEVEL
#  define _DEBUG_LEVEL 1
#endif

#if !defined _SPIFFS_FORMAT
#	define _SPIFFS_FORMAT 0
#endif

#if !defined _WIFIMANAGER				
#	define _WIFIMANAGER 0
#endif

#if !defined _DUSB
#	define _DUSB 1
#endif

#if !defined _MONITOR
#	define _MONITOR 1
#endif

#if !defined _STATISTICS
#	define _STATISTICS 3
#endif

#define EU863_870 1

#define _CLASS "A"

#define _UDPROUTER 1

#if !defined _CHANNEL
#	define _CHANNEL 2
#endif

#if !defined _SPREADING
#	define _SPREADING SF7
#endif

#define _PDEBUG  (P_RX | P_TX)   // Only show RX and TX messages, no scan spam

#define _CAD 0

#define _CRCCHECK 1

#define _SERVER 1
#define _REFRESH 1
#define _SERVERPORT 80
#define _MAXBUFSIZE 192

#if !defined _OTA
#	define _OTA 1
#endif

// TTGO LoRa32 pin out
#if !defined _PIN_OUT
#	define _PIN_OUT 4
#endif

#if !defined _STRICT_1CH
#	define _STRICT_1CH 0
#endif

#if !defined _RXDELAY1
#	define _RXDELAY1 0
#endif

#if !defined _RX2_SF
#	define _RX2_SF 5
#endif

#if !defined _REPEATER
#	define _REPEATER 0
#endif

#if !defined _OLED
#	define _OLED 0
#endif

#define _GATEWAYMGT 0

#if !defined _STAT_LOG
#	define _STAT_LOG 0
#endif

#define _LOCUDPPORT 1700

#if !defined _LOCALSERVER
#	define _LOCALSERVER 1
#endif

// Philippines timezone UTC+8
#define NTP_TIMESERVER "ph.pool.ntp.org"
#define NTP_TIMEZONES	8
#define SECS_IN_HOUR	3600
#define _NTP_INTR 0

#if !defined _GATEWAYNODE
#	define _GATEWAYNODE 0
#endif

#define _TRUSTED_NODES 1

#define _CONFIGFILE "/gwayConfig.txt"

#if !defined _MAXSTAT
#	define _MAXSTAT 30
#endif

#if !defined _MAXSEEN
#	define _MAXSEEN 15
#endif
#define _SEENFILE "/gwaySeen.txt"

#if !defined _MAXMONITOR
#	define _MAXMONITOR 20
#endif

#define _PULL_INTERVAL 16
#define _STAT_INTERVAL 120
#define _NTP_INTERVAL 3600
#define _WWW_INTERVAL 60
#define _FILE_INTERVAL 30
#define _MSG_INTERVAL 240
#define _RST_INTERVAL 3600

#define CFG_sx1276_radio		

#define _BAUDRATE 115200

#if !defined _MUTEX
#	define _MUTEX 0
#endif

#if !defined _GWAYSCAN
#	define _GWAYSCAN 0
#endif

#if !defined _EXPERT
#	define _EXPERT 0
#endif

#if _REPEATER==0
// ===== REPLACE WITH YOUR RASPBERRY PI IP =====
//#	define _TTNSERVER "192.168.100.126" // Home WiFi
//#	define _TTNSERVER "192.168.100.133" // Home Eth WiFi
//#	define _TTNSERVER "10.200.3.213" // Hotspot 2 jenel
#	define _TTNSERVER "10.92.238.150" // Hotspot
//#	define _TTNSERVER "192.168.6.214" // Liam Wifi
//#	define _TTNSERVER "192.168.1.8" // Liam Eth Wifi
//#	define _TTNSERVER "192.168.4.1" // Pi Hotspot

// =============================================
#	define _TTNPORT 1700
#endif