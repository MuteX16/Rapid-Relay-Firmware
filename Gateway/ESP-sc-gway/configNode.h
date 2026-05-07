#if _GATEWAYNODE==1
#	define _LCODE 1
#	define _CHECK_MIC 1
#	define _SENSOR_INTERVAL 300
#	define _DEVADDR { 0x00, 0x00, 0x00, 0x00 }
#	define _APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#	define _NWKSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#	define _GPS 1
#	define _BATTERY 1
#endif

#if _TRUSTED_NODES >= 1
struct nodex {
	uint32_t id;
	char nm[32];
};
nodex nodes[] = {
	{ 0x0037ad51 , "ESP32-Node1" }
};
#endif

#if _LOCALSERVER>=1
struct codex {
	uint32_t id;
	unsigned char nm[32];
	uint8_t nwkKey[16];
	uint8_t appKey[16];
};
// [DB] ESP32-Node1 ABP credentials — must match ChirpStack Activation tab
codex decodes[] = {
	{	0x0037ad51 , "ESP32-Node1",
		// Network Session Key (NwkSKey)
		{ 0x1a, 0x67, 0xc2, 0x2c, 0xd9, 0x80, 0xb1, 0x4a, 0xb0, 0x67, 0xa8, 0xcd, 0x8a, 0x33, 0x5f, 0x58 },
		// Application Session Key (AppSKey)
		{ 0x6b, 0x4a, 0x83, 0x08, 0x71, 0xa7, 0xce, 0x59, 0xfa, 0x05, 0xa6, 0xcb, 0x01, 0x1d, 0xbd, 0x1b }
	}
};
#endif

// ===== WIFI CREDENTIALS =====
struct wpas {
	char login[32];
	char passw[64];
};
wpas wpa[] = {
	//{ "PiGateway" , "grouptba123" }
	//{ "HUAWEI-2.4G-vJn3" , "8e9Cb4PZ" }
	{ "Room316" , "kissmuna" }
	//{ "NOVAPH" , "Liam36912@" }
	//{ "T.I.P.ian Employee" , "5W7ZMSYw&h" }
};
// ============================

// Gateway identification
#define _DESCRIPTION "FloodMonitor Gateway"
#define _EMAIL "mjwayneayabes@email.com"
#define _PLATFORM "ESP32"
#define _LAT 14.71134572186102 
#define _LON 120.9354169833886
#define _ALT 10

#if !defined(CFG_noassert)
#define ASSERT(cond) if(!(cond)) gway_failed(__FILE__, __LINE__)
#else
#define ASSERT(cond) /**/
#endif
