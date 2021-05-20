// ****************************************************************
// VIDEO DOORBELL by Craig Hoffmann
// 
// User configuration - UserConfig.h
//
// ****************************************************************

// Access Point SSID name and password (and MY_HOSTNAME is also used as the MQTT client name)
#define MY_HOSTNAME "cam_"           // ESP MAC(part) will be appended to this
#define AP_PASSWORD "ap_passwd"      // Access Point Password

// MQTT Settings - max string lengths for server ip, username, password, topic
#define MAX_SERVER_STR_LEN 32
#define MAX_USER_STR_LEN 32
#define MAX_PASSWORD_STR_LEN 32
#define MAX_TOPIC_STR_LEN 32
