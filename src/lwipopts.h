#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"



#if !NO_SYS
#define TCPIP_THREAD_STACKSIZE 4096
#define DEFAULT_THREAD_STACKSIZE 1024
#define ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0
#define TCPIP_THREAD_PRIO   7

// // not necessary, can be done either way
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// // ping_thread sets socket receive timeout, so enable this feature
#define LWIP_SO_RCVTIMEO 1
#endif

// Lwip features
#define LWIP_HTTPD_CGI                  1    // Enable HTTPCGI
#define LWIP_HTTPD_SUPPORT_POST         1    // Enable POST (firmware upload)
#define LWIP_HTTPD_CUSTOM_FILES         0   
#define LWIP_HTTPD_DYNAMIC_FILE_READ    0
#define LWIP_HTTPD_DYNAMIC_HEADERS      0
#define LWIP_HTTPD_MAX_REQUEST_URI_LEN  128
#define LWIP_HTTPD_MAX_CGI_PARAMETERS   32   // Settings forms send up to ~20 values
#define REST_MAX_QUERY_LEN              640  // Query string of a REST call (the request itself is limited to 1023)
#define LWIP_HTTPD_DYNAMIC_HEADERS      0
#define LWIP_SOCKETS 1

// MDNS
#define LWIP_MDNS_RESPONDER 1
#define LWIP_IGMP 1
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define MDNS_MAX_SERVICES 2  // increase from 1 to 2 
#define MDNS_RESP_USENETIF_EXTCALLBACK  1
#define MEMP_NUM_SYS_TIMEOUT (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)
#define MEMP_NUM_TCP_PCB 12

// Firmware download from GitHub by the trickler itself (Pico 2 W, display
// menu): HTTPS client with mbedtls (FW_DOWNLOAD is set in CMakeLists.txt)
#if FW_DOWNLOAD
#define LWIP_ALTCP                      1
#define LWIP_ALTCP_TLS                  1
#define LWIP_ALTCP_TLS_MBEDTLS          1
#define ALTCP_MBEDTLS_AUTHMODE          MBEDTLS_SSL_VERIFY_REQUIRED
#define LWIP_HTTPC_HAVE_FILE_IO         0
#define LWIP_DNS                        1
// After the handshake the TLS layer opens the TCP window only when a whole
// record (up to 16 kB) is decrypted: the window must hold a full record, or
// the download stalls. The pbuf pool (24 x 1.5 kB) can buffer it.
#undef TCP_WND
#define TCP_WND                         (16 * TCP_MSS)
#endif

#endif