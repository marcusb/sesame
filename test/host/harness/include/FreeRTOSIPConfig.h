#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

#include <stdio.h>

#define ipconfigUSE_IPv4 1
#define ipconfigUSE_IPv6 0
#define ipconfigIPv4_BACKWARD_COMPATIBLE 1
#define ipconfigCOMPATIBLE_WITH_SINGLE 0

#define ipconfigBYTE_ORDER pdFREERTOS_LITTLE_ENDIAN
#define ipconfigHAS_PRINTF 1
#define FreeRTOS_printf(x) printf x
#define ipconfigHAS_DEBUG_PRINTF 1
#define FreeRTOS_debug_printf(x) printf x

#define ipconfigSOCK_DEFAULT_RECEIVE_BLOCK_TIME (5000)
#define ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME (5000)

#define ipconfigUSE_DNS 1
#define ipconfigUSE_DNS_CACHE 1
#define ipconfigUSE_MDNS 1
#define ipconfigDNS_CACHE_NAME_LENGTH 48
#define ipconfigDNS_CACHE_ENTRIES 4
#define ipconfigDNS_REQUEST_ATTEMPTS 2

#define ipconfigIP_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define ipconfigIP_TASK_STACK_SIZE_WORDS (configMINIMAL_STACK_SIZE * 5)

#define ipconfigUSE_NETWORK_EVENT_HOOK 1

#define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS (5000U / portTICK_PERIOD_MS)

/* libslirp runs a DHCP server of its own; let our stack use it. */
#define ipconfigUSE_DHCP 1
#define ipconfigUSE_DHCPv6 0
#define ipconfigDHCP_REGISTER_HOSTNAME 0
#define ipconfigUSE_DHCP_HOOK 0
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD (120000U / portTICK_PERIOD_MS)

#define ipconfigARP_CACHE_ENTRIES 6
#define ipconfigMAX_ARP_RETRANSMISSIONS 5
#define ipconfigMAX_ARP_AGE 150

#define ipconfigINCLUDE_FULL_INET_ADDR 1

#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS 60
#define ipconfigEVENT_QUEUE_LENGTH (ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS + 5)
#define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND 1

#define ipconfigUDP_TIME_TO_LIVE 128
#define ipconfigTCP_TIME_TO_LIVE 128

#define ipconfigUSE_TCP 1
#define ipconfigUSE_TCP_WIN 1
#define ipconfigNETWORK_MTU 1500U

#define ipconfigREPLY_TO_INCOMING_PINGS 1
#define ipconfigSUPPORT_OUTGOING_PINGS 0
#define ipconfigSUPPORT_SELECT_FUNCTION 0

#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES 1
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES 1

#define ipconfigPACKET_FILLER_SIZE 2U
#define ipconfigTCP_WIN_SEG_COUNT 32
#define ipconfigTCP_RX_BUFFER_LENGTH (10000)
#define ipconfigTCP_TX_BUFFER_LENGTH (10000)

#define ipconfigIS_VALID_PROG_ADDRESS(x) ((x) != NULL)

#define ipconfigTCP_KEEP_ALIVE 1
#define ipconfigTCP_KEEP_ALIVE_INTERVAL 20

#define ipconfigSOCKET_HAS_USER_SEMAPHORE 0
#define ipconfigSOCKET_HAS_USER_WAKE_CALLBACK 1
#define ipconfigUSE_CALLBACKS 0

#define portINLINE __inline

#endif /* FREERTOS_IP_CONFIG_H */
