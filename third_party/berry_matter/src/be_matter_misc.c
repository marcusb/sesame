/*
  be_matter_misc.c - misc functions for `matter` module

  Copyright (C) 2023  Stephan Hadinger & Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/********************************************************************
 * Matter global module
 * 
 *******************************************************************/

#include "be_constobj.h"
#include "be_mapping.h"
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"

static uint8_t ip_bytes[16] = {};
// Convert an IP address from string to raw bytes
const void* matter_get_ip_bytes(const char* ip_str, size_t* ret_len) {
#if ( ipconfigUSE_IPv6 == 1 )
  if (strchr(ip_str, ':')) {
    // IPv6
    if (FreeRTOS_inet_pton(FREERTOS_AF_INET6, ip_str, ip_bytes) == pdPASS) {
      *ret_len = 16;
      return ip_bytes;
    }
  } else {
    // IPv4
    uint32_t ip_32 = FreeRTOS_inet_addr(ip_str);
    if (ip_32 != 0) {
      memcpy(ip_bytes, &ip_32, 4);
      *ret_len = 4;
      return ip_bytes;
    }
  }
#else
  uint32_t ip_32 = FreeRTOS_inet_addr(ip_str);
  if (ip_32 != 0) {
    memcpy(ip_bytes, &ip_32, 4);
    *ret_len = 4;
    return ip_bytes;
  }
#endif
  *ret_len = 0;
  return NULL;
}
