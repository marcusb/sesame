/* CHIP Inet interface enumeration stub for FreeRTOS+TCP.
 *
 * chip::Inet::InterfaceIterator calls if_nameindexImpl() / if_freenameindexImpl()
 * (declared in inet/InetInterfaceImpl.h) to enumerate network interfaces.  We
 * return a single entry for our one Ethernet interface ("mlan0", index 1).
 */

#include <inet/InetInterfaceImpl.h>
#include <net/if.h>

namespace chip {
namespace Inet {

struct if_nameindex * if_nameindexImpl()
{
    static char s_name[] = "mlan0";
    static struct if_nameindex s_intf[2] = {
        { 1, s_name },
        { 0, nullptr },
    };
    return s_intf;
}

void if_freenameindexImpl(struct if_nameindex * /* inArray */) {}

} // namespace Inet
} // namespace chip
