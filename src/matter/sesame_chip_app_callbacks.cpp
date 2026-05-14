/* Sesame-specific CHIP application callbacks.
 *
 * Compiled into chip_app (inside the linker group) so the linker can find
 * these when attribute-storage.cpp and related files create the demand.
 */

#include <app/util/generic-callbacks.h>

void emberAfClusterInitCallback(chip::EndpointId endpoint, chip::ClusterId clusterId)
{
    (void)endpoint;
    (void)clusterId;
}
