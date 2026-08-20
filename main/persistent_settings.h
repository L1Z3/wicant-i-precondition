#ifndef __PERSISTENT_SETTINGS_H__
#define __PERSISTENT_SETTINGS_H__

#include <stdbool.h>

// Load mirrored settings and start the low-priority NVS writer.
// This must be called before using any setting accessor.
void persistent_settings_init(void);

bool persistent_settings_get_precon_enabled(void);
void persistent_settings_set_precon_enabled(bool enabled);

#endif
