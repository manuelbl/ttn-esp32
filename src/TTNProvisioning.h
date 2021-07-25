/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#ifndef _ttnprovisioning_h_
#define _ttnprovisioning_h_

#include "ttn_provisioning.h"


class TTNProvisioning
{
public:
    TTNProvisioning() { ttn_provision_init(); }

    bool haveKeys() { return ttn_provision_have_keys(); }
    bool decodeKeys(const char *dev_eui, const char *app_eui, const char *app_key) { return ttn_provision_decode_keys(dev_eui, app_eui, app_key); }
    bool fromMAC(const char *app_eui, const char *app_key) { return ttn_provision_from_mac(app_eui, app_key); }
    bool saveKeys() { return ttn_provision_save_keys(); }
    bool restoreKeys(bool silent) { return ttn_provision_restore_keys(silent); }

#if defined(TTN_HAS_AT_COMMANDS)
    void startTask() { ttn_provision_start_task(); }
#endif
};

#endif
