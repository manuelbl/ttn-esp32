/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level C++ API for ttn-esp32.
 *******************************************************************************/

#include "TheThingsNetwork.h"

TTNRFSettings TheThingsNetwork::getRFSettings(TTNRxTxWindow window)
{
    ttn_rf_settings_t settings = ttn_get_rf_settings(static_cast<ttn_rx_tx_window_t>(window));
    TTNRFSettings result;
    result.spreadingFactor = static_cast<TTNSpreadingFactor>(settings.spreading_factor);
    result.bandwidth = static_cast<TTNBandwidth>(settings.bandwidth);
    result.frequency = settings.frequency;
    return result;
}
