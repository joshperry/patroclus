#include "CanMeter.h"

namespace patroclus {

bool CanMeter::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN,
        CAN_RX_PIN,
        #if CAN_ACTIVE_MODE
        TWAI_MODE_NORMAL
        #else
        TWAI_MODE_LISTEN_ONLY
        #endif
    );

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        DEBUG_PRINTF("TWAI driver install failed: %d\n", result);
        return false;
    }

    result = twai_start();
    if (result != ESP_OK) {
        DEBUG_PRINTF("TWAI start failed: %d\n", result);
        return false;
    }

    DEBUG_PRINTLN("TWAI initialized");
    #if CAN_ACTIVE_MODE
    DEBUG_PRINTLN("  Mode: ACTIVE (will ACK frames)");
    #else
    DEBUG_PRINTLN("  Mode: LISTEN_ONLY (passive)");
    #endif

    return true;
}

// Process a single CAN frame - VE.CAN energy meter format
void CanMeter::processFrame(const twai_message_t& msg) {
    // Dump to serial if enabled
    if (serialDump_) {
        if (msg.extd) {
            Serial.printf("CAN: %08X [%d]", msg.identifier, msg.data_length_code);
        } else {
            Serial.printf("CAN: %03X [%d]", msg.identifier, msg.data_length_code);
        }
        for (int i = 0; i < msg.data_length_code; i++) {
            Serial.printf(" %02X", msg.data[i]);
        }
        Serial.println();
    }

    // Buffer for MQTT capture if enabled
    if (captureEnabled_ && captureCount_ < CAN_CAPTURE_BUFFER_SIZE) {
        CanCapture& cap = captureBuffer_[captureCount_++];
        cap.id = msg.identifier;
        cap.len = msg.data_length_code;
        cap.extd = msg.extd;
        memcpy(cap.data, msg.data, 8);
    }

    unsigned long now = millis();

    // Handle register-based messages (0x19EFFF40)
    // Format: [seq_lo][seq_hi][register][sub_seq][value 32-bit LE]
    if (msg.identifier == CAN_ID_REGISTER) {
        uint8_t reg = msg.data[2];
        uint32_t regVal = (uint32_t)msg.data[4] |
                          ((uint32_t)msg.data[5] << 8) |
                          ((uint32_t)msg.data[6] << 16) |
                          ((uint32_t)msg.data[7] << 24);

        switch (reg) {
            case REG_ENERGY_FORWARD:
                // Energy in Wh, convert to kWh
                data_.totalEnergyForward = regVal / 1000.0f;
                break;
            //case REG_L1_PF:
            //    data_.phases[0].powerFactor = regVal / 1000.0f;
            //    break;
            //case REG_L2_PF:
            //    data_.phases[1].powerFactor = regVal / 1000.0f;
            //    break;
            //case REG_L3_PF:
            //    data_.phases[2].powerFactor = regVal / 1000.0f;
            //    break;
        }
        return;  // Don't fall through to 0x19F30xxx parsing
    }

    // Parse 0x19F30xxx VE.CAN energy meter messages
    // Format: [seq_lo][seq_hi][val1_lo][val1_hi][val2_b0][val2_b1][val2_b2][val2_b3]

    uint16_t val16 = (msg.data[3] << 8) | msg.data[2];
    int32_t val32 = (int32_t)msg.data[4] |
                    ((int32_t)msg.data[5] << 8) |
                    ((int32_t)msg.data[6] << 16) |
                    ((int32_t)msg.data[7] << 24);

    switch (msg.identifier) {
        // L1 Voltage/Frequency
        case CAN_ID_L1_VOLTAGE:
            data_.phases[0].voltage = val16 / 10.0f;
            data_.frequency = ((msg.data[7] << 8) | msg.data[6]) / 10.0f;
            data_.phases[0].lastUpdate = now;
            data_.phases[0].valid = true;
            break;

        // L1 Current/Power
        case CAN_ID_L1_POWER:
            data_.phases[0].current = val16 / 10.0f;
            data_.phases[0].power = (float)val32;  // Watts, negative = import
            data_.phases[0].lastUpdate = now;
            data_.phases[0].valid = true;
            break;

        // L2 Voltage/Frequency
        case CAN_ID_L2_VOLTAGE:
            data_.phases[1].voltage = val16 / 10.0f;
            data_.phases[1].lastUpdate = now;
            data_.phases[1].valid = true;
            break;

        // L2 Current/Power
        case CAN_ID_L2_POWER:
            data_.phases[1].current = val16 / 10.0f;
            data_.phases[1].power = (float)val32;
            data_.phases[1].lastUpdate = now;
            data_.phases[1].valid = true;
            break;

        // L3 Voltage/Frequency
        case CAN_ID_L3_VOLTAGE:
            data_.phases[2].voltage = val16 / 10.0f;
            data_.phases[2].lastUpdate = now;
            data_.phases[2].valid = true;
            break;

        // L3 Current/Power
        case CAN_ID_L3_POWER:
            data_.phases[2].current = val16 / 10.0f;
            data_.phases[2].power = (float)val32;
            data_.phases[2].lastUpdate = now;
            data_.phases[2].valid = true;
            break;
    }
}

void CanMeter::poll() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        processFrame(msg);
    }
}

}  // namespace patroclus
