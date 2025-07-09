#include <dshot.h>

static const char *TAG = "RMT-DSHOT";

// Create array of RMT channels, one for each pin
rmt_channel_handle_t rmt_channels[NUM_MOTORS] = {};

// Create array of RMT encoders
rmt_encoder_handle_t copy_encoders[NUM_MOTORS] = {};

// Create sync manager
// #if RMT_SYNC
// rmt_sync_manager_handle_t synchro = NULL;
// #endif

// RMT TX config
rmt_transmit_config_t tx_config = {
    // .loop_count = -1 // infinite loop
    .flags.eot_level = 0};

rmt_symbol_word_t dshot_tx_items[NUM_MOTORS][16] = {}; // 17 bits with the pause

// rmt_tx_event_callbacks_t cbs = {
//     .on_trans_done = your_callback_function,
// };
// rmt_tx_register_event_callbacks(tx_channel, &cbs, your_context);
// bool your_callback_function(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx)
// {
//     // Do something when transmission is done
//     return true; // Return true to yield from ISR if needed
// }

void setup_rmt_channels(gpio_num_t pins[NUM_MOTORS])
{
    ESP_LOGI(TAG, "Setting up RMT peripheral...");

    // Loop through each pin and init the RMT channels
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        // gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), pins[i]);

        rmt_tx_channel_config_t config = {
            .gpio_num = pins[i],
            .clk_src = RMT_CLK_SRC,
            .resolution_hz = RMT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 3,
            .flags.invert_out = false,
            .flags.with_dma = false, // Only 1 TX channel supports DMA
            .flags.allow_pd = false};

        ESP_LOGI(TAG, "Setting up TX channel %d...", i + 1);

        ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &rmt_channels[i]));
        ESP_ERROR_CHECK(rmt_enable(rmt_channels[i]));

        // Init RMT encoder for each channel
        rmt_copy_encoder_config_t encoder_config = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoders[i]));
    };

    // Sync the RMT channels
    // #if RMT_SYNC
    //     rmt_sync_manager_config_t synchro_config = {
    //         .tx_channel_array = rmt_channels,
    //         .array_size = sizeof(rmt_channels) / sizeof(rmt_channels[0])};
    //     ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));
    // #endif

    ESP_LOGI(TAG, "All TX channels configured.");
};

/*
// __attribute__((optimize("O0")))
void IRAM_ATTR send_dshot_frame(uint16_t (*throttle)[NUM_MOTORS], bool telemetry)
{
    // Create DSHOT frame
    // dshot_packet frame[NUM_MOTORS] = {};

    // Encode DSHOT command
    rmt_symbol_word_t dshot_tx_items[NUM_MOTORS][16] = {}; // 17 bits with the pause

    // Iterate over each motor to encode and send command
    for (uint8_t motor = 0; motor < NUM_MOTORS; motor++)
    {
        // Assemble DSHOT frame
        // frame[motor].throttle_value = (*throttle)[motor];
        // frame[motor].telemetry_request = telemetry;

        // frame[motor].raw = (frame[motor].throttle_value << 1) | frame[motor].telemetry_request;
        // // Calculate checksum
        // frame[motor].checksum = (frame[motor].raw ^ (frame[motor].raw >> 4) ^ (frame[motor].raw >> 8)) & 0x0F;
        // // Add checksum to end of packet
        // frame[motor].raw = (frame[motor].raw << 4) | frame[motor].checksum;

        // Save 5us by assembling without struct
        uint16_t packet = ((*throttle)[motor] << 1) | telemetry;
        packet = packet << 4 | ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F);

        // Iterate over each bit in the DSHOT command (16 bits)
        for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++)
        {
            uint8_t bit = (packet >> i) & 1;
            uint8_t index = DSHOT_FRAME_SIZE - 1 - i;

            if (bit == 1)
            {
                // set to one
                dshot_tx_items[motor][index].duration0 = DSHOT_BIT_1_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[motor][index].duration1 = DSHOT_BIT_1_LOW / DSHOT_TICK_TIME;
            }
            else
            {
                // set to zero
                dshot_tx_items[motor][index].duration0 = DSHOT_BIT_0_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[motor][index].duration1 = DSHOT_BIT_0_LOW / DSHOT_TICK_TIME;
            }
            dshot_tx_items[motor][index].level0 = 1;
            dshot_tx_items[motor][index].level1 = 0;
        };
        // Set pause bit (bit 17) to zero
        // dshot_tx_items[i][16].level0 = 0;
        // dshot_tx_items[i][16].level1 = 1;
        // dshot_tx_items[i][16].duration0 = 21; // pause between commands
        // dshot_tx_items[i][16].duration0 = 11;

        // Send DSHOT command (channels should already be synced)
        // ESP_LOGI(TAG, "Sending DSHOT command to hardware.");

        // Use if repeating on loop
        // ESP_ERROR_CHECK(rmt_disable(rmt_channels[i]));
        // ESP_ERROR_CHECK(rmt_enable(rmt_channels[i]));

        rmt_transmit(rmt_channels[motor],
                     copy_encoders[motor],
                     &dshot_tx_items[motor],
                     sizeof(rmt_symbol_word_t) * DSHOT_FRAME_SIZE, // 17 bits sent  (when including pause)
                     &tx_config);
    };

    // Restart RMT sync manager
    // #if RMT_SYNC
    //     // ESP_LOGI(TAG, "Restarting synchronization manager.");
    //     rmt_sync_reset(synchro);
    // #endif

    // ESP_LOGI(TAG, "DSHOT command sent %d - %d - %d - %d throttle. RAW PACKET: %x",
    //          frame[0].throttle_value,
    //          frame[1].throttle_value,
    //          frame[2].throttle_value,
    //          frame[3].throttle_value,
    //          frame[0].raw);
};
*/

void IRAM_ATTR send_dshot_frame(uint16_t (*throttle)[NUM_MOTORS], bool telemetry)
{
    // Create DSHOT frame
    // dshot_packet frame[NUM_MOTORS] = {};

    // Iterate over each motor to encode and send command
    for (uint8_t motor = 0; motor < NUM_MOTORS; motor++)
    {
        // Assemble DSHOT frame
        // frame[motor].throttle_value = (*throttle)[motor];
        // frame[motor].telemetry_request = telemetry;

        // frame[motor].raw = (frame[motor].throttle_value << 1) | frame[motor].telemetry_request;
        // // Calculate checksum
        // frame[motor].checksum = (frame[motor].raw ^ (frame[motor].raw >> 4) ^ (frame[motor].raw >> 8)) & 0x0F;
        // // Add checksum to end of packet
        // frame[motor].raw = (frame[motor].raw << 4) | frame[motor].checksum;

        // Save 5us by assembling without struct
        uint16_t packet = ((*throttle)[motor] << 1) | telemetry;
        packet = packet << 4 | ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F);

        // Iterate over each bit in the DSHOT command (16 bits)
        for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++)
        {
            uint8_t bit = (packet >> i) & 1;
            uint8_t index = DSHOT_FRAME_SIZE - 1 - i;

            if (bit == 1)
            {
                // set to one
                dshot_tx_items[motor][index].duration0 = DSHOT_BIT_1_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[motor][index].duration1 = DSHOT_BIT_1_LOW / DSHOT_TICK_TIME;
            }
            else
            {
                // set to zero
                dshot_tx_items[motor][index].duration0 = DSHOT_BIT_0_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[motor][index].duration1 = DSHOT_BIT_0_LOW / DSHOT_TICK_TIME;
            }
            dshot_tx_items[motor][index].level0 = 1;
            dshot_tx_items[motor][index].level1 = 0;
        };
        // Set pause bit (bit 17) to zero
        // dshot_tx_items[i][16].level0 = 0;
        // dshot_tx_items[i][16].level1 = 1;
        // dshot_tx_items[i][16].duration0 = 21; // pause between commands
        // dshot_tx_items[i][16].duration0 = 11;

        // Send DSHOT command (channels should already be synced)
        // ESP_LOGI(TAG, "Sending DSHOT command to hardware.");

        // Use if repeating on loop
        // ESP_ERROR_CHECK(rmt_disable(rmt_channels[i]));
        // ESP_ERROR_CHECK(rmt_enable(rmt_channels[i]));
    };

    // Send all commands at once
    for (uint8_t motor = 0; motor < NUM_MOTORS; motor++)
    {
        rmt_transmit(rmt_channels[motor],
                     copy_encoders[motor],
                     &dshot_tx_items[motor],
                     sizeof(rmt_symbol_word_t) * DSHOT_FRAME_SIZE, // 17 bits sent  (when including pause)
                     &tx_config);
    };
};