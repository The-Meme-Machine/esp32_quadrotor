#include <dshot.h>

#define CONFIG_RMT_ISR_IRAM_SAFE 1
#if CONFIG_RMT_ISR_IRAM_SAFE
#define RMT_CALLBACK_ATTR IRAM_ATTR
#else
#define RMT_CALLBACK_ATTR
#endif

static const char *TAG = "RMT";

// Create array of RMT channels, one for each pin
rmt_channel_handle_t rmt_channels[NUM_MOTORS] = {};

// Create array of RMT encoders
rmt_encoder_handle_t copy_encoders[NUM_MOTORS] = {};

// Create sync manager
#if RMT_SYNC
rmt_sync_manager_handle_t synchro = NULL;
#endif

// RMT TX config
rmt_transmit_config_t tx_config = {
    // .loop_count = -1 // infinite loop
};

// rmt_tx_event_callbacks_t cbs = {
//     .on_trans_done = your_callback_function,
// };
// rmt_tx_register_event_callbacks(tx_channel, &cbs, your_context);
// bool your_callback_function(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx)
// {
//     // Do something when transmission is done
//     return true; // Return true to yield from ISR if needed
// }

// motor_channels setup_rmt_channels(gpio_num_t pins[NUM_MOTORS])

void setup_rmt_channels(gpio_num_t pins[NUM_MOTORS])
{
    // // Create array of RMT channels, one for each pin
    // static rmt_channel_handle_t rmt_channels[NUM_MOTORS] = {};

    // // Create array of RMT encoders
    // static rmt_encoder_handle_t copy_encoders[NUM_MOTORS] = {};

    // Loop through each pin and init the RMT channels
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        // gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), pins[i]);

        rmt_tx_channel_config_t config = {
            .gpio_num = pins[i],
            .clk_src = RMT_CLK_SRC,
            .resolution_hz = RMT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 1,
            .flags.invert_out = false,
            .flags.with_dma = false,
            // .flags.with_dma = true, // Only 1 TX channel supports DMA
        };

        ESP_LOGI(TAG, "Setting up RMT TX channel %d.", i);

        ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &rmt_channels[i]));
        ESP_ERROR_CHECK(rmt_enable(rmt_channels[i]));

        // Init RMT encoder for each channel
        rmt_copy_encoder_config_t encoder_config = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoders[i]));
    };

    // Sync the RMT channels
#if RMT_SYNC
    rmt_sync_manager_config_t synchro_config = {
        .tx_channel_array = rmt_channels,
        .array_size = sizeof(rmt_channels) / sizeof(rmt_channels[0])};
    ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));
#endif

    // // Return the RMT channels
    // motor_channels channels = {
    //     .rmt_channels = &rmt_channels[0],
    //     .synchro = &synchro,
    //     .copy_encoders = &copy_encoders[0],
    // };

    ESP_LOGI(TAG, "All TX channels configured, synchronization set.");

    // return channels;
};

// RMT_CALLBACK_ATTR
// void send_dshot_frame(uint16_t throttle[NUM_MOTORS], bool telemetry[NUM_MOTORS], motor_channels channels)
void RMT_CALLBACK_ATTR send_dshot_frame(uint16_t (*throttle)[NUM_MOTORS], bool telemetry)
{
    // Assemble DSHOT frame
    dshot_packet frame[NUM_MOTORS] = {};
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        frame[i].throttle_value = (*throttle)[i];
        // frame[i].throttle_value = local_throttle[i];
        frame[i].telemetry_request = telemetry;

        frame[i].raw = (frame[i].throttle_value << 1) | frame[i].telemetry_request;
        // Calculate checksum
        frame[i].checksum = (frame[i].raw ^ (frame[i].raw >> 4) ^ (frame[i].raw >> 8)) & 0x0F;
        // Add checksum to end of packet
        frame[i].raw = (frame[i].raw << 4) | frame[i].checksum;
    };

    // Encode DSHOT command
    rmt_symbol_word_t dshot_tx_items[NUM_MOTORS][16] = {}; // 17 bits with the pause

    // Iterate over each motor to encode and send command
    // Commands will not be sent until all RMT channels are ready to transmit (due to sync manager)
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        // Iterate over each bit in the DSHOT command (16 bits)
        for (int8_t j = 15; j >= 0; j--)
        {
            uint16_t bit = (frame[i].raw >> j) & 1;

            if (bit == 1)
            {
                // set to one
                dshot_tx_items[i][j].duration0 = DSHOT_BIT_1_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[i][j].duration1 = DSHOT_BIT_1_LOW / DSHOT_TICK_TIME;
            }
            else
            {
                // set to zero
                dshot_tx_items[i][j].duration0 = DSHOT_BIT_0_HIGH / DSHOT_TICK_TIME;
                dshot_tx_items[i][j].duration1 = DSHOT_BIT_0_LOW / DSHOT_TICK_TIME;
            }
            dshot_tx_items[i][j].level0 = 1;
            dshot_tx_items[i][j].level1 = 0;
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

        rmt_transmit(rmt_channels[i],
                     copy_encoders[i],
                     &dshot_tx_items,
                     sizeof(rmt_symbol_word_t) * 16, // 17 bits sent  (when including pause)
                     &tx_config);
    };

// Restart RMT sync manager
#if RMT_SYNC
    // ESP_LOGI(TAG, "Restarting synchronization manager.");
    rmt_sync_reset(synchro);
#endif

    // ESP_LOGI(TAG, "DSHOT command sent %d - %d - %d - %d throttle. RAW PACKET: %x",
    //          frame[0].throttle_value,
    //          frame[1].throttle_value,
    //          frame[2].throttle_value,
    //          frame[3].throttle_value,
    //          frame[0].raw);
};

// RMT_CALLBACK_ATTR
// dshot_packet *assemble_dshot_frames(uint16_t desired_throttle[NUM_MOTORS], bool telemetry[NUM_MOTORS])
// {
//     static dshot_packet frame[NUM_MOTORS] = {};

//     for (int i = 0; i < NUM_MOTORS; i++)
//     {
//         frame[i].throttle_value = desired_throttle[i];
//         frame[i].telemetry_request = telemetry[i];

//         frame[i].raw = (desired_throttle[i] << 1) | telemetry[i];
//         // Calculate checksum
//         frame[i].checksum = (frame[i].raw ^ (frame[i].raw >> 4) ^ (frame[i].raw >> 8)) & 0x0F;
//         // Add checksum to end of packet
//         frame[i].raw = (frame[i].raw << 4) | frame[i].checksum;
//     };

//     return &frame;
// };

// RMT_CALLBACK_ATTR
// rmt_symbol_word_t encode_dshot(dshot_packet packet) {

// };