
#include <sdkconfig.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_check.h>
#include <esp_log.h>
#include <rom/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "beamsync";

// #define USE_RMT 1
// #define USE_PCNT 1
#define STACK_SIZE 4096

static uint16_t lx = 200, ly = 200;
static uint16_t last_x, last_y = 200;

static unsigned int vcount;
static int16_t hcount;
static bool hsync_disabled;
static uint32_t vsync_timestamp;

static TaskHandle_t xHandle2;
static QueueHandle_t jammer_queue = NULL;

static bool button_state;


#if USE_RMT
#include "hal/rmt_hal.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
rmt_channel_handle_t tx_chan = NULL;
rmt_encoder_t* copy_encoder;
#endif

#if USE_PCNT
#include "driver/pulse_cnt.h"
pcnt_unit_handle_t pcnt_unit = NULL;
#endif

static inline uint32_t asm_ccount(void) {
    int32_t r;
    asm volatile("rsr %0, ccount" : "=r"(r));
    return r;
}

IRAM_ATTR
static inline uint32_t start_timeout(uint32_t usecs) {
    return asm_ccount() + usecs * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
}

IRAM_ATTR
static inline unsigned int has_timed_out(uint32_t timeout) {
    return (int32_t)(timeout - asm_ccount()) < 0;
}

static inline void delay_us(unsigned int usecs) {
    // 160MHz clock, 160 cycles/us
    // 240MHz clock, 240 cycles/us
    volatile int32_t timeout =
        asm_ccount() + usecs * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    while ((int32_t)(timeout - asm_ccount()) >= 0);
}

static inline void delay_ms(unsigned int msecs) { delay_us(msecs * 1000); }


#if CONFIG_BEAMZAPPER_PIN_VSYNC < 32
#define GET_VSYNC() \
  (REG_READ(GPIO_IN_REG) & (1 << CONFIG_BEAMZAPPER_PIN_VSYNC))
#else
#define GET_VSYNC() \
  (REG_READ(GPIO_IN1_REG) & (1 << (CONFIG_BEAMZAPPER_PIN_VSYNC - 32)))
#endif

#if CONFIG_BEAMZAPPER_PIN_HSYNC < 32
#define GET_HSYNC() \
  (REG_READ(GPIO_IN_REG) & (1 << CONFIG_BEAMZAPPER_PIN_HSYNC))
#else
#define GET_HSYNC() \
  REG_READ(GPIO_IN1_REG) & (1 << (CONFIG_BEAMZAPPER_PIN_HSYNC - 32))
#endif

// gpio_set_level(CONFIG_BEAMZAPPER_PIN_BUTTON, 0);
// gpio_set_level(CONFIG_BEAMZAPPER_PIN_BUTTON, 1);

#if CONFIG_BEAMZAPPER_PIN_BUTTON < 32
#define SET_BUTTON_LOW() \
  REG_WRITE(GPIO_OUT_W1TC_REG, 1 << CONFIG_BEAMZAPPER_PIN_BUTTON)
#define SET_BUTTON_HIGH() \
  REG_WRITE(GPIO_OUT_W1TS_REG, 1 << CONFIG_BEAMZAPPER_PIN_BUTTON);
#else
#define SET_BUTTON_LOW() \
  REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (CONFIG_BEAMZAPPER_PIN_BUTTON - 32))
#define SET_BUTTON_HIGH() \
  REG_WRITE(GPIO_OUT1_W1TS_REG, 1 << (CONFIG_BEAMZAPPER_PIN_BUTTON - 32))
#endif

// Found by trial and error
// print peek(53267),peek(53268)
//#define VSYNC_OFFSET 2
#define VSYNC_OFFSET 10
#define HSYNC_OFFSET 42

/*
cpu freq = 160MHz
scanline = 64us = 10240 cycles
512 pixels = 125ns/pixel = 20 cycle/pixel
cpu freq = 240MHz
scanline = 64us = 15360 cycles
512 pixels = 125ns/pixel = 30 cycle/pixel
*/

#if USE_RMT

typedef enum {
    RMT_CHANNEL_DIRECTION_TX,
    RMT_CHANNEL_DIRECTION_RX,
} rmt_channel_direction_t;

typedef enum {
    RMT_FSM_INIT_WAIT,
    RMT_FSM_INIT,
    RMT_FSM_ENABLE_WAIT,
    RMT_FSM_ENABLE,
    RMT_FSM_RUN_WAIT,
    RMT_FSM_RUN,
} rmt_fsm_t;

enum {
    RMT_TX_QUEUE_READY,
    RMT_TX_QUEUE_PROGRESS,
    RMT_TX_QUEUE_COMPLETE,
    RMT_TX_QUEUE_MAX,
};

typedef struct rmt_group_t rmt_group_t;
typedef struct rmt_channel_t rmt_channel_t;
typedef struct rmt_tx_channel_t rmt_tx_channel_t;
typedef struct rmt_rx_channel_t rmt_rx_channel_t;
typedef struct rmt_sync_manager_t rmt_sync_manager_t;

struct rmt_group_t {
    int group_id;  // group ID, index from 0
    portMUX_TYPE
        spinlock;  // to protect per-group register level concurrent access
    rmt_hal_context_t hal;  // hal layer for each group
#if 0
    rmt_clock_source_t clk_src; // record the group clock source, group clock is shared by all channels
    uint32_t resolution_hz;     // resolution of group clock
    uint32_t occupy_mask;       // a set bit in the mask indicates the channel is not available
    rmt_tx_channel_t* tx_channels[SOC_RMT_TX_CANDIDATES_PER_GROUP]; // array of RMT TX channels
    rmt_rx_channel_t* rx_channels[SOC_RMT_RX_CANDIDATES_PER_GROUP]; // array of RMT RX channels
    rmt_sync_manager_t* sync_manager; // sync manager, this can be extended into an array if there're more sync controllers in one RMT group
    int intr_priority;     // RMT interrupt priority
#endif
};

struct rmt_channel_t {
    int channel_id;  // channel ID, index from 0
    int gpio_num;    // GPIO number used by RMT RX channel
    uint32_t
        channel_mask;  // mask of the memory blocks that occupied by the channel
    size_t mem_block_num;  // number of occupied RMT memory blocks
    rmt_group_t* group;    // which group the channel belongs to
#if 0
    portMUX_TYPE spinlock;  // prevent channel resource accessing by user and interrupt concurrently
    uint32_t resolution_hz; // channel clock resolution
    intr_handle_t intr;     // allocated interrupt handle for each channel
    _Atomic rmt_fsm_t fsm;  // channel life cycle specific FSM
    rmt_channel_direction_t direction; // channel direction
    rmt_symbol_word_t* hw_mem_base;    // base address of RMT channel hardware memory
    gdma_channel_handle_t dma_chan;    // DMA channel
    esp_pm_lock_handle_t pm_lock;      // power management lock
#if CONFIG_PM_ENABLE
    char pm_lock_name[RMT_PM_LOCK_NAME_LEN_MAX]; // pm lock name
#endif
    // RMT channel common interface
    // The following IO functions will have per-implementation for TX and RX channel
    esp_err_t(*del)(rmt_channel_t* channel);
    esp_err_t(*set_carrier_action)(rmt_channel_t* channel, const rmt_carrier_config_t* config);
    esp_err_t(*enable)(rmt_channel_t* channel);
    esp_err_t(*disable)(rmt_channel_t* channel);
#endif
};

typedef struct {
    rmt_encoder_handle_t encoder;  // encode user payload into RMT symbols
    const void* payload;           // encoder payload
    size_t payload_bytes;          // payload size
    int loop_count;  // transaction can be continued in a loop for specific times
    int remain_loop_count;  // user required loop count may exceed hardware
    // limitation, the driver will transfer them in
    // batches
    size_t transmitted_symbol_num;  // track the number of transmitted symbols
    struct {
        uint32_t
            eot_level : 1;  // Set the output level for the "End Of Transmission"
        uint32_t encoding_done : 1;  // Indicate whether the encoding has finished
        // (not the encoding of transmission)
    } flags;

} rmt_tx_trans_desc_t;

#define RMT_TX_CHANNEL_OFFSET_IN_GROUP 0

typedef struct {
    struct {
        rmt_symbol_word_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[SOC_RMT_CHANNELS_PER_GROUP];
} rmt_block_mem_t;

extern rmt_block_mem_t RMTMEM;
#endif

static void IRAM_ATTR hsync_isr_handler(void* arg) {
    volatile uint32_t now = asm_ccount();
    hcount++;
    if (button_state) {
        return; // Cannot do anything
    }
    if (hcount == ly) {
        if (last_x == lx && last_y == ly) {
            return; // cheating for jitter removal 
        }

#if USE_RMT
        rmt_transmit_config_t tx_config = {
            .loop_count = 0,  // once
        };

        rmt_symbol_word_t reset_code[] = {
            {.level0 = 0, .duration0 = 23 + lx, .level1 = 1, .duration1 = 1} };

#if RMT_LIB_DOES_NOT_SUCK
        ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, reset_code,
            sizeof(reset_code), &tx_config));
#else

        rmt_channel_t* channel = (rmt_channel_t*)tx_chan;
        rmt_group_t* group = channel->group;
        rmt_hal_context_t* hal = &group->hal;
        int channel_id = channel->channel_id;

        rmt_ll_tx_reset_pointer(hal->regs,
            channel_id);  // reset pointer for new transaction
        rmt_ll_tx_enable_loop(hal->regs, channel_id, 0);
        rmt_symbol_word_t* hw_mem_base =
            &RMTMEM.channels[channel_id + RMT_TX_CHANNEL_OFFSET_IN_GROUP]
            .symbols[0];

        hw_mem_base[0] = reset_code[0];
        // a RMT word whose duration is zero means a "stop" pattern
        hw_mem_base[1] = (rmt_symbol_word_t){
            .duration0 = 0,
            .level0 = 0,
            .duration1 = 0,
            .level1 = 1,
        };

        // FIXME button is broken
        rmt_ll_tx_fix_idle_level(hal->regs, channel_id, 1, true);
        rmt_ll_tx_start(hal->regs, channel_id);

#endif

#else

        uint32_t timeout = now + (HSYNC_OFFSET + (lx & 0x1ff)) * 64 *
            CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ / 512;
        while (!has_timed_out(timeout));
        SET_BUTTON_LOW();
        asm("nop; nop; nop; nop; ");
        // asm("nop; nop; nop; nop; nop; nop; nop; nop;");
        SET_BUTTON_HIGH();

#endif

        last_x = lx;
        last_y = ly;
        //gpio_intr_disable(CONFIG_BEAMZAPPER_PIN_HSYNC);
        hsync_disabled = true;
    }

    if ((hcount + 5) == ly) {
        xQueueSendFromISR(jammer_queue, &hcount, NULL);
    }
}

static void IRAM_ATTR vsync_isr_handler(void* arg) {
    // uint32_t gpio_num = (uint32_t)arg;
    vcount++;
    hcount = -VSYNC_OFFSET;

    if (button_state) {
        SET_BUTTON_LOW();
    } else {
        hsync_disabled = false;
        //gpio_intr_enable(CONFIG_BEAMZAPPER_PIN_HSYNC);
    }
    vsync_timestamp = asm_ccount();
#if USE_PCNT
    ESP_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit, prev_lx - 5));
    ESP_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit, prev_lx));
    FIXME
        ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, lx - 5));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, lx));
    prev_lx = lx;
    prev_ly = ly;

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
#endif

}

#if USE_COMMON_ISR_HANDLER
#else

static void IRAM_ATTR common_isr_handler(void* arg) {
    static uint32_t last_vsync;
    uint32_t vsync = GET_VSYNC();
    if (hsync_disabled || (!last_vsync && vsync)) {
        vsync_isr_handler(arg);
    } else {
        //if (GET_HSYNC()) {
        hsync_isr_handler(arg);
        //}
    }

    last_vsync = vsync;
    // Own handling is faster
    // read status to get interrupt status for GPIO0-31
    uint32_t gpio_intr_status = READ_PERI_REG(GPIO_STATUS_REG);
    // read status1 to get interrupt status for GPIO32-39
    uint32_t gpio_intr_status_h = READ_PERI_REG(GPIO_STATUS1_REG);
    if (hsync_disabled) {
        // I hope this prevents excess interrupts when there is no video signal
#if CONFIG_BEAMZAPPER_PIN_HSYNC < 32
        gpio_intr_status &= ~(1 << CONFIG_BEAMZAPPER_PIN_VSYNC);
#else
        gpio_intr_status_h &= ~(1 << (CONFIG_BEAMZAPPER_PIN_HSYNC - 32));
#endif
    }

    // Clear intr for gpio0-gpio31
    SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, gpio_intr_status);
    // Clear intr for gpio32-39
    SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, gpio_intr_status_h);
}
#endif


#if USE_PCNT

FIXME not working yet

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
    BaseType_t high_task_wakeup;
    volatile uint32_t now = asm_ccount();
    hsync_action(now);
    /*
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    if (edata)
        // send event data to queue, from this interrupt callback
        xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
    */
    return true;
}

static void init_pcnt() {
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = 512,
        .low_limit = 0,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    /*
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    */

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = CONFIG_BEAMZAPPER_PIN_HSYNC,
        //.level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    //ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    //QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, 0));

}
#endif

static void jammer_loop(void* arg) {
    // This jamms the core 0 for the time core 1 is processing syncline
    // Does not work very well
    uint32_t h;
    for (;;) {
        if (xQueueReceive(jammer_queue, &h, portMAX_DELAY)) {
            if (hcount + 5 >= ly && hcount <= (ly + 1)) {
                // Wait until next syncline after beam
                volatile uint32_t now = asm_ccount();
                uint32_t timeout = now + 5 * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 512 / 8;
                portDISABLE_INTERRUPTS();
                while (!has_timed_out(timeout) && hcount <= (ly + 1)) {
                    asm("nop;");
                }
                portENABLE_INTERRUPTS();
            } else {
                volatile int h1 = hcount, h2 = ly;
                ESP_LOGI(TAG, "Missed syncline hcount: %d y: %d", h1, h2);
            }
        }
    }
}

bool is_video() {
    volatile uint32_t now = asm_ccount();
    return (now - vsync_timestamp) < CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000 / 20; // Mostly correct
}

void set_xy(uint16_t x, uint16_t y) {
    lx = x;
    ly = y;
}

void set_joy_state(bool j0, bool j1, bool j2, bool j3, bool fire) {
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY0, !j0);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY1, !j1);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY2, !j2);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY3, !j3);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_BUTTON, !fire);
    button_state = fire;
    last_x = last_y = 0;
}

static const char* anim[] = {
    "\\ ",
    " \\",
    " /",
    "/ ",
};

void sync_debug() {
    /*
    lx += 10;
    if (lx > 500)
      lx = 0;
    ly += 10;
    if (ly > 250)
      ly = 0;
    */

    // vcount should be 50Hz * 5s = 250
    static int c = 0;

    ESP_LOGI(TAG,
        "%s x: %3d, y: %3d v: %3d h: %3d vs: %d hs: %d but: %d j0: %d j1: %d "
        "j2: %d j3: %d",
        anim[c & 3], lx, ly, vcount, (int)hcount,
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_VSYNC),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_HSYNC),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_BUTTON),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_JOY0),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_JOY1),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_JOY2),
        gpio_get_level(CONFIG_BEAMZAPPER_PIN_JOY3));
    c++;
    vcount = hcount = 0;
}


void init_gpio() {
    jammer_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_VSYNC, GPIO_MODE_INPUT);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_HSYNC, GPIO_MODE_INPUT);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_BUTTON, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_JOY0, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_JOY1, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_JOY2, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_JOY3, GPIO_MODE_INPUT_OUTPUT_OD);

    gpio_set_level(CONFIG_BEAMZAPPER_PIN_BUTTON, 1);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY0, 1);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY1, 1);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY2, 1);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY3, 1);

    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_JOY0, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_JOY1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_JOY2, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_JOY3, GPIO_PULLUP_ONLY);

    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_VSYNC, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CONFIG_BEAMZAPPER_PIN_HSYNC, GPIO_PULLUP_ONLY);

#if USE_COMMON_ISR_HANDLER
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
#endif

    gpio_set_intr_type(CONFIG_BEAMZAPPER_PIN_VSYNC, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(CONFIG_BEAMZAPPER_PIN_HSYNC, GPIO_INTR_POSEDGE);

#if USE_COMMON_ISR_HANDLER
    gpio_isr_handler_add(CONFIG_BEAMZAPPER_PIN_VSYNC, vsync_isr_handler,
        (void*)CONFIG_BEAMZAPPER_PIN_VSYNC);
    gpio_isr_handler_add(CONFIG_BEAMZAPPER_PIN_HSYNC, hsync_isr_handler,
        (void*)CONFIG_BEAMZAPPER_PIN_HSYNC);
#else

    gpio_intr_enable(CONFIG_BEAMZAPPER_PIN_VSYNC);
    gpio_intr_enable(CONFIG_BEAMZAPPER_PIN_HSYNC);

    gpio_isr_handle_t handle;
    gpio_isr_register(common_isr_handler, 0,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, &handle);
#endif

#if USE_RMT
#define RMT_RESOLUTION 8 * 1000 * 1000

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,            // select source clock
        .gpio_num = CONFIG_BEAMZAPPER_PIN_BUTTON,  // GPIO number
        .mem_block_symbols = 512,  // memory block size, 512 * 4 = 2048 Bytes
        .resolution_hz =
            RMT_RESOLUTION,  // 8 MHz tick resolution, i.e., 1 tick = 125 ns
        .trans_queue_depth =
            4,  // set the number of transactions that can pend in the background
        .flags.invert_out = true,
        .flags.with_dma = false,  // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan));

    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder));

    Here is another example

        // Reference https://esp-idf.readthedocs.io/en/v1.0/api/rmt.html

        rmt_config_t config;
    rmt_item32_t items[1];



    void setup() {
        // put your setup code here, to run once:
        config.rmt_mode = RMT_MODE_TX;
        config.channel = RMT_CHANNEL_0;
        config.gpio_num = STEP_PIN;
        config.mem_block_num = 1;
        config.tx_config.loop_en = 0;
        config.tx_config.carrier_en = 0;
        config.tx_config.idle_output_en = 1;
        config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
        config.clk_div = 80; // 80MHx / 80 = 1MHz 0r 1uS per count

        rmt_config(&amp;config);
        rmt_driver_install(config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)

        items[0].duration0 = 3;
        items[0].level0 = 1;
        items[0].duration1 = 0;
        items[0].level1 = 0;

    }

    void loop() {
        // esp_err_t rmt_write_items(rmt_channel_t channel, rmt_item32_t *rmt_item, int item_num, bool wait_tx_done)
        rmt_write_items(config.channel, items, 1, 0);
        delay(1);


    }

#endif
    /*
            xTaskCreatePinnedToCore(
                jammer_loop,           // Function that implements the task.
                "jammer_loop",         // Text name for the task.
                STACK_SIZE,            // Stack size in bytes, not words.
                (void*)jammer_queue,  // Parameter passed into the task.
                24,
                &xHandle2,  // Variable to hold the task's data structure.
                0);
    */
}
