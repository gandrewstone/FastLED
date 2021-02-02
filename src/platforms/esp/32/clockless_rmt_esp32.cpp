
#ifdef ESP32

#define FASTLED_INTERNAL
#include "FastLED.h"

// -- Forward reference
class ESP32RMTController;

// -- Array of all controllers
//    This array is filled at the time controllers are registered 
//    (Usually when the sketch calls addLeds)
static ESP32RMTController * gControllers[FASTLED_RMT_MAX_CONTROLLERS];

// -- Current set of active controllers, indexed by the RMT
//    channel assigned to them.
static ESP32RMTController * gOnChannel[FASTLED_RMT_MAX_CHANNELS];

static int gNumControllers = 0;
static int gNumStarted = 0;
static int gNumDone = 0;
static int gNext = 0;

static intr_handle_t gRMT_intr_handle = NULL;

// -- Global semaphore for the whole show process
//    Semaphore is not given until all data has been sent
static xSemaphoreHandle gTX_sem = NULL;

// -- Make sure we can't call show() too quickly
CMinWait<50>   gWait;

static bool gInitialized = false;

// -- Stored values for FASTLED_RMT_MAX_CHANNELS and FASTLED_RMT_MEM_BLOCKS
int ESP32RMTController::gMaxChannel;
int ESP32RMTController::gMemBlocks;

volatile uint64_t fastLEDlastFillDuration = 0;
volatile uint64_t fastLEDfillDuration = 0;
volatile uint32_t fastLEDnumFills = 0;

ESP32RMTController::ESP32RMTController(int DATA_PIN, int T1, int T2, int T3, int maxChannel, int memBlocks)
    : mPixelData(0),
      mSize(0),
      mWhichHalf(0),
      mBuffer(0),
      mBufferSize(0),
      mCurPulse(0)
{
    // -- Store the max channel and mem blocks parameters
    gMaxChannel = maxChannel;
    gMemBlocks = memBlocks;

    // -- Precompute rmt items corresponding to a zero bit and a one bit
    //    according to the timing values given in the template instantiation
    // T1H
    mOne.level0 = 1;
    mOne.duration0 = ESP_TO_RMT_CYCLES(T1+T2); // TO_RMT_CYCLES(T1+T2);
    // T1L
    mOne.level1 = 0;
    mOne.duration1 = ESP_TO_RMT_CYCLES(T3); // TO_RMT_CYCLES(T3);

    // T0H
    mZero.level0 = 1;
    mZero.duration0 = ESP_TO_RMT_CYCLES(T1); // TO_RMT_CYCLES(T1);
    // T0L
    mZero.level1 = 0;
    mZero.duration1 = ESP_TO_RMT_CYCLES(T2+T3); // TO_RMT_CYCLES(T2 + T3);

    mEmpty.level0 = 0;
    mEmpty.level1 = 0;
    mEmpty.duration0 = ESP_TO_RMT_CYCLES(C_NS(25000));
    mEmpty.duration1 = ESP_TO_RMT_CYCLES(C_NS(25001));  // 50 uSec min to latch

    gControllers[gNumControllers] = this;
    gNumControllers++;

    // -- Expected number of CPU cycles between buffer fills
    mCyclesPerFill = (T1 + T2 + T3) * PULSES_PER_FILL;

    // -- If there is ever an interval greater than 1.5 times
    //    the expected time, then bail out.
    mMaxCyclesPerFill = mCyclesPerFill + mCyclesPerFill/2;

    mPin = gpio_num_t(DATA_PIN);
}

// -- Get or create the buffer for the pixel data
//    We can't allocate it ahead of time because we don't have
//    the PixelController object until show is called.
uint8_t * ESP32RMTController::getPixelBuffer(int size_in_bytes)
{
    if (mPixelData == 0) {
        mSize = size_in_bytes;
        mPixelData = (uint8_t *) malloc(mSize);
        mEndPtr = mPixelData + size_in_bytes;
    }
    return mPixelData;
}

// -- Initialize RMT subsystem
//    This only needs to be done once
void ESP32RMTController::init(gpio_num_t pin)
{
    if (gInitialized) return;

    for (int i = 0; i < gMaxChannel; i += gMemBlocks) {
        gOnChannel[i] = NULL;

        // -- RMT configuration for transmission
        rmt_config_t rmt_tx;
        rmt_tx.channel = rmt_channel_t(i);
        rmt_tx.rmt_mode = RMT_MODE_TX;
        rmt_tx.gpio_num = pin;
        rmt_tx.mem_block_num = gMemBlocks;
        rmt_tx.clk_div = DIVIDER;
        rmt_tx.tx_config.loop_en = false;
        rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
        rmt_tx.tx_config.carrier_en = false;
        rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        rmt_tx.tx_config.idle_output_en = true;

        // -- Apply the configuration
        rmt_config(&rmt_tx);

        if (FASTLED_RMT_BUILTIN_DRIVER) {
            rmt_driver_install(rmt_channel_t(i), 0, 0);
        } else {
            // -- Set up the RMT to send 32 bits of the pulse buffer and then
            //    generate an interrupt. When we get this interrupt we
            //    fill the other part in preparation (like double-buffering)
            rmt_set_tx_thr_intr_en(rmt_channel_t(i), true, PULSES_PER_FILL);
        }
    }

    // -- Create a semaphore to block execution until all the controllers are done
    if (gTX_sem == NULL) {
        gTX_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(gTX_sem);
    }

    if ( ! FASTLED_RMT_BUILTIN_DRIVER) {
        // -- Allocate the interrupt if we have not done so yet. This
        //    interrupt handler must work for all different kinds of
        //    strips, so it delegates to the refill function for each
        //    specific instantiation of ClocklessController.
        if (gRMT_intr_handle == NULL)
            esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, interruptHandler, 0, &gRMT_intr_handle);
    }

    gInitialized = true;
}

// -- Show this string of pixels
//    This is the main entry point for the pixel controller
void IRAM_ATTR ESP32RMTController::showPixels()
{
    if (gNumStarted == 0) {
        // -- First controller: make sure everything is set up
        ESP32RMTController::init(mPin);

#if FASTLED_ESP32_FLASH_LOCK == 1
        // -- Make sure no flash operations happen right now
        spi_flash_op_lock();
#endif
    }

    // -- Keep track of the number of strips we've seen
    gNumStarted++;

    // -- The last call to showPixels is the one responsible for doing
    //    all of the actual worl
    if (gNumStarted == gNumControllers) {
        gNext = 0;

        // -- This Take always succeeds immediately
        xSemaphoreTake(gTX_sem, portMAX_DELAY);

        // -- Make sure it's been at least 50us since last show
        gWait.wait();  // If 50us wait is now in the RMT, remove this

        // -- First, fill all the available channels
        int channel = 0;
        while (channel < gMaxChannel && gNext < gNumControllers) {
            ESP32RMTController::startNext(channel);
            // -- Important: when we use more than one memory block, we need to
            //    skip the channels that would otherwise overlap in memory.
            channel += gMemBlocks;
        }

        // -- Wait here while the data is sent. The interrupt handler
        //    will keep refilling the RMT buffers until it is all
        //    done; then it gives the semaphore back.
        xSemaphoreTake(gTX_sem, portMAX_DELAY);
        xSemaphoreGive(gTX_sem);

        // -- Make sure we don't call showPixels too quickly
        gWait.mark();

        // -- Reset the counters
        gNumStarted = 0;
        gNumDone = 0;
        gNext = 0;

#if FASTLED_ESP32_FLASH_LOCK == 1
        // -- Release the lock on flash operations
        spi_flash_op_unlock();
#endif

    }
}

// -- Start up the next controller
//    This method is static so that it can dispatch to the
//    appropriate startOnChannel method of the given controller.
void IRAM_ATTR ESP32RMTController::startNext(int channel)
{
    if (gNext < gNumControllers) {
        ESP32RMTController * pController = gControllers[gNext];
        pController->startOnChannel(channel);
        gNext++;
    }
}

// -- Start this controller on the given channel
//    This function just initiates the RMT write; it does not wait
//    for it to finish.
void IRAM_ATTR ESP32RMTController::startOnChannel(int channel)
{
    // -- Assign this channel and configure the RMT
    mRMT_channel = rmt_channel_t(channel);

    // -- Store a reference to this controller, so we can get it
    //    inside the interrupt handler
    gOnChannel[channel] = this;

    // -- Assign the pin to this channel
    rmt_set_pin(mRMT_channel, RMT_MODE_TX, mPin);

    if (FASTLED_RMT_BUILTIN_DRIVER) {
        // -- Use the built-in RMT driver to send all the data in one shot
        rmt_register_tx_end_callback(doneOnChannel, 0);
        rmt_write_items(mRMT_channel, mBuffer, mBufferSize, false);
    } else {
        // -- Use our custom driver to send the data incrementally

        // -- Initialize the counters that keep track of where we are in
        //    the pixel data and the RMT buffer
        mRMT_mem_start = & (RMTMEM.chan[mRMT_channel].data32[0].val);
        mRMT_mem_ptr = mRMT_mem_start;
        mCurPtr = mPixelData;
        mCur = 0;
        mWhichHalf = 0;
        mLastFill = 0;

        // -- Fill both halves of the RMT buffer (a total of 64 bits of pixel data)
        fillNext(false);
        fillNext(false);

        // -- Turn on the interrupts
        rmt_set_tx_intr_en(mRMT_channel, true);

        // -- Kick off the transmission
        tx_start();
    }
}

// -- Start RMT transmission
//    Setting this RMT flag is what actually kicks off the peripheral
void IRAM_ATTR ESP32RMTController::tx_start()
{
    // rmt_tx_start(mRMT_channel, true);
    // Inline the code for rmt_tx_start, so it can be placed in IRAM
    RMT.conf_ch[mRMT_channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[mRMT_channel].conf1.mem_rd_rst = 0;
    RMT.int_ena.val &= ~(1 << (mRMT_channel * 3));
    RMT.int_ena.val |= (1 << (mRMT_channel * 3));
    RMT.conf_ch[mRMT_channel].conf1.tx_start = 1;
    mLastFill = __clock_cycles();
}

// -- A controller is done 
//    This function is called when a controller finishes writing
//    its data. It is called either by the custom interrupt
//    handler (below), or as a callback from the built-in
//    interrupt handler. It is static because we don't know which
//    controller is done until we look it up.
void IRAM_ATTR ESP32RMTController::doneOnChannel(rmt_channel_t channel, void * arg)
{
    ESP32RMTController * pController = gOnChannel[channel];

    // -- Turn off the interrupts
    // rmt_set_tx_intr_en(channel, false);
    // Inline the code for rmt_tx_stop, so it can be placed in IRAM
    RMT.int_ena.val &= ~(1 << (channel * 3));
    RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[channel].conf1.mem_rd_rst = 0;

    gOnChannel[channel] = NULL;
    gNumDone++;

    if (gNumDone == gNumControllers) {
        // -- If this is the last controller, signal that we are all done
        if (FASTLED_RMT_BUILTIN_DRIVER) {
            xSemaphoreGive(gTX_sem);
        } else {
            portBASE_TYPE HPTaskAwoken = 0;
            xSemaphoreGiveFromISR(gTX_sem, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) portYIELD_FROM_ISR();
        }
    } else {
        // -- Otherwise, if there are still controllers waiting, then
        //    start the next one on this channel
        if (gNext < gNumControllers) {
            startNext(channel);
        }
    }
}

// -- Custom interrupt handler
//    This interrupt handler handles two cases: a controller is
//    done writing its data, or a controller needs to fill the
//    next half of the RMT buffer with data.
void IRAM_ATTR ESP32RMTController::interruptHandler(void *arg)
{
    uint64_t start = __clock_cycles();
    // -- The basic structure of this code is borrowed from the
    //    interrupt handler in esp-idf/components/driver/rmt.c
    register uint32_t intr_st = RMT.int_st.val;
    register int channel;

    register uint32_t tx_done_bit = 1;
    register uint32_t tx_next_bit = 1<<24;
    for (channel = 0; channel < gMaxChannel; channel++, tx_done_bit <<= 3, tx_next_bit<<=1)
    {
        register ESP32RMTController * pController = gOnChannel[channel];
        if (pController != NULL)
        {
            if (intr_st & tx_next_bit)
            {
                // inline fillNext implementation since function calls are expensive
                {
                    register uint32_t one_val = pController->mOne.val;
                    register uint32_t zero_val = pController->mZero.val;

                    // -- Use locals for speed
                    register uint32_t * pItem =  (uint32_t *) pController->mRMT_mem_ptr;

                    register unsigned char* end = pController->mCurPtr + PULSES_PER_FILL/8;
                    if (end > pController->mEndPtr) end = pController->mEndPtr;
                    register unsigned char* curPtr = pController->mCurPtr;

                    while(curPtr < end)
                    {
                        // -- Get the next four bytes of pixel data
                        register uint32_t pixeldata4 = *((uint32_t*) curPtr);
                        curPtr+=4;
                        asm(
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 96\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 100\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 104\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 108\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 112\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 116\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 120\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 124\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 64\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 68\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 72\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 76\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 80\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 84\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 88\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 92\n"          // store a1 contents into *(pItem+0)
                

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 32\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 36\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 40\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 44\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 48\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 52\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 56\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 60\n"          // store a1 contents into *(pItem+0)

                            
                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 0\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 4\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 8\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 12\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 16\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 20\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 24\n"          // store a1 contents into *(pItem+0)

                            "slli   %3, %3, 1\n"           // Shift it to the top
                            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
                            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
                            "s32i   a11, %2, 28\n"          // store a1 contents into *(pItem+0)
                            :
                            : "r" (zero_val), "r" (one_val), "r" (pItem), "r" (pixeldata4)
                            : "a11");

                        pItem += 32;
                    }
                    if (end == pController->mEndPtr) *pItem++ = 0;  // tell RMT we are done
                    pController->mCurPtr = curPtr;

                    // -- Flip to the other half, resetting the pointer if necessary
                    pController->mWhichHalf++;
                    if (pController->mWhichHalf == 2)
                    {
                        pItem = (uint32_t*) pController->mRMT_mem_start;
                        pController->mWhichHalf = 0;
                    }

                    // -- Store the new pointer back into the object
                    pController->mRMT_mem_ptr = (volatile uint32_t*) pItem;

                }

                RMT.int_clr.val |= tx_next_bit;

            }
            else
            {
                // -- Transmission is complete on this channel
                if (intr_st & tx_done_bit)
                {
                    // GAS: set pin output before toggling RMT
                    gpio_set_level(pController->mPin,0);
                    gpio_matrix_out(pController->mPin, 0x100 , 0,0); // SIG_GPIO_OUT_IDX
                    RMT.int_clr.val &= ~tx_next_bit;
                    RMT.int_clr.val |= tx_done_bit;
                    doneOnChannel(rmt_channel_t(channel), 0);
                }
            }
        }
    }

    uint64_t end = __clock_cycles();
    if (end > start)  // ignore any wrap-around samples
    {
        fastLEDfillDuration += end - start;
        fastLEDlastFillDuration = end;
        fastLEDnumFills++;
    }
}

// -- Fill RMT buffer
//    Puts 32 bits of pixel data into the next 32 slots in the RMT memory
//    Each data bit is represented by a 32-bit RMT item that specifies how
//    long to hold the signal high, followed by how long to hold it low.

#if 0
void IRAM_ATTR ESP32RMTController::fillNext(bool check_time)
{
    uint64_t start = __clock_cycles();
    mLastFill = start;

    // -- Get the zero and one values into local variables
    register uint32_t one_val = mOne.val;
    register uint32_t zero_val = mZero.val;

    // -- Use locals for speed
    volatile register uint32_t * pItem =  mRMT_mem_ptr;

    for (register int i = 0; i < PULSES_PER_FILL/8; i++) {
        if (mCur < mSize) {

            // -- Get the next four bytes of pixel data
            register uint32_t pixeldata = mPixelData[mCur] << 24;
            mCur++;

            // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the
            // rmt_item32_t value corresponding to the buffered bit value
            for (register uint32_t j = 0; j < 8; j++) {
                *pItem++ = (pixeldata & 0x80000000L) ? one_val : zero_val;
                // Replaces: RMTMEM.chan[mRMT_channel].data32[mCurPulse].val = val;

                pixeldata <<= 1;
            }
        } else {
            // -- No more data; signal to the RMT we are done by filling the
            //    rest of the buffer with zeros
            *pItem++ = 0;
        }
    }

    // -- Flip to the other half, resetting the pointer if necessary
    mWhichHalf++;
    if (mWhichHalf == 2) {
        pItem = mRMT_mem_start;
        mWhichHalf = 0;
    }

    // -- Store the new pointer back into the object
    mRMT_mem_ptr = pItem;

    /*
      uint64_t end = __clock_cycles();
      if (end > start)  // ignore any wrap-around samples
      {
      fastLEDfillDuration += end - start;
      fastLEDlastFill = end;
      fastLEDnumFills++;
      }
    */
}

#else
void IRAM_ATTR ESP32RMTController::fillNext(bool check_time)
{
    // uint64_t start = xthal_get_ccount();  // __clock_cycles();

    // -- Get the zero and one values into local variables
    register uint32_t one_val = mOne.val;
    register uint32_t zero_val = mZero.val;

    // -- Use locals for speed
    register uint32_t * pItem =  (uint32_t *) mRMT_mem_ptr;

    register unsigned char* end = mCurPtr + PULSES_PER_FILL/8;
    if (end > mEndPtr) end = mEndPtr;
    register unsigned char* curPtr = mCurPtr;

    while(curPtr < end)
    {
        // -- Get the next four bytes of pixel data
        register uint32_t pixeldata4 = *((uint32_t*) curPtr);
        curPtr+=4;
        asm(
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 96\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 100\n"          // store a1 contents into *(pItem+0)
                                
            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 104\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 108\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 112\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 116\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 120\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 124\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 64\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 68\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 72\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 76\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 80\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 84\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 88\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 92\n"          // store a1 contents into *(pItem+0)
                

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 32\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 36\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 40\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 44\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 48\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 52\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 56\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 60\n"          // store a1 contents into *(pItem+0)
                
            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 0\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 4\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 8\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 12\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 16\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 20\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 24\n"          // store a1 contents into *(pItem+0)

            "slli   %3, %3, 1\n"           // Shift it to the top
            "movgez a11, %0, %3\n"         // if its zero load the zero_val into a1
            "movltz a11, %1, %3\n"         // if its 1 load the one_val into a1
            "s32i   a11, %2, 28\n"          // store a1 contents into *(pItem+0)
            :
            : "r" (zero_val), "r" (one_val), "r" (pItem), "r" (pixeldata4)
            : "a11");

            
        /*
          asm("movi a14, 1\n"                // Mask bit
          "slli a14, a14, 7\n"           // Shift it to the top
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 0\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 4\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 8\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 12\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 16\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 20\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 24\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 28\n"          // store a1 contents into *(pItem+0)


                
          "slli a14, a14, 15\n"           // Shift to next bit (weird because little endian)
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 32\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 36\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 40\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 44\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 48\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 52\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 56\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 60\n"          // store a1 contents into *(pItem+0)




          "slli a14, a14, 15\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 64\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 68\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 72\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 76\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 80\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 84\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 88\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 92\n"          // store a1 contents into *(pItem+0)


          "slli a14, a14, 15\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 96\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 100\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 104\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 108\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 112\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 116\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 120\n"          // store a1 contents into *(pItem+0)

          "srli a14, a14, 1\n"           // Shift to next bit
          "and a12, %3, a14\n"            // mask top bit of pixeldata4
          "moveqz a11, %0, a12\n"         // if its zero load the zero_val into a1
          "movnez a11, %1, a12\n"         // if its 1 load the one_val into a1
          "s32i   a11, %2, 124\n"          // store a1 contents into *(pItem+0)

          :
          : "r" (zero_val), "r" (one_val), "r" (pItem), "r" (pixeldata4)
          : "a11","a12","a13","a14");
        */

        mCurPtr+=4;
        pItem+=32;
#if 0
        for (register int j=0;j<4;j++)
        {
            // Seem backwards?  ESP32 is little endian.
            register uint32_t pixeldata = pixeldata4 & 255;
            pixeldata4 >>= 8;

            switch(pixeldata>>4)
            {
            default:
            case 0:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 1:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 2:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 3:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 4:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 5:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 6:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 7:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 8:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 9:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 10:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 11:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 12:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 13:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 14:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 15:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            }

            switch(pixeldata&0xf)
                //switch(pixeldata&0xe)
            {
            default:
            case 0:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 1:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 2:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 3:
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 4:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 5:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 6:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 7:
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 8:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 9:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 10:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 11:
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            case 12:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = zero_val;
                break;
            case 13:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                *pItem++ = one_val;
                break;
            case 14:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = zero_val;
                break;
            case 15:
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                *pItem++ = one_val;
                break;
            }
        }
#endif
            
    }
    if (end == mEndPtr) *pItem++ = 0;

    // -- Flip to the other half, resetting the pointer if necessary
    mWhichHalf++;
    if (mWhichHalf == 2)
    {
        pItem = (uint32_t*) mRMT_mem_start;
        mWhichHalf = 0;
    }

    // -- Store the new pointer back into the object
    mRMT_mem_ptr = (volatile uint32_t*) pItem;
}
#endif

// -- Init pulse buffer
//    Set up the buffer that will hold all of the pulse items for this
//    controller. 
//    This function is only used when the built-in RMT driver is chosen
void ESP32RMTController::initPulseBuffer(int size_in_bytes)
{
    if (mBuffer == 0) {
        // -- Each byte has 8 bits, each bit needs a 32-bit RMT item
        mBufferSize = size_in_bytes * 8 * 4;
        mBuffer = (rmt_item32_t *) calloc( mBufferSize, sizeof(rmt_item32_t));
    }
    mCurPulse = 0;
}

// -- Convert a byte into RMT pulses
//    This function is only used when the built-in RMT driver is chosen
void ESP32RMTController::convertByte(uint32_t byteval)
{
    // -- Write one byte's worth of RMT pulses to the big buffer
    byteval <<= 24;
    for (register uint32_t j = 0; j < 8; j++) {
        mBuffer[mCurPulse] = (byteval & 0x80000000L) ? mOne : mZero;
        byteval <<= 1;
        mCurPulse++;
    }
}

#endif
