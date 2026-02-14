#include <stdio.h>
#include <string.h>

#include "board.h"
#include "ztimer.h"
#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "thread.h"
#include "periph/adc.h"
#include "periph/rtc.h"
#include "periph/gpio.h"

#include "at24cxxx.h"
#include "at24cxxx_params.h"

/* ---- Defines ---- */

#define IPC_TYPE_BTN_A_PRESS        1
#define IPC_TYPE_BTN_B_PRESS        2
#define IPC_TYPE_BTN_TIMEOUT        4
#define IPC_TYPE_BTN_BOTH           16 // Both buttons pressed
#define IPC_TYPE_BTN_MODE           32 // Button pressed that triggers a mode change
#define IPC_TYPE_BTN_PORT           64 // Button pressed that triggers a port change

#define IPC_TYPE_SENSOR_PAUSE       128
#define IPC_TYPE_SENSOR_RESUME      256

#define ADC_RES ADC_RES_10BIT // Define the ADC resolution

#define BTN_BOTH_DETECTION_MS       100
#define BTN_DEBOUNCE_MS             100

// Platform specific defines
#define MAGIC                       0x4F
#define TYPE                        0xA3  // Packet
#define VERSION                     1

/* ---- Global variables ---- */

at24cxxx_t eeprom_dev;
at24cxxx_params_t eeprom_params = AT24CXXX_PARAMS; // Are overwritten with board specific values

kernel_pid_t button_thread_pid = KERNEL_PID_UNDEF;
static char _button_stack[THREAD_STACKSIZE_MEDIUM];

kernel_pid_t calibration_thread_pid = KERNEL_PID_UNDEF;
static char _calibration_stack[THREAD_STACKSIZE_MEDIUM];

kernel_pid_t sensor_thread_pid = KERNEL_PID_UNDEF;
static char _sensor_stack[THREAD_STACKSIZE_LARGE];

netif_t *netif = NULL;

uint16_t sensor_connected_bitmap = 0b0000000000000000; // For every sensor connected, the corresponding bit is set to 1
uint16_t sensor_calibration_bitmap = 0b0000000000000000; // For every sensor calibrated, the corresponding bit is set to 1
uint16_t sensor_data[12]; // Array to store the latest sensor data for each port

typedef struct
{
    uint8_t magic;
    uint8_t type;
    uint8_t version;
    uint16_t sensor_connected_bitmap; // Bitmap indicating which sensors are connected
    uint16_t sensor_calibration_bitmap; // Bitmap indicating which sensors are calibrated
    uint16_t sensor_values[12]; // Latest sensor values for each port
} rf_message_t;

typedef struct {
    adc_t adc_line;
    gpio_t pwr_pin;
    gpio_t fb_pin; // Feedback pin
} port_cfg_t;

static const port_cfg_t ports[] = {
    { ANALOG_PORT_0,  POWER_PORT_0_PIN, PRESENCE_PORT_0_PIN },
    { ANALOG_PORT_1,  POWER_PORT_1_PIN, PRESENCE_PORT_1_PIN },
    { ANALOG_PORT_2,  POWER_PORT_2_PIN, PRESENCE_PORT_2_PIN },
    { ANALOG_PORT_3,  POWER_PORT_3_PIN, PRESENCE_PORT_3_PIN },
    { ANALOG_PORT_4,  POWER_PORT_4_PIN, PRESENCE_PORT_4_PIN },
    { ANALOG_PORT_5,  POWER_PORT_5_PIN, PRESENCE_PORT_5_PIN },
    { ANALOG_PORT_6,  POWER_PORT_6_PIN, PRESENCE_PORT_6_PIN },
    { ANALOG_PORT_7,  POWER_PORT_7_PIN, PRESENCE_PORT_7_PIN },
    { ANALOG_PORT_8,  POWER_PORT_8_PIN, PRESENCE_PORT_8_PIN },
    { ANALOG_PORT_9,  POWER_PORT_9_PIN, PRESENCE_PORT_9_PIN },
    { ANALOG_PORT_10, POWER_PORT_10_PIN, PRESENCE_PORT_10_PIN },
    { ANALOG_PORT_11, POWER_PORT_11_PIN, PRESENCE_PORT_11_PIN },
};

/* ---- Button handling section ---- */

uint32_t last_btn_press_A = 0;

static void _btn_A_cb(void *arg)
{
    (void) arg;

    // Local debounce
    uint32_t now = ztimer_now(ZTIMER_USEC);

    if (now - last_btn_press_A < BTN_DEBOUNCE_MS * 1000)
    {
        return;
    }

    last_btn_press_A = now;

    msg_t ipc_msg;
    ipc_msg.type = IPC_TYPE_BTN_A_PRESS;
    msg_send(&ipc_msg, button_thread_pid);
}

uint32_t last_btn_press_B = 0;

static void _btn_B_cb(void *arg)
{
    (void) arg;

    // Local debounce
    uint32_t now = ztimer_now(ZTIMER_USEC);

    if (now - last_btn_press_B < BTN_DEBOUNCE_MS * 1000)
    {
        return;
    }

    last_btn_press_B = now;

    msg_t ipc_msg;
    ipc_msg.type = IPC_TYPE_BTN_B_PRESS;
    msg_send(&ipc_msg, button_thread_pid);
}

static void _btn_timeout_cb(void *arg)
{
    (void) arg;

    msg_t ipc_msg;
    ipc_msg.type = IPC_TYPE_BTN_TIMEOUT;
    msg_send(&ipc_msg, button_thread_pid);
}


static void *button_thread(void *arg)
{
    (void) arg;

    msg_t ipc_msg;
    msg_t ipc_msg_queue[16];

    /* Setup the message queue */
    msg_init_queue(ipc_msg_queue, 16);

    ztimer_t btn_timeout_timer;
    btn_timeout_timer.callback = _btn_timeout_cb;
    btn_timeout_timer.arg = NULL;

    uint8_t btn_flag_A = 0;
    uint8_t btn_flag_B = 0;

    while (1)
    {
        msg_receive(&ipc_msg);

        switch (ipc_msg.type)
        {
            case IPC_TYPE_BTN_A_PRESS:
                
                if (btn_flag_B)
                {
                    btn_flag_B = 0;
                    ztimer_remove(ZTIMER_MSEC, &btn_timeout_timer);

                    ipc_msg.type = IPC_TYPE_BTN_BOTH;
                    msg_send(&ipc_msg, calibration_thread_pid);
                }
                else if (!btn_flag_A)
                {
                    btn_flag_A = 1;
                    ztimer_set(ZTIMER_MSEC, &btn_timeout_timer, BTN_BOTH_DETECTION_MS);
                }

                break;

            case IPC_TYPE_BTN_B_PRESS:
                
                if (btn_flag_A)
                {

                    btn_flag_A = 0;
                    ztimer_remove(ZTIMER_MSEC, &btn_timeout_timer);

                    ipc_msg.type = IPC_TYPE_BTN_BOTH;
                    msg_send(&ipc_msg, calibration_thread_pid);

                }
                else if (!btn_flag_B)
                {
                    btn_flag_B = 1;
                    ztimer_set(ZTIMER_MSEC, &btn_timeout_timer, BTN_BOTH_DETECTION_MS);
                }

                break;

            case IPC_TYPE_BTN_TIMEOUT:
                
                if (btn_flag_A)
                {
                    ipc_msg.type = IPC_TYPE_BTN_PORT;
                    msg_send(&ipc_msg, calibration_thread_pid);

                    btn_flag_A = 0;
                }
                else if (btn_flag_B)
                {
                    ipc_msg.type = IPC_TYPE_BTN_MODE;
                    msg_send(&ipc_msg, calibration_thread_pid);

                    btn_flag_B = 0;
                }

                break;

            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

/* ---- Calibration section ---- */

typedef enum {
    STATE_IDLE = 0,
    STATE_CAL = 1
} cal_state_t;

typedef enum {
    CAL_TYPE_DRY = 0,
    CAL_TYPE_WET = 1
} cal_type_t;


static void _activate_cur_port(uint8_t cur_port)
{
    // Turn off all ports
    for (size_t i = 0; i < 12; i++)
    {
        gpio_set(ports[i].pwr_pin);
    }

    // Turn on cur port
    gpio_clear(ports[cur_port].pwr_pin);
}

static uint8_t _activate_next_port(uint8_t cur_port)
{
    uint8_t new_port = (uint8_t) ((cur_port + 1) % 12);
    
    // Turn off previous port
    gpio_set(ports[cur_port].pwr_pin);

    // Turn on new port
    gpio_clear(ports[new_port].pwr_pin);

    return new_port;
}

static void _deactivate_cur_port(uint8_t cur_port)
{
    gpio_set(ports[cur_port].pwr_pin);
}


static cal_type_t _change_cal_type(cal_type_t cal_type)
{
    if (cal_type == CAL_TYPE_DRY)
    {
        gpio_clear(CALIB_LED_BLUE_PIN);
        gpio_set(CALIB_LED_RED_PIN);

        return CAL_TYPE_WET;
    }
    else // if (cal_type == CAL_TYPE_WET)
    {
        gpio_clear(CALIB_LED_RED_PIN);
        gpio_set(CALIB_LED_BLUE_PIN);

        return CAL_TYPE_DRY;
    }
}

static void _set_cal_type_led(cal_type_t cal_type)
{
    if (cal_type == CAL_TYPE_DRY)
    {
        gpio_set(CALIB_LED_BLUE_PIN);
        gpio_clear(CALIB_LED_RED_PIN);
    }
    else // if (cal_type == CAL_TYPE_WET)
    {
        gpio_set(CALIB_LED_RED_PIN);
        gpio_clear(CALIB_LED_BLUE_PIN);
    }
}

static void _clear_cal_type_led(cal_type_t cal_type)
{
    if (cal_type == CAL_TYPE_DRY)
    {
        gpio_set(CALIB_LED_RED_PIN);
    }
    else // if (cal_type == CAL_TYPE_WET)
    {
        gpio_set(CALIB_LED_BLUE_PIN);
    }
}


static void _write_to_eeprom(uint8_t port, cal_type_t cal_type, uint16_t value)
{
    // Compute the EEPROM address based on the port number and calibration type
    uint32_t adr = (port * 2) + cal_type; // cal_type is either 0 or 1

    // Write the value to the EEPROM
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;

    at24cxxx_write(&eeprom_dev, adr, data, sizeof(data));
}


static void _calibrate_port(uint8_t cur_port, cal_type_t cal_type)
{
    /**
     * Calibration is performed only on actual connected ports.
     * The calibration is then performed for 15s, during which the selected sensors adc is sampled
     * multiple times. The first few samples are discarded, to allow the internal adc capacitor
     * to stabilize.
     * Then the average of all samples is computed and stored in the EEPROM, depending on the
     * calibration type (dry/wet) and the port number.
     */

    // Check if selected port is actually connected, if not, skip calibration and return to idle state
    if ((sensor_connected_bitmap & (1 << cur_port)) == 0)
    {
        _clear_cal_type_led(cal_type);
        return;
    } 

     // Turn on calibration process indicaton (blinking dry/wet indication)
    _clear_cal_type_led(cal_type);

    // Perform calibration for 15s, taking a sample every 100ms
    int32_t samples[100];

    for (size_t i = 0; i < 150; i++)
    {
        int sample = adc_sample(ports[cur_port].adc_line, ADC_RES);

        if (i < 50)
        {
            continue; // Throw away the first 50 samples -> equals 5s of stabilization time
        }

        samples[i - 50] = sample;

        _set_cal_type_led(cal_type); // Indicate that a sample is being taken
        ztimer_sleep(ZTIMER_MSEC, 50);
        _clear_cal_type_led(cal_type);
        ztimer_sleep(ZTIMER_MSEC, 50);
    }

    // Average all samples
    uint64_t sum = 0;

    for (size_t i = 0; i < 100; i++)
    {
        sum += samples[i];
    }

    uint32_t average = (uint32_t) (sum / 100);

    printf("Calibration result for port %u, type %s: %lu\n", cur_port, cal_type == CAL_TYPE_DRY ? "DRY" : "WET", average);

    // Write to EEPROM, depending on calibration type and port  
    _write_to_eeprom(cur_port, cal_type, (uint16_t) average); 
}


static void *calibration_thread(void *arg)
{
    (void) arg;

    msg_t ipc_msg;
    msg_t ipc_msg_queue[4];

    /* Setup the message queue */
    msg_init_queue(ipc_msg_queue, 4);

    cal_state_t cal_state = STATE_IDLE;
    cal_type_t cal_type = CAL_TYPE_DRY;

    uint8_t cur_port = 0;

    while (1)
    {
        msg_receive(&ipc_msg);

        if (cal_state == STATE_IDLE && ipc_msg.type == IPC_TYPE_BTN_BOTH)
        {
            cal_state = STATE_CAL;

            // Inform the sensor thread to pause measurements during calibration
            ipc_msg.type = IPC_TYPE_SENSOR_PAUSE;
            msg_send(&ipc_msg, sensor_thread_pid);

            ztimer_sleep(ZTIMER_MSEC, 100); // Short delay to ensure the sensor thread has paused before we start calibration

            _set_cal_type_led(cal_type); // Indicate initial calibration type as DRY
            _activate_cur_port(cur_port); // Activate port 0 at the start of calibration

            continue;
        }

        if (cal_state == STATE_CAL)
        {
            switch (ipc_msg.type)
            {
                case IPC_TYPE_BTN_PORT:

                    cur_port = _activate_next_port(cur_port);

                    break;

                case IPC_TYPE_BTN_MODE:

                    cal_type = _change_cal_type(cal_type);

                    break;

                case IPC_TYPE_BTN_BOTH:

                    // Perform calibration for the current port and type
                    _calibrate_port(cur_port, cal_type);

                    cal_state = STATE_IDLE;
                    cal_type = CAL_TYPE_DRY;

                    _deactivate_cur_port(cur_port);
                    cur_port = 0;

                    // Inform the sensor thread to resume measurements after calibration
                    ipc_msg.type = IPC_TYPE_SENSOR_RESUME;
                    msg_send(&ipc_msg, sensor_thread_pid);

                    break;

                default:
                    break;
            }
        }
    }

    /* never reached */
    return NULL;
}

/* ---- Sensor section ---- */

static uint8_t _check_pause_request(void)
{
    msg_t ipc_msg;

    // Check if there is a pause request in the message queue
    if (msg_try_receive(&ipc_msg))
    {
        if (ipc_msg.type == IPC_TYPE_SENSOR_PAUSE)
        {
            return 1; // Pause request received   
        }
    }

    return 0; // No pause request
}

static void _check_sensor_connection(void)
{
    for (size_t i = 0; i < 12; i++)
    {
        if (gpio_read(ports[i].fb_pin) == 0) // If port is connected, the feedback pin will be pulled low
        {
            sensor_connected_bitmap |= (1 << i);
        }
        else
        {
            sensor_connected_bitmap &= ~(1 << i);
        }
    }
}


static uint16_t _read_from_eeprom(uint8_t port, cal_type_t cal_type)
{
    // Compute the EEPROM address based on the port number and calibration type
    uint32_t adr = (port * 2) + cal_type; // cal_type is either 0 or 1

    // Read the value from the EEPROM
    uint8_t data[2];
    at24cxxx_read(&eeprom_dev, adr, data, sizeof(data));

    uint16_t value = ((uint16_t) data[0] << 8) | data[1];
    return value;
}


static uint8_t _check_calibration_validity(uint16_t dry, uint16_t wet)
{
    // Check if calibration values are valid
    if (dry == 0 || wet == 0 || dry == wet || dry < wet)
    {
        return 0; // Invalid calibration values
    }

    return 1; // Valid calibration values
}

static uint16_t _calculate_relative_soil_moisture(uint16_t raw, uint16_t dry, uint16_t wet)
{
    /**
     * This function calculates the relative soil moisture as a percentage based on the raw sensor
     * value and the dry/wet calibration values.
     */

    int32_t num = (int32_t) (dry - raw);
    int32_t den = (int32_t) (dry - wet);

    // Check bounds
    if (num <= 0)
    {
        return 0;
    }

    if (num >= den)
    {
        return 100;
    }

    return (uint16_t) (((num * 100) + (den / 2)) / den);
}


static void *sensor_thread(void *arg)
{
    (void) arg;

    msg_t ipc_msg;
    msg_t ipc_msg_queue[4];

    /* Setup the message queue */
    msg_init_queue(ipc_msg_queue, 4);

    uint8_t pause_requested = 0;

    uint8_t cur_port = 0; // Start at port 0

    while (1)
    {
        if (pause_requested)
        {
            // If a pause was requested, wait for a resume message
            msg_receive(&ipc_msg);

            if (ipc_msg.type == IPC_TYPE_SENSOR_RESUME)
            {
                pause_requested = 0; // Resume with paused measurement
            }
        }

        // At each iteration, check how many sensors are connected and update the bitmap accordingly
        _check_sensor_connection();

        // Turn on the current port
        gpio_clear(ports[cur_port].pwr_pin);

        // Perform measurement for 25s, taking a sample every 100ms
        int32_t samples[200];

        for (size_t i = 0; i < 250; i++)
        {
            int sample = adc_sample(ports[cur_port].adc_line, ADC_RES);

            if (i < 50)
            {
                ztimer_sleep(ZTIMER_MSEC, 100);

                if (_check_pause_request()) // Allow to pause the measurement process during the stabilization time
                {
                    pause_requested = 1;
                    break;
                }

                continue; // Throw away the first 50 samples -> equals 5s of stabilization time
            }

            samples[i - 50] = sample;

            ztimer_sleep(ZTIMER_MSEC, 100);

            if (_check_pause_request()) // Allow to pause the measurement process, during measurement 
            {
                pause_requested = 1;
                break;
            }
        }

        if (pause_requested)
        {
            continue; // Skip the rest of the iteration and repeat the iteration when not paused anymore
        }

        // Average all samples
        uint64_t sum = 0;

        for (size_t i = 0; i < 200; i++)
        {
            sum += samples[i];
        }

        uint32_t average = (uint32_t) (sum / 200);

        // Check if DRY and WET calibration values are available for the current port
        uint16_t cal_dry = 0;
        uint16_t cal_wet = 0;

        cal_dry = _read_from_eeprom(cur_port, CAL_TYPE_DRY);
        cal_wet = _read_from_eeprom(cur_port, CAL_TYPE_WET);

        uint16_t value = (uint16_t) average;
        // Check calibration validity, if invalid return raw value
        if (_check_calibration_validity(cal_dry, cal_wet))
        {
            // Update calibration bitmap to indicate that this sensor has valid calibration values
            sensor_calibration_bitmap |= (1 << cur_port);
            value = _calculate_relative_soil_moisture((uint16_t) average, cal_dry, cal_wet);
        }
        else
        {
            // Update calibration bitmap to indicate that this sensor does not have valid calibration values
            sensor_calibration_bitmap &= ~(1 << cur_port);
        }

        // Put into data array
        sensor_data[cur_port] = value;

        // Turn off the current port
        gpio_set(ports[cur_port].pwr_pin);
        cur_port = (cur_port + 1) % 12; // Move to the next port for the next iteration

        // When all ports have been measured, send data message
        if (cur_port == 0)
        {
            rf_message_t rf_msg;

            rf_msg.magic = MAGIC;
            rf_msg.type = TYPE; // The type distinguishes the platform (here soil moisture hub)
            rf_msg.version = VERSION;
            rf_msg.sensor_calibration_bitmap = sensor_calibration_bitmap;
            rf_msg.sensor_connected_bitmap = sensor_connected_bitmap;
            memcpy(rf_msg.sensor_values, sensor_data, sizeof(sensor_data));

            // TODO: Send
        }
    }

    /* never reached */
    return NULL;
}

/* ---- Init section ---- */

void initialize_adcs(void)
{

    for (size_t i = 0; i < 12; i++)
    {
        int res = adc_init(ports[i].adc_line);

        if (res < 0) {
            printf("Failed to initialize ADC line %u\n", ports[i].adc_line);
        }
    }
}

void initialize_buttons(void)
{
    gpio_init_int(CALIB_BUTTON_A_PIN, GPIO_IN_PU, GPIO_RISING, _btn_A_cb, NULL);
    gpio_init_int(CALIB_BUTTON_B_PIN, GPIO_IN_PU, GPIO_RISING, _btn_B_cb, NULL);
}


int main(void)
{
    netif = netif_iter(NULL);

    initialize_adcs();
    initialize_buttons();

    int res = at24cxxx_init(&eeprom_dev, &eeprom_params);

    if (res != AT24CXXX_OK) {
        printf("Failed to initialize EEPROM\n");
    }

    //at24cxxx_erase(&eeprom_dev);
    //uint16_t value = _read_from_eeprom(0, CAL_TYPE_DRY); // Test read from EEPROM
    //printf("Read value from EEPROM: %u\n", value);

    /* Create sensor thread */
    sensor_thread_pid = thread_create(_sensor_stack, sizeof(_sensor_stack), THREAD_PRIORITY_MAIN + 3,
                                      THREAD_CREATE_STACKTEST,
                                      sensor_thread, NULL, "sensor_thread");

    /* Create calibration thread */
    calibration_thread_pid = thread_create(_calibration_stack, sizeof(_calibration_stack), THREAD_PRIORITY_MAIN + 2,
                                          THREAD_CREATE_STACKTEST,
                                          calibration_thread, NULL, "calibration_thread");
    
    /* Create button thread */
    button_thread_pid = thread_create(_button_stack, sizeof(_button_stack), THREAD_PRIORITY_MAIN + 1,
                                    THREAD_CREATE_STACKTEST,                                    
                                    button_thread, NULL, "button_thread");

    thread_sleep();

    return 0;
}
