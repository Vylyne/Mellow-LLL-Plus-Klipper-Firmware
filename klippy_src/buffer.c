// LLL Buffer Plus - autonomous auto-feed task for Klipper MCU firmware
//
// Ports the state machine from Mellow's standalone firmware
// (Fly3DTeam/Buffer, lib/buffer/buffer.cpp) INTO Klipper MCU firmware, as
// an autonomous task. The board remains a normal Klipper MCU (the host
// reads PB7 via [filament_switch_sensor]) AND drives the buffer motor on
// its own.
//
// ADDED file (does not modify any existing Klipper file) -> survives git pull.
//
// Convention taken VERBATIM from buffer.cpp / buffer.h:
//   photo sensor: obstructed = 1, not obstructed = 0 (direct GPIO readback, NO inversion!)
//   endstop_3/filament switch (PB7): Normally Closed, Filament Present = 0, No Filament = 1
//   buttons: normally closed, pressed = 0, released = 1
//   HALL3 (PB4) obstructed -> Forward (feed)   [pos1]
//   HALL2 (PB3) obstructed -> Stop             [pos2]
//   HALL1 (PB2) obstructed -> Back (retract)   [pos3]
//
// ONLY change vs. the standalone: button hold is handled event-driven
// (non-blocking) instead of the original blocking while() loop, because
// blocking for seconds would break Klipper host comms. Behavior is
// otherwise identical.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"   // CONFIG_CLOCK_FREQ
#include "board/gpio.h" // gpio_in_setup / gpio_out_setup
#include "board/misc.h" // timer_read_time
#include "command.h"    // DECL_COMMAND / sendf
#include "sched.h"      // DECL_TASK / DECL_INIT

// ------------------------------------------------------------------
// Pins (Klipper encoding: GPIO(PORT,NUM) = (PORT-'A')*16 + NUM)
// ------------------------------------------------------------------
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))

#define HALL1_PIN GPIO('B', 2)             // Optical sensor 3 -> pos3 -> back limit, retract.
#define HALL2_PIN GPIO('B', 3)             // Optical sensor 2 -> pos2 -> mid, don't push or pull.
#define HALL3_PIN GPIO('B', 4)             // Optical sensor 1 -> pos1 -> forward limit, feed.
#define FIL_PIN GPIO('B', 7)               // ENDSTOP_3 / filament entry switch.
#define KEY_INTERNAL_RETRACT GPIO('B', 13) // KEY1 Retract (retract)
#define KEY_INTERNAL_FEED GPIO('B', 12)    // KEY2 Feed (feed)
#define KEY_EXTERNAL_RETRACT GPIO('A', 2)  // Exposed 3 pin header, external retract button
#define KEY_EXTERNAL_FEED GPIO('A', 3)     // Exposed 3 pin header, external feed button
#define KEY_EXTERNAL_UNLOAD GPIO('B', 10)  // Exposed 3 pin header, triggers UNLOAD job
#define KEY_EXTERNAL_LOAD GPIO('B', 11)    // Exposed 3 pin header, triggers LOAD job
#define MDM_FILAMENT_PIN GPIO('A', 4)      // MDM filament presence sensor
#define MDM_MOTION_PIN GPIO('A', 5)        // MDM motion sensor (encoder pulse)
#define FRONT_SIG_PIN GPIO('B', 5)         // mainboard "advance" signal (active low)
#define BACK_SIG_PIN GPIO('B', 6)          // mainboard "retract" signal (active low)
#define EXT1_PIN GPIO('C', 13)             // EXTENSION_PIN1
#define EXT2_PIN GPIO('C', 14)             // EXTENSION_PIN2
#define EN_PIN GPIO('A', 6)                // TMC enable (active low)
#define UART_PIN GPIO('B', 1)              // TMC2208 single-wire UART (PDN_UART)

// ASSUMPTION - VERIFY ON REAL HARDWARE: MDM_FILAMENT_PIN's raw read value
// meaning "no filament present". Your printer.cfg's
// [filament_switch_sensor T0_mdm_filament] uses a plain `^T0_buffer:PA4`
// (no `!` inversion) unlike the entrance switch's `^!`. This constant must
#define MDM_FIL_NO_FILAMENT_STATE 1

// ------------------------------------------------------------------
// Parameters (copied from standalone firmware)
// ------------------------------------------------------------------
// VACTUAL = SPEED * microsteps * 200 / 60 / 0.715   (buffer.cpp)
//   SPEED = 260 rpm, microsteps = 64 -> 77575 (matches the standalone's int cast)
#define SPEED_RPM 260
#define MICROSTEPS_DIV 64
#define VACTUAL_MAG ((int32_t)((int64_t)SPEED_RPM * MICROSTEPS_DIV * 200 / 60 * 1000 / 715))

#define FEED_TIMEOUT_MS 60000 // safety: stop if continuous FORWARD > 60s (buffer.cpp)

// ms -> MCU clock ticks conversion
#define MS_TICKS(ms) ((uint32_t)(ms) * (CONFIG_CLOCK_FREQ / 1000))

// Bit-bang UART: 9600 baud (matches the standalone's beginSerial(9600))
#define UART_BAUD 9600
#define BIT_TICKS (CONFIG_CLOCK_FREQ / UART_BAUD) // 48e6/9600 = 5000 ticks/bit

// ------------------------------------------------------------------
// TMC2208 registers (datasheet addresses) + values (DERIVED from TMCStepper)
// ------------------------------------------------------------------
#define TMC_GCONF 0x00
#define TMC_IHOLD_IRUN 0x10
#define TMC_VACTUAL 0x22
#define TMC_CHOPCONF 0x6C
#define TMC_PWMCONF 0x70
#define TMC_WRITE_FLAG 0x80
#define TMC_SLAVE_ADDR 0x00 // DRIVER_ADDRESS 0b00 (buffer.h)
#define TMC_SYNC 0x05

#define GCONF_VAL 0x000001C4UL      // i_scale_analog=0,spreadcycle,pdn_disable,mstep_reg_select,multistep_filt
#define GCONF_SHAFT 0x00000008UL    // bit3 shaft (FORWARD=1)
#define CHOPCONF_VAL 0x12020055UL   // toff=5,vsense=1,mres=2(64u),intpol=1
#define IHOLD_IRUN_VAL 0x00010F07UL // IHOLD=7,IRUN=15,iholddelay=1 (rms_current(500)@0.11)
#define PWMCONF_VAL 0xC10D0024UL    // TMC2208 default (pwm_autoscale)

// ------------------------------------------------------------------
// Motor state
// ------------------------------------------------------------------
enum
{
    ST_STOP = 0,
    ST_FORWARD,
    ST_BACK
};

// Hall / optical position sensors (buffer dancer arm)
static struct gpio_in hall1, hall2, hall3;

// Filament presence (entrance)
static struct gpio_in fil;

// Physical buttons - board-mounted
static struct gpio_in key_internal_retract, key_internal_feed;

// Physical buttons - external header
static struct gpio_in key_external_retract, key_external_feed;
static struct gpio_in key_external_load, key_external_unload;

// Mainboard control signals
static struct gpio_in front_sig, back_sig;

// MDM (downstream encoder module)
static struct gpio_in mdm_filament, mdm_motion;

static struct gpio_out en_out, uart_out, ext1, ext2;
static uint8_t last_state = ST_STOP;
static uint8_t tmc_ready = 0;
static uint32_t feed_start_time;
static uint8_t is_feeding = 0;
static uint8_t is_error = 0;

// Button state (replaces the standalone's attachInterrupt with polling)
static uint8_t key1_prev = 1, key2_prev = 1;
static uint32_t key1_press_t, key2_press_t, key1_rel_t, key2_rel_t;
static uint8_t key1_held = 0, key2_held = 0;
static uint8_t key1_rel = 0, key2_rel = 0;
static uint8_t key1_cnt = 0, key2_cnt = 0;
static uint8_t manual = 0; // 0=none, 1=forward, 2=back
static uint8_t inform = 0;
static uint32_t inform_t;

// ------------------------------------------------------------------
// Load/unload job state
// ------------------------------------------------------------------
enum
{
    JOB_NONE = 0,
    JOB_LOAD,
    JOB_UNLOAD
};
enum
{
    JOB_RESULT_OK = 0,
    JOB_RESULT_STALLED,
    JOB_RESULT_TIMEOUT,
    JOB_RESULT_INTERRUPTED
};

// PLACEHOLDER - not yet measured. No MDM_MOTION_PIN pulse for this long
// while a job is active is treated as a stall (filament isn't moving).
#define JOB_STALL_MS 5000
// Hard ceiling for button-triggered jobs. Host-triggered jobs
// (buffer_load/buffer_unload commands) pass their own timeout instead.
#define JOB_DEFAULT_TIMEOUT_MS 60000

static uint8_t current_job = JOB_NONE;
static uint8_t load_btn_prev = 1, unload_btn_prev = 1;
static uint8_t toolhead_sensor_present = 0; // host-forwarded EBB toolhead sensor state
static uint32_t job_start_time;
static uint32_t job_last_motion_time;
static uint32_t job_deadline;
static uint8_t mdm_motion_prev = 0;

static void start_job(uint8_t job, uint32_t timeout_ms);
static void send_job_result(uint8_t job, uint8_t status);

// ------------------------------------------------------------------
// TMC2208 UART: blocking bit-bang TX, 9600 baud, IRQs enabled.
// Byte framing: start(0), 8 data bits LSB-first, stop(1). Idle = high.
// ------------------------------------------------------------------
static void
bit_delay(void)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < BIT_TICKS)
        ;
}

static void
uart_send_byte(uint8_t b)
{
    gpio_out_write(uart_out, 0); // start bit
    bit_delay();
    for (uint8_t i = 0; i < 8; i++)
    { // 8 data bits, LSB first
        gpio_out_write(uart_out, (b >> i) & 0x01);
        bit_delay();
    }
    gpio_out_write(uart_out, 1); // stop bit
    bit_delay();
}

// TMC CRC8 (poly 0x07), over the first 7 bytes (identical to TMC2208Stepper::calcCRC)
static uint8_t
tmc_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t cur = data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (cur & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = crc << 1;
            cur >>= 1;
        }
    }
    return crc;
}

// TMC2208 register write (write-only) - datagram identical to write()
static void
tmc_write(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    dg[0] = TMC_SYNC;
    dg[1] = TMC_SLAVE_ADDR;
    dg[2] = reg | TMC_WRITE_FLAG;
    dg[3] = (val >> 24) & 0xFF;
    dg[4] = (val >> 16) & 0xFF;
    dg[5] = (val >> 8) & 0xFF;
    dg[6] = val & 0xFF;
    dg[7] = tmc_crc(dg, 7);
    for (uint8_t i = 0; i < 8; i++)
        uart_send_byte(dg[i]);
}

static void
tmc_delay_ms(uint32_t ms)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < MS_TICKS(ms))
        ;
}

// ------------------------------------------------------------------
// TMC2208 config - ported from buffer_motor_init. Called on the task's
// first tick (NOT from DECL_INIT: init order isn't guaranteed, the timer
// might not be running yet). Assumes 24V is present (24V first, USB after).
// ------------------------------------------------------------------
static void
tmc_configure(void)
{
    tmc_delay_ms(10); // TMC power stabilization
    tmc_write(TMC_GCONF, GCONF_VAL);
    tmc_delay_ms(2);
    tmc_write(TMC_CHOPCONF, CHOPCONF_VAL);
    tmc_delay_ms(2);
    tmc_write(TMC_IHOLD_IRUN, IHOLD_IRUN_VAL);
    tmc_delay_ms(2);
    tmc_write(TMC_PWMCONF, PWMCONF_VAL);
    tmc_delay_ms(2);
    tmc_write(TMC_VACTUAL, 0);
    tmc_delay_ms(2);
    last_state = ST_STOP;
}

// ------------------------------------------------------------------
// Host command support.
// ------------------------------------------------------------------

// constants for host driven operation support
enum
{
    HMODE_AUTO = 0,
    HMODE_FORWARD,
    HMODE_BACK,
    HMODE_STOP
};
static uint8_t host_mode = HMODE_AUTO;
static uint32_t host_mode_deadline;

void command_buffer_set_mode(uint32_t *args)
{
    host_mode = args[0];
    uint32_t timeout_ms = args[1];
    host_mode_deadline = timer_read_time() + MS_TICKS(timeout_ms);
}
DECL_COMMAND(command_buffer_set_mode, "buffer_set_mode mode=%c timeout=%u");

void command_buffer_query_state(uint32_t *args)
{
    sendf("buffer_state hall1=%c hall2=%c hall3=%c fil=%c error=%c state=%c host_mode=%c",
          gpio_in_read(hall1), gpio_in_read(hall2), gpio_in_read(hall3),
          gpio_in_read(fil), is_error, last_state, host_mode);
}
DECL_COMMAND(command_buffer_query_state, "buffer_query_state");

void command_buffer_load(uint32_t *args)
{
    start_job(JOB_LOAD, args[0]);
}
DECL_COMMAND(command_buffer_load, "buffer_load timeout=%u");

void command_buffer_unload(uint32_t *args)
{
    start_job(JOB_UNLOAD, args[0]);
}
DECL_COMMAND(command_buffer_unload, "buffer_unload timeout=%u");

void command_buffer_set_toolhead_sensor(uint32_t *args)
{
    toolhead_sensor_present = args[0] ? 1 : 0;
}
DECL_COMMAND(command_buffer_set_toolhead_sensor, "buffer_set_toolhead_sensor state=%c");

static void
start_job(uint8_t job, uint32_t timeout_ms)
{
    uint32_t now = timer_read_time();
    current_job = job;
    job_start_time = now;
    job_last_motion_time = now; // don't count time before the job started
    job_deadline = now + MS_TICKS(timeout_ms);
    mdm_motion_prev = gpio_in_read(mdm_motion); // avoid a false edge on the first tick
}

// status = JOB_RESULT_* outcome. reason = which job (JOB_LOAD/JOB_UNLOAD)
// this result is for.
static void
send_job_result(uint8_t job, uint8_t status)
{
    sendf("buffer_result status=%c job=%c", status, job);
}
// ------------------------------------------------------------------
// Init (at boot) - GPIO only (self-contained). Ports buffer_sensor_init.
// ------------------------------------------------------------------
void buffer_init(void)
{
    // Sensors / buttons: raw INPUT (standalone pinMode INPUT); mainboard
    // signals = PULLUP
    hall1 = gpio_in_setup(HALL1_PIN, 0);
    hall2 = gpio_in_setup(HALL2_PIN, 0);
    hall3 = gpio_in_setup(HALL3_PIN, 0);
    fil = gpio_in_setup(FIL_PIN, 0);
    key_internal_retract = gpio_in_setup(KEY_INTERNAL_RETRACT, 0);
    key_internal_feed = gpio_in_setup(KEY_INTERNAL_FEED, 0);
    key_external_retract = gpio_in_setup(KEY_EXTERNAL_RETRACT, 1);
    key_external_feed = gpio_in_setup(KEY_EXTERNAL_FEED, 1);
    key_external_load = gpio_in_setup(KEY_EXTERNAL_LOAD, 1);     // pulled up, idle=1
    key_external_unload = gpio_in_setup(KEY_EXTERNAL_UNLOAD, 1); // pulled up, idle=1
    mdm_filament = gpio_in_setup(MDM_FILAMENT_PIN, 0);
    mdm_motion = gpio_in_setup(MDM_MOTION_PIN, 0);

    front_sig = gpio_in_setup(FRONT_SIG_PIN, 1); // INPUT_PULLUP (active low)
    back_sig = gpio_in_setup(BACK_SIG_PIN, 1);   // INPUT_PULLUP (active low)
    // Outputs
    en_out = gpio_out_setup(EN_PIN, 1);     // motor disabled (active low)
    uart_out = gpio_out_setup(UART_PIN, 1); // UART line idle = high
    ext1 = gpio_out_setup(EXT1_PIN, 0);     // EXTENSION_PIN1 LOW (buffer.cpp)
    ext2 = gpio_out_setup(EXT2_PIN, 1);     // EXTENSION_PIN2 HIGH (buffer.cpp)
}
DECL_INIT(buffer_init);

// ------------------------------------------------------------------
// Apply a motor state (only writes to the TMC on change).
// Direction = shaft bit + POSITIVE VACTUAL + stop before reversing (motor_control).
// FORWARD = shaft=1 (buffer.h FORWARD=1); BACK = shaft=0.
// ------------------------------------------------------------------
static void
apply_state(uint8_t state)
{
    if (state == last_state)
        return;
    switch (state)
    {
    case ST_FORWARD:
        gpio_out_write(en_out, 0);
        if (last_state == ST_BACK)
        {
            tmc_write(TMC_VACTUAL, 0);
            tmc_delay_ms(2);
        }
        tmc_write(TMC_GCONF, GCONF_VAL | GCONF_SHAFT); // shaft=1
        tmc_delay_ms(2);
        tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
        break;
    case ST_BACK:
        gpio_out_write(en_out, 0);
        if (last_state == ST_FORWARD)
        {
            tmc_write(TMC_VACTUAL, 0);
            tmc_delay_ms(2);
        }
        tmc_write(TMC_GCONF, GCONF_VAL); // shaft=0
        tmc_delay_ms(2);
        tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
        break;
    case ST_STOP:
    default:
        tmc_write(TMC_VACTUAL, 0);
        gpio_out_write(en_out, 1);
        break;
    }
    last_state = state;
}

// ------------------------------------------------------------------
// Buttons + mainboard signals - ported from motor_control's button
// section, made non-blocking. Returns 1 if a manual mode is active
// (=> skip auto-feed this iteration), 0 otherwise.
// ------------------------------------------------------------------
static uint8_t
process_buttons(void)
{
    uint32_t now = timer_read_time();
    uint8_t k1 = gpio_in_read(key_internal_retract) && gpio_in_read(key_external_retract); // 1=released, 0=pressed
    uint8_t k2 = gpio_in_read(key_internal_feed) && gpio_in_read(key_external_feed);

    // Edge detection (replaces key_it_callback CHANGE)
    if (key1_prev && !k1)
    {
        key1_press_t = now;
        key1_cnt++;
        key1_held = 1;
    }
    else if (!key1_prev && k1)
    {
        if (now - key1_press_t <= MS_TICKS(500))
        {
            key1_rel = 1;
            key1_rel_t = now;
        }
        else
            key1_cnt = 0;
        key1_held = 0;
    }
    key1_prev = k1;
    if (key2_prev && !k2)
    {
        key2_press_t = now;
        key2_cnt++;
        key2_held = 1;
    }
    else if (!key2_prev && k2)
    {
        if (now - key2_press_t <= MS_TICKS(500))
        {
            key2_rel = 1;
            key2_rel_t = now;
        }
        else
            key2_cnt = 0;
        key2_held = 0;
    }
    key2_prev = k2;

    // inform off after 3s (motor_control)
    if (inform && now - inform_t >= MS_TICKS(3000))
    {
        inform = 0;
        gpio_out_write(ext2, 1);
        gpio_out_write(ext1, 0);
    }
    // KEY1 short click: 1x -> EXT2 LOW + inform; 2x+ -> is_error (pause)
    if (key1_rel && now - key1_rel_t > MS_TICKS(500))
    {
        key1_rel = 0;
        if (key1_cnt == 1)
        {
            gpio_out_write(ext2, 0);
            inform_t = now;
            inform = 1;
            is_error = 0;
        }
        else if (key1_cnt >= 2)
            is_error = 1;
        key1_cnt = 0;
    }
    // KEY2 short click: 1x -> EXT1 HIGH + inform; 2x+ -> is_error (pause)
    if (key2_rel && now - key2_rel_t > MS_TICKS(500))
    {
        key2_rel = 0;
        if (key2_cnt == 1)
        {
            gpio_out_write(ext1, 1);
            inform_t = now;
            inform = 1;
            is_error = 0;
        }
        else if (key2_cnt >= 2)
            is_error = 1;
        key2_cnt = 0;
    }

    // Long hold (>=500ms) OR mainboard signal low -> manual (motor_control)
    uint8_t back_act = (key1_held && now - key1_press_t >= MS_TICKS(500)) || !gpio_in_read(back_sig);
    uint8_t fwd_act = (key2_held && now - key2_press_t >= MS_TICKS(500)) || !gpio_in_read(front_sig);

    if (back_act || fwd_act)
    {
        if (current_job != JOB_NONE)
            send_job_result(current_job, JOB_RESULT_INTERRUPTED);
        current_job = JOB_NONE; // feed/retract always wins - a job never
                                // resumes after being interrupted
    }

    if (back_act)
    { // BACK priority (as in motor_control)
        if (manual != 2)
        {
            gpio_out_write(en_out, 0);
            tmc_write(TMC_VACTUAL, 0);
            tmc_delay_ms(2);
            tmc_write(TMC_GCONF, GCONF_VAL);
            tmc_delay_ms(2); // shaft=0 (BACK)
            tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
            manual = 2;
            last_state = ST_BACK;
        }
        return 1;
    }
    if (fwd_act)
    {
        if (manual != 1)
        {
            gpio_out_write(en_out, 0);
            tmc_write(TMC_VACTUAL, 0);
            tmc_delay_ms(2);
            tmc_write(TMC_GCONF, GCONF_VAL | GCONF_SHAFT);
            tmc_delay_ms(2); // shaft=1 (FWD)
            tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
            manual = 1;
            last_state = ST_FORWARD;
        }
        return 1;
    }
    if (manual)
    { // hold released -> stop
        tmc_write(TMC_VACTUAL, 0);
        tmc_delay_ms(2);
        gpio_out_write(en_out, 1);
        last_state = ST_STOP;
        if (manual == 2)
            is_error = 1; // standalone: is_error=true after a BACK long-press
        manual = 0;
        return 1;
    }

    uint8_t load_btn = gpio_in_read(key_external_load);
    uint8_t unload_btn = gpio_in_read(key_external_unload);

    if (load_btn_prev && !load_btn && current_job == JOB_NONE && !manual)
        start_job(JOB_LOAD, JOB_DEFAULT_TIMEOUT_MS);
    load_btn_prev = load_btn;

    if (unload_btn_prev && !unload_btn && current_job == JOB_NONE && !manual)
        start_job(JOB_UNLOAD, JOB_DEFAULT_TIMEOUT_MS);
    unload_btn_prev = unload_btn;

    return 0;
}

// ------------------------------------------------------------------
// Load/unload job execution - drives the motor directly while a job is
// active. Runs ahead of both the host_mode override and the autonomous
// hall state machine (see buffer_task). Returns 1 if a job is active and
// handled the motor this tick, 0 if no job is running.
// ------------------------------------------------------------------
static uint8_t
run_job(void)
{
    if (current_job == JOB_NONE)
        return 0;

    uint32_t now = timer_read_time();

    // Stall detection: no MDM encoder pulse for JOB_STALL_MS means filament
    // isn't actually moving, regardless of what the buffer motor is doing.
    uint8_t motion_now = gpio_in_read(mdm_motion);
    if (motion_now != mdm_motion_prev)
    {
        job_last_motion_time = now;
        mdm_motion_prev = motion_now;
    }

    if (now - job_last_motion_time > MS_TICKS(JOB_STALL_MS))
    {
        apply_state(ST_STOP);
        send_job_result(current_job, JOB_RESULT_STALLED);
        current_job = JOB_NONE;
        return 1;
    }

    if (now > job_deadline)
    {
        apply_state(ST_STOP);
        send_job_result(current_job, JOB_RESULT_TIMEOUT);
        current_job = JOB_NONE;
        return 1;
    }

    if (current_job == JOB_LOAD)
    {
        if (toolhead_sensor_present)
        {
            apply_state(ST_STOP);
            send_job_result(JOB_LOAD, JOB_RESULT_OK);
            current_job = JOB_NONE;
            return 1;
        }
        apply_state(ST_FORWARD);
        return 1;
    }

    // JOB_UNLOAD
    uint8_t mdm_empty = (gpio_in_read(mdm_filament) == MDM_FIL_NO_FILAMENT_STATE);
    uint8_t entrance_empty = gpio_in_read(fil); // FIL_PIN: 1 = no filament
    if (mdm_empty && entrance_empty)
    {
        apply_state(ST_STOP);
        send_job_result(JOB_UNLOAD, JOB_RESULT_OK);
        current_job = JOB_NONE;
        return 1;
    }
    apply_state(ST_BACK);
    return 1;
}

// ------------------------------------------------------------------
// Task - ports read_sensor_state + motor_control
// ------------------------------------------------------------------
void buffer_task(void)
{
    if (!tmc_ready)
    { // first call: timer is ready -> configure the TMC
        tmc_configure();
        tmc_ready = 1;
        return;
    }

    if (process_buttons()) // buttons/signals always take priority (manual)
        return;

    if (run_job()) // an active LOAD/UNLOAD job owns the motor this tick -
                   // deliberately runs ahead of the entrance-runout check
                   // below, since an empty entrance is the expected
                   // starting condition for a LOAD, not a fault
        return;

    if (host_mode != HMODE_AUTO)
    {
        if (timer_read_time() > host_mode_deadline)
        {
            host_mode = HMODE_AUTO; // watchdog: host went silent, don't run forever
        }
        else
        {
            uint8_t want = (host_mode == HMODE_FORWARD) ? ST_FORWARD
                           : (host_mode == HMODE_BACK)  ? ST_BACK
                                                        : ST_STOP;
            apply_state(want);
            return;
        }
    }

    // No filament at the entrance (PB7=1) -> stop (print runout = host on PB7)
    if (gpio_in_read(fil))
    {
        apply_state(ST_STOP);
        is_feeding = 0;
        is_error = 0;
        return;
    }

    if (is_error)
    { // pause (double-click / timeout) -> stop
        apply_state(ST_STOP);
        return;
    }

    // Hall state machine (obstructed = 1), same priority as buffer.cpp
    uint8_t state;
    if (gpio_in_read(hall3))
        state = ST_FORWARD; // pos1
    else if (gpio_in_read(hall2))
        state = ST_STOP; // pos2
    else if (gpio_in_read(hall1))
        state = ST_BACK; // pos3
    else
        state = last_state; // no hall triggered: hold state

    // Safety: continuous feed > 60s -> error (buffer.cpp front_time>timeout)
    if (state == ST_FORWARD)
    {
        if (!is_feeding)
        {
            is_feeding = 1;
            feed_start_time = timer_read_time();
        }
        else if (timer_read_time() - feed_start_time > MS_TICKS(FEED_TIMEOUT_MS))
        {
            is_error = 1;
            apply_state(ST_STOP);
            return;
        }
    }
    else
    {
        is_feeding = 0;
    }

    apply_state(state);
}
DECL_TASK(buffer_task);