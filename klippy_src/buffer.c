// LLL Buffer Plus - tâche autonome d'auto-feed pour firmware MCU Klipper
//
// Porte la machine à états du firmware standalone Mellow (Fly3DTeam/Buffer,
// lib/buffer/buffer.cpp) DANS le firmware MCU Klipper, sous forme d'une tâche
// autonome. La carte reste un MCU Klipper normal (le host lit PB7 via
// [filament_switch_sensor]) ET pilote le moteur du buffer toute seule.
//
// Fichier AJOUTÉ (ne modifie aucun fichier Klipper existant) -> survit aux git pull.
//
// Convention reprise MOT POUR MOT de buffer.cpp / buffer.h :
//   光感 (capteur photo) : obstrué = 1, non obstrué = 0  (lecture GPIO directe, PAS d'inversion ^!)
//   耗材开关 (PB7) : a du filament = 0, pas de filament = 1
//   按键 (boutons) : pressé = 0, relâché = 1
//   HALL3 (PB4) obstrué -> Forward (pousse)   [pos1, 耗材往前推]
//   HALL2 (PB3) obstrué -> Stop               [pos2, 电机停止]
//   HALL1 (PB2) obstrué -> Back (rétracte)     [pos3, 回退耗材]
//
// SEULE adaptation vs standalone : le maintien bouton (long-press) est géré
// en événementiel non-bloquant (la boucle while() bloquante du standalone
// bloquerait la comm Klipper plusieurs secondes). Comportement identique.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"   // CONFIG_CLOCK_FREQ
#include "board/gpio.h" // gpio_in_setup / gpio_out_setup
#include "board/misc.h" // timer_read_time
#include "command.h"    // DECL_COMMAND / sendf
#include "sched.h"      // DECL_TASK / DECL_INIT

// ------------------------------------------------------------------
// Pins (encodage Klipper : GPIO(PORT,NUM) = (PORT-'A')*16 + NUM)  [buffer.h]
// ------------------------------------------------------------------
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))

#define HALL1_PIN GPIO('B', 2)             // 光感3 -> pos3 -> Back
#define HALL2_PIN GPIO('B', 3)             // 光感2 -> pos2 -> Stop
#define HALL3_PIN GPIO('B', 4)             // 光感1 -> pos1 -> Forward
#define FIL_PIN GPIO('B', 7)               // ENDSTOP_3 / 耗材开关 (filament switch)
#define KEY_INTERNAL_RETRACT GPIO('B', 13) // KEY1 后退 (retract)
#define KEY_INTERNAL_FEED GPIO('B', 12)    // KEY2 前进 (feed)
#define KEY_EXTERNAL_RETRACT GPIO('A', 2)  // Exposed 3 pin header using for external Retract button
#define KEY_EXTERNAL_FEED GPIO('A', 3)     // Exposed 3 pin header using for external feed buttons
#define FRONT_SIG_PIN GPIO('B', 5)         // signal carte mère "avance" (actif bas)
#define BACK_SIG_PIN GPIO('B', 6)          // signal carte mère "recule" (actif bas)
#define EXT1_PIN GPIO('C', 13)             // EXTENSION_PIN1
#define EXT2_PIN GPIO('C', 14)             // EXTENSION_PIN2
#define EN_PIN GPIO('A', 6)                // TMC enable (actif bas)
#define UART_PIN GPIO('B', 1)              // TMC2208 single-wire UART (PDN_UART)

// ------------------------------------------------------------------
// Paramètres (repris du standalone, AUCUNE modif)
// ------------------------------------------------------------------
// VACTUAL = SPEED * microsteps * 200 / 60 / 0.715   (buffer.cpp)
//   SPEED = 260 r/min, microsteps = 64  -> 77575 (idem cast int du standalone)
#define SPEED_RPM 400
#define MICROSTEPS_DIV 64
#define VACTUAL_MAG ((int32_t)((int64_t)SPEED_RPM * MICROSTEPS_DIV * 200 / 60 * 1000 / 715))

#define FEED_TIMEOUT_MS 60000 // sécurité : stop si Forward continu > 60s (buffer.cpp)

// conversion ms -> ticks horloge MCU
#define MS_TICKS(ms) ((uint32_t)(ms) * (CONFIG_CLOCK_FREQ / 1000))

// Bit-bang UART : 9600 bauds (comme le standalone : beginSerial(9600))
#define UART_BAUD 9600
#define BIT_TICKS (CONFIG_CLOCK_FREQ / UART_BAUD) // 48e6/9600 = 5000 ticks/bit

// ------------------------------------------------------------------
// Registres TMC2208 (adresses datasheet) + valeurs (DÉRIVÉES de TMcstepper)
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
#define CHOPCONF_VAL 0x12020055UL   // toff=5,vsense=1,mres=2(64µ),intpol=1
#define IHOLD_IRUN_VAL 0x00010F07UL // IHOLD=7,IRUN=15,iholddelay=1 (rms_current(500)@0.11)
#define PWMCONF_VAL 0xC10D0024UL    // défaut TMC2208 (pwm_autoscale)

// ------------------------------------------------------------------
// État
// ------------------------------------------------------------------
enum
{
    ST_STOP = 0,
    ST_FORWARD,
    ST_BACK
};

static struct gpio_in hall1, hall2, hall3, fil, key_internal_retract, key_internal_feed, key_external_retract, key_external_feed, front_sig, back_sig;
static struct gpio_out en_out, uart_out, ext1, ext2;
static uint8_t last_state = ST_STOP;
static uint8_t tmc_ready = 0;
static uint32_t feed_start_time;
static uint8_t is_feeding = 0;
static uint8_t is_error = 0;

// état boutons (remplace les attachInterrupt du standalone par du polling)
static uint8_t key1_prev = 1, key2_prev = 1;
static uint32_t key1_press_t, key2_press_t, key1_rel_t, key2_rel_t;
static uint8_t key1_held = 0, key2_held = 0;
static uint8_t key1_rel = 0, key2_rel = 0;
static uint8_t key1_cnt = 0, key2_cnt = 0;
static uint8_t manual = 0; // 0=aucun, 1=forward, 2=back
static uint8_t inform = 0;
static uint32_t inform_t;

// ------------------------------------------------------------------
// UART TMC2208 : bit-bang TX bloquant, 9600 bauds, IRQ activées.
// Trame UART par octet : start(0), 8 bits LSB-first, stop(1). Idle = haut.
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

// CRC8 TMC (poly 0x07), sur les 7 premiers octets (identique TMC2208Stepper::calcCRC)
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

// Écriture d'un registre TMC2208 (write-only) — datagramme identique à write()
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
// Config TMC2208 — port de buffer_motor_init. Appelée au 1er appel de la tâche
// (PAS dans DECL_INIT : ordre d'init non garanti, le timer pourrait ne pas
// tourner). Suppose la 24V présente (24V d'abord, USB ensuite).
// ------------------------------------------------------------------
static void
tmc_configure(void)
{
    tmc_delay_ms(10); // stabilisation alim TMC
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

// ------------------------------------------------------------------
// Init (au boot) — GPIO uniquement (self-contained). Port buffer_sensor_init.
// ------------------------------------------------------------------
void buffer_init(void)
{
    // Capteurs / boutons : INPUT brut (standalone pinMode INPUT) ; signaux carte = PULLUP
    hall1 = gpio_in_setup(HALL1_PIN, 0);
    hall2 = gpio_in_setup(HALL2_PIN, 0);
    hall3 = gpio_in_setup(HALL3_PIN, 0);
    fil = gpio_in_setup(FIL_PIN, 0);
    key_internal_retract = gpio_in_setup(KEY_INTERNAL_RETRACT, 0);
    key_internal_feed = gpio_in_setup(KEY_INTERNAL_FEED, 0);
    key_external_retract = gpio_in_setup(KEY_EXTERNAL_RETRACT, 1);
    key_external_feed = gpio_in_setup(KEY_EXTERNAL_FEED, 1);
    front_sig = gpio_in_setup(FRONT_SIG_PIN, 1); // INPUT_PULLUP (actif bas)
    back_sig = gpio_in_setup(BACK_SIG_PIN, 1);   // INPUT_PULLUP (actif bas)
    // Sorties
    en_out = gpio_out_setup(EN_PIN, 1);     // moteur désactivé (actif bas)
    uart_out = gpio_out_setup(UART_PIN, 1); // ligne UART au repos = haut
    ext1 = gpio_out_setup(EXT1_PIN, 0);     // EXTENSION_PIN1 LOW (buffer.cpp)
    ext2 = gpio_out_setup(EXT2_PIN, 1);     // EXTENSION_PIN2 HIGH (buffer.cpp)
}
DECL_INIT(buffer_init);

// ------------------------------------------------------------------
// Application d'un état moteur (écrit au TMC que sur changement).
// Direction = bit shaft + VACTUAL POSITIF + stop avant inversion (motor_control).
// FORWARD = shaft=1 (buffer.h FORWARD=1) ; BACK = shaft=0.
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
// Boutons + signaux carte mère — port de la section boutons de motor_control,
// rendu non-bloquant. Retourne 1 si un mode manuel est actif (=> on saute
// l'auto-feed cette itération), 0 sinon.
// ------------------------------------------------------------------
static uint8_t
process_buttons(void)
{
    uint32_t now = timer_read_time();
    uint8_t k1 = gpio_in_read(key_internal_retract) && gpio_in_read(key_external_retract); // 1=relâché, 0=pressé
    uint8_t k2 = gpio_in_read(key_internal_feed) && gpio_in_read(key_external_feed);

    // détection de fronts (remplace key_it_callback CHANGE)
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

    // inform off après 3s (motor_control)
    if (inform && now - inform_t >= MS_TICKS(3000))
    {
        inform = 0;
        gpio_out_write(ext2, 1);
        gpio_out_write(ext1, 0);
    }
    // KEY1 clic court : 1x -> EXT2 LOW + inform ; 2x+ -> is_error (pause)
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
    // KEY2 clic court : 1x -> EXT1 HIGH + inform ; 2x+ -> is_error (pause)
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

    // Maintien long (>=500ms) OU signal carte bas -> manuel (motor_control)
    uint8_t back_act = (key1_held && now - key1_press_t >= MS_TICKS(500)) || !gpio_in_read(back_sig);
    uint8_t fwd_act = (key2_held && now - key2_press_t >= MS_TICKS(500)) || !gpio_in_read(front_sig);

    if (back_act)
    { // priorité BACK (comme motor_control)
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
    { // fin du maintien -> stop
        tmc_write(TMC_VACTUAL, 0);
        tmc_delay_ms(2);
        gpio_out_write(en_out, 1);
        last_state = ST_STOP;
        if (manual == 2)
            is_error = 1; // standalone : is_error=true après BACK long-press
        manual = 0;
        return 1;
    }
    return 0;
}

// ------------------------------------------------------------------
// Tâche — port de read_sensor_state + motor_control
// ------------------------------------------------------------------
void buffer_task(void)
{
    if (!tmc_ready)
    { // 1er appel : timer prêt -> config TMC
        tmc_configure();
        tmc_ready = 1;
        return;
    }

    if (process_buttons()) // boutons/signaux = priorité (manuel)
        return;

    // Adding support for Host commanded moves, while allowing hardware buttons to override.
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

    // Plus de filament à l'entrée (PB7=1) -> stop (le runout print = host sur PB7)
    if (gpio_in_read(fil))
    {
        apply_state(ST_STOP);
        is_feeding = 0;
        is_error = 0;
        return;
    }

    if (is_error)
    { // pause (double-clic / timeout) -> stop
        apply_state(ST_STOP);
        return;
    }

    // Machine à états halls (obstrué = 1), priorité comme buffer.cpp
    uint8_t state;
    if (gpio_in_read(hall3))
        state = ST_FORWARD; // pos1
    else if (gpio_in_read(hall2))
        state = ST_STOP; // pos2
    else if (gpio_in_read(hall1))
        state = ST_BACK; // pos3
    else
        state = last_state; // aucun hall : garde l'état

    // Sécurité : feed continu > 60s -> erreur (buffer.cpp front_time>timeout)
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
