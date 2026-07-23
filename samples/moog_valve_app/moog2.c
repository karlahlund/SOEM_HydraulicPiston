/*
  moog2.c  (single-file SOEM 2.x app, fixed slave order)

  Physical order (fixed):
    1 EK1100
    2 EL1008  (DI)
    3 EL2008  (DO)
    4 EL3062  (AI, 0..10V confirmed)
    5 EL4032  (AO +/-10V)

  Modes:
    sudo ./moog_valve_app eth0             -> normal control loop
    sudo ./moog_valve_app eth0 --monitor   -> bench monitor, manual outputs only
*/

#include "soem/soem.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

/* ===================== User config ===================== */
#define DEBUG_PRINT 0

#define CYCLE_USEC 100000            /* 100 ms */
#define SLEW_RATE_V_PER_SEC 5.0f
#define DIR_FLIP_NEUTRAL_HOLD_CYCLES 5

/* DI assignments (EL1008) */
#define DI_FWD_BIT 0
#define DI_REV_BIT 1

/* DO assignments (EL2008) */
#define DO_SOL_FWD_BIT 0
#define DO_SOL_REV_BIT 1

/* EL3062 value offsets within its own input area.
   Default assumes status(2)+value(2) per channel.
   If mapped value-only, use 0 and 2. */
#define EL3062_CH1_VALUE_OFFSET     2
#define EL3062_CH2_VALUE_OFFSET     6

#define EXPECTED_SLAVE_COUNT 5

/* ===================== Printing ===================== */
#define EPRINTF(...) do { fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)

#if DEBUG_PRINT
  #define DPRINTF(...) do { printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
  #define DPRINTF(...) do { } while(0)
#endif

/* ===================== Scaling helpers ===================== */
static inline float clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* EL3062 confirmed 0..10V: 32767 -> 10.000V.
   Negative raw on a unipolar terminal indicates wiring/config fault. */
static inline float raw_to_v_0_10(int16_t raw)
{
  if (raw < 0) raw = 0;
  return ((float)raw) * 10.0f / 32767.0f;
}

/* EL4032 is bipolar: command carries direction for the servo valve. */
static inline int16_t v_to_raw_pm10(float v)
{
  v = clampf(v, -10.0f, 10.0f);
  return (int16_t)(v * 32767.0f / 10.0f);
}

static float slew_limit(float target, float prev, float dt_sec, float slew_v_per_sec)
{
  float max_step = slew_v_per_sec * dt_sec;
  float delta = target - prev;
  if (delta >  max_step) delta =  max_step;
  if (delta < -max_step) delta = -max_step;
  return prev + delta;
}

/* ===================== Fieldbus wrapper ===================== */
typedef struct {
  ecx_contextt ctx;
  char *iface;
  uint8_t group;
  uint8_t iomap[4096];
  int roundtrip_us;
} Fieldbus;

static void fieldbus_initialize(Fieldbus *fb, char *iface)
{
  memset(fb, 0, sizeof(*fb));
  fb->iface = iface;
  fb->group = 0;
}

static int fieldbus_roundtrip(Fieldbus *fb)
{
  ec_timet start, end, diff;
  int wkc;

  start = osal_current_time();
  ecx_send_processdata(&fb->ctx);
  wkc = ecx_receive_processdata(&fb->ctx, EC_TIMEOUTRET);
  end = osal_current_time();

  osal_time_diff(&start, &end, &diff);
  fb->roundtrip_us = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);
  return wkc;
}

static int fieldbus_expected_wkc(Fieldbus *fb)
{
  ec_groupt *grp = fb->ctx.grouplist + fb->group;
  return grp->outputsWKC * 2 + grp->inputsWKC;
}

/* Expected identities. Verify these against your slaveinfo output;
   mismatches only produce warnings, not failures. */
static const struct { uint32_t pid; const char *name; } expected_slaves[EXPECTED_SLAVE_COUNT] = {
  { 0x044c2c52, "EK1100" },
  { 0x03f03052, "EL1008" },
  { 0x07d83052, "EL2008" },
  { 0x0bf63052, "EL3062" },
  { 0x0fc03052, "EL4032" },
};

static void dump_bad_states(Fieldbus *fb, uint16_t want)
{
  ecx_readstate(&fb->ctx);
  for (int i = 1; i <= fb->ctx.slavecount; ++i) {
    ec_slavet *s = fb->ctx.slavelist + i;
    if (s->state != want) {
      EPRINTF("  slave %d %-10s state=0x%02x AL=0x%04x : %s\n",
              i, s->name, s->state, s->ALstatuscode,
              ec_ALstatuscode2string(s->ALstatuscode));
    }
  }
}

static boolean fieldbus_start(Fieldbus *fb)
{
  ec_slavet *slave0;
  int i, chk, used;

  EPRINTF("ecx_init(%s)\n", fb->iface);
  if (!ecx_init(&fb->ctx, fb->iface)) {
    EPRINTF("FAIL: ecx_init(%s) - no socket. Running as root? Correct interface?\n", fb->iface);
    return FALSE;
  }

  slave0 = fb->ctx.slavelist;

  if (ecx_config_init(&fb->ctx) <= 0) {
    EPRINTF("FAIL: ecx_config_init found no slaves\n");
    return FALSE;
  }
  EPRINTF("Found %d slaves\n", fb->ctx.slavecount);

  if (fb->ctx.slavecount != EXPECTED_SLAVE_COUNT) {
    EPRINTF("FAIL: expected %d slaves, found %d\n",
            EXPECTED_SLAVE_COUNT, fb->ctx.slavecount);
    return FALSE;
  }

  for (i = 1; i <= fb->ctx.slavecount; ++i) {
    ec_slavet *s = fb->ctx.slavelist + i;
    EPRINTF("  %d: %-12s man=0x%08x id=0x%08x rev=0x%08x\n",
            i, s->name, s->eep_man, s->eep_id, s->eep_rev);
    if (s->eep_id != expected_slaves[i-1].pid) {
      EPRINTF("     WARN: expected %s (0x%08x), got 0x%08x - fixed mapping may be wrong\n",
              expected_slaves[i-1].name, expected_slaves[i-1].pid, s->eep_id);
    }
  }

  used = ecx_config_map_group(&fb->ctx, fb->iomap, fb->group);
  EPRINTF("IOmap used %d of %d bytes\n", used, (int)sizeof(fb->iomap));
  if (used <= 0 || used > (int)sizeof(fb->iomap)) {
    EPRINTF("FAIL: iomap size out of range\n");
    return FALSE;
  }

  ecx_configdc(&fb->ctx);

  EPRINTF("Waiting for SAFE_OP...\n");
  ecx_statecheck(&fb->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  if (slave0->state != EC_STATE_SAFE_OP) {
    EPRINTF("FAIL: not all slaves reached SAFE_OP (group state 0x%02x)\n", slave0->state);
    dump_bad_states(fb, EC_STATE_SAFE_OP);
    return FALSE;
  }

  /* Valid process data before requesting OP, or the SM watchdog trips
     and slaves fall back to SAFE_OP + ERROR */
  for (i = 0; i < 4; ++i) {
    ecx_send_processdata(&fb->ctx);
    ecx_receive_processdata(&fb->ctx, EC_TIMEOUTRET);
  }

  EPRINTF("Requesting OPERATIONAL...\n");
  slave0->state = EC_STATE_OPERATIONAL;
  ecx_writestate(&fb->ctx, 0);

  chk = 200;
  do {
    ecx_send_processdata(&fb->ctx);
    ecx_receive_processdata(&fb->ctx, EC_TIMEOUTRET);
    ecx_statecheck(&fb->ctx, 0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (slave0->state != EC_STATE_OPERATIONAL));

  if (slave0->state == EC_STATE_OPERATIONAL) {
    EPRINTF("All slaves OPERATIONAL\n");
    return TRUE;
  }

  EPRINTF("FAIL: not all slaves reached OP\n");
  dump_bad_states(fb, EC_STATE_OPERATIONAL);
  return FALSE;
}

static void fieldbus_stop(Fieldbus *fb)
{
  ec_slavet *slave0 = fb->ctx.slavelist;
  if (slave0) {
    slave0->state = EC_STATE_INIT;
    ecx_writestate(&fb->ctx, 0);
  }
  ecx_close(&fb->ctx);
}

static void fieldbus_check_state(Fieldbus *fb)
{
  ec_groupt *grp = fb->ctx.grouplist + fb->group;
  grp->docheckstate = FALSE;
  ecx_readstate(&fb->ctx);

  for (int i = 1; i <= fb->ctx.slavecount; ++i) {
    ec_slavet *s = fb->ctx.slavelist + i;
    if (s->group != fb->group) continue;

    if (s->state != EC_STATE_OPERATIONAL) {
      grp->docheckstate = TRUE;

      if (s->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
        s->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
        ecx_writestate(&fb->ctx, i);
      } else if (s->state == EC_STATE_SAFE_OP) {
        s->state = EC_STATE_OPERATIONAL;
        ecx_writestate(&fb->ctx, i);
      } else if (s->state > EC_STATE_NONE) {
        if (ecx_reconfig_slave(&fb->ctx, i, EC_TIMEOUTRET)) {
          s->islost = FALSE;
        }
      } else if (!s->islost) {
        ecx_statecheck(&fb->ctx, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (s->state == EC_STATE_NONE) s->islost = TRUE;
      }
    } else if (s->islost) {
      if (s->state != EC_STATE_NONE) {
        s->islost = FALSE;
      } else if (ecx_recover_slave(&fb->ctx, i, EC_TIMEOUTRET)) {
        s->islost = FALSE;
      }
    }
  }
}

/* ===================== Fixed IO mapping (pointer-based) ===================== */
typedef struct {
  int s_ek1100, s_el1008, s_el2008, s_el3062, s_el4032;

  uint8_t *di;    /* EL1008 inputs  */
  uint8_t *dout;  /* EL2008 outputs */
  uint8_t *ai;    /* EL3062 inputs  */
  uint8_t *ao;    /* EL4032 outputs */

  uint32_t di_bytes, do_bytes, ai_bytes, ao_bytes;
} IOMap;

static int iomap_bind_fixed(IOMap *io, Fieldbus *fb)
{
  ec_slavet *sl = fb->ctx.slavelist;

  memset(io, 0, sizeof(*io));

  io->s_ek1100 = 1;
  io->s_el1008 = 2;
  io->s_el2008 = 3;
  io->s_el3062 = 4;
  io->s_el4032 = 5;

  if (fb->ctx.slavecount < EXPECTED_SLAVE_COUNT) return 0;

  io->di   = sl[io->s_el1008].inputs;
  io->dout = sl[io->s_el2008].outputs;
  io->ai   = sl[io->s_el3062].inputs;
  io->ao   = sl[io->s_el4032].outputs;

  io->di_bytes = sl[io->s_el1008].Ibytes;
  io->do_bytes = sl[io->s_el2008].Obytes;
  io->ai_bytes = sl[io->s_el3062].Ibytes;
  io->ao_bytes = sl[io->s_el4032].Obytes;

  if (!io->di || !io->dout || !io->ai || !io->ao) {
    EPRINTF("FAIL: null IO pointer (di=%p do=%p ai=%p ao=%p)\n",
            (void *)io->di, (void *)io->dout, (void *)io->ai, (void *)io->ao);
    return 0;
  }

  /* EL3062 needs 8 bytes for status+value on two channels;
     EL4032 needs 4 for two 16-bit outputs. Guard the casts below. */
  if (io->ai_bytes < (uint32_t)(EL3062_CH2_VALUE_OFFSET + 2)) {
    EPRINTF("FAIL: EL3062 input area %u bytes, need >= %d. "
            "Terminal likely mapped value-only - set EL3062_CH*_VALUE_OFFSET to 0/2.\n",
            io->ai_bytes, EL3062_CH2_VALUE_OFFSET + 2);
    return 0;
  }
  if (io->ao_bytes < 2) {
    EPRINTF("FAIL: EL4032 output area %u bytes, need >= 2\n", io->ao_bytes);
    return 0;
  }

  return 1;
}

static inline void outputs_safe(IOMap *io)
{
  io->dout[0] &= (uint8_t)~((1u << DO_SOL_FWD_BIT) | (1u << DO_SOL_REV_BIT));
  *(int16_t *)(io->ao + 0) = v_to_raw_pm10(0.0f);
}

/* ===================== Monitor / bench mode ===================== */
static void print_slave_states(Fieldbus *fb)
{
  ecx_readstate(&fb->ctx);
  for (int i = 1; i <= fb->ctx.slavecount; ++i) {
    ec_slavet *s = fb->ctx.slavelist + i;
    long ioff = s->inputs  ? (long)(s->inputs  - fb->iomap) : -1;
    long ooff = s->outputs ? (long)(s->outputs - fb->iomap) : -1;
    printf("  %d %-10s state=0x%02x AL=0x%04x  Ibits=%3u Ibytes=%2u @%4ld  "
           "Obits=%3u Obytes=%2u @%4ld\n",
           i, s->name, s->state, s->ALstatuscode,
           s->Ibits, s->Ibytes, ioff,
           s->Obits, s->Obytes, ooff);
  }
}

static void monitor_loop(Fieldbus *fb, IOMap *io)
{
  const int expected = fieldbus_expected_wkc(fb);
  uint8_t manual_do = 0;
  float   manual_ao = 0.0f;
  int cycle = 0;
  struct termios oldt, newt;
  int oldfl;

  printf("\n=== MONITOR MODE - outputs driven only by manual keys ===\n");
  printf("Expected WKC: %d\n", expected);
  print_slave_states(fb);
  printf("\nKeys: 0/1 toggle DO0/DO1   +/- AO step 0.5V   z zero AO   q quit\n\n");

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= (tcflag_t)~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldfl = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldfl | O_NONBLOCK);

  for (;;)
  {
    int wkc = fieldbus_roundtrip(fb);
    int c = getchar();

    if (c == 'q') break;
    else if (c == '0') manual_do ^= (uint8_t)(1u << DO_SOL_FWD_BIT);
    else if (c == '1') manual_do ^= (uint8_t)(1u << DO_SOL_REV_BIT);
    else if (c == '+') manual_ao = clampf(manual_ao + 0.5f, -10.0f, 10.0f);
    else if (c == '-') manual_ao = clampf(manual_ao - 0.5f, -10.0f, 10.0f);
    else if (c == 'z') manual_ao = 0.0f;

    io->dout[0] = manual_do;
    *(int16_t *)(io->ao + 0) = v_to_raw_pm10(manual_ao);

    uint8_t  di      = io->di[0];
    int16_t  ai1_raw = *(int16_t *)(io->ai + EL3062_CH1_VALUE_OFFSET);
    int16_t  ai2_raw = *(int16_t *)(io->ai + EL3062_CH2_VALUE_OFFSET);
    uint16_t ai1_sts = *(uint16_t *)(io->ai + 0);
    uint16_t ai2_sts = *(uint16_t *)(io->ai + 4);

    if ((cycle++ % 5) == 0) {
      int f = (di >> DI_FWD_BIT) & 1;
      int r = (di >> DI_REV_BIT) & 1;

      printf("\033[H\033[J");
      printf("WKC %d/%d   group state 0x%02x   roundtrip %d us\n\n",
             wkc, expected, fb->ctx.slavelist[0].state, fb->roundtrip_us);

      printf("DI (EL1008) raw 0x%02X   ", di);
      for (int b = 7; b >= 0; --b) putchar((di >> b) & 1 ? '1' : '0');
      printf("\n   FWD(bit%d)=%d  REV(bit%d)=%d   -> %s\n",
             DI_FWD_BIT, f, DI_REV_BIT, r,
             (f && r) ? "FAULT(both)" : f ? "FWD" : r ? "REV" : "OFF");

      printf("\nAI (EL3062)\n");
      printf("   CH1 raw %6d  %7.3f V   status 0x%04X %s\n",
             ai1_raw, raw_to_v_0_10(ai1_raw), ai1_sts,
             (ai1_sts & 0x01) ? "[UNDERRANGE]" : (ai1_sts & 0x02) ? "[OVERRANGE]" : "");
      printf("   CH2 raw %6d  %7.3f V   status 0x%04X\n",
             ai2_raw, raw_to_v_0_10(ai2_raw), ai2_sts);

      printf("\nDO (EL2008) raw 0x%02X   ", manual_do);
      for (int b = 7; b >= 0; --b) putchar((manual_do >> b) & 1 ? '1' : '0');
      printf("\n   SOL_FWD=%d  SOL_REV=%d\n",
             (manual_do >> DO_SOL_FWD_BIT) & 1,
             (manual_do >> DO_SOL_REV_BIT) & 1);

      printf("\nAO (EL4032) CH1 %+7.3f V   raw %6d\n",
             manual_ao, v_to_raw_pm10(manual_ao));

      printf("\n[0/1] toggle DO   [+/-] AO 0.5V   [z] zero   [q] quit\n");
      fflush(stdout);
    }

    osal_usleep(CYCLE_USEC);
  }

  io->dout[0] = 0;
  *(int16_t *)(io->ao + 0) = 0;
  fieldbus_roundtrip(fb);

  fcntl(STDIN_FILENO, F_SETFL, oldfl);
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  printf("\nMonitor exit - outputs zeroed.\n");
}

/* ===================== Control state ===================== */
typedef enum {
  DIR_OFF = 0,
  DIR_FWD = 1,
  DIR_REV = 2
} Direction;

/* ===================== Main ===================== */
int main(int argc, char *argv[])
{
  Fieldbus fb;
  IOMap io;
  int monitor = 0;

  if (argc < 2) {
    fprintf(stderr, "usage: %s <iface> [--monitor]\n", argv[0]);
    return 1;
  }
  if (argc >= 3 && strcmp(argv[2], "--monitor") == 0) monitor = 1;

  fieldbus_initialize(&fb, argv[1]);

  if (!fieldbus_start(&fb)) {
    fieldbus_stop(&fb);
    return 1;
  }

  if (!iomap_bind_fixed(&io, &fb)) {
    EPRINTF("FAIL: iomap bind\n");
    fieldbus_stop(&fb);
    return 1;
  }

  EPRINTF("IO bound: DI@%ld(%u B)  DO@%ld(%u B)  AI@%ld(%u B)  AO@%ld(%u B)\n",
          (long)(io.di - fb.iomap),   io.di_bytes,
          (long)(io.dout - fb.iomap), io.do_bytes,
          (long)(io.ai - fb.iomap),   io.ai_bytes,
          (long)(io.ao - fb.iomap),   io.ao_bytes);

  if (monitor) {
    monitor_loop(&fb, &io);
    fieldbus_stop(&fb);
    return 0;
  }

  /* Headless only after a successful init */
  if (!DEBUG_PRINT) {
    if (!freopen("/dev/null", "w", stdout)) { /* ignore */ }
    if (!freopen("/dev/null", "w", stderr)) { /* ignore */ }
  }

  const int expected = fieldbus_expected_wkc(&fb);

  float cmd_prev = 0.0f;
  const float dt_sec = (float)CYCLE_USEC / 1000000.0f;

  Direction last_dir = DIR_OFF;
  int neutral_hold = 0;

  for (;;)
  {
    int wkc = fieldbus_roundtrip(&fb);

    int ok = 1;
    if (wkc < expected) ok = 0;
    if (fb.ctx.slavelist[0].state != EC_STATE_OPERATIONAL) ok = 0;

    if (!ok) {
      outputs_safe(&io);
      cmd_prev = 0.0f;
      last_dir = DIR_OFF;
      neutral_hold = 0;

      fieldbus_check_state(&fb);
      osal_usleep(CYCLE_USEC);
      continue;
    }

    /* ---- READ DI: switch ---- */
    uint8_t di = io.di[0];
    int fwd = (di >> DI_FWD_BIT) & 0x01;
    int rev = (di >> DI_REV_BIT) & 0x01;

    Direction dir = DIR_OFF;
    if (fwd && !rev) dir = DIR_FWD;
    else if (rev && !fwd) dir = DIR_REV;

    /* ---- READ AI: pot on EL3062 CH1 (0..10V) ---- */
    int16_t raw_pot = *(int16_t *)(io.ai + EL3062_CH1_VALUE_OFFSET);
    float pot_v = clampf(raw_to_v_0_10(raw_pot), 0.0f, 10.0f);

    /* ---- Direction flip neutral hold ---- */
    if ((last_dir == DIR_FWD && dir == DIR_REV) || (last_dir == DIR_REV && dir == DIR_FWD)) {
      neutral_hold = DIR_FLIP_NEUTRAL_HOLD_CYCLES;
    }
    last_dir = dir;

    int sol_fwd = 0, sol_rev = 0;
    float target_cmd_v = 0.0f;

    if (neutral_hold > 0) {
      neutral_hold--;
    } else {
      if (dir == DIR_FWD) {
        sol_fwd = 1;
        target_cmd_v = +pot_v;
      } else if (dir == DIR_REV) {
        sol_rev = 1;
        target_cmd_v = -pot_v;
      }
    }

    /* ---- Slew-limit analog command ---- */
    float cmd_v = slew_limit(target_cmd_v, cmd_prev, dt_sec, SLEW_RATE_V_PER_SEC);
    cmd_prev = cmd_v;

    /* ---- WRITE DO: solenoids ---- */
    uint8_t out = io.dout[0];
    out &= (uint8_t)~((1u << DO_SOL_FWD_BIT) | (1u << DO_SOL_REV_BIT));
    if (sol_fwd) out |= (uint8_t)(1u << DO_SOL_FWD_BIT);
    if (sol_rev) out |= (uint8_t)(1u << DO_SOL_REV_BIT);
    io.dout[0] = out;

    /* ---- WRITE AO: EL4032 CH1 ---- */
    *(int16_t *)(io.ao + 0) = v_to_raw_pm10(cmd_v);

    osal_usleep(CYCLE_USEC);
  }

  return 0;
}
