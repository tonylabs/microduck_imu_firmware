/**
 * @file    dxl_slave.c
 * @brief   Dynamixel Protocol 2.0 slave over USART1, half-duplex.
 *
 * ## Owning one shared wire
 *
 * Sixteen devices share DATA and take turns. The protocol layer here is the
 * same on every board revision; what differs is how this MCU lets go of the
 * wire when it is not its turn, and that lives in line_drive() / line_release()
 * behind DXL_BUFFERED from board.h.
 *
 * **Rev 1, DXL_BUFFERED = 0.** PB6 runs straight to the connector, and
 * USART1_RX cannot be routed to it, so USART1 runs single-wire half-duplex
 * (HDSEL), feeding its receiver from the TX pin. HDSEL alone would idle the pin
 * high and fight every servo; the fix is the output type. PB6 sits in
 * alternate-function **open-drain** while receiving, so an idle-high
 * transmitter releases it, and flips to push-pull for the few hundred
 * microseconds it takes to send. Open-drain the whole time would be simpler,
 * but at 1 Mbps the rise time through the bus pull-up alone is far too slow.
 *
 * **Rev 2, DXL_BUFFERED = 1.** A 74LVC1G125 sits between PB6 and the bus, and
 * the bus comes back to PB7 as a normal USART1_RX. Its OE# is driven by
 * USART1's own driver-enable output on PA12: with DEM set, the USART asserts DE
 * before every start bit and drops it after the last stop bit, timed in
 * sixteenths of a bit by DEAT/DEDT. The firmware never touches the line and
 * line_drive() / line_release() are empty. Because PB7 is on the bus,
 * everything this board transmits is also received - RE is dropped for the
 * duration of a transmission, exactly as HDSEL requires on Rev 1.
 *
 * ## Latency
 *
 * The whole protocol runs in the USART1 interrupt: by the time the last CRC
 * byte of a request has arrived the response is built and the first byte is in
 * TDR. return_delay_time is 0, matching what the host asserts of the servos.
 *
 * Packets addressed to someone else - every servo status packet on the bus,
 * about fifteen per tick - are dropped at the ID byte, before any CRC is
 * computed. Byte stuffing guarantees FF FF FD 00 never appears inside a
 * payload, so resynchronising on the header is safe.
 */

#include "dxl_slave.h"

#include <string.h>

#include "board.h"

/* CMSIS names differ between FIFO-capable and plain USART headers. */
#ifndef USART_ISR_RXNE_RXFNE
#define USART_ISR_RXNE_RXFNE USART_ISR_RXNE
#endif
#ifndef USART_ISR_TXE_TXFNF
#define USART_ISR_TXE_TXFNF USART_ISR_TXE
#endif
#ifndef USART_CR1_RXNEIE_RXFNEIE
#define USART_CR1_RXNEIE_RXFNEIE USART_CR1_RXNEIE
#endif
#ifndef USART_CR1_TXEIE_TXFNFIE
#define USART_CR1_TXEIE_TXFNFIE USART_CR1_TXEIE
#endif
#ifndef USART_ICR_NECF
#define USART_ICR_NECF USART_ICR_NCF
#endif

/* -- protocol constants --------------------------------------------------- */

#define INST_PING      0x01u
#define INST_READ      0x02u
#define INST_WRITE     0x03u
#define INST_SYNC_READ 0x82u
#define INST_STATUS    0x55u

#define ERR_NONE          0x00u
#define ERR_DATA_RANGE    0x04u
#define ERR_DATA_LENGTH   0x05u
#define ERR_INSTRUCTION   0x02u

/** Longest request that is ours to buffer: a broadcast SYNC_WRITE of goal
 *  positions for fifteen servos is 88 bytes. 160 leaves room for a longer one
 *  without a second thought. */
#define DXL_RX_MAX 160u

/** Header + id + len + inst + err + 20 params + worst-case stuffing + CRC. */
#define DXL_TX_MAX 64u

/* -- state ---------------------------------------------------------------- */

static volatile uint8_t  s_table[DXL_TABLE_SIZE];
static volatile uint32_t s_replies;

static uint8_t  s_rx[DXL_RX_MAX];
static uint16_t s_rx_len;     /* LEN field of the packet being received    */
static uint16_t s_rx_pos;     /* bytes of the LEN-counted body received    */
static uint8_t  s_state;
static uint16_t s_last_rx_us; /* TIM3 stamp of the previous byte           */

/** Inter-byte gap, in microseconds, that abandons a part-received packet.
 *
 *  Bytes inside a packet arrive back to back - 10 us apart at 1 Mbps - so any
 *  real gap means the rest of that packet is never coming. Without this a
 *  truncated packet keeps counting down its LEN into the *next* packet's bytes
 *  and costs a second transaction as well as the one that was lost, which on
 *  this bus is a whole missed control tick.
 *
 *  Resynchronising on the header instead would be tempting, since byte stuffing
 *  keeps FF FF FD 00 out of any payload - but not out of the two CRC bytes,
 *  which are not stuffed, so a packet whose CRC happened to be 00 FD after two
 *  FF payload bytes would be thrown away. A gap cannot be faked that way. */
#define RX_IDLE_RESET_US 2000u

/* 2 ms, not the 100 us it was. The host's UART loads its transmit FIFO in
 * chunks and the line idles until an interrupt refills it, and on the Radxa
 * that interrupt is routinely 100-300 us late: measured with the v4 counters,
 * the 100 us rule abandoned 9 of 300 fifteen-byte requests and 26 of 300
 * thirty-byte ones - every one of them clean, no CRC or line error, just a
 * pause in the middle. A truncated packet is still abandoned long before the
 * next request, which is a control tick (20 ms) away. */

/* -- diagnostic counters ---------------------------------------------------
 *
 * Served in the control table at DXL_DIAG_ADDR, u16 little-endian each, so a
 * host can read them over the bus and see where a request it never got an
 * answer to actually died: never assembled (bytes lost - compare `ours` to
 * what was sent), assembled but corrupt (`crc_fail`), a gap mid-packet
 * (`gap_reset`), the receiver flagging framing/noise/overrun (`line_err`),
 * served but the reply never completed (`served` vs `tx_done`). Wrap at 65536
 * and are never cleared; the host takes differences. */
enum {
    DIAG_OURS = 0,   /* headers with our ID or broadcast, reached packet_complete */
    DIAG_CRC_FAIL,   /* ... and failed CRC */
    DIAG_GAP_RESET,  /* part-received packet abandoned on a >100 us gap */
    DIAG_LINE_ERR,   /* FE / NE / ORE seen by the receiver */
    DIAG_SERVED_SR,  /* SYNC_READ answered */
    DIAG_SERVED_RD,  /* every reply framed, SYNC_READ ones included */
    DIAG_TX_DONE,    /* replies whose last bit left the wire (TC) */
    DIAG_MAX_GAP_US, /* largest mid-packet inter-byte gap tolerated so far */
    DIAG_LAST_GAP_US,/* the gap that caused the most recent reset, us (saturates) */
    DIAG_LAST_GAP_AT,/* ... and how many bytes into the packet it struck */
    DIAG_OVERRUN,    /* receiver overruns (ORE), now that they are reported */
    DIAG_COUNT
};
#define DXL_DIAG_ADDR 8u

static uint16_t s_diag[DIAG_COUNT];

static inline void diag_store(uint8_t which)
{
    s_table[DXL_DIAG_ADDR + (which * 2u)]      = (uint8_t)(s_diag[which] & 0xFFu);
    s_table[DXL_DIAG_ADDR + (which * 2u) + 1u] = (uint8_t)(s_diag[which] >> 8);
}

static inline void diag_bump(uint8_t which)
{
    s_diag[which]++;
    diag_store(which);
}

static inline void diag_set(uint8_t which, uint16_t v)
{
    s_diag[which] = v;
    diag_store(which);
}

static inline void diag_max(uint8_t which, uint16_t v)
{
    if (v > s_diag[which]) {
        s_diag[which] = v;
        diag_store(which);
    }
}

static uint8_t  s_tx[DXL_TX_MAX];
static uint16_t s_tx_len;
static volatile uint16_t s_tx_pos;

enum {
    S_FF1 = 0,
    S_FF2,
    S_FD,
    S_00,
    S_ID,
    S_LEN_L,
    S_LEN_H,
    S_BODY
};

/* -- microsecond clock ---------------------------------------------------- */

/*
 * TIM3 free-runs at 1 MHz. It exists for one thing: when this board is not
 * first in a SYNC_READ ID list it has to hold off until the devices ahead of it
 * have answered, and that wait is hundreds of microseconds - far below SysTick
 * resolution. The host lists it first, so in practice the wait is zero.
 */

static void us_timer_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    TIM3->PSC = (uint16_t)((SystemCoreClock / 1000000u) - 1u);
    TIM3->ARR = 0xFFFFu;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

static void delay_us(uint16_t us)
{
    uint16_t t0 = (uint16_t)TIM3->CNT;
    while ((uint16_t)((uint16_t)TIM3->CNT - t0) < us) {
        /* spin */
    }
}

/* -- CRC ------------------------------------------------------------------ */

/*
 * CRC-16/IBM, polynomial 0x8005, initial value 0, not reflected - the Dynamixel
 * 2.0 CRC. Note it covers the packet *after* byte stuffing, header included.
 *
 * Table-driven, and the table is the whole point. Every request is checked and
 * every reply is signed from inside USART1_IRQHandler, with the receiver
 * holding at most eight bytes - 80 us at 1 Mbps - behind it. Bitwise, the two
 * CRCs around one SYNC_READ cost about 50 us of deaf ISR on a 48 MHz M0+, and
 * the 88-byte SYNC_WRITE that precedes it on every tick cost 90 us on its own;
 * measured on the bus, the board answered about 85% of the host's SYNC_READs
 * and was silent for the rest. The table brings each CRC to about 2 us. 512
 * bytes of flash, on a part with 32 KB of which this firmware uses 4.
 */
static const uint16_t s_crc_table[256] = {
    0x0000u, 0x8005u, 0x800Fu, 0x000Au, 0x801Bu, 0x001Eu, 0x0014u, 0x8011u,
    0x8033u, 0x0036u, 0x003Cu, 0x8039u, 0x0028u, 0x802Du, 0x8027u, 0x0022u,
    0x8063u, 0x0066u, 0x006Cu, 0x8069u, 0x0078u, 0x807Du, 0x8077u, 0x0072u,
    0x0050u, 0x8055u, 0x805Fu, 0x005Au, 0x804Bu, 0x004Eu, 0x0044u, 0x8041u,
    0x80C3u, 0x00C6u, 0x00CCu, 0x80C9u, 0x00D8u, 0x80DDu, 0x80D7u, 0x00D2u,
    0x00F0u, 0x80F5u, 0x80FFu, 0x00FAu, 0x80EBu, 0x00EEu, 0x00E4u, 0x80E1u,
    0x00A0u, 0x80A5u, 0x80AFu, 0x00AAu, 0x80BBu, 0x00BEu, 0x00B4u, 0x80B1u,
    0x8093u, 0x0096u, 0x009Cu, 0x8099u, 0x0088u, 0x808Du, 0x8087u, 0x0082u,
    0x8183u, 0x0186u, 0x018Cu, 0x8189u, 0x0198u, 0x819Du, 0x8197u, 0x0192u,
    0x01B0u, 0x81B5u, 0x81BFu, 0x01BAu, 0x81ABu, 0x01AEu, 0x01A4u, 0x81A1u,
    0x01E0u, 0x81E5u, 0x81EFu, 0x01EAu, 0x81FBu, 0x01FEu, 0x01F4u, 0x81F1u,
    0x81D3u, 0x01D6u, 0x01DCu, 0x81D9u, 0x01C8u, 0x81CDu, 0x81C7u, 0x01C2u,
    0x0140u, 0x8145u, 0x814Fu, 0x014Au, 0x815Bu, 0x015Eu, 0x0154u, 0x8151u,
    0x8173u, 0x0176u, 0x017Cu, 0x8179u, 0x0168u, 0x816Du, 0x8167u, 0x0162u,
    0x8123u, 0x0126u, 0x012Cu, 0x8129u, 0x0138u, 0x813Du, 0x8137u, 0x0132u,
    0x0110u, 0x8115u, 0x811Fu, 0x011Au, 0x810Bu, 0x010Eu, 0x0104u, 0x8101u,
    0x8303u, 0x0306u, 0x030Cu, 0x8309u, 0x0318u, 0x831Du, 0x8317u, 0x0312u,
    0x0330u, 0x8335u, 0x833Fu, 0x033Au, 0x832Bu, 0x032Eu, 0x0324u, 0x8321u,
    0x0360u, 0x8365u, 0x836Fu, 0x036Au, 0x837Bu, 0x037Eu, 0x0374u, 0x8371u,
    0x8353u, 0x0356u, 0x035Cu, 0x8359u, 0x0348u, 0x834Du, 0x8347u, 0x0342u,
    0x03C0u, 0x83C5u, 0x83CFu, 0x03CAu, 0x83DBu, 0x03DEu, 0x03D4u, 0x83D1u,
    0x83F3u, 0x03F6u, 0x03FCu, 0x83F9u, 0x03E8u, 0x83EDu, 0x83E7u, 0x03E2u,
    0x83A3u, 0x03A6u, 0x03ACu, 0x83A9u, 0x03B8u, 0x83BDu, 0x83B7u, 0x03B2u,
    0x0390u, 0x8395u, 0x839Fu, 0x039Au, 0x838Bu, 0x038Eu, 0x0384u, 0x8381u,
    0x0280u, 0x8285u, 0x828Fu, 0x028Au, 0x829Bu, 0x029Eu, 0x0294u, 0x8291u,
    0x82B3u, 0x02B6u, 0x02BCu, 0x82B9u, 0x02A8u, 0x82ADu, 0x82A7u, 0x02A2u,
    0x82E3u, 0x02E6u, 0x02ECu, 0x82E9u, 0x02F8u, 0x82FDu, 0x82F7u, 0x02F2u,
    0x02D0u, 0x82D5u, 0x82DFu, 0x02DAu, 0x82CBu, 0x02CEu, 0x02C4u, 0x82C1u,
    0x8243u, 0x0246u, 0x024Cu, 0x8249u, 0x0258u, 0x825Du, 0x8257u, 0x0252u,
    0x0270u, 0x8275u, 0x827Fu, 0x027Au, 0x826Bu, 0x026Eu, 0x0264u, 0x8261u,
    0x0220u, 0x8225u, 0x822Fu, 0x022Au, 0x823Bu, 0x023Eu, 0x0234u, 0x8231u,
    0x8213u, 0x0216u, 0x021Cu, 0x8219u, 0x0208u, 0x820Du, 0x8207u, 0x0202u,
};

static uint16_t dxl_crc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0u;
    for (uint16_t i = 0u; i < len; i++) {
        crc = (uint16_t)((crc << 8) ^ s_crc_table[((crc >> 8) ^ data[i]) & 0xFFu]);
    }
    return crc;
}

/* -- line direction ------------------------------------------------------- */

/*
 * These two are the only place the board revision reaches into the protocol.
 * Both must be fast and both must be safe to call from the ISR.
 */

#if DXL_BUFFERED

/* Hardware DE owns the buffer. Nothing to do, on purpose - these stay so the
 * transmit path reads the same on both revisions. */
static inline void line_release(void) { }
static inline void line_drive(void)   { }

#else

static inline void line_release(void)
{
    DXL_PORT->OTYPER |= DXL_TX_PIN; /* open-drain: an idle-high TX lets go */
}

static inline void line_drive(void)
{
    DXL_PORT->OTYPER &= ~DXL_TX_PIN; /* push-pull: real edges at 1 Mbps */
}

#endif

/* -- transmit ------------------------------------------------------------- */

static void tx_start(void)
{
    USART1->CR1 &= ~USART_CR1_RE; /* the receiver is on the same wire */
    line_drive();
    s_tx_pos = 0u;
    USART1->ICR = USART_ICR_TCCF;
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    s_replies++;
}

/**
 * Frame and queue a status packet.
 *
 * @param err      error byte, 0 on success
 * @param params   payload after the error byte, may be NULL
 * @param nparams  length of @p params
 */
static void respond(uint8_t err, const volatile uint8_t *params, uint16_t nparams)
{
    uint16_t n = 0u;

    s_tx[n++] = 0xFFu;
    s_tx[n++] = 0xFFu;
    s_tx[n++] = 0xFDu;
    s_tx[n++] = 0x00u;
    s_tx[n++] = (uint8_t)DXL_ID;
    uint16_t len_at = n;
    n += 2u; /* LEN is filled in once stuffing is known */

    uint16_t body_at = n;
    s_tx[n++] = INST_STATUS;
    s_tx[n++] = err;

    /* Byte stuffing: an FD following FF FF inside the payload gets a second FD
     * so the sequence can never be mistaken for a header. */
    uint8_t run = 0u; /* consecutive 0xFF seen, capped at 2 */
    for (uint16_t i = 0u; i < nparams; i++) {
        uint8_t b = params[i];
        if (n >= DXL_TX_MAX - 3u) {
            break;
        }
        s_tx[n++] = b;
        if (b == 0xFFu) {
            if (run < 2u) {
                run++;
            }
        } else if (b == 0xFDu && run == 2u) {
            s_tx[n++] = 0xFDu;
            run = 0u;
        } else {
            run = 0u;
        }
    }

    uint16_t len = (uint16_t)(n - body_at + 2u);
    s_tx[len_at] = (uint8_t)(len & 0xFFu);
    s_tx[len_at + 1u] = (uint8_t)(len >> 8);

    uint16_t crc = dxl_crc(s_tx, n);
    s_tx[n++] = (uint8_t)(crc & 0xFFu);
    s_tx[n++] = (uint8_t)(crc >> 8);

    s_tx_len = n;
    diag_bump(DIAG_SERVED_RD);
    tx_start();
}

/* -- request dispatch ----------------------------------------------------- */

/** Status packet airtime for @p len payload bytes, in microseconds, plus a
 *  little slack. Header 4 + id + len 2 + inst + err + CRC 2 = 11 overhead
 *  bytes, ten bit times each at 1 Mbps. */
static uint16_t slot_us(uint16_t len)
{
    return (uint16_t)(((11u + len) * 10u) + 20u);
}

static void serve_read(uint16_t addr, uint16_t len)
{
    if (len == 0u || addr >= DXL_TABLE_SIZE || (uint32_t)addr + len > DXL_TABLE_SIZE) {
        respond(ERR_DATA_RANGE, NULL, 0u);
        return;
    }
    respond(ERR_NONE, &s_table[addr], len);
}

static void handle_packet(uint8_t id, const uint8_t *body, uint16_t nbody)
{
    if (nbody < 1u) {
        return;
    }
    uint8_t inst = body[0];
    const uint8_t *p = &body[1];
    uint16_t np = (uint16_t)(nbody - 1u);

    switch (inst) {
    case INST_PING: {
        uint8_t info[3];
        info[0] = (uint8_t)(DXL_MODEL_NUMBER & 0xFFu);
        info[1] = (uint8_t)(DXL_MODEL_NUMBER >> 8);
        info[2] = DXL_FIRMWARE_VERSION;
        if (id == DXL_BROADCAST_ID) {
            /* A bus scan pings the broadcast address and expects one answer per
             * device, spaced by ID so they do not collide. */
            delay_us((uint16_t)(DXL_ID * 3u));
        }
        respond(ERR_NONE, info, 3u);
        break;
    }

    case INST_READ:
        if (id != DXL_ID) {
            break;
        }
        if (np < 4u) {
            respond(ERR_DATA_LENGTH, NULL, 0u);
            break;
        }
        serve_read((uint16_t)(p[0] | ((uint16_t)p[1] << 8)),
                   (uint16_t)(p[2] | ((uint16_t)p[3] << 8)));
        break;

    case INST_WRITE:
        if (id != DXL_ID) {
            break;
        }
        /* Nothing here is writable: identity is compiled in and the sensor
         * block is an output. Acknowledged rather than refused so a tool that
         * writes a harmless register during a scan does not report the board
         * as broken. */
        respond(ERR_NONE, NULL, 0u);
        break;

    case INST_SYNC_READ: {
        if (np < 5u) {
            break;
        }
        uint16_t addr = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
        uint16_t len = (uint16_t)(p[2] | ((uint16_t)p[3] << 8));
        const uint8_t *ids = &p[4];
        uint16_t nids = (uint16_t)(np - 4u);

        for (uint16_t i = 0u; i < nids; i++) {
            if (ids[i] != DXL_ID) {
                continue;
            }
            /* Devices answer in list order. The host puts this board at index 0
             * so the wait is normally zero; the general case is here so a
             * reordered list degrades into a late answer rather than a
             * collision. */
            if (i > 0u) {
                delay_us((uint16_t)(i * slot_us(len)));
            }
            serve_read(addr, len);
            diag_bump(DIAG_SERVED_SR);
            break;
        }
        break;
    }

    default:
        if (id == DXL_ID) {
            respond(ERR_INSTRUCTION, NULL, 0u);
        }
        break;
    }
}

/* -- receive -------------------------------------------------------------- */

static void packet_complete(void)
{
    uint16_t total = (uint16_t)(7u + s_rx_len);

    diag_bump(DIAG_OURS);
    uint16_t want = dxl_crc(s_rx, (uint16_t)(total - 2u));
    uint16_t got = (uint16_t)(s_rx[total - 2u] | ((uint16_t)s_rx[total - 1u] << 8));
    if (want != got) {
        diag_bump(DIAG_CRC_FAIL);
        return;
    }

    /* Un-stuff in place over the body, CRC excluded. */
    uint8_t *body = &s_rx[7];
    uint16_t raw = (uint16_t)(s_rx_len - 2u);
    uint16_t out = 0u;
    uint8_t run = 0u;
    for (uint16_t i = 0u; i < raw; i++) {
        uint8_t b = body[i];
        if (b == 0xFDu && run == 2u) {
            /* The inserted duplicate: keep one FD, drop the next byte. */
            body[out++] = b;
            if (i + 1u < raw && body[i + 1u] == 0xFDu) {
                i++;
            }
            run = 0u;
            continue;
        }
        body[out++] = b;
        if (b == 0xFFu) {
            if (run < 2u) {
                run++;
            }
        } else {
            run = 0u;
        }
    }

    handle_packet(s_rx[4], body, out);
}

static void rx_byte(uint8_t b)
{
    uint16_t now = (uint16_t)TIM3->CNT;
    if (s_state != S_FF1) {
        uint16_t gap = (uint16_t)(now - s_last_rx_us);
        if (gap > RX_IDLE_RESET_US) {
            /* How far the abandoned packet had got: header bytes by state,
             * then the body count. */
            uint16_t at = (s_state == S_BODY) ? (uint16_t)(7u + s_rx_pos) : (uint16_t)s_state;
            diag_set(DIAG_LAST_GAP_US, gap);
            diag_set(DIAG_LAST_GAP_AT, at);
            s_state = S_FF1;
            diag_bump(DIAG_GAP_RESET);
        } else {
            diag_max(DIAG_MAX_GAP_US, gap);
        }
    }
    s_last_rx_us = now;

    switch (s_state) {
    case S_FF1:
        if (b == 0xFFu) {
            s_state = S_FF2;
        }
        break;

    case S_FF2:
        s_state = (b == 0xFFu) ? S_FD : S_FF1;
        break;

    case S_FD:
        if (b == 0xFDu) {
            s_state = S_00;
        } else if (b != 0xFFu) {
            s_state = S_FF1;
        }
        break;

    case S_00:
        if (b == 0x00u) {
            s_state = S_ID;
        } else {
            s_state = (b == 0xFFu) ? S_FF2 : S_FF1;
        }
        break;

    case S_ID:
        /* Everything else on this bus - fifteen servo status packets a tick -
         * is dropped here, before it costs a CRC. */
        if (b != DXL_ID && b != DXL_BROADCAST_ID) {
            s_state = S_FF1;
            break;
        }
        s_rx[0] = 0xFFu;
        s_rx[1] = 0xFFu;
        s_rx[2] = 0xFDu;
        s_rx[3] = 0x00u;
        s_rx[4] = b;
        s_state = S_LEN_L;
        break;

    case S_LEN_L:
        s_rx[5] = b;
        s_state = S_LEN_H;
        break;

    case S_LEN_H:
        s_rx[6] = b;
        s_rx_len = (uint16_t)(s_rx[5] | ((uint16_t)b << 8));
        if (s_rx_len < 3u || s_rx_len > (DXL_RX_MAX - 7u)) {
            s_state = S_FF1;
            break;
        }
        s_rx_pos = 0u;
        s_state = S_BODY;
        break;

    case S_BODY:
        /* The first body byte is the instruction. A broadcast that is not a
         * PING or a SYNC_READ is nobody's business here - it is the host's
         * 88-byte SYNC_WRITE of goal positions, once a tick - so it is dropped
         * now rather than buffered, CRC-checked and then ignored. Buffering
         * it cost the receiver the ~90 us the CRC took, which is exactly the
         * moment the next SYNC_READ starts arriving after a tick the host had
         * to skip. */
        if (s_rx_pos == 0u && s_rx[4] == DXL_BROADCAST_ID
            && b != INST_PING && b != INST_SYNC_READ) {
            s_state = S_FF1;
            break;
        }
        s_rx[7u + s_rx_pos] = b;
        s_rx_pos++;
        if (s_rx_pos >= s_rx_len) {
            s_state = S_FF1;
            packet_complete();
        }
        break;

    default:
        s_state = S_FF1;
        break;
    }
}

/* -- interrupt ------------------------------------------------------------ */

void USART1_IRQHandler(void)
{
    /* FIFO mode: RXFNE means "not empty", so drain it - one entry per pass
     * would leave the rest for a re-entry each, and the whole point of the
     * FIFO is to absorb the ISR's own worst case. Re-read ISR each time; a
     * byte can land between two iterations. */
    while ((USART1->ISR & USART_ISR_RXNE_RXFNE) != 0u) {
        uint32_t flags = USART1->ISR & (USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE);
        if (flags != 0u) {
            USART1->ICR = USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
            diag_bump(((flags & USART_ISR_ORE) != 0u) ? DIAG_OVERRUN : DIAG_LINE_ERR);
        }
        rx_byte((uint8_t)USART1->RDR);
    }

    uint32_t cr1 = USART1->CR1;
    if ((cr1 & USART_CR1_TXEIE_TXFNFIE) != 0u) {
        /* TXFNF means "not full": push as much of the reply as fits. */
        while (s_tx_pos < s_tx_len && (USART1->ISR & USART_ISR_TXE_TXFNF) != 0u) {
            USART1->TDR = s_tx[s_tx_pos++];
        }
        if (s_tx_pos >= s_tx_len) {
            USART1->CR1 = (USART1->CR1 & ~USART_CR1_TXEIE_TXFNFIE) | USART_CR1_TCIE;
        }
    }

    uint32_t isr = USART1->ISR;
    cr1 = USART1->CR1;

    if (((isr & USART_ISR_TC) != 0u) && ((cr1 & USART_CR1_TCIE) != 0u)) {
        USART1->ICR = USART_ICR_TCCF;
        USART1->CR1 &= ~USART_CR1_TCIE;
        diag_bump(DIAG_TX_DONE);
        line_release();
        /* Drop anything the receiver latched while the line was ours. RE was
         * clear, so this should be nothing; read the FIFO out regardless
         * rather than trust that. */
        while ((USART1->ISR & USART_ISR_RXNE_RXFNE) != 0u) {
            (void)USART1->RDR;
        }
        USART1->RQR = USART_RQR_RXFRQ;
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        USART1->CR1 |= USART_CR1_RE;
    }
}

/* -- public --------------------------------------------------------------- */

/*
 * GPIO comes up in two steps around the USART, not one before it.
 *
 * dxl_gpio_prepare() sets everything about the pins except their mode - AF
 * number, pull, speed, output type - while they are still the high-impedance
 * inputs reset left them as. dxl_gpio_connect() then switches them to
 * alternate function, and is only called once USART1 is clocked, configured
 * and enabled. Put into AF mode earlier, a pin is driven by a peripheral that
 * does not exist yet, and the level it gets is whatever the unclocked mux
 * happens to output - low, in practice. On Rev 2 that is OE# low: U5 enabled,
 * the bus driven from an idle PB6 until the USART wakes up. On Rev 1 it is an
 * open-drain PB6 pulled low. Either is a glitch on a wire fifteen servos are
 * listening to, at exactly the moment they are booting too.
 *
 * Until connect(), R7 holds OE# high through PA12's high impedance and the bus
 * pull-up holds DATA high through PB6's, so the board is silent by default.
 */

static void dxl_gpio_prepare(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    DXL_PORT->AFR[0] = (DXL_PORT->AFR[0] & ~(0xFu << (DXL_TX_PIN_NUM * 4)))
                     | ((uint32_t)DXL_TX_AF << (DXL_TX_PIN_NUM * 4));
    DXL_PORT->OSPEEDR |= (3u << (DXL_TX_PIN_NUM * 2));
    DXL_PORT->PUPDR &= ~(3u << (DXL_TX_PIN_NUM * 2));

#if DXL_BUFFERED
    /* PB6 only ever drives U5's input: ordinary push-pull. */
    DXL_PORT->OTYPER &= ~DXL_TX_PIN;

    /* PB7 reads the bus. The pull-up is deliberate: with U5 released and every
     * servo silent, nothing else on this board defines the idle level, and a
     * floating CMOS input under a UART receiver is framing errors. ~40k is
     * weak enough never to fight a driver. */
    DXL_PORT->AFR[0] = (DXL_PORT->AFR[0] & ~(0xFu << (DXL_RX_PIN_NUM * 4)))
                     | ((uint32_t)DXL_RX_AF << (DXL_RX_PIN_NUM * 4));
    DXL_PORT->PUPDR = (DXL_PORT->PUPDR & ~(3u << (DXL_RX_PIN_NUM * 2)))
                    | (1u << (DXL_RX_PIN_NUM * 2));
    DXL_PORT->MODER = (DXL_PORT->MODER & ~(3u << (DXL_RX_PIN_NUM * 2)))
                    | (2u << (DXL_RX_PIN_NUM * 2));

    /* PA12 as USART1_DE - AF number now, AF mode in connect(). Belt and
     * braces: ODR is set high as well, so if anything ever does put this pin
     * into plain output mode it releases U5 rather than enabling it. */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    DXL_DE_PORT->BSRR = DXL_DE_PIN;
    DXL_DE_PORT->AFR[DXL_DE_PIN_NUM / 8u] =
        (DXL_DE_PORT->AFR[DXL_DE_PIN_NUM / 8u] & ~(0xFu << ((DXL_DE_PIN_NUM % 8u) * 4u)))
        | ((uint32_t)DXL_DE_AF << ((DXL_DE_PIN_NUM % 8u) * 4u));
    DXL_DE_PORT->OSPEEDR |= (3u << (DXL_DE_PIN_NUM * 2));
#else
    DXL_PORT->OTYPER |= DXL_TX_PIN; /* released until there is something to say */
#endif
}

static void dxl_gpio_connect(void)
{
#if DXL_BUFFERED
    /* DE first, then TX. With DEM|DEP already set and the transmitter idle,
     * the USART is holding DE inactive-high, so OE# goes from R7-high to
     * driven-high and U5 never sees an edge. */
    DXL_DE_PORT->MODER = (DXL_DE_PORT->MODER & ~(3u << (DXL_DE_PIN_NUM * 2)))
                       | (2u << (DXL_DE_PIN_NUM * 2));
#endif
    DXL_PORT->MODER = (DXL_PORT->MODER & ~(3u << (DXL_TX_PIN_NUM * 2)))
                    | (2u << (DXL_TX_PIN_NUM * 2));
}

void dxl_slave_init(void)
{
    memset((void *)s_table, 0, sizeof(s_table));
    s_table[0] = (uint8_t)(DXL_MODEL_NUMBER & 0xFFu);
    s_table[1] = (uint8_t)(DXL_MODEL_NUMBER >> 8);
    s_table[2] = DXL_FIRMWARE_VERSION;
    s_table[3] = (uint8_t)DXL_ID;
    s_table[4] = 3u; /* baud rate code 3 = 1 Mbps, matching the servos */
    s_table[5] = 0u; /* return delay time */

    s_state = S_FF1;

    us_timer_init();
    dxl_gpio_prepare();

    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    USART1->CR1 = 0u;
    USART1->BRR = (uint16_t)((SystemCoreClock + (DXL_BAUD / 2u)) / DXL_BAUD);
    USART1->CR2 = 0u;
    /* OVRDIS keeps a byte lost during a transmission from latching an overrun
     * the parser would then have to clear before it could resynchronise. */
#if DXL_BUFFERED
    /* DEM: the USART drives DE around each transmission. DEP: active low, to
     * match U5's OE#. DEAT/DEDT: assert 4/16 of a bit before the start bit and
     * hold 4/16 after the stop bit - 250 ns each at 1 Mbps, which is fifty
     * times U5's enable time and invisible next to the 10 us byte. */
    /* ORE is reported (no OVRDIS): with a FIFO an overrun means the ISR was
     * away for eight byte times, which is worth counting, and the TC handler
     * flushes the receiver and clears the flag before RE comes back on. */
    USART1->CR3 = USART_CR3_DEM | USART_CR3_DEP;
    USART1->CR1 = (4u << USART_CR1_DEAT_Pos) | (4u << USART_CR1_DEDT_Pos);
#else
    /* HDSEL wires the receiver to the TX pin - the only way to receive on a
     * board where USART1_RX has nowhere to go. */
    USART1->CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
#endif
    /* FIFO mode: eight bytes of receive buffering, so the ISR may be away for
     * up to 80 us at 1 Mbps before a byte is lost, against 10 us with the bare
     * RDR. That covers a CRC, a reply being framed, and the SPI poll's
     * critical section together. Must be set before UE. */
    USART1->CR1 |= USART_CR1_FIFOEN;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART1->CR1 |= USART_CR1_UE;

    /* Only now, with TX idling high and DE idling inactive, do the pins join
     * the peripheral. */
    dxl_gpio_connect();

    NVIC_SetPriority(USART1_IRQn, 0u);
    NVIC_EnableIRQ(USART1_IRQn);
}

void dxl_slave_publish(const uint8_t *block)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint16_t i = 0u; i < DXL_SENSOR_LEN; i++) {
        s_table[DXL_SENSOR_ADDR + i] = block[i];
    }
    __set_PRIMASK(primask);
}

uint32_t dxl_slave_replies(void)
{
    return s_replies;
}
