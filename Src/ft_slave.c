/**
 * @file    ft_slave.c
 * @brief   Feetech FT-SCS slave over USART1, half-duplex.
 *
 * ## Owning one shared wire
 *
 * Sixteen devices share DATA and take turns. The protocol layer here is the
 * same on every board revision; what differs is how this MCU lets go of the
 * wire when it is not its turn, and that lives in line_drive() / line_release()
 * behind DXL_BUFFERED from board.h. (The board.h names stay DXL_*: those are
 * the net names on the schematic, and the copper does not care which protocol
 * rides it.)
 *
 * **Rev 1, DXL_BUFFERED = 0.** PB6 runs straight to the connector, and
 * USART1_RX cannot be routed to it, so USART1 runs single-wire half-duplex
 * (HDSEL), feeding its receiver from the TX pin. HDSEL alone would idle the pin
 * high and fight every servo; the fix is the output type. PB6 sits in
 * alternate-function **open-drain** while receiving, so an idle-high
 * transmitter releases it, and flips to push-pull for the few hundred
 * microseconds it takes to send. Open-drain the whole time would be simpler,
 * but even at 500 kbps the rise time through the bus pull-up alone is too slow.
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
 * ## The wire format
 *
 * FT-SCS, the Feetech protocol the HD1910 speaks. Both directions are
 *
 *     FF FF  ID  LEN  (INST | ERR)  PARAM...  CHK
 *
 * with LEN counting everything after itself including CHK - so LEN = nparams
 * + 2 - and CHK the bitwise complement of the 8-bit sum of ID, LEN, the
 * instruction-or-error byte and the parameters. The two header bytes are not
 * summed. Instruction packets carry an instruction byte here; status packets
 * carry an error byte.
 *
 * Three things follow from that, and all three cost code below:
 *
 * * **There is no byte stuffing.** FF FF can and does occur inside a payload
 *   - a gyro reading of -1 is two of them - so the header is not a reliable
 *   resynchronisation point. The inter-byte gap below is what recovers a
 *   stream, not the header.
 * * **LEN is one byte**, so a packet is at most 259 bytes and a read is at
 *   most 253. The buffers are sized for the maximum rather than for what this
 *   board expects, so no legal request can overrun them or be answered with
 *   the wrong number of bytes.
 * * **The checksum is eight bits.** It is a sum, not a CRC: it catches a
 *   mangled byte but not every transposition. Nothing to be done about that -
 *   it is what the servos on the same wire use.
 *
 * ## Latency
 *
 * The whole protocol runs in the USART1 interrupt: by the time the checksum
 * byte of a request has arrived the response is built and the first byte is in
 * TDR. The return delay is 0.
 *
 * Packets addressed to someone else - every servo status packet on the bus,
 * about fifteen per tick - are dropped at the ID byte, before the checksum is
 * computed.
 */

#include "ft_slave.h"

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

#define INST_PING       0x01u
#define INST_READ       0x02u
#define INST_WRITE      0x03u
#define INST_REG_WRITE  0x04u
#define INST_ACTION     0x05u
#define INST_RESET      0x06u
#define INST_REBOOT     0x08u
#define INST_SYNC_READ  0x82u
#define INST_SYNC_WRITE 0x83u

#define ERR_NONE 0x00u

/** Microseconds one byte occupies on the wire: ten bit times, 8N1. Derived
 *  from FT_BAUD so that nothing timed in byte times goes stale when the rate
 *  changes - every such figure below is written in terms of this. */
#define FT_BYTE_US (10000000u / FT_BAUD)

/** Largest FT-SCS packet: FF FF ID LEN + 255 counted bytes. Sized for the
 *  protocol rather than for the 24-byte SYNC_READ this board actually expects,
 *  so no legal request can be truncated into a shorter one that happens to
 *  checksum. */
#define FT_RX_MAX 260u

/** FF FF ID LEN ERR + 253 params + CHK. 253 is the longest payload LEN can
 *  describe, so every read a host can legally ask for is answerable with a
 *  correctly framed packet. */
#define FT_TX_MAX 259u

/** Longest payload a status packet can carry, from LEN being one byte. */
#define FT_PARAMS_MAX 253u

/* -- state ---------------------------------------------------------------- */

static volatile uint8_t  s_table[FT_TABLE_SIZE];
static volatile uint32_t s_replies;
static volatile uint8_t  s_err; /* error byte sent with every status packet */

static uint8_t  s_rx[FT_RX_MAX];
static uint16_t s_rx_len;     /* LEN field of the packet being received    */
static uint16_t s_rx_pos;     /* bytes of the LEN-counted body received    */
static uint8_t  s_state;
static uint16_t s_last_rx_us; /* TIM3 stamp of the previous byte           */

/** Inter-byte gap, in microseconds, that abandons a part-received packet.
 *
 *  Bytes inside a packet arrive back to back - FT_BYTE_US apart, 20 us at
 *  500 kbps - so any real gap means the rest of that packet is never coming. Without this a
 *  truncated packet keeps counting down its LEN into the *next* packet's bytes
 *  and costs a second transaction as well as the one that was lost, which on
 *  this bus is a whole missed control tick.
 *
 *  Under FT-SCS this is not merely the better option, it is the only one.
 *  Protocol 2.0 could resynchronise on its four-byte header because byte
 *  stuffing kept that sequence out of payloads; FF FF is not reserved in any
 *  way here and turns up inside ordinary sensor data, so a parser that trusts
 *  the header will happily start a packet in the middle of one. A gap cannot
 *  be faked that way. */
#define RX_IDLE_RESET_US 2000u

/* 2 ms, and deliberately not scaled with the bit rate: it is sized against the
 * host, not against the wire. 2 ms, not the 100 us it once was. The host's UART loads its transmit FIFO in
 * chunks and the line idles until an interrupt refills it, and on the Radxa
 * that interrupt is routinely 100-300 us late: measured with the v4 counters,
 * the 100 us rule abandoned 9 of 300 fifteen-byte requests and 26 of 300
 * thirty-byte ones - every one of them clean, no checksum or line error, just
 * a pause in the middle. A truncated packet is still abandoned long before the
 * next request, which is a control tick (20 ms) away. */

/* -- diagnostic counters ---------------------------------------------------
 *
 * Served in the control table at FT_DIAG_ADDR, u16 little-endian each, so a
 * host can read them over the bus and see where a request it never got an
 * answer to actually died: never assembled (bytes lost - compare `ours` to
 * what was sent), assembled but corrupt (`chk_fail`), a gap mid-packet
 * (`gap_reset`), the receiver flagging framing/noise/overrun (`line_err`),
 * served but the reply never completed (`served` vs `tx_done`). Wrap at 65536
 * and are never cleared; the host takes differences. */
enum {
    DIAG_OURS = 0,   /* headers with our ID or broadcast, reached packet_complete */
    DIAG_CHK_FAIL,   /* ... and failed the checksum */
    DIAG_GAP_RESET,  /* part-received packet abandoned on a gap */
    DIAG_LINE_ERR,   /* FE / NE seen by the receiver */
    DIAG_SERVED_SR,  /* SYNC_READ answered */
    DIAG_SERVED_RD,  /* every reply framed, SYNC_READ ones included */
    DIAG_TX_DONE,    /* replies whose last bit left the wire (TC) */
    DIAG_MAX_GAP_US, /* largest mid-packet inter-byte gap tolerated so far */
    DIAG_LAST_GAP_US,/* the gap that caused the most recent reset, us (saturates) */
    DIAG_LAST_GAP_AT,/* ... and how many bytes into the packet it struck */
    DIAG_OVERRUN,    /* receiver overruns (ORE) */
    DIAG_COUNT
};

/** Not 8: on FT-SCS address 8 is the response status level, part of the
 *  header every scanner reads. The counters start above the whole of it. */
#define FT_DIAG_ADDR 16u

static uint16_t s_diag[DIAG_COUNT];

static inline void diag_store(uint8_t which)
{
    s_table[FT_DIAG_ADDR + (which * 2u)]      = (uint8_t)(s_diag[which] & 0xFFu);
    s_table[FT_DIAG_ADDR + (which * 2u) + 1u] = (uint8_t)(s_diag[which] >> 8);
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

static uint8_t  s_tx[FT_TX_MAX];
static uint16_t s_tx_len;
static volatile uint16_t s_tx_pos;

enum {
    S_FF1 = 0,
    S_FF2,
    S_ID,
    S_LEN,
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

/* -- checksum ------------------------------------------------------------- */

/*
 * The FT-SCS checksum: ~(ID + LEN + INST/ERR + sum(PARAMS)), eight bits,
 * wrapping. The two 0xFF header bytes are not included.
 *
 * The Dynamixel firmware this replaced needed a 512-byte table to get its
 * CRC-16 down to a cost the ISR could afford; a byte-wide sum is a few cycles
 * per byte and needs nothing. That is the one place where FT-SCS being the
 * weaker protocol makes this board's job easier.
 */
static uint8_t ft_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0u;
    for (uint16_t i = 0u; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)~sum;
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
    DXL_PORT->OTYPER &= ~DXL_TX_PIN; /* push-pull: real edges on a long bus */
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

/*
 * A status packet is framed in place in s_tx:
 *
 *   index  0    1    2    3    4    5 ...        n
 *          FF   FF   ID   LEN  ERR  params...    CHK
 *
 * tx_begin() lays down everything that does not depend on the payload,
 * tx_finish() fills in LEN (which is only known once the payload is written),
 * appends the checksum and hands the frame to the ISR. Splitting it this way
 * lets serve_read() write its payload straight out of the volatile table and
 * the zero fill past its end, without a second buffer to copy through.
 */

static void tx_begin(uint8_t err)
{
    s_tx[0] = 0xFFu;
    s_tx[1] = 0xFFu;
    s_tx[2] = (uint8_t)FT_ID;
    /* s_tx[3] is LEN, written by tx_finish() */
    s_tx[4] = err;
}

/** @param n  bytes written so far, i.e. 5 + the number of parameters. */
static void tx_finish(uint16_t n)
{
    s_tx[3] = (uint8_t)(n - 3u); /* ERR + params + CHK */
    s_tx[n] = ft_checksum(&s_tx[2], (uint16_t)(n - 2u));
    s_tx_len = (uint16_t)(n + 1u);
    diag_bump(DIAG_SERVED_RD);
    tx_start();
}

/** The error byte this board reports. Zero unless the IMU has stopped
 *  answering and FT_ERR_MIRRORS_FAULTS is on. */
static inline uint8_t status_err(void)
{
    return s_err;
}

/** Frame and queue a status packet carrying a short fixed payload. */
static void respond(const uint8_t *params, uint8_t nparams)
{
    tx_begin(status_err());
    uint16_t n = 5u;
    for (uint8_t i = 0u; i < nparams; i++) {
        s_tx[n++] = params[i];
    }
    tx_finish(n);
}

/* -- request dispatch ----------------------------------------------------- */

/** Status packet airtime for @p len payload bytes, in microseconds, plus a
 *  little slack. FF FF + id + len + err + chk = 6 overhead bytes. */
static uint16_t slot_us(uint16_t len)
{
    return (uint16_t)(((6u + len) * FT_BYTE_US) + (2u * FT_BYTE_US));
}

/**
 * Answer a READ or a SYNC_READ.
 *
 * Addresses past the end of the table read as zero rather than as an error.
 * FT-SCS has no "that address does not exist" to report - the error byte is
 * eight hardware fault bits and nothing else - and the alternative, staying
 * silent, is worse than useless on a shared bus: the host is reading status
 * packets back in ID order, so a missing reply does not cost one device's
 * data, it costs every device queued behind us too. A reply of the right
 * length always keeps the bus in step. The Feetech memory table has
 * `Undefined` rows of its own, so zero is also what a real servo does here.
 */
static void serve_read(uint8_t addr, uint8_t len)
{
    if (len == 0u || len > FT_PARAMS_MAX) {
        return; /* LEN could not describe the reply; nothing honest to send */
    }
    tx_begin(status_err());
    uint16_t n = 5u;
    for (uint16_t i = 0u; i < len; i++) {
        uint16_t a = (uint16_t)(addr + i);
        s_tx[n++] = (a < FT_TABLE_SIZE) ? s_table[a] : 0u;
    }
    tx_finish(n);
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
    case INST_PING:
        /* Unicast only. A Feetech bus is scanned by pinging each ID in turn
         * (servocom's scan does exactly that), so answering the broadcast buys
         * nothing and would put this board's reply on the wire at the same
         * moment as fifteen servos' - the scan would come back garbage for
         * all sixteen. A Feetech ping carries no payload; the scanner reads
         * the model at address 3 afterwards. */
        if (id != FT_ID) {
            break;
        }
        respond(NULL, 0u);
        break;

    case INST_READ:
        if (id != FT_ID || np < 2u) {
            break;
        }
        serve_read(p[0], p[1]);
        break;

    case INST_WRITE:
    case INST_REG_WRITE:
    case INST_ACTION:
    case INST_RESET:
    case INST_REBOOT:
        if (id != FT_ID) {
            break;
        }
        /* Nothing here is writable: identity is compiled in and the sensor
         * block is an output. Acknowledged rather than ignored so a tool that
         * writes a harmless register during a scan does not report the board
         * as missing - and because on FT-SCS silence is the only way to
         * refuse, which is indistinguishable from a dead board. */
        respond(NULL, 0u);
        break;

    case INST_SYNC_READ: {
        /* addr, len, then one byte per device. */
        if (np < 3u) {
            break;
        }
        uint8_t addr = p[0];
        uint8_t len = p[1];
        const uint8_t *ids = &p[2];
        uint16_t nids = (uint16_t)(np - 2u);

        for (uint16_t i = 0u; i < nids; i++) {
            if (ids[i] != FT_ID) {
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

    case INST_SYNC_WRITE:
    default:
        /* Not ours, or nothing we can refuse in a way FT-SCS can express.
         * Either way: silence. */
        break;
    }
}

/* -- receive -------------------------------------------------------------- */

static void packet_complete(void)
{
    uint16_t total = (uint16_t)(4u + s_rx_len);

    diag_bump(DIAG_OURS);
    /* Over ID, LEN, INST and the parameters - everything but the two header
     * bytes and the checksum itself. */
    uint8_t want = ft_checksum(&s_rx[2], (uint16_t)(total - 3u));
    if (want != s_rx[total - 1u]) {
        diag_bump(DIAG_CHK_FAIL);
        return;
    }

    /* No un-stuffing: FT-SCS sends payload bytes exactly as they are. */
    handle_packet(s_rx[2], &s_rx[4], (uint16_t)(s_rx_len - 1u));
}

static void rx_byte(uint8_t b)
{
    uint16_t now = (uint16_t)TIM3->CNT;
    if (s_state != S_FF1) {
        uint16_t gap = (uint16_t)(now - s_last_rx_us);
        if (gap > RX_IDLE_RESET_US) {
            /* How far the abandoned packet had got: header bytes by state,
             * then the body count. */
            uint16_t at = (s_state == S_BODY) ? (uint16_t)(4u + s_rx_pos) : (uint16_t)s_state;
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
        s_state = (b == 0xFFu) ? S_ID : S_FF1;
        break;

    case S_ID:
        /* A run of three or more 0xFF means the header was the *last* two of
         * them, so stay here rather than reading an FF as an ID - no device
         * has ID 255. */
        if (b == 0xFFu) {
            break;
        }
        /* Everything else on this bus - fifteen servo status packets a tick -
         * is dropped here, before it costs a checksum. */
        if (b != FT_ID && b != FT_BROADCAST_ID) {
            s_state = S_FF1;
            break;
        }
        s_rx[0] = 0xFFu;
        s_rx[1] = 0xFFu;
        s_rx[2] = b;
        s_state = S_LEN;
        break;

    case S_LEN:
        /* LEN counts the instruction, the parameters and the checksum, so the
         * shortest legal packet (a PING) has LEN 2. */
        s_rx[3] = b;
        s_rx_len = b;
        if (s_rx_len < 2u || s_rx_len > (FT_RX_MAX - 4u)) {
            s_state = S_FF1;
            break;
        }
        s_rx_pos = 0u;
        s_state = S_BODY;
        break;

    case S_BODY:
        /* The first body byte is the instruction. A broadcast that is not a
         * SYNC_READ is nobody's business here - it is the host's SYNC_WRITE of
         * goal positions, once a tick - so it is dropped now rather than
         * buffered, checksummed and then ignored. (A broadcast PING is not
         * answered either; see handle_packet.) */
        if (s_rx_pos == 0u && s_rx[2] == FT_BROADCAST_ID && b != INST_SYNC_READ) {
            s_state = S_FF1;
            break;
        }
        s_rx[4u + s_rx_pos] = b;
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
 * ft_gpio_prepare() sets everything about the pins except their mode - AF
 * number, pull, speed, output type - while they are still the high-impedance
 * inputs reset left them as. ft_gpio_connect() then switches them to
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

static void ft_gpio_prepare(void)
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

static void ft_gpio_connect(void)
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

void ft_slave_init(void)
{
    memset((void *)s_table, 0, sizeof(s_table));
    /* The FT-SCS header, at the addresses the protocol fixes. A scan pings an
     * ID and then reads two bytes at 3, so those two are what decide whether
     * this board shows up as itself or as an unknown. */
    s_table[0] = FT_FIRMWARE_VERSION;
    s_table[1] = FT_FIRMWARE_MINOR;
    s_table[2] = 0u; /* END = 0: little-endian, like every FT-SCS device */
    s_table[3] = (uint8_t)(FT_MODEL_NUMBER & 0xFFu);
    s_table[4] = (uint8_t)(FT_MODEL_NUMBER >> 8);
    s_table[5] = (uint8_t)FT_ID;
    s_table[6] = FT_BAUD_CODE;
    s_table[7] = 0u; /* return delay */
    s_table[8] = 1u; /* response status level: answer every instruction */

    s_state = S_FF1;

    us_timer_init();
    ft_gpio_prepare();

    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    USART1->CR1 = 0u;
    USART1->BRR = (uint16_t)((SystemCoreClock + (FT_BAUD / 2u)) / FT_BAUD);
    USART1->CR2 = 0u;
#if DXL_BUFFERED
    /* DEM: the USART drives DE around each transmission. DEP: active low, to
     * match U5's OE#. DEAT/DEDT: assert 4/16 of a bit before the start bit and
     * hold 4/16 after the stop bit - 500 ns each at 500 kbps, which is a
     * hundred times U5's enable time and invisible next to the 20 us byte. */
    /* ORE is reported (no OVRDIS): with a FIFO an overrun means the ISR was
     * away for eight byte times, which is worth counting, and the TC handler
     * flushes the receiver and clears the flag before RE comes back on. */
    USART1->CR3 = USART_CR3_DEM | USART_CR3_DEP;
    USART1->CR1 = (4u << USART_CR1_DEAT_Pos) | (4u << USART_CR1_DEDT_Pos);
#else
    /* HDSEL wires the receiver to the TX pin - the only way to receive on a
     * board where USART1_RX has nowhere to go. OVRDIS keeps a byte lost during
     * a transmission from latching an overrun the parser would then have to
     * clear before it could resynchronise. */
    USART1->CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
#endif
    /* FIFO mode: eight bytes of receive buffering, so the ISR may be away for
     * up to eight byte times - 160 us at 500 kbps - before a byte is lost,
     * against one byte time with the bare RDR. That covers a checksum, a reply being framed, and the SPI poll's
     * critical section together. Must be set before UE. */
    USART1->CR1 |= USART_CR1_FIFOEN;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART1->CR1 |= USART_CR1_UE;

    /* Only now, with TX idling high and DE idling inactive, do the pins join
     * the peripheral. */
    ft_gpio_connect();

    NVIC_SetPriority(USART1_IRQn, 0u);
    NVIC_EnableIRQ(USART1_IRQn);
}

void ft_slave_publish(const uint8_t *block)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint16_t i = 0u; i < FT_SENSOR_LEN; i++) {
        s_table[FT_SENSOR_ADDR + i] = block[i];
    }
    __set_PRIMASK(primask);
}

void ft_slave_set_fault(bool fault)
{
#if FT_ERR_MIRRORS_FAULTS
    s_err = fault ? FT_ERR_SENSOR : ERR_NONE;
#else
    (void)fault;
    s_err = ERR_NONE;
#endif
}

uint32_t ft_slave_replies(void)
{
    return s_replies;
}
