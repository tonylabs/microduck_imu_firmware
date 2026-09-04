/**
 * @file    dxl_slave.c
 * @brief   Dynamixel Protocol 2.0 slave over USART1 single-wire half-duplex.
 *
 * ## Sharing one wire with no transceiver
 *
 * DATA_TX runs from PB6 straight to the JST connectors: no 74LVC bus buffer, no
 * direction pin. PB6 is USART1_TX and USART1_RX cannot be routed to it, so the
 * only way to receive is HDSEL - single-wire half-duplex, where the USART feeds
 * its receiver from the TX pin.
 *
 * HDSEL alone is not enough. With TE set the transmitter idles the pin high,
 * which on a shared bus would fight every servo that tries to answer. The fix
 * is the output type: the pin sits in alternate-function **open-drain** while
 * receiving, so an idle-high transmitter releases it, and flips to push-pull
 * for the few hundred microseconds it takes to send a status packet. Open-drain
 * the whole time would be simpler, but at 1 Mbps the rise time through the
 * bus pull-up alone is far too slow.
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
#define RX_IDLE_RESET_US 100u

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
 * 2.0 CRC. Computed bitwise rather than from a table: 512 bytes of table would
 * buy about 20 us per packet on a part with 32 KB of flash and a 20 ms tick.
 * Note it covers the packet *after* byte stuffing, header included.
 */
static uint16_t dxl_crc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0u;
    for (uint16_t i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x8005u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* -- line direction ------------------------------------------------------- */

static inline void line_release(void)
{
    DXL_PORT->OTYPER |= DXL_PIN; /* open-drain: an idle-high TX lets go */
}

static inline void line_drive(void)
{
    DXL_PORT->OTYPER &= ~DXL_PIN; /* push-pull: real edges at 1 Mbps */
}

/* -- transmit ------------------------------------------------------------- */

static void tx_start(void)
{
    USART1->CR1 &= ~USART_CR1_RE; /* HDSEL would otherwise echo us back */
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

    uint16_t want = dxl_crc(s_rx, (uint16_t)(total - 2u));
    uint16_t got = (uint16_t)(s_rx[total - 2u] | ((uint16_t)s_rx[total - 1u] << 8));
    if (want != got) {
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
    if (s_state != S_FF1 && (uint16_t)(now - s_last_rx_us) > RX_IDLE_RESET_US) {
        s_state = S_FF1;
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
    uint32_t isr = USART1->ISR;
    uint32_t cr1 = USART1->CR1;

    if ((isr & USART_ISR_RXNE_RXFNE) != 0u) {
        rx_byte((uint8_t)USART1->RDR);
    }

    if (((isr & USART_ISR_TXE_TXFNF) != 0u) && ((cr1 & USART_CR1_TXEIE_TXFNFIE) != 0u)) {
        if (s_tx_pos < s_tx_len) {
            USART1->TDR = s_tx[s_tx_pos++];
        } else {
            USART1->CR1 = (USART1->CR1 & ~USART_CR1_TXEIE_TXFNFIE) | USART_CR1_TCIE;
        }
    }

    if (((isr & USART_ISR_TC) != 0u) && ((cr1 & USART_CR1_TCIE) != 0u)) {
        USART1->ICR = USART_ICR_TCCF;
        USART1->CR1 &= ~USART_CR1_TCIE;
        line_release();
        /* Drop anything the receiver latched while the line was ours. */
        USART1->RQR = USART_RQR_RXFRQ;
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        USART1->CR1 |= USART_CR1_RE;
    }
}

/* -- public --------------------------------------------------------------- */

static void dxl_gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    DXL_PORT->AFR[0] = (DXL_PORT->AFR[0] & ~(0xFu << (DXL_PIN_NUM * 4)))
                     | ((uint32_t)DXL_PIN_AF << (DXL_PIN_NUM * 4));
    DXL_PORT->OTYPER |= DXL_PIN; /* released until there is something to say */
    DXL_PORT->OSPEEDR |= (3u << (DXL_PIN_NUM * 2));
    DXL_PORT->PUPDR &= ~(3u << (DXL_PIN_NUM * 2));
    DXL_PORT->MODER = (DXL_PORT->MODER & ~(3u << (DXL_PIN_NUM * 2)))
                    | (2u << (DXL_PIN_NUM * 2));
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
    dxl_gpio_init();

    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    USART1->CR1 = 0u;
    USART1->BRR = (uint16_t)((SystemCoreClock + (DXL_BAUD / 2u)) / DXL_BAUD);
    USART1->CR2 = 0u;
    /* HDSEL wires the receiver to the TX pin. OVRDIS keeps a byte lost during a
     * transmission from latching an overrun the parser would then have to clear
     * before it could resynchronise. */
    USART1->CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART1->CR1 |= USART_CR1_UE;

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
