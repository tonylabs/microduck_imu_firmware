/**
 * @file    mount.c
 * @brief   Mounting correction - see mount.h.
 *
 * Plain float arithmetic. The STM32C011 has no FPU, so every operation here is
 * a software routine; a full correction is on the order of two hundred of
 * them, well under a millisecond at 48 MHz, and it runs from the 2 ms poll in
 * the main loop, never from the USART ISR. No libm: the one square root is
 * Newton's method on a value already known to lie in [0, 1].
 */
#include "mount.h"

static const float CW = MOUNT_CORR_W, CX = MOUNT_CORR_X, CY = MOUNT_CORR_Y, CZ = MOUNT_CORR_Z;

/* v' = q v q^-1 for a unit quaternion q, the same form duck-control uses. */
static void rotate(float qw, float qx, float qy, float qz, float v[3])
{
    float tx = 2.0f * (qy * v[2] - qz * v[1]);
    float ty = 2.0f * (qz * v[0] - qx * v[2]);
    float tz = 2.0f * (qx * v[1] - qy * v[0]);
    float cx = qy * tz - qz * ty;
    float cy = qz * tx - qx * tz;
    float cz = qx * ty - qy * tx;
    v[0] += qw * tx + cx;
    v[1] += qw * ty + cy;
    v[2] += qw * tz + cz;
}

static float sqrt01(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    float r = 0.5f * (x + 1.0f); /* >= sqrt(x) on [0,1]: Newton converges from above */
    for (int i = 0; i < 6; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
}

/* IEEE binary16 <-> float, normals and subnormals, no NaN/inf handling needed
 * for unit-quaternion components. */
static float half_to_float(uint16_t h)
{
    uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
    uint32_t exp  = (h >> 10) & 0x1Fu;
    uint32_t frac = h & 0x3FFu;
    uint32_t bits;
    if (exp == 0u) {
        if (frac == 0u) {
            bits = sign;
        } else {
            /* subnormal: renormalise */
            int e = -1;
            do { e++; frac <<= 1; } while ((frac & 0x400u) == 0u);
            frac &= 0x3FFu;
            bits = sign | ((uint32_t)(127 - 15 - e) << 23) | (frac << 13);
        }
    } else if (exp == 0x1Fu) {
        bits = sign | 0x7F800000u | (frac << 13);
    } else {
        bits = sign | ((exp + 127u - 15u) << 23) | (frac << 13);
    }
    union { uint32_t u; float f; } u = { bits };
    return u.f;
}

static uint16_t float_to_half(float f)
{
    union { float f; uint32_t u; } u = { f };
    uint32_t sign = (u.u >> 16) & 0x8000u;
    int32_t  exp  = (int32_t)((u.u >> 23) & 0xFFu) - 127 + 15;
    uint32_t frac = u.u & 0x7FFFFFu;
    if (exp >= 0x1F) {
        return (uint16_t)(sign | 0x7C00u);
    }
    if (exp <= 0) {
        if (exp < -10) {
            return (uint16_t)sign;
        }
        frac |= 0x800000u;
        uint32_t shift = (uint32_t)(14 - exp);
        uint32_t half = frac >> shift;
        uint32_t rem  = frac & ((1u << shift) - 1u);
        uint32_t mid  = 1u << (shift - 1u);
        if (rem > mid || (rem == mid && (half & 1u))) {
            half++;
        }
        return (uint16_t)(sign | half);
    }
    uint32_t half = sign | ((uint32_t)exp << 10) | (frac >> 13);
    uint32_t rem = frac & 0x1FFFu;
    if (rem > 0x1000u || (rem == 0x1000u && (half & 1u))) {
        half++; /* carries into the exponent correctly */
    }
    return (uint16_t)half;
}

static int16_t clamp_i16(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

void mount_correct_vec(int16_t v[3])
{
#if MOUNT_CORR_ENABLED
    float f[3] = { (float)v[0], (float)v[1], (float)v[2] };
    rotate(CW, CX, CY, CZ, f);
    v[0] = clamp_i16(f[0]);
    v[1] = clamp_i16(f[1]);
    v[2] = clamp_i16(f[2]);
#else
    (void)v;
#endif
}

void mount_correct_quat(uint16_t q_half[3])
{
#if MOUNT_CORR_ENABLED
    float x = half_to_float(q_half[0]);
    float y = half_to_float(q_half[1]);
    float z = half_to_float(q_half[2]);
    float n2 = x * x + y * y + z * z;
    if (n2 > 1.0f) {
        n2 = 1.0f;
    }
    float w = sqrt01(1.0f - n2);

    /* q' = q * conj(C): the measured sensor frame is the corrected frame rotated
     * by C, so the world<-corrected map is world<-measured composed with that. */
    float cw = CW, cx = -CX, cy = -CY, cz = -CZ;
    float rw = w * cw - x * cx - y * cy - z * cz;
    float rx = w * cx + x * cw + y * cz - z * cy;
    float ry = w * cy - x * cz + y * cw + z * cx;
    float rz = w * cz + x * cy - y * cx + z * cw;
    if (rw < 0.0f) { /* the host reconstructs w as +sqrt: keep the hemisphere */
        rx = -rx; ry = -ry; rz = -rz;
    }
    q_half[0] = float_to_half(rx);
    q_half[1] = float_to_half(ry);
    q_half[2] = float_to_half(rz);
#else
    (void)q_half;
#endif
}

void mount_correction_i16(int16_t out[4])
{
#if MOUNT_CORR_ENABLED
    out[0] = clamp_i16(CW * 10000.0f);
    out[1] = clamp_i16(CX * 10000.0f);
    out[2] = clamp_i16(CY * 10000.0f);
    out[3] = clamp_i16(CZ * 10000.0f);
#else
    out[0] = 10000; out[1] = 0; out[2] = 0; out[3] = 0;
#endif
}
