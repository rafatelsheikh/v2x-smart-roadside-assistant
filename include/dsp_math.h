#ifndef DSP_MATH_H
#define DSP_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Returned to the driver (and from there to Python via ctypes).
 * Plain C struct: int32 + float = 8 bytes, no padding surprises. */
typedef struct {
    int   peak_index;      /* bin of strongest spike, DC-CENTERED:
                              0 = -Fs/2, N/2 = 0 Hz (centre freq), N-1 = +Fs/2.
                              Offset in Hz = (peak_index - N/2) * Fs / N     */
    float peak_magnitude;  /* raw RSSI of that bin in dBFS (0 dBFS = full-scale tone).
                              Uncalibrated: Kareem adds gain offsets.         */
} PeakData;

/* iq      : interleaved int16 samples I0,Q0,I1,Q1,... (libiio AD9361 format)
 * n       : number of COMPLEX samples (iq holds 2*n int16). Must be a power
 *           of two, 8 <= n <= 1<<20.
 * On invalid input returns {-1, -200.0f}. */
PeakData find_peak(const short *iq, int n);

/* Same, but ignores +/- dc_guard bins around 0 Hz (AD9361 DC offset / LO
 * leakage spike). dc_guard = 0 behaves like find_peak. */
PeakData find_peak_ex(const short *iq, int n, int dc_guard);

#ifdef __cplusplus
}
#endif
#endif
