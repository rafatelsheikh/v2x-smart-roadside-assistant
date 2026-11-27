#include "dsp_math.h"
#include <cmath>
#include <vector>
#include <complex>
#include <utility>

namespace {
constexpr double kPi        = 3.14159265358979323846;
constexpr double kFullScale = 2048.0;   // AD9361 12-bit samples held in int16

bool is_pow2(int n) { return n > 0 && (n & (n - 1)) == 0; }

// In-place iterative radix-2 Cooley-Tukey FFT.
void fft(std::vector<std::complex<double>> &a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {          // bit-reversal permutation
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * kPi / static_cast<double>(len);
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; ++k) {
                auto u = a[i + k];
                auto v = a[i + k + len / 2] * w;
                a[i + k]           = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}
} // namespace

PeakData find_peak_ex(const short *iq, int n, int dc_guard) {
    PeakData bad = {-1, -200.0f};
    if (!iq || !is_pow2(n) || n < 8 || n > (1 << 20) || dc_guard < 0) return bad;

    // Hann window (reduces leakage so weak signals aren't hidden by a strong one)
    std::vector<std::complex<double>> x(n);
    double win_sum = 0.0;
    for (int i = 0; i < n; ++i) {
        double w = 0.5 * (1.0 - std::cos(2.0 * kPi * i / n));
        win_sum += w;
        x[i] = std::complex<double>(iq[2 * i] * w, iq[2 * i + 1] * w);
    }
    fft(x);

    // Search in DC-centred order: shifted[k] = X[(k + n/2) % n]
    const int half = n / 2;
    int best = -1;
    double best_pwr = -1.0;
    for (int k = 0; k < n; ++k) {
        if (dc_guard > 0 && std::abs(k - half) <= dc_guard) continue;
        double p = std::norm(x[(k + half) % n]);
        if (p > best_pwr) { best_pwr = p; best = k; }
    }
    if (best < 0) return bad;

    // Amplitude normalised so a full-scale complex tone = 0 dBFS
    double amp = std::sqrt(best_pwr) / (win_sum * kFullScale);
    double db  = 20.0 * std::log10(amp > 1e-12 ? amp : 1e-12);
    PeakData r = {best, static_cast<float>(db)};
    return r;
}

PeakData find_peak(const short *iq, int n) { return find_peak_ex(iq, n, 0); }
