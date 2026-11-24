#ifndef V2X_FMCOMMS_DRIVER_H
#define V2X_FMCOMMS_DRIVER_H

#include <cstddef>
#include <string>

#include <iio.h>

#include "dsp_math.h"

namespace v2x {

/**
 * @brief 1Rx libiio hardware driver for ADI FMCOMMS2/3/4 (AD9361) on ZedBoard.
 *
 * Configures the AD9361 transceiver (ad9361-phy) for single-channel receive
 * with manual gain control, allocates a non-cyclic IIO capture buffer on
 * cf-ad9361-lpc, pulls raw interleaved 16-bit I/Q samples into RAM, hands the
 * buffer pointer to DSP engine (dsp_math.h), and returns PeakData.
 *
 * Assigned module: (include/fmcomms_driver.h, src/fmcomms_driver.cpp)
 */
class FMCOMMSDriver {
public:
    FMCOMMSDriver() = default;
    ~FMCOMMSDriver();

    // Non-copyable (owns libiio context and DMA buffer handles)
    FMCOMMSDriver(const FMCOMMSDriver&) = delete;
    FMCOMMSDriver& operator=(const FMCOMMSDriver&) = delete;

    // Movable
    FMCOMMSDriver(FMCOMMSDriver&& other) noexcept;
    FMCOMMSDriver& operator=(FMCOMMSDriver&& other) noexcept;

    /**
     * @brief Initializes the local (or URI) IIO context, configures ad9361-phy,
     *        enables RX0 I/Q channels, and allocates the capture iio_buffer.
     *
     * @param center_freq   RX local oscillator frequency in Hz (e.g., 5900000000LL for 5.9 GHz V2X).
     * @param sample_rate   Baseband sampling frequency in Hz (e.g., 10000000LL for 10 MSPS).
     * @param rx_gain_db    Manual RX hardware gain in dB (coordinated with SignalProcessor).
     * @param sample_count  Number of complex I/Q samples per capture buffer (power of 2, matches FFT size).
     * @param rf_bandwidth  Analog RF filter bandwidth in Hz (0 = match sample_rate).
     * @param dc_guard      Number of +/- FFT bins around 0 Hz to ignore (AD9361 DC/LO leakage).
     * @param uri           Optional libiio context URI (empty = local ZedBoard context).
     * @return true on success, false if any hardware/IIO step fails.
     */
    bool init(long long center_freq,
              long long sample_rate,
              double rx_gain_db,
              int sample_count = 4096,
              long long rf_bandwidth = 0,
              int dc_guard = 2,
              const std::string& uri = "");

    /**
     * @brief Releases the IIO buffer, disables RX channels, and destroys the context.
     */
    void shutdown();

    /**
     * @brief Pulls one buffer of raw I/Q samples from the AD9361 into RAM,
     *        passes the raw buffer pointer and sample count to dsp_math.cpp,
     *        and returns the detected spectral peak.
     *
     * @return PeakData from DSP module.
     *         Returns {-1, -200.0f} if the driver is uninitialized or refill fails.
     */
    PeakData captureAndProcess();

    // Runtime reconfiguration helpers
    bool setCenterFrequency(long long center_freq_hz);
    bool setSampleRate(long long sample_rate_hz);
    bool setRfBandwidth(long long rf_bandwidth_hz);
    bool setRxGain(double rx_gain_db);
    void setDcGuard(int dc_guard_bins) noexcept { dc_guard_ = (dc_guard_bins >= 0) ? dc_guard_bins : 0; }

    // State accessors (for coordination with SignalProcessor / main.cpp)
    bool isInitialized() const noexcept { return initialized_; }
    long long getCenterFrequency() const noexcept { return center_freq_hz_; }
    long long getSampleRate() const noexcept { return sample_rate_hz_; }
    long long getRfBandwidth() const noexcept { return rf_bandwidth_hz_; }
    double getRxGain() const noexcept { return rx_gain_db_; }
    int getSampleCount() const noexcept { return sample_count_; }
    int getDcGuard() const noexcept { return dc_guard_; }

private:
    bool configurePhy(long long center_freq,
                      long long sample_rate,
                      long long rf_bandwidth,
                      double rx_gain_db);
    bool setupRxBuffer(int sample_count);

    // libiio handles
    struct iio_context* ctx_ = nullptr;
    struct iio_device* phy_dev_ = nullptr;     // "ad9361-phy"
    struct iio_device* rx_dev_ = nullptr;      // "cf-ad9361-lpc"
    struct iio_channel* rx_phy_ch_ = nullptr;  // ad9361-phy "voltage0" (input)
    struct iio_channel* rx_lo_ch_ = nullptr;   // ad9361-phy "altvoltage0" (output, RX_LO)
    struct iio_channel* rx0_i_ = nullptr;      // cf-ad9361-lpc "voltage0" (input, I)
    struct iio_channel* rx0_q_ = nullptr;      // cf-ad9361-lpc "voltage1" (input, Q)
    struct iio_buffer* rx_buf_ = nullptr;      // Non-cyclic RX DMA buffer

    // Cached configuration
    long long center_freq_hz_ = 0;
    long long sample_rate_hz_ = 0;
    long long rf_bandwidth_hz_ = 0;
    double rx_gain_db_ = 0.0;
    int sample_count_ = 0;
    int dc_guard_ = 2;
    bool initialized_ = false;
};

} // namespace v2x

#endif // V2X_FMCOMMS_DRIVER_H