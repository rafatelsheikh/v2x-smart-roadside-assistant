#include "fmcomms_driver.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <utility>

namespace v2x {

namespace {

constexpr const char* kPhyDeviceName = "ad9361-phy";
constexpr const char* kRxStreamDeviceName = "cf-ad9361-lpc";
constexpr const char* kRxRfPort = "A_BALANCED";
constexpr const char* kGainControlMode = "manual";

void logIioError(const char* action, int err_code) {
    char err_buf[256] = {0};
    iio_strerror(err_code < 0 ? -err_code : err_code, err_buf, sizeof(err_buf));
    std::cerr << "[FMCOMMSDriver] " << action << " failed (" << err_code
              << "): " << err_buf << '\n';
}

} // namespace

FMCOMMSDriver::~FMCOMMSDriver() {
    shutdown();
}

FMCOMMSDriver::FMCOMMSDriver(FMCOMMSDriver&& other) noexcept
    : ctx_(std::exchange(other.ctx_, nullptr)),
      phy_dev_(std::exchange(other.phy_dev_, nullptr)),
      rx_dev_(std::exchange(other.rx_dev_, nullptr)),
      rx_phy_ch_(std::exchange(other.rx_phy_ch_, nullptr)),
      rx_lo_ch_(std::exchange(other.rx_lo_ch_, nullptr)),
      rx0_i_(std::exchange(other.rx0_i_, nullptr)),
      rx0_q_(std::exchange(other.rx0_q_, nullptr)),
      rx_buf_(std::exchange(other.rx_buf_, nullptr)),
      center_freq_hz_(other.center_freq_hz_),
      sample_rate_hz_(other.sample_rate_hz_),
      rf_bandwidth_hz_(other.rf_bandwidth_hz_),
      rx_gain_db_(other.rx_gain_db_),
      sample_count_(other.sample_count_),
      dc_guard_(other.dc_guard_),
      initialized_(std::exchange(other.initialized_, false)) {}

FMCOMMSDriver& FMCOMMSDriver::operator=(FMCOMMSDriver&& other) noexcept {
    if (this != &other) {
        shutdown();
        ctx_ = std::exchange(other.ctx_, nullptr);
        phy_dev_ = std::exchange(other.phy_dev_, nullptr);
        rx_dev_ = std::exchange(other.rx_dev_, nullptr);
        rx_phy_ch_ = std::exchange(other.rx_phy_ch_, nullptr);
        rx_lo_ch_ = std::exchange(other.rx_lo_ch_, nullptr);
        rx0_i_ = std::exchange(other.rx0_i_, nullptr);
        rx0_q_ = std::exchange(other.rx0_q_, nullptr);
        rx_buf_ = std::exchange(other.rx_buf_, nullptr);
        center_freq_hz_ = other.center_freq_hz_;
        sample_rate_hz_ = other.sample_rate_hz_;
        rf_bandwidth_hz_ = other.rf_bandwidth_hz_;
        rx_gain_db_ = other.rx_gain_db_;
        sample_count_ = other.sample_count_;
        dc_guard_ = other.dc_guard_;
        initialized_ = std::exchange(other.initialized_, false);
    }
    return *this;
}

bool FMCOMMSDriver::init(long long center_freq,
                         long long sample_rate,
                         double rx_gain_db,
                         int sample_count,
                         long long rf_bandwidth,
                         int dc_guard,
                         const std::string& uri) {
    if (initialized_) {
        shutdown();
    }

    if (center_freq <= 0 || sample_rate <= 0 || sample_count <= 0 || dc_guard < 0) {
        std::cerr << "[FMCOMMSDriver] Invalid configuration parameters "
                  << "(center_freq=" << center_freq
                  << ", sample_rate=" << sample_rate
                  << ", sample_count=" << sample_count
                  << ", dc_guard=" << dc_guard << ").\n";
        return false;
    }

    const long long effective_bw = (rf_bandwidth > 0) ? rf_bandwidth : sample_rate;
    dc_guard_ = dc_guard;

    // 1. Acquire IIO context (local on ZedBoard ARM, or remote URI if provided)
    if (uri.empty()) {
        ctx_ = iio_create_local_context();
    } else {
        ctx_ = iio_create_context_from_uri(uri.c_str());
    }

    if (!ctx_) {
        std::cerr << "[FMCOMMSDriver] Failed to create IIO context ("
                  << std::strerror(errno) << ").\n";
        return false;
    }

    // 2. Locate AD9361 PHY and RX streaming DMA devices
    phy_dev_ = iio_context_find_device(ctx_, kPhyDeviceName);
    if (!phy_dev_) {
        std::cerr << "[FMCOMMSDriver] Could not find device '" << kPhyDeviceName << "'.\n";
        shutdown();
        return false;
    }

    rx_dev_ = iio_context_find_device(ctx_, kRxStreamDeviceName);
    if (!rx_dev_) {
        std::cerr << "[FMCOMMSDriver] Could not find device '" << kRxStreamDeviceName << "'.\n";
        shutdown();
        return false;
    }

    // 3. Configure AD9361 PHY (LO, sample rate, RF bandwidth, manual gain)
    if (!configurePhy(center_freq, sample_rate, effective_bw, rx_gain_db)) {
        shutdown();
        return false;
    }

    // 4. Enable 1Rx I/Q streaming channels and allocate DMA buffer
    if (!setupRxBuffer(sample_count)) {
        shutdown();
        return false;
    }

    initialized_ = true;
    return true;
}

bool FMCOMMSDriver::configurePhy(long long center_freq,
                                 long long sample_rate,
                                 long long rf_bandwidth,
                                 double rx_gain_db) {
    // RX baseband/front-end configuration channel (input voltage0)
    rx_phy_ch_ = iio_device_find_channel(phy_dev_, "voltage0", false);
    if (!rx_phy_ch_) {
        std::cerr << "[FMCOMMSDriver] Could not find PHY channel 'voltage0' (RX).\n";
        return false;
    }

    // RX Local Oscillator channel (output altvoltage0 -> RX_LO)
    rx_lo_ch_ = iio_device_find_channel(phy_dev_, "altvoltage0", true);
    if (!rx_lo_ch_) {
        std::cerr << "[FMCOMMSDriver] Could not find LO channel 'altvoltage0' (RX_LO).\n";
        return false;
    }

    // Select balanced RF input port A (FMCOMMS2/3/4 RX1A)
    ssize_t str_ret = iio_channel_attr_write(rx_phy_ch_, "rf_port_select", kRxRfPort);
    if (str_ret < 0) {
        logIioError("write rf_port_select", static_cast<int>(str_ret));
        return false;
    }

    if (!setRfBandwidth(rf_bandwidth)) {
        return false;
    }

    if (!setSampleRate(sample_rate)) {
        return false;
    }

    if (!setCenterFrequency(center_freq)) {
        return false;
    }

    // Switch AGC to manual mode before setting hardwaregain so RSSI calibration stays deterministic
    str_ret = iio_channel_attr_write(rx_phy_ch_, "gain_control_mode", kGainControlMode);
    if (str_ret < 0) {
        logIioError("write gain_control_mode", static_cast<int>(str_ret));
        return false;
    }

    if (!setRxGain(rx_gain_db)) {
        return false;
    }

    return true;
}

bool FMCOMMSDriver::setupRxBuffer(int sample_count) {
    // 1Rx complex stream: voltage0 = RX0_I, voltage1 = RX0_Q
    rx0_i_ = iio_device_find_channel(rx_dev_, "voltage0", false);
    rx0_q_ = iio_device_find_channel(rx_dev_, "voltage1", false);

    if (!rx0_i_ || !rx0_q_) {
        std::cerr << "[FMCOMMSDriver] Could not find RX I/Q streaming channels "
                     "('voltage0', 'voltage1') on '" << kRxStreamDeviceName << "'.\n";
        return false;
    }

    iio_channel_enable(rx0_i_);
    iio_channel_enable(rx0_q_);

    // Allocate non-cyclic receive buffer for sample_count complex I/Q pairs
    rx_buf_ = iio_device_create_buffer(rx_dev_, static_cast<std::size_t>(sample_count), false);
    if (!rx_buf_) {
        std::cerr << "[FMCOMMSDriver] Failed to allocate IIO RX buffer of "
                  << sample_count << " samples (" << std::strerror(errno) << ").\n";
        return false;
    }

    sample_count_ = sample_count;
    return true;
}

bool FMCOMMSDriver::setCenterFrequency(long long center_freq_hz) {
    if (!rx_lo_ch_) {
        return false;
    }
    int ret = iio_channel_attr_write_longlong(rx_lo_ch_, "frequency", center_freq_hz);
    if (ret < 0) {
        logIioError("write RX_LO frequency", ret);
        return false;
    }
    center_freq_hz_ = center_freq_hz;
    return true;
}

bool FMCOMMSDriver::setSampleRate(long long sample_rate_hz) {
    if (!rx_phy_ch_) {
        return false;
    }
    int ret = iio_channel_attr_write_longlong(rx_phy_ch_, "sampling_frequency", sample_rate_hz);
    if (ret < 0) {
        logIioError("write sampling_frequency", ret);
        return false;
    }
    sample_rate_hz_ = sample_rate_hz;
    return true;
}

bool FMCOMMSDriver::setRfBandwidth(long long rf_bandwidth_hz) {
    if (!rx_phy_ch_) {
        return false;
    }
    int ret = iio_channel_attr_write_longlong(rx_phy_ch_, "rf_bandwidth", rf_bandwidth_hz);
    if (ret < 0) {
        logIioError("write rf_bandwidth", ret);
        return false;
    }
    rf_bandwidth_hz_ = rf_bandwidth_hz;
    return true;
}

bool FMCOMMSDriver::setRxGain(double rx_gain_db) {
    if (!rx_phy_ch_) {
        return false;
    }
    int ret = iio_channel_attr_write_double(rx_phy_ch_, "hardwaregain", rx_gain_db);
    if (ret < 0) {
        logIioError("write hardwaregain", ret);
        return false;
    }

    // Read back the quantized gain actually applied by the AD9361 gain table
    double applied_gain = rx_gain_db;
    if (iio_channel_attr_read_double(rx_phy_ch_, "hardwaregain", &applied_gain) == 0) {
        rx_gain_db_ = applied_gain;
    } else {
        rx_gain_db_ = rx_gain_db;
    }
    return true;
}

PeakData FMCOMMSDriver::captureAndProcess() {
    if (!initialized_ || !rx_buf_ || !rx0_i_) {
        std::cerr << "[FMCOMMSDriver] captureAndProcess() called before successful init().\n";
        return PeakData{-1, -200.0f};
    }

    // Pull a fresh block of interleaved 16-bit I/Q samples into RAM via DMA
    ssize_t nbytes_rx = iio_buffer_refill(rx_buf_);
    if (nbytes_rx < 0) {
        logIioError("iio_buffer_refill", static_cast<int>(nbytes_rx));
        return PeakData{-1, -200.0f};
    }

    // Pointer to the first enabled RX sample (interleaved int16_t I, int16_t Q)
    void* raw_iq_buffer = iio_buffer_first(rx_buf_, rx0_i_);
    if (!raw_iq_buffer) {
        std::cerr << "[FMCOMMSDriver] iio_buffer_first() returned null pointer.\n";
        return PeakData{-1, -200.0f};
    }

    // Internal handoff to DSP module (dsp_math.h / dsp_math.cpp)
    return find_peak_ex(static_cast<const short*>(raw_iq_buffer), sample_count_, dc_guard_);
}

void FMCOMMSDriver::shutdown() {
    if (rx_buf_) {
        iio_buffer_destroy(rx_buf_);
        rx_buf_ = nullptr;
    }
    if (rx0_i_) {
        iio_channel_disable(rx0_i_);
        rx0_i_ = nullptr;
    }
    if (rx0_q_) {
        iio_channel_disable(rx0_q_);
        rx0_q_ = nullptr;
    }
    if (ctx_) {
        iio_context_destroy(ctx_);
        ctx_ = nullptr;
    }

    phy_dev_ = nullptr;
    rx_dev_ = nullptr;
    rx_phy_ch_ = nullptr;
    rx_lo_ch_ = nullptr;
    sample_count_ = 0;
    initialized_ = false;
}

} // namespace v2x