// 2x oversampling with a 23-tap halfband FIR (flat to 18 kHz, -54 dB above
// 30 kHz at 96 kHz). Polyphase: one branch is a pure delay, the other a
// 12-tap symmetric filter, so each direction costs 6 multiplies per sample.

#ifndef ORINAMI_CLOUDS_DSP_OVERSAMPLER_H_
#define ORINAMI_CLOUDS_DSP_OVERSAMPLER_H_

namespace orinami_clouds {

const int kHalfbandTaps = 6;
const float kHalfband[kHalfbandTaps] = {
  0.313969934f, -0.093640136f, 0.044627008f,
  -0.022049426f, 0.009917661f, -0.003798434f
};

class HalfbandLine {
 public:
  void Init() {
    for (int i = 0; i < kSize; ++i) x_[i] = 0.0f;
    head_ = 0;
  }
  inline void Push(float x) {
    head_ = (head_ - 1) & kMask;
    x_[head_] = x;
  }
  // Sample pushed n calls ago (0 = newest).
  inline float at(int n) const { return x_[(head_ + n) & kMask]; }
  // Symmetric 12-tap branch (unit DC gain when doubled).
  inline float Fir() const {
    float acc = 0.0f;
    for (int j = 0; j < kHalfbandTaps; ++j) {
      acc += kHalfband[j] *
          (at(kHalfbandTaps - 1 - j) + at(kHalfbandTaps + j));
    }
    return acc;
  }
  inline float Delayed() const { return at(kHalfbandTaps - 1); }
 private:
  enum { kSize = 16, kMask = kSize - 1 };   // ring buffer, >= 2 * kHalfbandTaps
  float x_[kSize];
  int head_;
};

class Upsampler2x {
 public:
  void Init() { line_.Init(); }
  // One input sample in, two out (earlier one first).
  inline void Process(float in, float* out) {
    line_.Push(in);
    out[0] = 2.0f * line_.Fir();
    out[1] = line_.Delayed();
  }
 private:
  HalfbandLine line_;
};

class Downsampler2x {
 public:
  void Init() { a_.Init(); b_.Init(); }
  // Two input samples in (earlier one first), one out.
  inline float Process(const float* in) {
    a_.Push(in[0]);
    b_.Push(in[1]);
    return 0.5f * a_.Delayed() + b_.Fir();
  }
 private:
  HalfbandLine a_, b_;
};

}  // namespace orinami_clouds

#endif  // ORINAMI_CLOUDS_DSP_OVERSAMPLER_H_
