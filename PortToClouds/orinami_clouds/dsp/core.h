// Orinami for Clouds hardware: DSP core, single precision.
//
// Same model as the Rack module (see OrinamiCore.hpp there):
//   T = 1/2 M u'^2 + 1/2 I(u) th'^2,   M = m/2,   I = 2 m (r(u)^2 + r0^2)
//   V = 1/2 kf (u - u0)^2 + 1/2 kr (th - th0)^2
// u is the separation (1 flat, 0 ends touching, -3 four wraps), th the
// rotation, r(u) the distance of the end masses from the hinge.
//
// Differences from the Rack core, all forced by the Cortex-M4F:
//  - float only. The four state variables use compensated (Kahan) sums so
//    that slow LFO motion is not lost to rounding at 48 kHz. This relies on
//    strict IEEE arithmetic: do not build with -ffast-math.
//  - 17 x 512 screen tables and a sine table, filled at boot (74 kB of RAM).
//  - stiffness-derived constants are refreshed once per block, not per sample.
//
// No hardware dependencies: this header also compiles into the desktop test.

#ifndef ORINAMI_CLOUDS_DSP_CORE_H_
#define ORINAMI_CLOUDS_DSP_CORE_H_

#include <cmath>
#include <cstddef>

namespace orinami_clouds {

const int kNumShapes = 17;       // shape -1..+1 in 16 steps; row 8 is the circle
const int kTableSize = 512;      // samples per 2*pi of turning
const int kSineSize = 1024;
const float kPi = 3.14159265358979f;
const float kTwoPi = 6.28318530717959f;

const float kSepMin = -3.0f;     // overcurl stop (four wraps)
const float kR0Sq = 0.01f;       // inertia floor, r0 = 0.1
const float kFoldLim = 0.25f;    // limit cycle settles near +/- 2x this
const float kRotLim = 0.78539816f;
const float kBounce = 0.5f;      // restitution at the stops

struct Tables {
  float c[kNumShapes][kTableSize + 1];
  float s[kNumShapes][kTableSize + 1];
  float sine[kSineSize + 1];

  // Screen profile: tangent angle phi against the turning angle tau a circle
  // would have. Circle: phi = tau. L1 side: 180 degree creases at
  // pi/2 + k*pi. Linf side: 90 degree corners at pi/4 + k*pi/2.
  void Init() {
    const int fine = 8;
    const int n = kTableSize * fine;
    const float h = kTwoPi / static_cast<float>(n);
    for (int j = 0; j < kNumShapes; ++j) {
      float shape = -1.0f + 2.0f * static_cast<float>(j) / (kNumShapes - 1);
      float q = shape < 0.0f ? kPi : 0.5f * kPi;
      float w = fabsf(shape);
      float pc = 1.0f, ps = 0.0f, ac = 0.0f, as = 0.0f;
      c[j][0] = 0.0f;
      s[j][0] = 0.0f;
      for (int i = 1; i <= n; ++i) {
        float tau = static_cast<float>(i) * h;
        float phi = (1.0f - w) * tau + w * q * floorf(tau / q + 0.5f);
        float cc = cosf(phi), ss = sinf(phi);
        ac += 0.5f * h * (pc + cc);
        as += 0.5f * h * (ps + ss);
        pc = cc;
        ps = ss;
        if (i % fine == 0) {
          c[j][i / fine] = ac;
          s[j][i / fine] = as;
        }
      }
      // Remove the small closure error from integrating across creases.
      float ec = c[j][kTableSize], es = s[j][kTableSize];
      for (int i = 0; i <= kTableSize; ++i) {
        float frac = static_cast<float>(i) / kTableSize;
        c[j][i] -= ec * frac;
        s[j][i] -= es * frac;
      }
    }
    for (int i = 0; i <= kSineSize; ++i) {
      sine[i] = sinf(kTwoPi * static_cast<float>(i) / kSineSize);
    }
  }
};

inline float ShapeIndex(float shape) {
  return (shape + 1.0f) * 0.5f * (kNumShapes - 1);
}

// Arc position s on a screen with half-turn alpha -> (y, z), screen frame.
inline void ScreenMap(
    const Tables& t, float s, float alpha, float shape_index,
    float* y, float* z) {
  float tau = alpha * s;
  float p = fabsf(tau) * (kTableSize / kTwoPi);
  int wraps = static_cast<int>(p * (1.0f / kTableSize));
  p -= static_cast<float>(wraps * kTableSize);
  int i0 = static_cast<int>(p);
  if (i0 > kTableSize - 1) i0 = kTableSize - 1;
  float f = p - static_cast<float>(i0);

  int j0 = static_cast<int>(shape_index);
  if (j0 > kNumShapes - 2) j0 = kNumShapes - 2;
  if (j0 < 0) j0 = 0;
  float g = shape_index - static_cast<float>(j0);

  const float* ca = &t.c[j0][i0];
  const float* cb = &t.c[j0 + 1][i0];
  const float* sa = &t.s[j0][i0];
  const float* sb = &t.s[j0 + 1][i0];
  float c0 = ca[0] + f * (ca[1] - ca[0]);
  float c1 = cb[0] + f * (cb[1] - cb[0]);
  float s0, s1;
  if (i0 == 0) {
    // S starts out quadratic in tau, not linear.
    s0 = f * f * sa[1];
    s1 = f * f * sb[1];
  } else {
    s0 = sa[0] + f * (sa[1] - sa[0]);
    s1 = sb[0] + f * (sb[1] - sb[0]);
  }
  float cv = c0 + g * (c1 - c0);
  float sv = s0 + g * (s1 - s0);
  float inv = 1.0f / alpha;
  *y = (tau < 0.0f ? -cv : cv) * inv;      // C is odd in tau
  *z = sv * inv;
}

// sin and cos of an unbounded angle from the table.
inline void SinCos(const Tables& t, float angle, float* sn, float* cs) {
  float p = angle * (1.0f / kTwoPi);
  p -= floorf(p);
  p *= kSineSize;
  int i = static_cast<int>(p);
  float f = p - static_cast<float>(i);
  i &= kSineSize - 1;
  *sn = t.sine[i] + f * (t.sine[i + 1] - t.sine[i]);
  int k = (i + kSineSize / 4) & (kSineSize - 1);
  *cs = t.sine[k] + f * (t.sine[k + 1] - t.sine[k]);
}

// Compensated accumulator: keeps the rounding error of each add and feeds it
// back into the next one.
struct Accumulator {
  float value;
  float carry;
  inline void Set(float v) { value = v; carry = 0.0f; }
  inline void Add(float x) {
    float y = x - carry;
    float t = value + y;
    carry = (t - value) - y;
    value = t;
  }
};

// Slowly varying inputs, set once per block.
struct BlockParameters {
  float shape_index;   // from ShapeIndex()
  float kf;            // fold stiffness
  float m;             // mass of each end
  float zf;            // fold damping ratio (negative self-oscillates)
  float kr;            // rotation stiffness
  float zr;            // rotation damping ratio
  float couple;        // centrifugal coupling, 0..1
  bool level_comp;
};

class Core {
 public:
  Core() { }

  void Init(const Tables* tables, float sample_rate) {
    t_ = tables;
    dt_ = 1.0f / sample_rate;
    primed_ = false;
    fold_neg_ = rot_neg_ = false;
    u_.Set(1.0f);
    v_.Set(0.0f);
    th_.Set(0.0f);
    l_.Set(0.0f);
    u_prev_ = 1.0f;
    th_prev_ = 0.0f;
    fold_kick_ = rot_kick_ = 0.0f;
  }

  // Queue a kick. Amounts are the peak excursion the velocity impulse
  // should produce: separation units, radians.
  inline void KickFold(float amount) { fold_kick_ += amount; }
  inline void KickRotation(float amount) { rot_kick_ += amount; }

  inline float separation() const { return u_.value; }
  inline float rotation() const { return th_.value; }

  // Once per block.
  void Prepare(const BlockParameters& p) {
    p_ = p;
    big_m_ = 0.5f * p.m;
    float m_min = 4.0f * p.kf * dt_ * dt_;      // keeps w0*dt <= 1/2
    if (big_m_ < m_min) big_m_ = m_min;
    dt_over_m_ = dt_ / big_m_;
    wf_ = sqrtf(p.kf / big_m_);
    cf_ = 2.0f * sqrtf(p.kf * big_m_);
    i_min_ = 4.0f * p.kr * dt_ * dt_;

    // An axis that has just been given negative damping needs a nudge to
    // leave equilibrium.
    if (p.zf < 0.0f && !fold_neg_) fold_kick_ += kFoldLim;
    if (p.zr < 0.0f && !rot_neg_) rot_kick_ += kRotLim;
    fold_neg_ = p.zf < 0.0f;
    rot_neg_ = p.zr < 0.0f;
  }

  // Once per sample: advance fold and rotation toward their rest values.
  inline void Step(float sep0, float th0) {
    if (!primed_) {
      u_.Set(sep0);
      th_.Set(th0);
      v_.Set(0.0f);
      l_.Set(0.0f);
      primed_ = true;
    }
    u_prev_ = u_.value;
    th_prev_ = th_.value;

    // Inertia from the end masses, and its slope in u.
    const float du = 4e-3f;
    float u = u_.value;
    float a0 = kPi * (1.0f - u);
    if (a0 < 1e-4f) a0 = 1e-4f;
    float ye, ze, yb, zb;
    ScreenMap(*t_, 1.0f, a0, p_.shape_index, &ye, &ze);
    ScreenMap(*t_, 1.0f, a0 + kPi * du, p_.shape_index, &yb, &zb);   // at u - du
    float r2 = ye * ye + ze * ze;
    float dr2 = (r2 - (yb * yb + zb * zb)) * (1.0f / du);
    float inertia = 2.0f * p_.m * (r2 + kR0Sq);
    if (inertia < i_min_) inertia = i_min_;
    float inv_i = 1.0f / inertia;
    float w = l_.value * inv_i;

    if (fold_kick_ != 0.0f) {
      v_.Add(fold_kick_ * wf_);
      fold_kick_ = 0.0f;
    }
    if (rot_kick_ != 0.0f) {
      l_.Add(rot_kick_ * sqrtf(p_.kr * inertia));
      rot_kick_ = 0.0f;
    }

    // Fold: spring toward rest, damping, centrifugal push from the spin.
    {
      float e = u - sep0;
      float z = p_.zf;
      if (z < 0.0f) {
        float q = e * (1.0f / kFoldLim);
        z *= 1.0f - q * q;
      }
      float d = z * cf_;
      float force = -p_.kf * e + p_.couple * p_.m * dr2 * w * w;
      float dv = (force - d * v_.value) * dt_over_m_;
      if (d > 0.0f) dv /= 1.0f + d * dt_over_m_;     // implicit in the damping
      v_.Add(dv);
      u_.Add(v_.value * dt_);
      if (u_.value > 1.0f) {
        u_.Set(1.0f);
        if (v_.value > 0.0f) v_.Set(-kBounce * v_.value);
      } else if (u_.value < kSepMin) {
        u_.Set(kSepMin);
        if (v_.value < 0.0f) v_.Set(-kBounce * v_.value);
      }
    }

    // Rotation, as angular momentum.
    {
      float e = th_.value - th0;
      float z = p_.zr;
      if (z < 0.0f) {
        float q = e * (1.0f / kRotLim);
        z *= 1.0f - q * q;
      }
      float d = z * 2.0f * sqrtf(p_.kr * inertia);
      float g = d * dt_ * inv_i;
      float dl = (-p_.kr * e - d * w) * dt_;
      if (d > 0.0f) dl /= 1.0f + g;
      l_.Add(dl);
      th_.Add(l_.value * inv_i * dt_);
    }

    // Recover from anything non-finite.
    float check = u_.value + v_.value + th_.value + l_.value;
    if (!(check == check) || check > 1e30f || check < -1e30f) {
      u_.Set(sep0);
      th_.Set(th0);
      v_.Set(0.0f);
      l_.Set(0.0f);
      u_prev_ = sep0;
      th_prev_ = th0;
    }
  }

  // Map one input sample. frac in (0, 1] places it between the previous and
  // the current dynamics step (0.5 and 1 for 2x oversampling).
  inline void Project(float s, float frac, float* out_def, float* out_orth)
      const {
    float u = u_prev_ + (u_.value - u_prev_) * frac;
    float th = th_prev_ + (th_.value - th_prev_) * frac;
    float alpha = kPi * (1.0f - u);
    if (alpha < 1e-4f) alpha = 1e-4f;
    float y, z;
    ScreenMap(*t_, s, alpha, p_.shape_index, &y, &z);
    if (p_.level_comp && alpha > 1.0f) {
      y *= alpha;
      z *= alpha;
    }
    float sn, cs;
    SinCos(*t_, th, &sn, &cs);
    *out_def = y * cs - z * sn;
    *out_orth = y * sn + z * cs;
  }

 private:
  const Tables* t_;
  BlockParameters p_;
  float dt_;
  float big_m_, dt_over_m_, wf_, cf_, i_min_;
  Accumulator u_, v_, th_, l_;
  float u_prev_, th_prev_;
  float fold_kick_, rot_kick_;
  bool primed_, fold_neg_, rot_neg_;
};

}  // namespace orinami_clouds

#endif  // ORINAMI_CLOUDS_DSP_CORE_H_
