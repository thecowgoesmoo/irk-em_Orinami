// Orinami DSP core. No Rack dependencies, so the same file can be compiled
// into a test harness or a Teensy build.
//
// The input sample picks an arc position s on a screen of fixed height. The
// screen is curled (separation) and spun about its hinge (rotation); the two
// outputs are the projections of the struck point onto the default axis and
// the axis orthogonal to it.
//
// Fold and rotation are each a sprung second-order system:
//   T = 1/2 M u'^2 + 1/2 I(u) th'^2,   M = m/2,   I = 2 m (r(u)^2 + r0^2)
//   V = 1/2 kf (u - u0)^2 + 1/2 kr (th - th0)^2
// u is the separation coordinate (1 flat, 0 endpoints touching, -3 four
// wraps), th the rotation, and r(u) the distance of the endpoint masses from
// the hinge, taken from the same geometry that shapes the audio. Rotation is
// integrated as angular momentum, so curling in speeds the spin up and the
// spin throws the masses back out. Negative damping is bounded with a
// van der Pol term, which turns each axis into an oscillator.
#pragma once
#include <cmath>
#include <vector>

namespace orinami {

const int NSHAPE = 65;     // shape -1..+1 in 64 steps; index 32 is the circle
const int NTAU = 2048;     // samples per 2*pi of turning
const int NFINE = 8;       // integration substeps per table sample
const double PI = 3.141592653589793;
const double TWO_PI = 6.283185307179586;

// One period of the screen profile for every shape.
// C(tau) = integral of cos(phi), S(tau) = integral of sin(phi), where phi is
// the tangent angle. Circle: phi = tau. L1 side: turning bunched into 180 deg
// creases at pi/2 + k*pi. Linf side: 90 deg corners at pi/4 + k*pi/2.
// Blending phi between the circle and the staircase keeps every shape flat
// at zero curl.
struct ScreenTables {
	std::vector<float> C, S;

	ScreenTables() : C(NSHAPE * (NTAU + 1)), S(NSHAPE * (NTAU + 1)) {
		const int n = NTAU * NFINE;
		const double h = TWO_PI / n;
		std::vector<double> c(n + 1), s(n + 1);
		for (int j = 0; j < NSHAPE; j++) {
			double shape = -1.0 + 2.0 * j / (NSHAPE - 1);
			double q = (shape < 0.0) ? PI : PI / 2;
			double w = std::fabs(shape);
			double pc = 1.0, ps = 0.0;
			c[0] = 0.0;
			s[0] = 0.0;
			for (int i = 1; i <= n; i++) {
				double tau = i * h;
				double phi = (1.0 - w) * tau + w * q * std::round(tau / q);
				double cc = std::cos(phi), ss = std::sin(phi);
				c[i] = c[i - 1] + 0.5 * h * (pc + cc);
				s[i] = s[i - 1] + 0.5 * h * (ps + ss);
				pc = cc;
				ps = ss;
			}
			// Remove the small closure error from integrating across creases.
			for (int i = 0; i <= NTAU; i++) {
				double frac = (double) i / NTAU;
				C[j * (NTAU + 1) + i] = (float) (c[i * NFINE] - c[n] * frac);
				S[j * (NTAU + 1) + i] = (float) (s[i * NFINE] - s[n] * frac);
			}
		}
	}
};

inline const ScreenTables& screenTables() {
	static ScreenTables t;
	return t;
}

// shape -1..+1 -> fractional table row
inline float shapeIndex(float shape) {
	return (shape + 1.f) * 0.5f * (NSHAPE - 1);
}

// Arc position s on a screen with half-turn alpha -> (y, z) in the screen frame.
inline void screenMap(const ScreenTables& T, float s, float alpha, float shapeIdx, float& y, float& z) {
	float tau = alpha * s;
	float t = std::fabs(tau) * (float) (NTAU / TWO_PI);
	t -= NTAU * std::floor(t / NTAU);
	int i0 = (int) t;
	if (i0 > NTAU - 1)
		i0 = NTAU - 1;
	if (i0 < 0)
		i0 = 0;
	float f = t - i0;

	int j0 = (int) shapeIdx;
	if (j0 > NSHAPE - 2)
		j0 = NSHAPE - 2;
	if (j0 < 0)
		j0 = 0;
	float g = shapeIdx - j0;

	const float* Ca = &T.C[j0 * (NTAU + 1) + i0];
	const float* Cb = Ca + (NTAU + 1);
	const float* Sa = &T.S[j0 * (NTAU + 1) + i0];
	const float* Sb = Sa + (NTAU + 1);

	float ca = Ca[0] + f * (Ca[1] - Ca[0]);
	float cb = Cb[0] + f * (Cb[1] - Cb[0]);
	float sa, sb;
	if (i0 == 0) {
		// S starts out quadratic in tau, not linear.
		sa = f * f * Sa[1];
		sb = f * f * Sb[1];
	}
	else {
		sa = Sa[0] + f * (Sa[1] - Sa[0]);
		sb = Sb[0] + f * (Sb[1] - Sb[0]);
	}
	float cv = ca + g * (cb - ca);
	float sv = sa + g * (sb - sa);

	y = (tau < 0.f ? -cv : cv) / alpha;      // C is odd in tau
	z = sv / alpha;
}

struct Controls {
	double sep0 = 0.5;      // rest separation, 1 flat .. -3
	double th0 = 0.0;       // rest rotation, radians
	float shapeIdx = 32.f;  // from shapeIndex()
	double kf = 1e3;        // fold stiffness
	double m = 1.0;         // mass of each endpoint
	double zf = 0.05;       // fold damping ratio (negative self-oscillates)
	double kr = 1e2;        // rotation stiffness
	double zr = 0.05;       // rotation damping ratio
	double couple = 1.0;    // scales the centrifugal force on the fold
};

struct Core {
	// Model constants
	double sepMin = -3.0;        // overcurl stop
	double r0sq = 0.01;          // inertia floor (r0 = 0.1)
	double foldLim = 0.25;       // limit cycle settles near +/- 2x this
	double rotLim = PI / 4;      // same, radians
	double bounce = 0.5;         // restitution at the stops

	// State (double: LFO-rate increments are tiny against the state)
	double u = 1.0, v = 0.0;     // separation and its rate
	double th = 0.0, L = 0.0;    // rotation and angular momentum
	double uPrev = 1.0, thPrev = 0.0;
	bool primed = false;
	bool foldNeg = false, rotNeg = false;

	void reset() {
		primed = false;
	}

	// Advance the dynamics by dt. foldKick and rotKick are zero, or the peak
	// excursion (separation units, radians) a velocity impulse should produce.
	void step(const Controls& c, double dt, double foldKick = 0.0, double rotKick = 0.0) {
		const ScreenTables& T = screenTables();
		if (!primed) {
			u = c.sep0;
			th = c.th0;
			v = 0.0;
			L = 0.0;
			foldNeg = rotNeg = false;
			primed = true;
		}
		uPrev = u;
		thPrev = th;

		// Inertia from the endpoint masses, and its slope in u
		const float du = 1e-3f;
		float ye, ze, ye2, ze2;
		float a0 = std::max((float) PI * (1.f - (float) u), 1e-6f);
		float a1 = std::max((float) PI * (1.f - (float) u + du), 1e-6f);   // at u - du
		screenMap(T, 1.f, a0, c.shapeIdx, ye, ze);
		screenMap(T, 1.f, a1, c.shapeIdx, ye2, ze2);
		double r2 = (double) ye * ye + (double) ze * ze;
		double dr2 = (r2 - ((double) ye2 * ye2 + (double) ze2 * ze2)) / du;

		double M = std::max(0.5 * c.m, 4.0 * c.kf * dt * dt);              // keeps w0*dt <= 1/2
		double I = std::max(2.0 * c.m * (r2 + r0sq), 4.0 * c.kr * dt * dt);
		double w = L / I;

		v += foldKick * std::sqrt(c.kf / M);
		L += rotKick * std::sqrt(c.kr * I);
		// An axis that has just been given negative damping needs a nudge
		// to leave equilibrium.
		if (c.zf < 0.0 && !foldNeg)
			v += foldLim * std::sqrt(c.kf / M);
		if (c.zr < 0.0 && !rotNeg)
			L += rotLim * std::sqrt(c.kr * I);
		foldNeg = c.zf < 0.0;
		rotNeg = c.zr < 0.0;

		// Fold: spring toward rest, damping, centrifugal push from the spin
		{
			double e = u - c.sep0;
			double z = c.zf;
			if (z < 0.0)
				z *= 1.0 - (e / foldLim) * (e / foldLim);
			double d = z * 2.0 * std::sqrt(c.kf * M);
			double F = -c.kf * e + c.couple * c.m * dr2 * w * w;
			if (d > 0.0)
				v = (v + F * dt / M) / (1.0 + d * dt / M);     // implicit in the damping term
			else
				v += (F - d * v) * dt / M;
			u += v * dt;
			if (u > 1.0) {
				u = 1.0;
				if (v > 0.0)
					v = -bounce * v;
			}
			else if (u < sepMin) {
				u = sepMin;
				if (v < 0.0)
					v = -bounce * v;
			}
		}

		// Rotation, as angular momentum
		{
			double e = th - c.th0;
			double z = c.zr;
			if (z < 0.0)
				z *= 1.0 - (e / rotLim) * (e / rotLim);
			double d = z * 2.0 * std::sqrt(c.kr * I);
			if (d > 0.0)
				L = (L - c.kr * e * dt) / (1.0 + d * dt / I);
			else
				L += (-c.kr * e - d * w) * dt;
			th += (L / I) * dt;
		}

		if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(th) || !std::isfinite(L)) {
			u = uPrev = c.sep0;
			th = thPrev = c.th0;
			v = 0.0;
			L = 0.0;
		}
	}

	// Map one (possibly oversampled) input sample. frac in (0, 1] places it
	// between the previous and current dynamics step.
	void project(float s, float frac, float shapeIdx, bool levelComp, float& outDef, float& outOrth) const {
		const ScreenTables& T = screenTables();
		float uk = (float) (uPrev + (u - uPrev) * frac);
		float tk = (float) (thPrev + (th - thPrev) * frac);
		float alpha = std::max((float) PI * (1.f - uk), 1e-6f);
		float y, z;
		screenMap(T, s, alpha, shapeIdx, y, z);
		if (levelComp) {
			float g = std::max(1.f, alpha);
			y *= g;
			z *= g;
		}
		float cs = std::cos(tk), sn = std::sin(tk);
		outDef = y * cs - z * sn;
		outOrth = y * sn + z * cs;
	}
};

} // namespace orinami
