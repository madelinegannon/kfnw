﻿#pragma once

#include "ofMain.h"

/**
 * From https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/PID-Cpp-Implementation.html#real-world-implementation.
 */

/// Standard PID (proportional, integral, derivative) controller. Derivative
/// component is filtered using an exponential moving average filter.
class PID {
public:
	PID() = default;
	/// @param  kp
	///         Proportional gain
	/// @param  ki
	///         Integral gain
	/// @param  kd
	///         Derivative gain
	/// @param  Ts
	///         Sampling time (seconds)
	/// @param  fc
	///         Cutoff frequency of derivative EMA filter (Hertz),
	///         zero to disable the filter entirely
	PID(float kp, float ki, float kd, float Ts, float f_c = 0,
		float maxOutput = 300)
		: Ts(Ts), maxOutput(maxOutput) {
		setKp(kp);
		setKi(ki);
		setKd(kd);
		setEMACutoff(f_c);
	}

	/// Update the controller: given the current position, compute the control
	/// action.
	float update(uint16_t input) {
		// The error is the difference between the reference (setpoint) and the
		// actual position (input)
		int16_t error = setpoint - input;
		// The integral or sum of current and previous errors
		int32_t newIntegral = integral + error;
		// Compute the difference between the current and the previous input,
		// but compute a weighted average using a factor α ∊ (0,1]
		float diff = emaAlpha * (prevInput - input);
		// Update the average
		prevInput -= diff;

		// Check if we can turn off the motor
		if (activityCount >= activityThres && activityThres) {
			float filtError = setpoint - prevInput;
			if (filtError >= -errThres && filtError <= errThres) {
				errThres = 2; // hysteresis
				integral = newIntegral;
				return 0;
			}
			else {
				errThres = 1;
			}
		}
		else {
			++activityCount;
			errThres = 1;
		}

		bool backward = false;
		int32_t calcIntegral = backward ? newIntegral : integral;

		// Standard PID rule
		float output = kp * error + ki_Ts * calcIntegral + kd_Ts * diff;

		// Clamp and anti-windup
		if (output > maxOutput)
			output = maxOutput;
		else if (output < -maxOutput)
			output = -maxOutput;
		else
			integral = newIntegral;

		return output;
	}

	void setKp(float kp) { this->kp = kp; }               ///< Proportional gain
	void setKi(float ki) { this->ki_Ts = ki * this->Ts; } ///< Integral gain
	void setKd(float kd) { this->kd_Ts = kd / this->Ts; } ///< Derivative gain

	float getKp() const { return kp; }         ///< Proportional gain
	float getKi() const { return ki_Ts / Ts; } ///< Integral gain
	float getKd() const { return kd_Ts * Ts; } ///< Derivative gain

	float calcAlphaEMA(float fn) {
		if (fn <= 0)
			return 1;
		// α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
		const float c = std::cos(2 * float(PI) * fn);
		return c - 1 + std::sqrt(c * c - 4 * c + 3);
	}

	/// Set the cutoff frequency (-3 dB point) of the exponential moving average
	/// filter that is applied to the input before taking the difference for
	/// computing the derivative term.
	void setEMACutoff(float f_c) {
		float f_n = f_c * Ts; // normalized sampling frequency
		this->emaAlpha = f_c == 0 ? 1 : calcAlphaEMA(f_n);
	}

	/// Set the reference/target/setpoint of the controller.
	void setSetpoint(uint16_t setpoint) {
		if (this->setpoint != setpoint) this->activityCount = 0;
		this->setpoint = setpoint;
	}
	/// @see @ref setSetpoint(int16_t)
	uint16_t getSetpoint() const { return setpoint; }

	/// Set the maximum control output magnitude. Default is 255, which clamps
	/// the control output in [-255, +255].
	void setMaxOutput(float maxOutput) { this->maxOutput = maxOutput; }
	/// @see @ref setMaxOutput(float)
	float getMaxOutput() const { return maxOutput; }

	/// Reset the activity counter to prevent the motor from turning off.
	void resetActivityCounter() { this->activityCount = 0; }
	/// Set the number of seconds after which the motor is turned off, zero to
	/// keep it on indefinitely.
	void setActivityTimeout(float s) {
		if (s == 0)
			activityThres = 0;
		else
			activityThres = uint16_t(s / Ts) == 0 ? 1 : s / Ts;
	}

	/// Reset the sum of the previous errors to zero.
	void resetIntegral() { integral = 0; }

private:
	float Ts = 1;               ///< Sampling time (seconds)
	float maxOutput = 255;      ///< Maximum control output magnitude
	float kp = 1;               ///< Proportional gain
	float ki_Ts = 0;            ///< Integral gain times Ts
	float kd_Ts = 0;            ///< Derivative gain divided by Ts
	float emaAlpha = 1;         ///< Weight factor of derivative EMA filter.
	float prevInput = 0;        ///< (Filtered) previous input for derivative.
	uint16_t activityCount = 0; ///< How many ticks since last setpoint change.
	uint16_t activityThres = 0; ///< Threshold for turning off the output.
	uint8_t errThres = 1;       ///< Threshold with hysteresis.
	int32_t integral = 0;       ///< Sum of previous errors for integral.
	uint16_t setpoint = 0;      ///< Position reference.
};

