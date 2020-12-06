#ifndef INC_BLDC_MOTOR_CONTROL_H_
#define INC_BLDC_MOTOR_CONTROL_H_

#include "common_def.h"
#include "complex.h"
#include "discrete_systems.h"
#include "system_identification.h"
#include "systems_analyze.h"

namespace SystemControl {
namespace MotorControl {

static constexpr real_t sqrt3 = M_SQRT3;
static constexpr real_t sqrt3div2 = sqrt3 / 2.0F;
static constexpr real_t two_thirds = 2.0F / 3.0F;
static constexpr real_t four_thirds = 4.0F / 3.0F;

static constexpr real_t pi = M_PI;
static constexpr real_t two_PI = 2.0F * M_PI;
static constexpr real_t half_PI = M_PI / 2.0F;

namespace Transform {
real_t sin(real_t);
real_t cos(real_t);
}

class Vector2Phase: public Complex {
public:
	Vector2Phase(real_t x = 0.0, real_t y = 0.0) :
			Complex(x, y), x(real), y(imag) {
	}
	Vector2Phase(const Complex& C) :
			Vector2Phase(C.get_real(), C.get_imag()) {
	}
	inline Vector2Phase& operator=(const Vector2Phase& other) {
		x = other.x;
		y = other.y;
		return *this;
	}

	inline Vector2Phase rotate(real_t fi) const {
		real_t sin_fi = Transform::sin(fi);
		real_t cos_fi = Transform::cos(fi);
		return Vector2Phase(cos_fi * x - sin_fi * y, cos_fi * y + sin_fi * x);
	}
	inline Vector2Phase rotate(const Vector2Phase& vecB) const {
		const Vector2Phase& vecA = *this;
		return Vector2Phase(vecA * vecB);
	}

	inline Vector2Phase normalize() const {
		real_t size = magnitude();
		static constexpr real_t min_size = 1e-3;
		if (size < min_size)
			size += min_size;
		return Vector2Phase(x / size, y / size);
	}
	inline Vector2Phase normalize_fast() const {
		real_t size = magnitude_fast();
		static constexpr real_t min_size = 1e-3;
		if (size < min_size)
			size += min_size;
		return Vector2Phase(x / size, y / size);
	}

	inline Vector2Phase transformDQtoAB(real_t fi) const {
		return rotate(fi);
	}
	inline Vector2Phase transformABtoDQ(real_t fi) const {
		return rotate(-fi);
	}

	inline Vector2Phase transformDQtoAB(const Vector2Phase& posVector) const {
		const Vector2Phase& vecDQ = *this;
		return vecDQ * posVector;
	}
	inline Vector2Phase transformABtoDQ(const Vector2Phase& posVector) const {
		const Vector2Phase& vecAB = *this;
		return vecAB * (posVector.conj());
	}

	real_t& x;
	real_t& y;

private:

};
class Vector3Phase {
public:

	enum phase_enum {
		phaseA, phaseB, phaseC
	};

	Vector3Phase() :
			Vector3Phase(NAN, NAN, NAN) {
	}

	Vector3Phase(real_t a, real_t b, real_t c) :
			a(a), b(b), c(c) {
	}

	Vector3Phase(real_t a, real_t b, real_t c, phase_enum phase_unknown) :
			a(a), b(b), c(c) {
		switch (phase_unknown) {
		case phaseA:
			this->a = -b - c;
			break;
		case phaseB:
			this->b = -a - c;
			break;
		case phaseC:
			this->c = -a - b;
			break;
		default:
			break;
		};
	}
	Vector3Phase(real_t u, phase_enum phase_known) {
		switch (phase_known) {
		case phaseA:
			a = u;
			b = -u / 2.0;
			c = -u / 2.0;
			break;
		case phaseB:
			b = u;
			a = -u / 2.0;
			c = -u / 2.0;
			break;
		case phaseC:
			c = u;
			a = -u / 2.0;
			b = -u / 2.0;
			break;
		default:
			break;
		};
	}
	inline Vector3Phase& operator=(const Vector3Phase& other) {
		a = other.a;
		b = other.b;
		c = other.c;
		return *this;
	}

	inline bool is_symmetric() const {
		return (a + b + c == 0);
	}
	inline bool is_positive() const {
		return (a >= 0 && b >= 0 && c >= 0);
	}
	inline real_t min() const {
		return MIN(MIN(a,b), c);
	}
	inline real_t max() const {
		return MAX(MAX(a,b), c);
	}

	inline Vector3Phase toSymmetric() const {
		return Vector3Phase((2.0 * a - b - c) / 3.0, (2.0 * b - a - c) / 3.0, (2.0 * c - a - b) / 3.0);
	}

	union {
		struct {
			real_t a;
			real_t b;
			real_t c;
		};
		real_t data[3];
	};
private:

};

namespace Transform {

inline Vector3Phase Vector2PhaseToVector3Phase(const Vector2Phase& vec2Phase) {
	real_t x_half = 0.5 * vec2Phase.x;
	real_t y_sqrt3div2 = sqrt3div2 * vec2Phase.y;
	return Vector3Phase(vec2Phase.x, -x_half + y_sqrt3div2, -x_half - y_sqrt3div2);
}
inline Vector2Phase Vector3PhaseSymmetricToVector2Phase(const Vector3Phase& vec3Phase) {
	return Vector2Phase(vec3Phase.a, (vec3Phase.b - vec3Phase.c) / sqrt3);
}
inline Vector2Phase Vector3PhaseNonSymmetricToVector2Phase(const Vector3Phase& vec3Phase) {
	return Vector2Phase(two_thirds * vec3Phase.a - (vec3Phase.b + vec3Phase.c) / 3.0, (vec3Phase.b - vec3Phase.c) / sqrt3);
}

inline real_t mod(real_t num, real_t den) {
	return (num - ((int) (num / den) * den));
}

template<size_t N>
class LookupTable {
public:
	virtual real_t operator()(real_t arg) const {
		int idx = int((arg - interval_low) / interval_length * (N - 1));
#ifdef ASSERT_DIMENSIONS
		if (idx >= (int) N || idx < 0)
		throw exception_INDEX_OUT_OF_RANGE;
#endif
		return values[idx];
	}

	LookupTable(std::function<real_t(real_t)> fcn, real_t interval_low, real_t interval_high) :
			interval_low(interval_low), interval_length(interval_high - interval_low) {
		for (uint i = 0; i < N; i++) {
			values[i] = fcn((real_t) (i * interval_length / (N - 1) + interval_low));
		}
	}

protected:
	const real_t interval_low;
	const real_t interval_length;
	real_t values[N];

};

template<size_t N>
class SinLookup: public LookupTable<N> {
public:
	SinLookup() :
			LookupTable<N>(std::function<real_t(real_t)>((real_t (*)(real_t))&std::sin), 0, half_PI) {}
			inline real_t operator()(real_t fi) const {
				int sign = 1;
				if (fi < 0) {
					sign *= -1;
					fi = -fi;
				}
				fi = mod(fi, two_PI);
				if (fi >= pi) {
					sign *= -1;
					fi -= pi;
				}
				if (fi >= half_PI) {
					fi = pi - fi;
				}
				return sign * LookupTable<N>::operator()(fi);
			}
		};

real_t sinTaylor(real_t, uint = 3);

}

struct MotorParametersElectrical {
	union {
		struct {
			real_t Rs;
			real_t Ld;
			real_t Lq;
			real_t Psi_f;
		};
		real_t data[4] = { 0 };
	};
	uint p;
};
struct MotorParametersMechanical {
	union {
		struct {
			real_t J;
			real_t B;
		};
		real_t data[2] = { 0 };
	};
};

struct MotorParameters: MotorParametersElectrical, MotorParametersMechanical {
};

class VoltageInverter {

public:
	VoltageInverter(real_t Ucc) :
			Ucc(Ucc) {
	}
	virtual void setVoltage(const Vector3Phase&)=0;
	virtual void setDuty(const Vector3Phase&)=0;
	virtual void setFreq(uint)=0;
	virtual real_t getFreq() const=0;
	virtual void start()=0;
	virtual void stop()=0;
protected:
	const real_t Ucc = 0;

};

class VoltageInverterSVPWM: public VoltageInverter {

	using VoltageInverter::VoltageInverter;
public:
	void setVoltage(const Vector3Phase&);
};

class VoltageInverterSVPWMmin: public VoltageInverter {
	using VoltageInverter::VoltageInverter;
public:
	void setVoltage(const Vector3Phase&);
};

class CurrentFeedback {

public:
	CurrentFeedback(real_t k, uint resolution, uint16_t offset) :
			k(k), offset(offset), full_scale((1 << resolution) - 1) {
	}
	virtual const Vector3Phase& getCurrent() const=0;

protected:
	inline real_t rawData2Current(uint16_t data) const {
		return (real_t) ((int32_t) data - (int32_t) offset) / (real_t) full_scale * k;
	}

	const real_t k;
	const uint16_t offset;
	const uint16_t full_scale;

};

class UpdateCallback {
public:
	~UpdateCallback() {
	}
	virtual Vector3Phase operator()(const Vector3Phase& current) {
		return Vector3Phase(0, 0, 0);
	}
};

class PositionAndSpeed {

public:
	PositionAndSpeed(real_t phi = 0.0, real_t omega = 0.0, int32_t n_revs = 0) :
			phi(phi), n_revs(n_revs), omega(omega) {
	}

	inline real_t getAbsolutePosition() const {
		return n_revs * two_PI + phi;
	}

	real_t phi = 0;
	int32_t n_revs = 0;
	real_t omega = 0;
private:
};

class PositionFeedback {
public:
	virtual PositionAndSpeed getPositionAndSpeed(const Vector2Phase&)=0;
};

class BEMFObserverGeneric {
public:
	virtual Vector2Phase estimate(const Vector2Phase&, const Vector2Phase&, real_t)=0;
};

class BEMFObserverLuenberg: public BEMFObserverGeneric {

public:
	BEMFObserverLuenberg(const MotorParametersElectrical&, approximation_method, real_t, real_t, real_t);
	BEMFObserverLuenberg(const MotorParametersElectrical&, approximation_method, real_t, Complex, Complex);
	Vector2Phase estimate(const Vector2Phase&, const Vector2Phase&, real_t = 0);
private:
	const real_t g1;
	const real_t g2;
	const MotorParametersElectrical& parameters;
public:
	DiscreteIntegrator iAlfaIntegrator;
	DiscreteIntegrator iBetaIntegrator;
	DiscreteIntegratorLimited eAlfaIntegrator;
	DiscreteIntegratorLimited eBetaIntegrator;

};

class DiscreteIntegratorModulo: public DiscreteIntegrator {
	using DiscreteIntegrator::DiscreteIntegrator;
public:

	inline void setModulo(real_t modulo) {
		this->modulo = modulo;
	}
	inline int getIntegerPart() const {
		return integer_part;
	}

private:
	real_t modify_output(real_t y) {
		integer_part = (int) (y / modulo);
		return y - integer_part * modulo;
	}
	int integer_part = 0;
	real_t modulo = two_PI;
};

class BEMFPositionFeedbackPLLGeneric: public PositionFeedback {
public:
	BEMFPositionFeedbackPLLGeneric(const MotorParametersElectrical& motorParameters, approximation_method method, real_t Ts, real_t P_gain, real_t I_gain) :
			integrator(method, Ts), PI_regulator(Ts, P_gain, I_gain, method) {
	}
	PositionAndSpeed getPositionAndSpeed(const Vector2Phase&);
	inline real_t get_error() const {
		return error;
	}
protected:
	virtual real_t calculateError(const Vector2Phase&)=0;
	PositionAndSpeed phi_omega;
	DiscreteIntegratorModulo integrator;
private:
	real_t error = 0;
public:
	DiscretePIregulator PI_regulator;

};

class BEMF_PositionFeedbackPLLStandard: public BEMFPositionFeedbackPLLGeneric {
	using BEMFPositionFeedbackPLLGeneric::BEMFPositionFeedbackPLLGeneric;
private:
	real_t calculateError(const Vector2Phase&);
};

class BEMF_PositionFeedbackPLLImproved: public BEMFPositionFeedbackPLLGeneric {
	using BEMFPositionFeedbackPLLGeneric::BEMFPositionFeedbackPLLGeneric;
private:
	real_t calculateError(const Vector2Phase&);
};

class OpenLoopControlGeneric {

public:
	OpenLoopControlGeneric(real_t Ts, approximation_method method, real_t dOmega) :
			integrator_phi(method, Ts), integrator_omega(method, Ts), dOmega(dOmega) {
	}

	inline real_t getOmega() const {
		return integrator_omega.output(0);
	}
	inline real_t getPhi() const {
		return integrator_phi.output(0);
	}
	inline void set_dOmega(real_t dOmega) {
		this->dOmega = dOmega;
	}
	inline void set_omegaRef(real_t omegaRef) {
		this->omegaRef = omegaRef;
	}
	PositionAndSpeed operator()();

	void setStates(const PositionAndSpeed&);
protected:

	DiscreteIntegratorModulo integrator_phi;
	DiscreteIntegrator integrator_omega;
	real_t dOmega;
	real_t omegaRef = 0;
	static constexpr real_t omega_error_threshold = 10;
};

class OpenLoopControlIF: public OpenLoopControlGeneric {
public:
	OpenLoopControlIF(real_t Ts, approximation_method method, real_t dOmega, real_t P_gain, real_t I_gain) :
			OpenLoopControlGeneric(Ts, method, dOmega), iReg(Ts, P_gain, I_gain, method) {
	}
	Vector2Phase operator()(const Vector2Phase&);

	inline void set_iRef(real_t iRef) {
		this->iRef = iRef;
	}
	DiscretePIregulator iReg;
private:

	real_t iRef = 0;

};

class OpenLoopControlUF: public OpenLoopControlGeneric {
public:
	OpenLoopControlUF(real_t Ts, approximation_method method, real_t dOmega, real_t u_per_omega) :
			OpenLoopControlGeneric(Ts, method, dOmega), u_per_omega(u_per_omega) {
	}
	Vector2Phase operator()(const Vector2Phase&);

private:
	const real_t u_per_omega;
};

class CurrentControl: public UpdateCallback {

public:
	CurrentControl(const MotorParametersElectrical& motorParameters, approximation_method method, real_t Ts, real_t P_gain, real_t I_gain, BEMFObserverGeneric& observer, BEMFPositionFeedbackPLLGeneric& positionFeedback) : //open_loop_control(Ts, method, open_loop_dOmega, open_loop_P_gain, open_loop_I_gain)  //
			idReg(Ts, P_gain, I_gain, method), iqReg(Ts, P_gain, I_gain, method), parameters(motorParameters), BEMF_observer(observer), positionFeedback(positionFeedback), open_loop_control(Ts, method, open_loop_dOmega) {
		open_loop_control.set_omegaRef(open_loop_omegaRef);
	}
	Vector3Phase operator()(const Vector3Phase&);
	inline void set_iDQRef(const Vector2Phase& iRef) {
		iDQRef = iRef;
	}
	inline void set_iDRef(real_t iD_ref) {
		iDQRef.x = iD_ref;
	}
	inline void set_iQRef(real_t iQ_ref) {
		iDQRef.y = iQ_ref;
	}

	inline const PositionAndSpeed& get_PositionAndSpeed() const {
		return phi_omega;
	}
	inline const Vector2Phase& get_iDQ() const {
		return iDQ;
	}
	inline const Vector2Phase& get_uAB() const {
		return uAB;
	}
	inline const Vector2Phase& get_BEMF() const {
		return BEMF;
	}
	inline const real_t get_BEMF_magnitude() const {
		return BEMF.magnitude_fast();
	}
	inline real_t getTheta() const {
		return iDQ.phase();
	}
	inline real_t getMoment() const {
		return 1.5 * parameters.p * (parameters.Psi_f * iDQ.y + (parameters.Ld - parameters.Lq) * iDQ.x * iDQ.y);
	}

	inline bool isClosedLoop() const {
		return closed_loop;
	}
	inline void set_decoupling_enable(bool new_state) {
		decoupling_enable = new_state;
	}

	DiscreteIPregulator idReg;
	DiscreteIPregulator iqReg;

private:
	const MotorParametersElectrical& parameters;

	Vector2Phase iDQRef;
	Vector2Phase iDQ;
	Vector2Phase uAB;
	Vector2Phase BEMF;
	PositionAndSpeed phi_omega;

	BEMFObserverGeneric& BEMF_observer;
	BEMFPositionFeedbackPLLGeneric& positionFeedback;
	bool decoupling_enable = true;
	bool closed_loop = false;

	OpenLoopControlGeneric open_loop_control;

	static constexpr real_t open_loop_dOmega = 400;
	static constexpr real_t open_loop_omegaRef = 500;
	static constexpr real_t open_loop_iQRef = 0.75;

	static constexpr real_t omega_threshold_low = 25;
	static constexpr real_t omega_threshold_high = 200;
	static constexpr real_t phi_error_threshold_low = M_PI / 30.0;
	static constexpr real_t BEMF_mag_threshold_low = 0.15;
	static constexpr real_t BEMF_mag_threshold_high = 0.75;

};
class IdentificationGeneric {
public:
	virtual uint get_length() const=0;
	inline uint get_idx() const {
		return idx;
	}
	virtual bool is_running() const {
		return idx < get_length();
	}
protected:
	uint idx = 0;
};

class IdentificationRsLdLq: public UpdateCallback, public IdentificationGeneric {

public:
	IdentificationRsLdLq(uint n_samples, real_t Ts, const VectorReal& u_vector, bool different_LqLd = false) :
			different_LqLd(different_LqLd), Ts(Ts), u_vector(u_vector), id_data(n_samples), iq_data(different_LqLd ? n_samples : 1), settle_iterations_mechanical((uint) (settle_time_mechanical / Ts)), settle_iterations_electrical((uint) (settle_time_electrical / Ts)) {
		u_vector.assert();
		if (u_vector.get_length() != n_samples)
			throw exception_code(exception_WRONG_DIMENSIONS);

	}
	bool is_running() const {
		return state < state_end;
	}
	uint get_length() const {
		return id_data.get_length();
	}
	void reset();
	Vector3Phase operator()(const Vector3Phase&);
	void estimate(MotorParametersElectrical&);
private:
	bool different_LqLd = false;
	uint state = 0;
	const real_t Ts = 0.0;
	const VectorReal& u_vector;
	VectorReal id_data;
	VectorReal iq_data;

	const uint settle_iterations_mechanical;
	const uint settle_iterations_electrical;

	static constexpr real_t u_aling = 2.0;
	static constexpr uint state_end = 6;
	static constexpr real_t settle_time_mechanical = 1.5;
	static constexpr real_t settle_time_electrical = 0.1;

};

class IdentificationPsi_f: public IdentificationGeneric {

public:
	IdentificationPsi_f(uint n_samples) :
			y(n_samples), H(n_samples, 1) {
	}
	void operator()(real_t, const Vector2Phase&);
	void estimate(MotorParametersElectrical&);
	uint get_length() const {
		return y.get_length();
	}

protected:
	static constexpr real_t omega_min_threshold = 50;
	VectorReal y;
	Matrix H;
};

class IdentificationPsi_f_improved: public IdentificationPsi_f {

public:
	IdentificationPsi_f_improved(uint n_samples, approximation_method method, real_t Ts, real_t N, const MotorParametersElectrical& parameters) :
			IdentificationPsi_f(n_samples), parameters(parameters), iq_diff(method, Ts, N) {
	}
	void operator()(real_t, const Vector2Phase&, const Vector2Phase&);

private:
	const MotorParametersElectrical& parameters;
	DiscreteDerivative iq_diff;
};

class IdentificationMechanical {

public:
	IdentificationMechanical(uint n_samples, real_t Ts) :
			Ts(Ts), moment_data(n_samples), omega_data(n_samples) {
	}

	void operator()(real_t, real_t);
	void estimate(MotorParametersMechanical&);
	inline bool is_running() const {
		return idx < moment_data.get_length();
	}
	inline uint get_idx() const {
		return idx;
	}
	void reset();
private:
	uint idx = 0;
	const real_t Ts;
	VectorReal moment_data;
	VectorReal omega_data;

};

class SpeedControl {

public:
	SpeedControl(real_t Ts, real_t P_gain, real_t I_gain, approximation_method method) :
			regulator(Ts, P_gain, I_gain, method) {
	}
	inline void setOmegaRef(real_t omega_ref) {
		this->omega_ref = omega_ref;
	}
	real_t operator()(real_t);

	DiscreteIPregulator regulator;
private:
	real_t omega_ref = 0;
};

class LoadMomentObserver {
public:
	LoadMomentObserver(const MotorParametersMechanical&, approximation_method, real_t, real_t, real_t);
	LoadMomentObserver(const MotorParametersMechanical&, approximation_method, real_t, Complex, Complex);
	real_t estimate(real_t, real_t);
private:
	real_t g1;
	real_t g2;
	const MotorParametersMechanical& parameters;

public:
	DiscreteIntegrator omegaIntegrator;
	DiscreteIntegrator loadMomentIntegrator;

};

template<typename T> void sort3(T arr[]) {
	if (arr[0] > arr[1])
		swap<T>(arr[0], arr[1]);
	if (arr[0] > arr[2])
		swap<T>(arr[0], arr[2]);
	if (arr[1] > arr[2])
		swap<T>(arr[1], arr[2]);

}

}
}

#endif /* INC_BLDC_MOTOR_CONTROL_H_ */
