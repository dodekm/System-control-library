#include "bldc_motor_control.h"

using namespace SystemControl;
using namespace MotorControl;

static Transform::SinLookup<256> SinLUT;

real_t Transform::sin(real_t fi) {
	return SinLUT(fi);
}

real_t Transform::cos(real_t fi) {
	return SinLUT(fi + half_PI);
}

void OpenLoopControlGeneric::setStates(const PositionAndSpeed& phi_omega) {
	integrator_omega.states_set(0, phi_omega.omega);
	integrator_phi.states_set(phi_omega.omega, phi_omega.phi);
}

PositionAndSpeed OpenLoopControlGeneric::operator()() {

	real_t omega_error = omegaRef - getOmega();
	if (abs(omega_error) > omega_error_threshold) {
		integrator_omega.input_port_real_value(0) = dOmega * SIGN(omega_error);
		integrator_omega.step();
	}
	real_t omega = getOmega();
	integrator_phi.input_port_real_value(0) = omega;
	integrator_phi.step();
	real_t phi = getPhi();
	return PositionAndSpeed(phi, omega);
}

Vector2Phase OpenLoopControlIF::operator()(const Vector2Phase& iAB) {

	OpenLoopControlGeneric::operator()();
	real_t phi = getPhi();
	iReg.input_port_real_value(0) = iRef - iAB.transformABtoDQ(phi).x;
	iReg.step();
	return Vector2Phase(iReg.output_port_real_value(0), 0).transformDQtoAB(phi);
}

Vector2Phase OpenLoopControlUF::operator()(const Vector2Phase& iAB) {
	(void) iAB;
	OpenLoopControlGeneric::operator()();
	return Vector2Phase(u_per_omega * abs(getOmega()), 0).transformDQtoAB(getPhi());
}

Vector3Phase CurrentControl::operator()(const Vector3Phase& current) {

	Vector2Phase iAB = Transform::Vector3PhaseSymmetricToVector2Phase(current);
	BEMF = BEMF_observer.estimate(uAB, iAB, phi_omega.omega);
	phi_omega = positionFeedback.getPositionAndSpeed(BEMF);

	if (closed_loop) {
		iDQ = iAB.transformABtoDQ(phi_omega.phi);
		Vector2Phase error(iDQRef - iDQ);
		idReg.input_port_real_value(0) = error.x;
		iqReg.input_port_real_value(0) = error.y;
		idReg.input_port_real_value(1) = iDQ.x;
		iqReg.input_port_real_value(1) = iDQ.y;
	} else {
		open_loop_control.set_omegaRef(open_loop_omegaRef * SIGN(iDQRef.y));
		open_loop_control();
		iDQ = iAB.transformABtoDQ(open_loop_control.getPhi());
		idReg.input_port_real_value(0) = 0;
		iqReg.input_port_real_value(0) = open_loop_iQRef - iDQ.y;
		idReg.input_port_real_value(1) = 0;
		iqReg.input_port_real_value(1) = iDQ.y;
	}

	idReg.step();
	iqReg.step();

	Vector2Phase uDQ(idReg.output_port_real_value(0), iqReg.output_port_real_value(0));
	if (closed_loop) {
		if (decoupling_enable)
			uDQ += Vector2Phase(-phi_omega.omega * iDQ.y * parameters.Lq, phi_omega.omega * iDQ.x * parameters.Ld);
		uAB = uDQ.transformDQtoAB(phi_omega.phi);
		if (abs(phi_omega.omega) < omega_threshold_low && get_BEMF_magnitude() < BEMF_mag_threshold_low) {
			open_loop_control.setStates(phi_omega);
			idReg.integrator.states_set(0);
			iqReg.integrator.states_set(0);
			closed_loop = false;
		}

	} else {
		uAB = uDQ.transformDQtoAB(open_loop_control.getPhi());
		if (abs(positionFeedback.get_error()) < phi_error_threshold_low && abs(open_loop_control.getOmega()) > omega_threshold_high && abs(phi_omega.omega) > omega_threshold_high && get_BEMF_magnitude() > BEMF_mag_threshold_high) {
			closed_loop = true;
		}
	}
	return Transform::Vector2PhaseToVector3Phase(uAB);
}

void VoltageInverterSVPWM::setVoltage(const Vector3Phase& u) {
	using namespace Operators;
	Vector3Phase u_norm=Vector3Phase(u.a/Ucc,u.b/Ucc,u.c/Ucc);
	real_t uN = (u_norm.max()+u_norm.min()-1.0)/2.0;
	setDuty(Vector3Phase(saturate((u_norm.a - uN), 1.0, 0), saturate((u_norm.b - uN), 1.0, 0), saturate((u_norm.c - uN), 1.0, 0)));
}

void VoltageInverterSVPWMmin::setVoltage(const Vector3Phase& u) {
	using namespace Operators;
	Vector3Phase u_norm=Vector3Phase(u.a/Ucc,u.b/Ucc,u.c/Ucc);
	real_t uN = u_norm.min();
	setDuty(Vector3Phase(saturate((u_norm.a - uN), 1.0, 0), saturate((u_norm.b - uN), 1.0, 0), saturate((u_norm.c - uN), 1.0, 0)));
}

BEMFObserverLuenberg::BEMFObserverLuenberg(const MotorParametersElectrical& parameters, approximation_method method, real_t Ts, real_t g1, real_t g2) :
		g1(g1), g2(g2), parameters(parameters), iAlfaIntegrator(method, Ts), iBetaIntegrator(method, Ts), eAlfaIntegrator(method, Ts), eBetaIntegrator(method, Ts) {
}

BEMFObserverLuenberg::BEMFObserverLuenberg(const MotorParametersElectrical& parameters, approximation_method method, real_t Ts, Complex lambda_1, Complex lambda_2) :
		BEMFObserverLuenberg(parameters, method, Ts, -lambda_1.get_real() - lambda_2.get_real() - parameters.Rs / parameters.Ld, -(lambda_1 * lambda_2).get_real() * parameters.Ld) {

}

Vector2Phase BEMFObserverLuenberg::estimate(const Vector2Phase& uS, const Vector2Phase& iS, real_t omega) {

	real_t iAlfa = iAlfaIntegrator.output_port_real_value(0);
	real_t iBeta = iBetaIntegrator.output_port_real_value(0);

	real_t eAlfa = eAlfaIntegrator.output_port_real_value(0);
	real_t eBeta = eBetaIntegrator.output_port_real_value(0);

	real_t error_iAlfa = iS.x - iAlfa;
	real_t error_iBeta = iS.y - iBeta;

	eAlfaIntegrator.input_port_real_value(0) = error_iAlfa * g2;
	eBetaIntegrator.input_port_real_value(0) = error_iBeta * g2;

	real_t Ld_minus_LQ_mul_omega = omega = (parameters.Ld - parameters.Lq) * omega;

	iAlfaIntegrator.input_port_real_value(0) = (uS.x - iAlfa * parameters.Rs - eAlfa - iS.y * Ld_minus_LQ_mul_omega) / parameters.Ld + error_iAlfa * g1;
	iBetaIntegrator.input_port_real_value(0) = (uS.y - iBeta * parameters.Rs - eBeta + iS.x * Ld_minus_LQ_mul_omega) / parameters.Ld + error_iBeta * g1;

	eAlfaIntegrator.step();
	eBetaIntegrator.step();

	iAlfaIntegrator.step();
	iBetaIntegrator.step();

	return Vector2Phase(eAlfaIntegrator.output_port_real_value(0), eBetaIntegrator.output_port_real_value(0));
}

real_t BEMF_PositionFeedbackPLLStandard::calculateError(const Vector2Phase& e) {
	return -e.x * Transform::cos(phi_omega.phi) - e.y * Transform::sin(phi_omega.phi);
}

real_t BEMF_PositionFeedbackPLLImproved::calculateError(const Vector2Phase& e) {
	return -e.x * e.y * Transform::cos(2.0 * phi_omega.phi) + 0.5 * (POW2(e.x) - POW2(e.y)) * Transform::sin(2.0 * phi_omega.phi);
}

PositionAndSpeed BEMFPositionFeedbackPLLGeneric::getPositionAndSpeed(const Vector2Phase& e) {

	error = calculateError(e.normalize_fast());
	PI_regulator.input_port_real_value(0) = error;
	PI_regulator.step();
	phi_omega.omega = PI_regulator.output_port_real_value(0);
	integrator.input_port_real_value(0) = phi_omega.omega;
	integrator.step();
	phi_omega.phi = integrator.output_port_real_value(0);
	phi_omega.n_revs += integrator.getIntegerPart();
	return phi_omega;
}

void IdentificationRsLdLq::reset() {
	state = 0;
	idx = 0;
	id_data.set_all(0);
	iq_data.set_all(0);
}

Vector3Phase IdentificationRsLdLq::operator()(const Vector3Phase& current) {

	Vector2Phase u(0, 0);
	switch (state) {
	case 0: {
		u.x = u_aling;
		if (idx >= settle_iterations_mechanical) {
			idx = 0;
			state++;
		} else
			idx++;
		break;
	}
	case 1: {
		if (idx >= settle_iterations_electrical) {
			idx = 0;
			state++;
		} else
			idx++;
		break;
	}
	case 2: {
		u.x = u_vector[idx];
		id_data[idx] = Transform::Vector3PhaseSymmetricToVector2Phase(current).x;
		if (idx >= id_data.get_length()) {
			idx = 0;
			state++;
		} else
			idx++;
		break;
	}
	case 3: {
		if (idx >= settle_iterations_electrical) {
			idx = 0;
			state++;
			if (!different_LqLd)
				state += 2;
		} else
			idx++;
		break;
	}
	case 4: {
		u.y = u_vector[idx];
		iq_data[idx] = Transform::Vector3PhaseSymmetricToVector2Phase(current).y;
		if (idx >= iq_data.get_length()) {
			idx = 0;
			state++;
		} else
			idx++;
		break;
	}
	case 5: {
		if (idx >= settle_iterations_electrical) {
			idx = 0;
			state++;
		} else
			idx++;
		break;
	}

	default:
		break;
	}

	return Transform::Vector2PhaseToVector3Phase(u);
}

void IdentificationRsLdLq::estimate(MotorParametersElectrical& parameters) {
	if (is_running())
		return;

	Polynom num(2);
	Polynom den(2);
	{
		SystemIdentification::estimate_discrete_transfer_function(u_vector, id_data, num, den, 1, 1);
		real_t K = num.sum() / den.sum();
		real_t T = -1.0 / (SystemsConvert::discrete_root_to_continuous_root(Complex(-den[0], 0.0), Ts).get_real());
		parameters.Rs = 1.0 / K;
		parameters.Ld = T / K;
	}
	if (different_LqLd) {
		SystemIdentification::estimate_discrete_transfer_function(u_vector, iq_data, num, den, 1, 1);
		real_t K = num.sum() / den.sum();
		real_t T = -1.0 / (SystemsConvert::discrete_root_to_continuous_root(Complex(-den[0], 0.0), Ts).get_real());
		real_t Rs = 1.0 / K;
		parameters.Rs = (parameters.Rs + Rs) / 2;
		parameters.Lq = T / K;
	} else {
		parameters.Lq = parameters.Ld;
	}

}

void IdentificationPsi_f::estimate(MotorParametersElectrical& parameters) {
	if (is_running())
		return;
	VectorReal theta(1, &parameters.Psi_f);
	SystemIdentification::linear_regression(H, y, theta);
}

void IdentificationPsi_f::operator()(real_t omega, const Vector2Phase& BEMF) {
	if (!is_running())
		return;
	if (abs(omega) < omega_min_threshold)
		return;
	y[idx] = BEMF.magnitude_fast();
	H(idx, 0) = abs(omega);
	idx++;

}

void IdentificationPsi_f_improved::operator()(real_t omega, const Vector2Phase& BEMF, const Vector2Phase& iDQ) {
	if (!is_running())
		return;
	if (abs(omega) < omega_min_threshold)
		return;
	if (idx == 0)
		iq_diff.states_set(iDQ.y, 0);

	y[idx] = BEMF.magnitude_fast() - (parameters.Ld - parameters.Lq) * (omega * iDQ.x - iq_diff.step(iDQ.y));
	H(idx, 0) = abs(omega);
	idx++;

}

void IdentificationMechanical::operator()(real_t moment, real_t omega) {
	if (!is_running())
		return;
	moment_data[idx] = moment;
	omega_data[idx] = omega;
	idx++;

}
void IdentificationMechanical::estimate(MotorParametersMechanical& parameters) {
	if (is_running())
		return;
	Polynom num(2);
	Polynom den(2);

	SystemIdentification::estimate_discrete_transfer_function(moment_data, omega_data, num, den, 1, 1);
	real_t K = num.sum() / den.sum();
	real_t T = -1.0 / (SystemsConvert::discrete_root_to_continuous_root(Complex(-den[0], 0.0), Ts).get_real());
	parameters.B = 1.0 / K;
	parameters.J = T / K;

}

void IdentificationMechanical::reset() {
	idx = 0;
	moment_data.set_all(0);
	omega_data.set_all(0);
}

real_t SpeedControl::operator()(real_t omega) {
	regulator.input_port_real_value(0) = omega_ref - omega;
	regulator.input_port_real_value(1) = omega;
	regulator.step();
	return regulator.output_port_real_value(0);
}

LoadMomentObserver::LoadMomentObserver(const MotorParametersMechanical& parameters, approximation_method method, real_t Ts, real_t g1, real_t g2) :
		g1(g1), g2(g2), parameters(parameters), omegaIntegrator(method, Ts), loadMomentIntegrator(method, Ts) {
}

LoadMomentObserver::LoadMomentObserver(const MotorParametersMechanical& parameters, approximation_method method, real_t Ts, Complex lambda_1, Complex lambda_2) :
		LoadMomentObserver(parameters, method, Ts, -lambda_1.get_real() - lambda_2.get_real() - parameters.B / parameters.J, -(lambda_1 * lambda_2).get_real() * parameters.J) {
}

real_t LoadMomentObserver::estimate(real_t motor_moment, real_t omega) {

	real_t omega_est = omegaIntegrator.output_port_real_value(0);
	real_t load_moment_est = loadMomentIntegrator.output_port_real_value(0);
	real_t error = omega - omega_est;

	loadMomentIntegrator.input_port_real_value(0) = error * g2;
	omegaIntegrator.input_port_real_value(0) = (motor_moment - omega_est * parameters.B - load_moment_est) / parameters.J + error * g1;

	loadMomentIntegrator.step();
	omegaIntegrator.step();

	return loadMomentIntegrator.output_port_real_value(0);
}
