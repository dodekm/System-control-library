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

Vector2Phase OpenLoopControl::operator()(void) {
	integrator_omega.step();
	real_t omega=integrator_omega.output_port_real_value(0);
	integrator_phi.input_port_real_value(0) = omega;
	integrator_phi.step();
	real_t phi=integrator_phi.output_port_real_value(0);
	return Vector2Phase(0, uA).rotate(phi);
}
void OpenLoopControl::setStates(const PositionAndSpeed& phi_omega) {
	integrator_omega.states_set(0, phi_omega.omega);
	integrator_phi.states_set(phi_omega.omega, phi_omega.phi);
}

Vector3Phase CurrentControl::operator()(const Vector3Phase& current) {

	Vector2Phase iAB = Transform::Vector3PhaseSymmetricToVector2Phase(current);
	BEMF = observer.estimate(uAB, iAB);
	phi_omega = positionFeedback.getPositionAndSpeed(BEMF);

	if (abs(iDQRef.y) > iQ_ref_off_threshold) {
		iDQ = iAB.transformABtoDQ(phi_omega.phi);
		Vector2Phase error(iDQRef - iDQ);
		idReg.input_port_real_value(0) = error.x;
		iqReg.input_port_real_value(0) = error.y;
		idReg.input_port_real_value(1) = iDQ.x;
		iqReg.input_port_real_value(1) = iDQ.y;
		idReg.step();
		iqReg.step();
		if (closed_loop) {

			Vector2Phase uDQ(idReg.output_port_real_value(0), iqReg.output_port_real_value(0));
			if (cross_compensation_enable)
				uDQ += Vector2Phase(-phi_omega.omega * iDQ.y * parameters.Lq, phi_omega.omega * iDQ.x * parameters.Ld);

			uAB = uDQ.transformDQtoAB(phi_omega.phi);
			if (abs(phi_omega.omega) < omega_threshold_low && get_BEMF_magnitude() < BEMF_threshold_low) {
				positionFeedback.setDirection(SIGN(iDQRef.y));
				open_loop_control.setStates(phi_omega);
				open_loop_control.set_d_omega(open_loop_accel * SIGN(iDQRef.y));
				closed_loop = false;
			}

		} else {
			uAB = open_loop_control();
			if (abs(open_loop_control.getOmega()) > omega_threshold_high && abs(phi_omega.omega) > omega_threshold_high && get_BEMF_magnitude() > BEMF_threshold_high) {
				closed_loop = true;
			}

		}
	} else {
		uAB = Vector2Phase(0, 0);
	}

	return Transform::Vector2PhaseToVector3Phase(uAB);

}

void VoltageInverter::setVoltage(const Vector3Phase& u) {
	Vector3Phase uNonsymmetricPositive = u.symetricToNonSymmetricPositive();
	setDuty(Vector3Phase(CLIP_TOP(uNonsymmetricPositive.a / Ucc, 1.0), CLIP_TOP(uNonsymmetricPositive.b / Ucc, 1.0), CLIP_TOP(uNonsymmetricPositive.c / Ucc, 1.0)));
}

BEMF_Observer::BEMF_Observer(const MotorParametersElectrical& parameters, approximation_method method, real_t Ts, Complex lambda_1, Complex lambda_2) :
		parameters(parameters), iAlfaIntegrator(method, Ts), iBetaIntegrator(method, Ts), eAlfaIntegrator(method, Ts), eBetaIntegrator(method, Ts) {
	g1 = -lambda_1.get_real() - lambda_2.get_real() - parameters.Rs / parameters.Ld;
	g2 = -(lambda_1 * lambda_2).get_real() * parameters.Ld;

}

Vector2Phase BEMF_Observer::estimate(const Vector2Phase& uS, const Vector2Phase& iS) {

	real_t iAlfa = iAlfaIntegrator.output_port_real_value(0);
	real_t iBeta = iBetaIntegrator.output_port_real_value(0);

	real_t eAlfa = eAlfaIntegrator.output_port_real_value(0);
	real_t eBeta = eBetaIntegrator.output_port_real_value(0);

	real_t error_iAlfa = iS.x - iAlfa;
	real_t error_iBeta = iS.y - iBeta;

	eAlfaIntegrator.input_port_real_value(0) = error_iAlfa * g2;
	eBetaIntegrator.input_port_real_value(0) = error_iBeta * g2;

	iAlfaIntegrator.input_port_real_value(0) = (uS.x - iAlfa * parameters.Rs - eAlfa) / parameters.Ld + error_iAlfa * g1;
	iBetaIntegrator.input_port_real_value(0) = (uS.y - iBeta * parameters.Rs - eBeta) / parameters.Ld + error_iBeta * g1;

	eAlfaIntegrator.step();
	eBetaIntegrator.step();

	iAlfaIntegrator.step();
	iBetaIntegrator.step();

	return Vector2Phase(eAlfaIntegrator.output_port_real_value(0), eBetaIntegrator.output_port_real_value(0));
}

void BEMF_PositionFeedbackPLLGeneric::estimate(real_t error) {
	PI_regulator.input_port_real_value(0) = error;
	PI_regulator.step();
	phi_omega.omega = PI_regulator.output_port_real_value(0);
	integrator.input_port_real_value(0) = phi_omega.omega;
	integrator.step();
	phi_omega.phi = integrator.output_port_real_value(0);
	phi_omega.n_revs += integrator.getIntegerPart();
}

PositionAndSpeed BEMF_PositionFeedbackPLLGeneric::getPositionAndSpeed(const Vector2Phase& e) {

	real_t error = -e.x * Transform::cos(phi_omega.phi) - e.y * Transform::sin(phi_omega.phi);
	error *= (real_t) direction;
	estimate(error);
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
		u.x = u_step;
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
		u.x = u_step;
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
		u.y = u_step;
		iq_data[idx] = Transform::Vector3PhaseSymmetricToVector2Phase(current).y;
		if (idx >= id_data.get_length()) {
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

	VectorReal u_vector(id_data.get_length());
	u_vector.set_all(u_step);
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
	parameters.Psi_f = psi_f_data.mean();
}

void IdentificationPsi_f::operator()(real_t omega, const Vector2Phase& BEMF) {
	if (!is_running())
		return;
	if (abs(omega) < omega_min_threshold) {
		idx = 0;
	} else {
		real_t psi_f = BEMF.magnitude_fast() / abs(omega);
		psi_f_data[idx] = psi_f;
		idx++;
	}
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

LoadMomentObserver::LoadMomentObserver(const MotorParametersMechanical& parameters, approximation_method method, real_t Ts, Complex lambda_1, Complex lambda_2) :
		parameters(parameters), omegaIntegrator(method, Ts), loadMomentIntegrator(method, Ts) {
	g1 = -lambda_1.get_real() - lambda_2.get_real() - parameters.B / parameters.J;
	g2 = -(lambda_1 * lambda_2).get_real() * parameters.J;
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
