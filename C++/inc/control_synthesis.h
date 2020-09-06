#ifndef CONTROL_SYNTHESIS_H_
#define CONTROL_SYNTHESIS_H_

#include "systems.h"
#include "vector_numeric.h"
#include "polynom.h"
#include "matrix.h"

namespace SystemControl {
namespace ControlSynthesis {
Complex convert_continuous_root_to_discrete_root(Complex, real_t);
Complex convert_discrete_root_to_continuous_root(Complex, real_t);

void create_discrete_polynom_from_continuous_poles(const VectorComplex&, Polynom&, real_t);
void create_discrete_polynom_from_multiple_complex_root(Polynom&, uint, Complex);
void create_discrete_aperiodic_polynom_for_multiple_time_constant(Polynom&, uint, real_t, real_t);

void convolution_matrix(const VectorReal&, Matrix&, uint);
Matrix convolution_matrix(const VectorReal&, uint);


void RST_poleplace(const Polynom&, const Polynom&, const Polynom&, Polynom&, Polynom&, Polynom&);
void PI_poleplace(const System1stOrderParams&, const ReferencePolynom2ndOrder&, PID_regulator_params&);
void PIV_poleplace(const System1stOrderParams&, const ReferencePolynom3rdOrder&, PID_regulator_params&, PID_regulator_params&);

}
}
#endif /* CONTROL_SYNTHESIS_H_ */
