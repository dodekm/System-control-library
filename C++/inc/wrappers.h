#ifndef INC_GSL_WRAPPER_H_
#define INC_GSL_WRAPPER_H_

#include "common_def.h"
#include "matrix.h"
#include "vector.h"
#include "signal_process.h"

namespace SystemControl {

namespace Convert
{

exception_code gsl_error_code_to_return_code(int);
int return_code_to_gsl_error_code(exception_code);
VectorReal Matrix2VectorReal(Matrix&);
const VectorReal Matrix2VectorReal(const Matrix&);
Matrix VectorReal2rowMatrix(VectorReal&);
Matrix VectorReal2colMatrix(VectorReal&);
const Matrix VectorReal2rowMatrix(const VectorReal&);
const Matrix VectorReal2colMatrix(const VectorReal&);
}
}
#endif /* INC_GSL_WRAPPER_H_ */

