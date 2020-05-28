/**
 * @file system_control.h
 * @brief System control library common modules include.
 */

/** @mainpage About this library
 *	This library applies the group of algorithms and functions from the field of numerical system control and its synthesis as well as systems identification.
 *	The functionality we have mentioned is contained in a C language library and it's application interface.
 *	All implemented systems and object are represented as structures. Anonymous structures are used for inheritance concept.
 */

/** @page Building
 *  To build this library you need add compiler flag -fms-extensions -std=c11 for anonymous structures support.
 *  Library needs C standard library functions from stddef.h, stdlib.h, stdint.h and string.h and dynamic allocation as well.
 */

/** @page Configuration
 *  Library uses configuration switches (defines) to optimize and customize resulting binary.
 *  These configuration defines are located in @ref common_def.h file.
 *
 *	@section data_type Floating point data type
 *	All numeric algorithms implemented in this library use floating point arithmetic.
 *	Common real number data type is defined @ref real_t and thus can be selected as float or double
 *	To select desired data type - define macro @ref NUMERIC_FLOATING_USE as @ref _FLOAT or @ref _DOUBLE
 *
 *	@warning In case of using GSL library, the defined real number data type is fixed as double, because GSL function use double as native data type.
 *
 *  @section GSL_support GSL support
 *  > The GNU Scientific Library (or GSL) is a software library for numerical computations in applied mathematics and science.
 *
 *  Some functionality in this library requires GSL library to be linked into project.
 *  To allow and enable these functions, define macro @ref USE_GSL
 *  @warning Without defining @ref USE_GSL and linking GSL library, some functions become un-available
 *
 *	@defgroup examples Examples
 *
 *
 */

#ifndef INC_SYSTEM_CONTROL_H_
#define INC_SYSTEM_CONTROL_H_

#include "common_def.h"
#include "vector.h"
#include "matrix.h"
#include "complex.h"
#include "gsl_wrapper.h"
#include "statistic.h"
#include "signal_process.h"
#include "polynom.h"
#include "systems.h"
#include "signal_sources.h"
#include "signal_sinks.h"
#include "static_systems.h"
#include "discrete_systems.h"
#include "continuous_systems.h"
#include "ode_solver.h"
#include "model.h"

#include "systems_analyze.h"
#include "control_synthesis.h"
#include "system_identification.h"
#include "adaptive_control.h"
#include "io_interface.h"
#include "io_sources_sinks.h"

#endif /* INC_SYSTEM_CONTROL_H_ */
