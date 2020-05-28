
#ifndef SYSTEMS_ANALYZE_H_
#define SYSTEMS_ANALYZE_H_

#include "systems.h"


namespace SystemControl {

namespace SystemsAnalyze
{

bool roots_stability(const VectorComplex&, System::system_time_domain);


}

namespace SystemsConvert
{

Complex continuous_root_to_discrete_root(Complex, real_t);
Complex discrete_root_to_continuous_root(Complex, real_t);


}


}




#endif /* SYSTEMS_ANALYZE_H_ */
