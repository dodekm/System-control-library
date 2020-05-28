#ifndef MODEL_H_
#define MODEL_H_

#include "common_def.h"
#include "systems.h"
#include "continuous_systems.h"

namespace SystemControl {

class ContinuousSubsystem {
protected:
	ContinuousSubsystem(SubSystem&);
	uint add_continuous_systems_to_list(System*, uint);
	SubSystem& subsystem;
	List<ContinuousSystem*> continuous_systems_list;
};

class ModelContinuous: public ContinuousSubsystem, public ContinuousSystemInterface {
public:
	ModelContinuous(SubSystem&);
private:
	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);

};

}

#endif /* MODEL_H_ */
