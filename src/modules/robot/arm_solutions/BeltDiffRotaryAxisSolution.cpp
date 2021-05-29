#include "BeltDiffRotaryAxisSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>

void BeltDiffRoraryAxisSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm) const
{
    actuator_mm[DELTA_STEPPER] = cartesian_mm[A_AXIS] + cartesian_mm[C_AXIS];
    actuator_mm[EPSILON_STEPPER] = cartesian_mm[B_AXIS];
    actuator_mm[ZETA_STEPPER] = cartesian_mm[A_AXIS] - cartesian_mm[C_AXIS];
}

void BeltDiffRoraryAxisSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[]) const
{
    cartesian_mm[A_AXIS] = 0.5F * (actuator_mm[DELTA_STEPPER] + actuator_mm[ZETA_STEPPER]);
    cartesian_mm[B_AXIS] = actuator_mm[EPSILON_STEPPER];
    cartesian_mm[C_AXIS] = 0.5F * (actuator_mm[DELTA_STEPPER] - actuator_mm[ZETA_STEPPER]);
}