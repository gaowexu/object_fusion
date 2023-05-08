///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_PREDICTION_AGENT_PREDICTOR_H
#define MODULES_PREDICTION_AGENT_PREDICTOR_H

#include "modules/types/types.h"

namespace perception
{
namespace object_fusion
{

class AgentPredictor
{
  public:
    AgentPredictor();
    void Predict(const Clock::duration& duration, BEVOFStateVector& state, BEVOFStateCovarianceMatrix& covariance);

  private:
    BEVOFStateTransformMatrix GetStateTransformMatrix(const Clock::duration& duration);
    BEVOFProcessNoiseCovarianceMatrix GetProcessNoiseCovarianceMatrix(const Clock::duration& duration);
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_PREDICTION_AGENT_PREDICTOR_H
