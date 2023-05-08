///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_FUSION_AGENT_UPDATER_H
#define MODULES_FUSION_AGENT_UPDATER_H

#include "modules/types/types.h"

namespace perception
{
namespace object_fusion
{

class AgentUpdater
{
  public:
    AgentUpdater();
    void Fuse(const BEVMeasurement& measurement, const BEVSensorType& sensor_type, BEVOFTrackObject& track);

    float FuseExistence(const float a, const float b);
    BEVDetClassificationType FuseClassificationVector(const BEVDetClassificationType& a,
                                                      const BEVDetClassificationType& b);

  private:
    BEVMeasurementTransformMatrix H_;
    BEVMeasurementCovarianceMatrix R_;
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_FUSION_AGENT_UPDATER_H
