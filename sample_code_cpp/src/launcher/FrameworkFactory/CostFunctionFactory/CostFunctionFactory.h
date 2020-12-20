#ifndef LAUNCHER_COSTFUNCTIONFACTORY_H_
#define LAUNCHER_COSTFUNCTIONFACTORY_H_


#include <iostream>
#include <map>
#include <string>

#include "CostFunction.h"
#include "CostFunction_EuclideanDistance.h"
#include "CostFunction_PerpendicularDistance.h"


namespace sample_slam {

enum class CostFunctionType {
  EUCLIDEAN_DISTANCE = 0,
  PERPENDICULAR_DISTANCE = 1,
};


class CostFunctionFactory {
 public:
  CostFunctionFactory() {}

  ~CostFunctionFactory() {}

  static CostFunction* ProduceCostFunctionPtr(const std::string& chars) {
    std::map<CostFunctionType, std::string> cost_function_type = {
      { CostFunctionType::EUCLIDEAN_DISTANCE, "euclidean_distance" },
      { CostFunctionType::PERPENDICULAR_DISTANCE, "perpendicular_distance" },
    };

    if (chars == cost_function_type[CostFunctionType::EUCLIDEAN_DISTANCE]) {
      return new CostFunction_EuclideanDistance;
    } else if (chars == cost_function_type[CostFunctionType::PERPENDICULAR_DISTANCE]) {
      return new CostFunction_PerpendicularDistance;
    } else {
      std::cerr << "[CostFunctionFactory::ProduceCostFunctionPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }

  static CostFunction* ProduceCostFunctionPtr(const CostFunctionType& enumerate) {
    if (enumerate == CostFunctionType::EUCLIDEAN_DISTANCE) {
      return new CostFunction_EuclideanDistance;
    } else if (enumerate == CostFunctionType::PERPENDICULAR_DISTANCE) {
      return new CostFunction_PerpendicularDistance;
    } else {
      std::cerr << "[CostFunctionFactory::ProduceCostFunctionPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_COSTFUNCTIONFACTORY_H_