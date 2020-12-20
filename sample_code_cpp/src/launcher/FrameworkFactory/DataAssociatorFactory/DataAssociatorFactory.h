#ifndef LAUNCHER_DATAASSOCIATORFACTORY_H_
#define LAUNCHER_DATAASSOCIATORFACTORY_H_


#include <iostream>
#include <map>
#include <string>

#include "DataAssociator.h"
#include "DataAssociator_LinearSearch.h"
#include "DataAssociator_GridTable.h"


namespace sample_slam {

enum class DataAssociatorType {
  LINEAR_SEARCH = 0,
  GRID_TABLE = 1,
};


class DataAssociatorFactory {
 public:
  DataAssociatorFactory() {}

  ~DataAssociatorFactory() {}

  static DataAssociator* ProduceDataAssociatorPtr(const std::string& chars) {
    std::map<DataAssociatorType, std::string> data_associator_type = {
      { DataAssociatorType::LINEAR_SEARCH, "linear_search" },
      { DataAssociatorType::GRID_TABLE, "grid_table" },
    };

    if (chars == data_associator_type[DataAssociatorType::LINEAR_SEARCH]) {
      return new DataAssociator_LinearSearch;
    } else if (chars == data_associator_type[DataAssociatorType::GRID_TABLE]) {
      return new DataAssociator_GridTable;
    } else {
      std::cerr << "[DataAssociatorFactory::ProduceDataAssociatorPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }

  static DataAssociator* ProduceDataAssociatorPtr(const DataAssociatorType& enumerate) {
    if (enumerate == DataAssociatorType::LINEAR_SEARCH) {
      return new DataAssociator_LinearSearch;
    } else if (enumerate == DataAssociatorType::GRID_TABLE) {
      return new DataAssociator_GridTable;
    } else {
      std::cerr << "[DataAssociatorFactory::ProduceDataAssociatorPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_DATAASSOCIATORFACTORY_H_