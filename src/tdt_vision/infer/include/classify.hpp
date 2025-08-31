#ifndef __CLASSIFY_HPP__
#define __CLASSIFY_HPP__

#include <future>
#include <memory>
#include <string>
#include <vector>
#include "BaseInfer.hpp"
#include "InferTool.hpp"

namespace classify {

using namespace tdt_radar;

enum class Type : int { densenet121 = 0 };

std::shared_ptr<Infer<int>> load(const std::string& engine_file, Type type);

const char* type_name(Type type);

}  // namespace classify

#endif  // __CLASSIFY_HPP__