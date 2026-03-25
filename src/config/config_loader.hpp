#pragma once

#include "common_types.hpp"
#include "logger/logger.hpp"

#include <filesystem>
#include <optional>
#include <string>

/**
 * 将 JSON 中的 log_level 整数 (0~3) 转为 Logger 使用的枚举。
 */
LogLevel log_level_from_int(int v);

/** 与 JSON 字段一致时的默认配置（文件缺失或解析失败时可回退） */
void system_config_set_defaults(SystemConfig& cfg);

/**
 * 从项目内 `config/system.json` 解析并写入 `out`。
 * @param path  JSON 文件绝对或相对路径
 * @param err   若非空，失败时写入原因
 * @return      成功 true；失败时 `out` 仍为调用前内容，应配合 `system_config_set_defaults` 使用
 */
bool load_system_config_json(const std::filesystem::path& path, SystemConfig& out, std::string* err = nullptr);

/**
 * 解析配置路径：环境变量 `MIDDLEWARE_CONFIG` > 若干相对路径探测。
 * @return 找到的可读文件路径；否则 nullopt
 */
std::optional<std::filesystem::path> resolve_system_config_path(int argc, char** argv);
