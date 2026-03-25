#include "config_loader.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

LogLevel log_level_from_int(int v) {
    const int c = std::max(0, std::min(3, v));
    switch (c) {
    case 0: return LogLevel::DEBUG;
    case 1: return LogLevel::INFO;
    case 2: return LogLevel::WARN;
    default: return LogLevel::ERROR;
    }
}

void system_config_set_defaults(SystemConfig& cfg) {
    cfg.serial_left  = {"/dev/ttyS3", 115200, 100};
    cfg.serial_right = {"/dev/ttyS9", 115200, 100};
    cfg.trajectory   = {1.0, 2.0, 256, 10.0};
    cfg.log_dir      = "/tmp/middleware_log";
    cfg.log_level    = 0;
    cfg.watchdog_timeout_ms = 2000;
    cfg.telemetry_stale_ms  = 1500;
}

static void parse_serial(const nlohmann::json& j, SerialConfig& s) {
    s.port        = j.at("port").get<std::string>();
    s.baud_rate   = j.at("baud_rate").get<int>();
    s.timeout_ms  = j.at("timeout_ms").get<int>();
}

static void parse_trajectory(const nlohmann::json& j, TrajectoryConfig& t) {
    t.max_velocity       = j.at("max_velocity").get<double>();
    t.max_acceleration   = j.at("max_acceleration").get<double>();
    t.buffer_size        = j.at("buffer_size").get<int>();
    t.control_period_ms  = j.at("control_period_ms").get<double>();
}

bool load_system_config_json(const std::filesystem::path& path, SystemConfig& out, std::string* err) {
    std::ifstream ifs(path);
    if (!ifs) {
        if (err) {
            *err = "cannot open file: " + path.string();
        }
        return false;
    }

    nlohmann::json root;
    try {
        ifs >> root;
    } catch (const std::exception& e) {
        if (err) {
            *err = std::string("JSON parse error: ") + e.what();
        }
        return false;
    }

    SystemConfig tmp{};
    try {
        parse_serial(root.at("serial_left"), tmp.serial_left);
        parse_serial(root.at("serial_right"), tmp.serial_right);
        parse_trajectory(root.at("trajectory"), tmp.trajectory);
        tmp.log_dir = root.at("log_dir").get<std::string>();
        tmp.log_level = root.at("log_level").get<int>();
        tmp.watchdog_timeout_ms = root.at("watchdog_timeout_ms").get<int>();
        if (root.contains("telemetry_stale_ms") && !root["telemetry_stale_ms"].is_null()) {
            tmp.telemetry_stale_ms = root["telemetry_stale_ms"].get<int>();
        } else {
            tmp.telemetry_stale_ms = 1500;
        }
    } catch (const std::exception& e) {
        if (err) {
            *err = std::string("missing or invalid field: ") + e.what();
        }
        return false;
    }

    out = std::move(tmp);
    return true;
}

std::optional<std::filesystem::path> resolve_system_config_path(int argc, char** argv) {
    if (const char* env = std::getenv("MIDDLEWARE_CONFIG")) {
        std::filesystem::path p(env);
        if (std::filesystem::is_regular_file(p)) {
            return std::filesystem::weakly_canonical(p);
        }
    }

    static const char* candidates[] = {
        "config/system.json",
        "../config/system.json",
        "../../config/system.json",
    };

    for (const char* rel : candidates) {
        std::filesystem::path p(rel);
        if (std::filesystem::is_regular_file(p)) {
            return std::filesystem::weakly_canonical(std::filesystem::absolute(p));
        }
    }

    if (argc > 0 && argv[0]) {
        std::filesystem::path exe(argv[0]);
        const std::filesystem::path base = exe.parent_path().empty()
            ? std::filesystem::current_path()
            : std::filesystem::weakly_canonical(exe.parent_path());

        const std::filesystem::path rels[] = {
            base / ".." / "config" / "system.json",
            base / ".." / ".." / "config" / "system.json",
            base / "config" / "system.json",
        };
        for (const auto& p : rels) {
            if (std::filesystem::is_regular_file(p)) {
                return std::filesystem::weakly_canonical(p);
            }
        }
    }

    return std::nullopt;
}
