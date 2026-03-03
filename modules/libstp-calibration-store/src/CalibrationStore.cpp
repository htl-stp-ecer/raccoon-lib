#include "CalibrationStore.hpp"
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "foundation/logging.hpp"

using namespace libstp::calibration_store;

CalibrationStore::~CalibrationStore() = default;

CalibrationStore & CalibrationStore::instance() {
    static CalibrationStore instance;
    return instance;
}

bool CalibrationStore::doesFileExist() const {
    return std::filesystem::exists(FILE_PATH);
}

static std::string typeKeyFor(CalibrationType type) {
    switch (type) {
        case IR_SENSOR: return "ir-calibration";
        default:        return "unknown-calibration";
    }
}

/// If the type node contains flat white_tresh/black_tresh keys (old format),
/// migrate them into a "default" sub-node in place.
static void migrateFlat(YAML::Node typeNode) {
    if (typeNode["white_tresh"] && typeNode["black_tresh"] &&
        typeNode["white_tresh"].IsScalar()) {
        float w = typeNode["white_tresh"].as<float>();
        float b = typeNode["black_tresh"].as<float>();
        typeNode.remove("white_tresh");
        typeNode.remove("black_tresh");
        typeNode["default"]["white_tresh"] = w;
        typeNode["default"]["black_tresh"] = b;
    }
}

void CalibrationStore::storeReading(float black_tresh, float white_tresh,
                                     CalibrationType type,
                                     const std::string& set_name) const {
    if (!doesFileExist()) {
        std::ofstream(FILE_PATH).close();
    }

    YAML::Node root;
    if (std::filesystem::file_size(FILE_PATH) > 0) {
        root = YAML::LoadFile(FILE_PATH);
    }

    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        calibrationNode = root["root"] = YAML::Node(YAML::NodeType::Map);
    }

    std::string typeKey = typeKeyFor(type);

    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        typeNode = calibrationNode[typeKey] = YAML::Node(YAML::NodeType::Map);
    }

    migrateFlat(typeNode);

    YAML::Node setNode = typeNode[set_name];
    if (!setNode) {
        setNode = typeNode[set_name] = YAML::Node(YAML::NodeType::Map);
    }

    setNode["white_tresh"] = white_tresh;
    setNode["black_tresh"] = black_tresh;

    std::ofstream out(FILE_PATH);
    out << root;
}

bool CalibrationStore::hasReadings(CalibrationType type,
                                    const std::string& set_name) const {
    if (!doesFileExist() || std::filesystem::file_size(FILE_PATH) == 0) {
        return false;
    }

    YAML::Node root = YAML::LoadFile(FILE_PATH);
    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        return false;
    }

    std::string typeKey = typeKeyFor(type);
    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        return false;
    }

    migrateFlat(typeNode);

    YAML::Node setNode = typeNode[set_name];
    if (!setNode) {
        return false;
    }

    return setNode["white_tresh"] && setNode["black_tresh"];
}


std::vector<float> CalibrationStore::getReadings(CalibrationType type,
                                                  const std::string& set_name) const {
    if (!doesFileExist() || std::filesystem::file_size(FILE_PATH) == 0) {
        return {0.0, 0.0};
    }

    YAML::Node root = YAML::LoadFile(FILE_PATH);
    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        return {0.0, 0.0};
    }

    std::string typeKey = typeKeyFor(type);
    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        return {0.0, 0.0};
    }

    migrateFlat(typeNode);

    YAML::Node setNode = typeNode[set_name];
    if (!setNode) {
        return {0.0, 0.0};
    }

    LIBSTP_LOG_INFO(setNode["white_tresh"].Scalar());
    auto white_tresh = setNode["white_tresh"].as<float>();
    auto black_tresh = setNode["black_tresh"].as<float>();

    return {white_tresh, black_tresh};
}

std::vector<std::string> CalibrationStore::getSetNames(CalibrationType type) const {
    std::vector<std::string> names;

    if (!doesFileExist() || std::filesystem::file_size(FILE_PATH) == 0) {
        return names;
    }

    YAML::Node root = YAML::LoadFile(FILE_PATH);
    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        return names;
    }

    std::string typeKey = typeKeyFor(type);
    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        return names;
    }

    migrateFlat(typeNode);

    for (auto it = typeNode.begin(); it != typeNode.end(); ++it) {
        if (it->second.IsMap()) {
            names.push_back(it->first.as<std::string>());
        }
    }

    return names;
}
