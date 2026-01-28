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

void CalibrationStore::storeReading(float black_tresh, float white_tresh, CalibrationType type) const {
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

    std::string typeKey;
    switch (type) {
        case IR_SENSOR:
            typeKey = "ir-calibration";
            break;
        default:
            typeKey = "unknown-calibration";
            break;
    }

    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        typeNode = calibrationNode[typeKey] = YAML::Node(YAML::NodeType::Map);
    }

    typeNode["white_tresh"] = white_tresh;
    typeNode["black_tresh"] = black_tresh;

    std::ofstream out(FILE_PATH);
    out << root;
}

bool CalibrationStore::hasReadings(CalibrationType type) const {
    if (!doesFileExist() || std::filesystem::file_size(FILE_PATH) == 0) {
        return false;
    }

    YAML::Node root = YAML::LoadFile(FILE_PATH);
    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        return false;
    }

    std::string typeKey;
    switch (type) {
        case IR_SENSOR:
            typeKey = "ir-calibration";
            break;
        default:
            return false;
    }

    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        return false;
    }

    return typeNode["white_tresh"] && typeNode["black_tresh"];
}


std::vector<float> CalibrationStore::getReadings(CalibrationType type) const {
    if (!doesFileExist() || std::filesystem::file_size(FILE_PATH) == 0) {
        return {0.0, 0.0};
    }

    YAML::Node root = YAML::LoadFile(FILE_PATH);
    YAML::Node calibrationNode = root["root"];
    if (!calibrationNode) {
        return {0.0, 0.0};
    }

    std::string typeKey;
    switch (type) {
        case IR_SENSOR:
            typeKey = "ir-calibration";
            break;
        default:
            return {0.0, 0.0};
    }

    YAML::Node typeNode = calibrationNode[typeKey];
    if (!typeNode) {
        return {0.0, 0.0};
    }
    LIBSTP_LOG_INFO(typeNode["white_tresh"].Scalar());
    auto white_tresh = typeNode["white_tresh"].as<float>();
    auto black_tresh = typeNode["black_tresh"].as<float>();

    return {white_tresh, black_tresh};
}
