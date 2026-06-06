#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace
{
    std::string load_servo_source()
    {
        const auto test_dir = std::filesystem::path(__FILE__).parent_path();
        const auto source = test_dir / "../../../../../modules/libstp-platforms/wombat/servo/src/Servo.cpp";

        std::ifstream in(source);
        EXPECT_TRUE(in.is_open()) << "failed to open " << source;
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }

    std::string extract_function_body(const std::string& source, const std::string& signature)
    {
        const auto sig_pos = source.find(signature);
        EXPECT_NE(sig_pos, std::string::npos) << "signature not found: " << signature;

        const auto brace_open = source.find('{', sig_pos);
        EXPECT_NE(brace_open, std::string::npos) << "missing opening brace for " << signature;

        int depth = 0;
        for (std::size_t i = brace_open; i < source.size(); ++i)
        {
            if (source[i] == '{') ++depth;
            if (source[i] == '}')
            {
                --depth;
                if (depth == 0)
                {
                    return source.substr(brace_open + 1, i - brace_open - 1);
                }
            }
        }

        ADD_FAILURE() << "missing closing brace for " << signature;
        return {};
    }
}

TEST(TransportWriter, EnableOnlyPublishesServoMode)
{
    const auto source = load_servo_source();
    const auto body = extract_function_body(source, "void libstp::hal::servo::Servo::enable() const");

    EXPECT_NE(body.find("setServoMode(port, 2)"), std::string::npos);
    EXPECT_EQ(body.find("setServo(port"), std::string::npos);
}

TEST(TransportWriter, SetPositionOnlyPublishesServoPosition)
{
    const auto source = load_servo_source();
    const auto body = extract_function_body(source, "void libstp::hal::servo::Servo::setPosition(const float position)");

    EXPECT_NE(body.find("storedPosition = position"), std::string::npos);
    EXPECT_NE(body.find("setServo(port, position)"), std::string::npos);
    EXPECT_EQ(body.find("setServoMode("), std::string::npos);
}
