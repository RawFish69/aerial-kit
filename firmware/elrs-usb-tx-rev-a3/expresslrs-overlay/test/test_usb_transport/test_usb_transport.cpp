#include <cstring>
#include <string>
#include <unity.h>

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "crsf_protocol.h"
#include "common.h"
#include "aerialkit_boot_logic.h"
#include "aerialkit_led_state_machine.h"
#include "aerialkit_serial_io_guard.h"

CRSFRouter crsfRouter;
uint32_t ChannelData[CRSF_NUM_CHANNELS];

extern char device_name[];
extern const char version[];

class CaptureEndpoint final : public CRSFEndpoint
{
public:
    CaptureEndpoint() : CRSFEndpoint(CRSF_ADDRESS_CRSF_RECEIVER) {}

    bool handleRaw(const crsf_header_t *message) override
    {
        if (message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            rcFrame = *message;
            sawRc = true;
        }
        return false;
    }

    void handleMessage(const crsf_header_t *message) override
    {
        if (message->type == CRSF_FRAMETYPE_BATTERY_SENSOR)
        {
            sawTelemetry = true;
        }
    }

    bool sawRc = false;
    bool sawTelemetry = false;
    crsf_header_t rcFrame {};
};

class CaptureConnector final : public CRSFConnector
{
public:
    void forwardMessage(const crsf_header_t *message) override
    {
        lastType = message->type;
        lastSize = message->frame_size;
        sawMessage = true;
    }

    bool sawMessage = false;
    crsf_frame_type_e lastType = CRSF_FRAMETYPE_HEARTBEAT;
    uint8_t lastSize = 0;
};

class DeviceInfoEndpoint final : public CRSFEndpoint
{
public:
    DeviceInfoEndpoint() : CRSFEndpoint(CRSF_ADDRESS_CRSF_TRANSMITTER) {}

    void handleMessage(const crsf_header_t *message) override
    {
        if (message->type == CRSF_FRAMETYPE_DEVICE_PING)
        {
            sendDeviceInformationPacket();
        }
    }
};

static CaptureEndpoint endpoint;
static DeviceInfoEndpoint deviceEndpoint;
static CaptureConnector connector;
static CRSFParser parser;
static int rcCallbacks;
static bool registered;

static void makeChannelsFrame(uint8_t frame[26])
{
    crsf_channels_s channels {};
    channels.ch0 = 172;
    channels.ch1 = 992;
    channels.ch2 = 992;
    channels.ch3 = 172;
    channels.ch4 = 992;
    channels.ch5 = 172;

    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = CRSF_FRAME_SIZE(sizeof(channels));
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memcpy(frame + 3, &channels, sizeof(channels));
    frame[25] = crsfRouter.crsf_crc.calc(frame + 2, 23);
}

static void makeBatteryFrame(uint8_t frame[12])
{
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 10;
    frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
    memset(frame + 3, 0, 8);
    frame[11] = crsfRouter.crsf_crc.calc(frame + 2, 9);
}

static void makeDevicePing(uint8_t frame[6])
{
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 4;
    frame[2] = CRSF_FRAMETYPE_DEVICE_PING;
    frame[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    frame[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    frame[5] = crsfRouter.crsf_crc.calc(frame + 2, 3);
}

void setUp()
{
    if (!registered)
    {
        crsfRouter.addEndpoint(&endpoint);
        crsfRouter.addEndpoint(&deviceEndpoint);
        crsfRouter.addConnector(&connector);
        connector.addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
        registered = true;
    }
    endpoint.sawRc = false;
    endpoint.sawTelemetry = false;
    connector.sawMessage = false;
    rcCallbacks = 0;
    parser.Reset();
}

void tearDown()
{
}

void test_tx_usb_rc_input_reaches_router_scheduler_hook()
{
    uint8_t frame[26];
    makeChannelsFrame(frame);
    parser.processBytes(&connector, frame, sizeof(frame), [](const crsf_header_t *message) {
        if (message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            rcCallbacks++;
        }
    });
    TEST_ASSERT_TRUE(endpoint.sawRc);
    TEST_ASSERT_EQUAL(1, rcCallbacks);
}

void test_tx_usb_telemetry_output_is_routable()
{
    uint8_t buffer[CRSF_MAX_PACKET_LEN] {};
    auto frame = reinterpret_cast<crsf_header_t *>(buffer);
    crsfRouter.makeLinkStatisticsPacket(frame);
    crsfRouter.deliverMessage(nullptr, frame);
    TEST_ASSERT_TRUE(connector.sawMessage);
    TEST_ASSERT_EQUAL(CRSF_FRAMETYPE_LINK_STATISTICS, connector.lastType);
}

void test_rx_usb_telemetry_input_reaches_router()
{
    uint8_t frame[12];
    makeBatteryFrame(frame);
    parser.processBytes(&connector, frame, sizeof(frame));
    TEST_ASSERT_TRUE(endpoint.sawTelemetry);
}

void test_usb_device_ping_reports_device_information()
{
    uint8_t frame[6];
    makeDevicePing(frame);
    parser.processBytes(&connector, frame, sizeof(frame));
    TEST_ASSERT_TRUE(connector.sawMessage);
    TEST_ASSERT_EQUAL(CRSF_FRAMETYPE_DEVICE_INFO, connector.lastType);
}

void test_role_heartbeat_and_mismatch_logic()
{
    const auto tx = aerialkit_heartbeat_spec(false);
    const auto rx = aerialkit_heartbeat_spec(true);
    TEST_ASSERT_EQUAL(1, tx.pulses);
    TEST_ASSERT_EQUAL(26, tx.blue);
    TEST_ASSERT_EQUAL(0, tx.green);
    TEST_ASSERT_EQUAL(2, rx.pulses);
    TEST_ASSERT_EQUAL(26, rx.green);
    TEST_ASSERT_EQUAL(0, rx.blue);
    TEST_ASSERT_EQUAL(350, tx.on_ms);
    TEST_ASSERT_EQUAL(350, rx.on_ms);
    TEST_ASSERT_TRUE(aerialkit_role_matches(false, false));
    TEST_ASSERT_TRUE(aerialkit_role_matches(true, true));
    TEST_ASSERT_FALSE(aerialkit_role_matches(true, false));
    TEST_ASSERT_FALSE(aerialkit_role_matches(false, true));
}

void test_led_state_priority_and_pattern_mapping()
{
    const AerialKitLedInputs searching = {
        false, false, false, false, false, false, false};
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedState::Searching),
        static_cast<int>(aerialkit_led_decide(searching).state));
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::FastBreath),
        static_cast<int>(aerialkit_led_decide(searching).pattern));

    AerialKitLedInputs inputs = searching;
    inputs.host_seen = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::SinglePulse),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.host_present = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::DoublePulse),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.rf_connected = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::Steady),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.bind_active = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::SlowBreath),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.boot_active = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::BootIdentity),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.radio_fault = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::RedBlink),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));

    inputs.role_or_model_mismatch = true;
    TEST_ASSERT_EQUAL(
        static_cast<int>(AerialKitLedPattern::AmberTriple),
        static_cast<int>(aerialkit_led_decide(inputs).pattern));
}

void test_led_role_colors_and_breathing_are_locked()
{
    const AerialKitLedRgb tx = aerialkit_role_color(false);
    const AerialKitLedRgb rx = aerialkit_role_color(true);
    TEST_ASSERT_EQUAL(0, tx.red);
    TEST_ASSERT_EQUAL(0, tx.green);
    TEST_ASSERT_EQUAL(255, tx.blue);
    TEST_ASSERT_EQUAL(0, rx.red);
    TEST_ASSERT_EQUAL(255, rx.green);
    TEST_ASSERT_EQUAL(0, rx.blue);

    const uint8_t slow_min = aerialkit_breath_level(0, 3000);
    const uint8_t slow_max = aerialkit_breath_level(1500, 3000);
    TEST_ASSERT_EQUAL(AERIALKIT_LED_RAW_MIN, slow_min);
    TEST_ASSERT_EQUAL(AERIALKIT_LED_RAW_MAX, slow_max);
    TEST_ASSERT_EQUAL(
        AERIALKIT_LED_RAW_MIN, aerialkit_breath_level(0, 1000));
    TEST_ASSERT_EQUAL(
        AERIALKIT_LED_RAW_MAX, aerialkit_breath_level(500, 1000));

    uint8_t previous_tx = AERIALKIT_LED_RAW_MIN;
    uint8_t previous_rx = AERIALKIT_LED_RAW_MIN;
    for (uint32_t elapsed = 0; elapsed < 6000; elapsed += 20)
    {
        const AerialKitLedRgb tx_frame =
            aerialkit_led_frame(AerialKitLedPattern::SlowBreath, elapsed, false);
        const AerialKitLedRgb rx_frame =
            aerialkit_led_frame(AerialKitLedPattern::FastBreath, elapsed, true);
        TEST_ASSERT_EQUAL(0, tx_frame.red);
        TEST_ASSERT_EQUAL(0, tx_frame.green);
        TEST_ASSERT_GREATER_OR_EQUAL(AERIALKIT_LED_RAW_MIN, tx_frame.blue);
        TEST_ASSERT_EQUAL(0, rx_frame.red);
        TEST_ASSERT_EQUAL(0, rx_frame.blue);
        TEST_ASSERT_GREATER_OR_EQUAL(AERIALKIT_LED_RAW_MIN, rx_frame.green);
        const uint8_t tx_delta =
            tx_frame.blue > previous_tx
                ? tx_frame.blue - previous_tx
                : previous_tx - tx_frame.blue;
        const uint8_t rx_delta =
            rx_frame.green > previous_rx
                ? rx_frame.green - previous_rx
                : previous_rx - rx_frame.green;
        TEST_ASSERT_LESS_OR_EQUAL(20, tx_delta);
        TEST_ASSERT_LESS_OR_EQUAL(20, rx_delta);
        previous_tx = tx_frame.blue;
        previous_rx = rx_frame.green;
    }
}

void test_led_service_patterns_use_required_timings()
{
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::DoublePulse, 50, false));
    TEST_ASSERT_FALSE(
        aerialkit_led_on(AerialKitLedPattern::DoublePulse, 150, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::DoublePulse, 250, false));
    TEST_ASSERT_FALSE(
        aerialkit_led_on(AerialKitLedPattern::DoublePulse, 500, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::SinglePulse, 50, false));
    TEST_ASSERT_FALSE(
        aerialkit_led_on(AerialKitLedPattern::SinglePulse, 150, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::RedBlink, 50, false));
    TEST_ASSERT_FALSE(
        aerialkit_led_on(AerialKitLedPattern::RedBlink, 150, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::AmberTriple, 50, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::AmberTriple, 250, false));
    TEST_ASSERT_TRUE(
        aerialkit_led_on(AerialKitLedPattern::AmberTriple, 450, false));
    TEST_ASSERT_FALSE(
        aerialkit_led_on(AerialKitLedPattern::AmberTriple, 700, false));

    const AerialKitLedRgb fault =
        aerialkit_led_frame(AerialKitLedPattern::RedBlink, 50, false);
    const AerialKitLedRgb mismatch =
        aerialkit_led_frame(AerialKitLedPattern::AmberTriple, 50, false);
    TEST_ASSERT_EQUAL(255, fault.red);
    TEST_ASSERT_EQUAL(0, fault.green);
    TEST_ASSERT_EQUAL(0, fault.blue);
    TEST_ASSERT_EQUAL(255, mismatch.red);
    TEST_ASSERT_TRUE(mismatch.green > 0);
    TEST_ASSERT_EQUAL(0, mismatch.blue);
}

void test_null_serial_io_is_rejected()
{
    void *instance = nullptr;
    void **gate = nullptr;
    void **missing_instance = &instance;
    TEST_ASSERT_FALSE(aerialkit_serial_io_ready(gate));
    TEST_ASSERT_FALSE(aerialkit_serial_io_ready(missing_instance));
    instance = &instance;
    TEST_ASSERT_TRUE(aerialkit_serial_io_ready(missing_instance));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_tx_usb_rc_input_reaches_router_scheduler_hook);
    RUN_TEST(test_tx_usb_telemetry_output_is_routable);
    RUN_TEST(test_rx_usb_telemetry_input_reaches_router);
    RUN_TEST(test_usb_device_ping_reports_device_information);
    RUN_TEST(test_role_heartbeat_and_mismatch_logic);
    RUN_TEST(test_led_state_priority_and_pattern_mapping);
    RUN_TEST(test_led_role_colors_and_breathing_are_locked);
    RUN_TEST(test_led_service_patterns_use_required_timings);
    RUN_TEST(test_null_serial_io_is_rejected);
    return UNITY_END();
}
