#include <cstring>
#include <string>
#include <unity.h>

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "crsf_protocol.h"

CRSFRouter crsfRouter;

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

static CaptureEndpoint endpoint;
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

void setUp()
{
    if (!registered)
    {
        crsfRouter.addEndpoint(&endpoint);
        crsfRouter.addConnector(&connector);
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

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_tx_usb_rc_input_reaches_router_scheduler_hook);
    RUN_TEST(test_tx_usb_telemetry_output_is_routable);
    RUN_TEST(test_rx_usb_telemetry_input_reaches_router);
    return UNITY_END();
}
