#include <cstring>
#include <unity.h>

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "common.h"
#include "crsf_protocol.h"
#include "msptypes.h"

CRSFRouter crsfRouter;
uint32_t ChannelData[CRSF_NUM_CHANNELS];
extern char device_name[];
extern const char version[];

class TxLikeEndpoint final : public CRSFEndpoint
{
public:
    TxLikeEndpoint() : CRSFEndpoint(CRSF_ADDRESS_CRSF_TRANSMITTER) {}

    bool handleRaw(const crsf_header_t *message) override
    {
        if (message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            rcFrames++;
            return true;
        }
        return false;
    }

    void handleMessage(const crsf_header_t *message) override
    {
        const auto ext = (const crsf_ext_header_t *)message;
        if (message->type != CRSF_FRAMETYPE_MSP_REQ ||
            ext->payload[2] != MSP_ELRS_RXTX_CONFIG ||
            ext->payload[3] != (uint8_t)MSP_ELRS_RXTX_CONFIG_SUBCMD::UID)
        {
            return;
        }

        mspPacket_t response;
        response.reset();
        response.makeResponse();
        response.function = MSP_ELRS_RXTX_CONFIG;
        response.addByte((uint8_t)MSP_ELRS_RXTX_CONFIG_SUBCMD::UID);
        for (uint8_t index = 0; index < 6; index++)
        {
            response.addByte(0);
        }
        crsfRouter.AddMspMessage(&response, ext->orig_addr, getDeviceId());
    }

    int rcFrames = 0;
};

class CaptureConnector final : public CRSFConnector
{
public:
    void forwardMessage(const crsf_header_t *message) override
    {
        sawMessage = true;
        lastType = message->type;
        lastFrameSize = message->frame_size;
        if (message->type == CRSF_FRAMETYPE_MSP_RESP)
        {
            const auto ext = (const crsf_ext_header_t *)message;
            responsePayloadLength = ext->payload[1];
            responseSubcommand = ext->payload[3];
        }
    }

    bool sawMessage = false;
    crsf_frame_type_e lastType = CRSF_FRAMETYPE_HEARTBEAT;
    uint8_t lastFrameSize = 0;
    uint8_t responsePayloadLength = 0;
    uint8_t responseSubcommand = 0xff;
};

static TxLikeEndpoint endpoint;
static CaptureConnector connector;
static CRSFParser parser;
static bool registered;

static void makeRcFrame(uint8_t frame[26])
{
    crsf_channels_s channels {};
    channels.ch0 = CRSF_CHANNEL_VALUE_STD_MIN;
    channels.ch1 = CRSF_CHANNEL_VALUE_MID;
    channels.ch2 = CRSF_CHANNEL_VALUE_STD_MIN;
    channels.ch3 = CRSF_CHANNEL_VALUE_MID;
    channels.ch4 = CRSF_CHANNEL_VALUE_MID;
    channels.ch5 = CRSF_CHANNEL_VALUE_STD_MIN;
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = CRSF_FRAME_SIZE(sizeof(channels));
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memcpy(frame + 3, &channels, sizeof(channels));
    frame[25] = crsfRouter.crsf_crc.calc(frame + 2, 23);
}

void setUp()
{
    if (!registered)
    {
        crsfRouter.addEndpoint(&endpoint);
        crsfRouter.addConnector(&connector);
        registered = true;
    }
    endpoint.rcFrames = 0;
    connector.sawMessage = false;
    connector.lastType = CRSF_FRAMETYPE_HEARTBEAT;
    connector.responsePayloadLength = 0;
    connector.responseSubcommand = 0xff;
    parser.Reset();
}

void tearDown() {}

void test_tx_uid_response_is_routed_while_rc_path_is_active()
{
    const uint8_t uidRequest[] = {
        0xC8, 0x0C, 0x7A, 0xEE, 0xEF,
        0x50, 0x00, 0x2D, 0x00, 0x01, 0x00, 0x00, 0x1A, 0x14,
    };
    uint8_t rc[26];
    makeRcFrame(rc);

    parser.processBytes(&connector, rc, sizeof(rc));
    parser.processBytes(&connector, uidRequest, sizeof(uidRequest));
    parser.processBytes(&connector, rc, sizeof(rc));

    TEST_ASSERT_EQUAL(2, endpoint.rcFrames);
    TEST_ASSERT_TRUE(connector.sawMessage);
    TEST_ASSERT_EQUAL(CRSF_FRAMETYPE_MSP_RESP, connector.lastType);
    TEST_ASSERT_EQUAL(7, connector.responsePayloadLength);
    TEST_ASSERT_EQUAL((uint8_t)MSP_ELRS_RXTX_CONFIG_SUBCMD::UID,
                      connector.responseSubcommand);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_tx_uid_response_is_routed_while_rc_path_is_active);
    return UNITY_END();
}
