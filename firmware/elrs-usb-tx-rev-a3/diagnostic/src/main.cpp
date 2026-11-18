#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdarg.h>

namespace {

constexpr uint8_t kFemTxEnable = 6;
constexpr uint8_t kFemRxEnable = 7;
constexpr uint8_t kRoleSelect = 4;
constexpr uint8_t kUartTx = 43;
constexpr uint8_t kUartRx = 44;
constexpr uint8_t kLedData = 21;

constexpr uint8_t kRadioNss = 10;
constexpr uint8_t kRadioMosi = 11;
constexpr uint8_t kRadioSck = 12;
constexpr uint8_t kRadioMiso = 13;
constexpr uint8_t kRadioReset = 14;
constexpr uint8_t kRadioDio1 = 8;
constexpr uint8_t kRadioBusy = 9;

constexpr uint8_t kI2cSda = 34;
constexpr uint8_t kI2cScl = 48;
constexpr uint8_t kLedBrightness = 26;  // 10% of 255

HardwareSerial uart0(0);
uint16_t radioVersion = 0;
bool radioSpiOk = false;

void forceRfShutdown()
{
    pinMode(kFemTxEnable, OUTPUT);
    digitalWrite(kFemTxEnable, LOW);
    pinMode(kFemRxEnable, OUTPUT);
    digitalWrite(kFemRxEnable, LOW);
}

void report(const char *fmt, ...)
{
    char line[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    Serial.println(line);
    uart0.println(line);
}

bool waitRadioNotBusy()
{
    const uint32_t start = micros();
    while (digitalRead(kRadioBusy) == HIGH && micros() - start < 20000)
    {
        delayMicroseconds(10);
    }
    return digitalRead(kRadioBusy) == LOW;
}

uint8_t readRadioRegister(uint16_t address)
{
    if (!waitRadioNotBusy())
    {
        return 0;
    }

    const uint8_t request[5] = {
        0x19,  // SX1280 RADIO_READ_REGISTER
        static_cast<uint8_t>(address >> 8),
        static_cast<uint8_t>(address),
        0x00,
        0x00,
    };
    uint8_t response[5] = {};

    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(kRadioNss, LOW);
    delayMicroseconds(2);
    for (uint8_t i = 0; i < sizeof(request); ++i)
    {
        response[i] = SPI.transfer(request[i]);
    }
    delayMicroseconds(2);
    digitalWrite(kRadioNss, HIGH);
    SPI.endTransaction();

    return response[4];
}

void testI2c(const char *name, uint8_t sda, uint8_t scl)
{
    Wire.begin(sda, scl);
    Wire.setClock(100000);
    uint8_t found[16] = {};
    uint8_t count = 0;

    for (uint8_t address = 1; address < 127; ++address)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0 && count < sizeof(found))
        {
            found[count++] = address;
        }
    }
    Wire.end();

    if (count == 0)
    {
        report("DIAG i2c %s SDA=%u SCL=%u: no devices", name, sda, scl);
        return;
    }

    report("DIAG i2c %s SDA=%u SCL=%u: %u device(s)", name, sda, scl, count);
    for (uint8_t i = 0; i < count; ++i)
    {
        report("DIAG i2c %s address=0x%02X", name, found[i]);
    }
}

void runHardwareDiagnostic()
{
    forceRfShutdown();

    pinMode(kRoleSelect, INPUT_PULLUP);
    pinMode(0, INPUT_PULLUP);
    pinMode(3, INPUT);
    pinMode(45, INPUT);
    pinMode(46, INPUT);
    pinMode(35, INPUT);
    pinMode(36, INPUT);
    pinMode(37, INPUT);
    pinMode(38, INPUT);

    pinMode(kRadioNss, OUTPUT);
    digitalWrite(kRadioNss, HIGH);
    pinMode(kRadioReset, OUTPUT);
    pinMode(kRadioDio1, INPUT);
    pinMode(kRadioBusy, INPUT);
    SPI.begin(kRadioSck, kRadioMiso, kRadioMosi, -1);

    report("DIAG Aerial Kit Rev A3 non-RF self-test");
    report("DIAG product=ESP32-S3-MINI-1U-N8 SX1280 RFX2401C");
    report("DIAG usb=ok uart0_tx=43 uart0_rx=44");
    report("DIAG role_gpio4=%s button_gpio0=%s",
           digitalRead(kRoleSelect) == LOW ? "low" : "high",
           digitalRead(0) == LOW ? "pressed" : "released");
    report("DIAG straps gpio3=%u gpio45=%u gpio46=%u",
           digitalRead(3), digitalRead(45), digitalRead(46));
    report("DIAG breakout gpio35=%u gpio36=%u gpio37=%u gpio38=%u",
           digitalRead(35), digitalRead(36), digitalRead(37), digitalRead(38));

    digitalWrite(kFemTxEnable, HIGH);
    report("DIAG fem txen_high readback=%u", digitalRead(kFemTxEnable));
    digitalWrite(kFemTxEnable, LOW);
    digitalWrite(kFemRxEnable, HIGH);
    report("DIAG fem rxen_high readback=%u", digitalRead(kFemRxEnable));
    digitalWrite(kFemRxEnable, LOW);
    forceRfShutdown();
    report("DIAG fem txen=%u rxen=%u final=shutdown",
           digitalRead(kFemTxEnable), digitalRead(kFemRxEnable));

    digitalWrite(kRadioReset, LOW);
    delay(10);
    digitalWrite(kRadioReset, HIGH);
    delay(30);
    const bool notBusy = waitRadioNotBusy();
    radioVersion = static_cast<uint16_t>(readRadioRegister(0x0153) << 8) |
                   readRadioRegister(0x0154);
    radioSpiOk = notBusy && radioVersion != 0x0000 && radioVersion != 0xFFFF;
    report("DIAG sx1280 spi=%s version=0x%04X busy=%u dio1=%u",
           radioSpiOk ? "ok" : "failed",
           radioVersion,
           digitalRead(kRadioBusy),
           digitalRead(kRadioDio1));

    testI2c("j3", kI2cSda, kI2cScl);

    neopixelWrite(kLedData, kLedBrightness, 0, 0);
    report("DIAG ws2812 gpio21=red at %u/255", kLedBrightness);
    forceRfShutdown();
}

}  // namespace

void setup()
{
    forceRfShutdown();
    uart0.begin(115200, SERIAL_8N1, kUartRx, kUartTx);
    Serial.begin(115200);
    delay(1000);
    runHardwareDiagnostic();
}

void loop()
{
    forceRfShutdown();

    static uint32_t lastReport = 0;
    static uint32_t lastColor = 0;
    const uint32_t now = millis();

    if (now - lastColor >= 2000)
    {
        static uint8_t phase = 0;
        const uint8_t colors[3][3] = {
            {kLedBrightness, 0, 0},
            {0, kLedBrightness, 0},
            {0, 0, kLedBrightness},
        };
        neopixelWrite(kLedData, colors[phase][0], colors[phase][1], colors[phase][2]);
        phase = (phase + 1) % 3;
        lastColor = now;
    }

    if (now - lastReport >= 5000)
    {
        report("DIAG alive uptime=%lu role_gpio4=%s radio_spi=%s version=0x%04X",
               static_cast<unsigned long>(now),
               digitalRead(kRoleSelect) == LOW ? "low" : "high",
               radioSpiOk ? "ok" : "failed",
               radioVersion);
        lastReport = now;
    }
}
