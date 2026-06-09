#include <Arduino.h>

#define FINGER_BAUDRATE 921600
#define FINGER_RX_PIN 3
#define FINGER_TX_PIN 1

#define DEBUG_BAUDRATE 115200
#define DEBUG_RX_PIN 18
#define DEBUG_TX_PIN 19

#define RESPONSE_TIMEOUT_MS 50
#define RESPONSE_BUFFER_SIZE 64

HardwareSerial &FingerSerial = Serial;
HardwareSerial DebugSerial(1);

uint8_t checksumFrom(const uint8_t *buf, size_t start, size_t endInclusive) {
    uint8_t checksum = 0;
    for (size_t i = start; i <= endInclusive; i++) {
        checksum += buf[i];
    }
    return checksum;
}

void printHexByte(uint8_t b) {
    if (b < 0x10) {
        DebugSerial.print("0");
    }
    DebugSerial.print(b, HEX);
}

void printHexBuffer(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printHexByte(buf[i]);
        DebugSerial.print(i + 1 == len ? "\n" : " ");
    }
    if (len == 0) {
        DebugSerial.println("(none)");
    }
}

void buildStatusQuery(uint8_t id, uint8_t frame[8]) {
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = 0x03;
    frame[3] = id;
    frame[4] = 0x04;
    frame[5] = 0x00;
    frame[6] = 0x22;
    frame[7] = checksumFrom(frame, 2, 6);
}

size_t readRawResponse(uint8_t *buf, size_t maxLen, uint32_t timeoutMs) {
    size_t storedCount = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeoutMs) {
        while (FingerSerial.available() > 0) {
            uint8_t value = FingerSerial.read();
            if (storedCount < maxLen) {
                buf[storedCount++] = value;
            }
        }
    }

    return storedCount;
}

int findHeaderAA55(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i + 1 < len; i++) {
        if (buf[i] == 0xAA && buf[i + 1] == 0x55) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool parseAndPrintStatus(const uint8_t *frame, size_t len, uint8_t expectedId) {
    if (len < 22) {
        DebugSerial.println("Parse error: response shorter than 22 bytes");
        return false;
    }

    if (frame[0] != 0xAA || frame[1] != 0x55 ||
        frame[2] != 0x11 || frame[3] != expectedId ||
        frame[4] != 0x04 || frame[5] != 0x00 || frame[6] != 0x22) {
        DebugSerial.println("Parse error: response header or command fields invalid");
        return false;
    }

    uint8_t expectedChecksum = checksumFrom(frame, 2, 20);
    if (frame[21] != expectedChecksum) {
        DebugSerial.print("Parse error: checksum received=0x");
        printHexByte(frame[21]);
        DebugSerial.print(" expected=0x");
        printHexByte(expectedChecksum);
        DebugSerial.println();
        return false;
    }

    uint16_t targetPosition = static_cast<uint16_t>(frame[7]) |
                              (static_cast<uint16_t>(frame[8]) << 8);
    int16_t currentPosition = static_cast<int16_t>(
        static_cast<uint16_t>(frame[9]) |
        (static_cast<uint16_t>(frame[10]) << 8)
    );
    int8_t temperatureC = static_cast<int8_t>(frame[11]);
    uint16_t currentMa = static_cast<uint16_t>(frame[12]) |
                         (static_cast<uint16_t>(frame[13]) << 8);
    int16_t forceG = static_cast<int16_t>(
        static_cast<uint16_t>(frame[14]) |
        (static_cast<uint16_t>(frame[16]) << 8)
    );
    uint8_t errorFlags = frame[15];
    uint16_t internal1 = static_cast<uint16_t>(frame[17]) |
                         (static_cast<uint16_t>(frame[18]) << 8);
    uint16_t internal2 = static_cast<uint16_t>(frame[19]) |
                         (static_cast<uint16_t>(frame[20]) << 8);

    DebugSerial.println("Status response valid:");
    DebugSerial.print("  target_position: ");
    DebugSerial.println(targetPosition);
    DebugSerial.print("  current_position: ");
    DebugSerial.println(currentPosition);
    DebugSerial.print("  temperature_c: ");
    DebugSerial.println(temperatureC);
    DebugSerial.print("  current_ma: ");
    DebugSerial.println(currentMa);
    DebugSerial.print("  force_g: ");
    DebugSerial.println(forceG);
    DebugSerial.print("  error_flags: 0x");
    printHexByte(errorFlags);
    DebugSerial.println();
    DebugSerial.print("    stall: ");
    DebugSerial.println((errorFlags & 0x01) ? "yes" : "no");
    DebugSerial.print("    over_temperature: ");
    DebugSerial.println((errorFlags & 0x02) ? "yes" : "no");
    DebugSerial.print("    over_current: ");
    DebugSerial.println((errorFlags & 0x04) ? "yes" : "no");
    DebugSerial.print("    motor_abnormal: ");
    DebugSerial.println((errorFlags & 0x08) ? "yes" : "no");
    DebugSerial.print("  internal_1: ");
    DebugSerial.println(internal1);
    DebugSerial.print("  internal_2: ");
    DebugSerial.println(internal2);

    return true;
}

void setup() {
    DebugSerial.begin(DEBUG_BAUDRATE, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
    DebugSerial.println();
    DebugSerial.println("=== DH6 Finger Actuator Status Query Debug ===");
    DebugSerial.println("Diagnostic only: sends status query 0x22; no motion commands.");

    FingerSerial.begin(FINGER_BAUDRATE, SERIAL_8N1, FINGER_RX_PIN, FINGER_TX_PIN);

    DebugSerial.println("Finger UART: Serial/UART0, 921600 8N1, RX=GPIO3, TX=GPIO1");
    DebugSerial.println("Debug UART: Serial1, 115200 8N1, RX=GPIO18, TX=GPIO19");
    DebugSerial.println("Disconnect palm servo signal from GPIO19 during this test.");
    DebugSerial.println("USB-TTL RX -> GPIO19, USB-TTL GND -> ESP32 GND");
    delay(500);
}

void loop() {
    for (uint8_t id = 1; id <= 5; id++) {
        size_t staleByteCount = 0;
        while (FingerSerial.available() > 0) {
            FingerSerial.read();
            staleByteCount++;
        }

        uint8_t queryFrame[8] = {0};
        uint8_t response[RESPONSE_BUFFER_SIZE] = {0};
        buildStatusQuery(id, queryFrame);

        DebugSerial.println();
        DebugSerial.print("--- Actuator ID ");
        DebugSerial.print(id);
        DebugSerial.println(" ---");
        DebugSerial.print("Stale bytes flushed: ");
        DebugSerial.println(staleByteCount);
        DebugSerial.print("Query frame: ");
        printHexBuffer(queryFrame, sizeof(queryFrame));

        FingerSerial.write(queryFrame, sizeof(queryFrame));
        FingerSerial.flush();

        size_t receivedCount = readRawResponse(
            response,
            sizeof(response),
            RESPONSE_TIMEOUT_MS
        );

        DebugSerial.print("Received byte count: ");
        DebugSerial.println(receivedCount);
        DebugSerial.print("Raw response: ");
        printHexBuffer(response, receivedCount);

        int headerIndex = findHeaderAA55(response, receivedCount);
        DebugSerial.print("AA 55 header: ");
        if (headerIndex >= 0) {
            DebugSerial.print("found at index ");
            DebugSerial.println(headerIndex);
        } else {
            DebugSerial.println("not found");
        }

        if (headerIndex >= 0 &&
            receivedCount - static_cast<size_t>(headerIndex) >= 22) {
            parseAndPrintStatus(
                response + headerIndex,
                receivedCount - static_cast<size_t>(headerIndex),
                id
            );
        } else if (receivedCount == 0) {
            DebugSerial.println("TIMEOUT: no bytes received");
        } else if (headerIndex < 0) {
            DebugSerial.println("PARTIAL/NO HEADER: bytes received but AA 55 not found");
        } else {
            DebugSerial.println("PARTIAL: AA 55 found but fewer than 22 bytes remain");
        }
    }

    DebugSerial.println();
    DebugSerial.println("Scan complete. Waiting 1000 ms...");
    delay(1000);
}
