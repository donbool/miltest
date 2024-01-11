#include <iostream>
#include <vector>

//setup of serial driver simulation
class SerialDriver {
public:
    Packet handlePacket(const Packet& packet);

private:

    bool killStatus = false;
    float thrusters[8] = {0};

    Packet handleAck(const Packet& packet);
    Packet handleNack(const Packet& packet);
    Packet handleKill(const Packet& packet);
    Packet handleUnkill(const Packet& packet);
    Packet handleSetThrust(const Packet& packet);
    Packet handleGetKillStatus(const Packet& packet);

    unsigned char bsdChecksum(const std::vector<std::uint8_t>& data);
};

//setup of packet
struct Packet {
    static const int startBits = 0x4744;
    uint8_t identifier;
    std::vector<uint8_t> payload;
    uint8_t checksum;

    // Serialize function
    std::vector<uint8_t> Packet::serialize() const {
    std::vector<uint8_t> data; //data bus is a vector containing bits
    // Add start bits
        data.push_back(static_cast<uint8_t>(startBits >> 8)); // High byte, check endianness****
        data.push_back(static_cast<uint8_t>(startBits & 0xFF)); // Low byte

    // add identifier
    data.push_back(identifier);

    // add payload 
    data.insert(data.end(), payload.begin(), payload.end());

    // add checksum to data (?)
    uint8_t calculatedChecksum = bsdChecksum(data); // Implement this function
    data.push_back(calculatedChecksum);

    return data;
}
};

unsigned char bsdChecksum(const std::vector<std::uint8_t>& data) {
    uint8_t checksum = 0; //check over this ****
    for (auto byte : data) {
        checksum = (checksum >> 1) + ((checksum & 1) << 7);
        checksum += byte;
        checksum &= 0xFF; // Keep it to one byte, not 16 bits
    }
    return checksum;
}

Packet SerialDriver::handlePacket(const Packet& packet) {
    switch(packet.identifier) {
        case 0x00: return handleAck(packet);
        case 0x01: return handleNack(packet);
        case 0x02: return handleGetKillStatus(packet);
        case 0x05: return handleKill(packet);
        case 0x06: return handleUnkill(packet);
        case 0x07: return handleSetThrust(packet);
        // ... other cases ...
        default:   return createNackPacket(); // Handle unknown identifiers
    }
}

Packet SerialDriver::handleAck(const Packet& packet) {
    return createNackPacket();
}

Packet SerialDriver::handleNack(const Packet& packet) {
    return createNackPacket();
}

Packet SerialDriver::handleKill(const Packet& packet) {
    if (killStatus) {
        return createNackPacket(); // Already killed
    } else {
        killStatus = true;
        return createAckPacket(); // Successfully killed
    }
}

Packet SerialDriver::handleUnkill(const Packet& packet) {
    if (!killStatus) {
        return createNackPacket(); // Not killed
    } else {
        killStatus = false;
        return createAckPacket(); // Successfully un-killed
    }
}

Packet SerialDriver::handleSetThrust(const Packet& packet) {
    // Assume the first byte of payload is thruster ID and next 4 bytes are the thrust value
    uint8_t thrusterID = packet.payload[0];
    if (thrusterID >= 0 && thrusterID < 8) {
        // Extract thrust value and set it
        float thrustValue = extractFloatFromPayload(packet.payload, 1);
        thrusters[thrusterID] = thrustValue;
        return createAckPacket();
    } else {
        return createNackPacket(); // Invalid thruster ID
    }
}

Packet createAckPacket() {
    // Create and return an ACK packet
    // ...
}

Packet createNackPacket() {
    // Create and return a NACK packet
    // ...
}

float extractFloatFromPayload(const std::vector<uint8_t>& payload, size_t start) {
    // Extract a float value starting from 'start' index in payload
    // ...
}

//CAN DO THIS IN MAIN
// Packet parsePacket(const std::vector<uint8_t>& data) {
//     Packet packet;
//     // enter example data like identifier and payload. checksum and startbits are always the same
//     return packet;
// }

int main() {
    SerialDriver driver;
    // Simulate receiving data
    std::vector<uint8_t> receivedData = /* ... */;
    Packet packet = parsePacket(receivedData);
    Packet response = driver.handlePacket(packet);
    // Send response
    // ...
    return 0;
}
