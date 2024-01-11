#include <iostream>
#include <vector>
#include <chrono>

//setup of serial driver simulation
class SerialDriver {
public:
    Packet handlePacket(const Packet& packet);
    bool isKilled() const;
    unsigned char bsdChecksum(const std::vector<std::uint8_t>& data);

private:
    bool killStatus = false;
    float thrusters[8] = {0};

    Packet handleAck(const Packet& packet);
    Packet handleNack(const Packet& packet);
    Packet handleKill(const Packet& packet);
    Packet handleUnkill(const Packet& packet);
    Packet handleSetThrust(const Packet& packet);
    Packet handleGetKillStatus(const Packet& packet);    
    Packet createNackPacket();
    Packet createAckPacket();
    std::chrono::steady_clock::time_point lastHeartbeat;
    void handleHeartBeat();
};

//setup of packet
struct Packet {
    static const int startBits = 0x4744;
    uint8_t identifier;
    std::vector<uint8_t> payload;
    uint8_t checksum;

    /* format of data:
    * startbits (always 0x4744)
    * identifier (1 byte)
    * payload (variable bytes)^^^handle variability
    * checksum (1 byte) ^^do bsd calculation
    */

    // to serialize into a data packet
    std::vector<uint8_t> Packet::serialize() const {
        std::vector<uint8_t> data; 

        data.push_back(static_cast<uint8_t>(startBits >> 8)); 
        data.push_back(static_cast<uint8_t>(startBits & 0xFF)); 

        data.push_back(identifier);

        data.insert(data.end(), payload.begin(), payload.end());

        uint8_t calculatedChecksum = bsdChecksum(data);
        data.push_back(calculatedChecksum);

        return data;
    }
};

unsigned char bsdChecksum(const std::vector<std::uint8_t>& data) {
    uint8_t checksum = 0;
    for (auto byte : data) {
        checksum = (checksum >> 1) + ((checksum & 1) << 7); //bsd calc
        checksum += byte;
        checksum &= 0xFF; // ensuring it stays within 1 byte
    }
    return checksum;
}

Packet SerialDriver::handlePacket(const Packet& packet) {
    if (isKilled() && packet.identifier != 0x04) { 
        return createNackPacket(); //return nack if killed
    } else {
        switch(packet.identifier) { //checking identifier and handling cases
            case 0x00: return handleAck(packet);
            case 0x01: return handleNack(packet);
            case 0x02: return handleGetKillStatus(packet); //also handles case 0x03
            case 0x04: 
                handleHeartBeat();
                break;
            case 0x05: return handleKill(packet);
            case 0x06: return handleUnkill(packet);
            case 0x07: return handleSetThrust(packet);
            default:   return createNackPacket(); // error handling
        }
    }
}

void SerialDriver::handleHeartBeat() { //using chrono
    lastHeartbeat = std::chrono::steady_clock::now();
}

bool SerialDriver::isKilled() const {
    auto now = std::chrono::steady_clock::now();
    auto timeSinceLastHeartbeat = std::chrono::duration_cast<std::chrono::seconds>(now - lastHeartbeat).count();
    return killStatus || (timeSinceLastHeartbeat >= 1); //checking if heartbeat missed
}

Packet SerialDriver::handleGetKillStatus(const Packet& packet){
    Packet response;
    response.startBits = Packet::startBits; 
    response.identifier = 0x03;

    // set payload based on kill status
    if (killStatus) {
        response.payload = {0x01}; // 0x01 = kill enabled
    } else {
        response.payload = {0x00}; // 0x00 = kill disabled
    }

    // checksum
    std::vector<uint8_t> dataForChecksum = response.serialize();
    response.checksum = bsdChecksum(dataForChecksum);

    return response;
}

Packet SerialDriver::handleAck(const Packet& packet) {
    return createNackPacket();
}

Packet SerialDriver::handleNack(const Packet& packet) {
    return createNackPacket();
}

Packet SerialDriver::handleKill(const Packet& packet) {
    if (!killStatus) {
        killStatus = true;
        return createAckPacket();
    }
    return createNackPacket();
}

Packet SerialDriver::handleUnkill(const Packet& packet) {
    if (killStatus) {
        killStatus = false;
        return createAckPacket();
    }
    return createNackPacket();
}

Packet SerialDriver::handleSetThrust(const Packet& packet) {
    uint8_t thrusterID = packet.payload[0];
    if (thrusterID >= 0 && thrusterID < 8) {
        if (packet.payload.size() < 5) {
            return createNackPacket(); // error handling
        }

        // get float
        float thrustValue;
        // ensuring float size
        if (sizeof(float) == 4) {
            memcpy(&thrustValue, &packet.payload[1], sizeof(float));
        } else {
            return createNackPacket(); // error handling
        }

        // error handling
        if (thrustValue < 0.0f || thrustValue > 1.0f) {
            return createNackPacket();
        }

        // set thrust is kill is disabled
        if (!killStatus) {
            thrusters[thrusterID] = thrustValue;
        }

        return createAckPacket();

    } else {
        return createNackPacket(); // error handling
    }
}

Packet createAckPacket() {
    Packet ackPacket;
    ackPacket.identifier = 0x00; 
    ackPacket.payload.clear(); 

    // get checksum
    std::vector<uint8_t> dataForChecksum = ackPacket.serialize(); 
    ackPacket.checksum = bsdChecksum(dataForChecksum);

    return ackPacket;
}

Packet createNackPacket() {
    Packet nackPacket;
    nackPacket.identifier = 0x01; 
    nackPacket.payload.clear(); 

    std::vector<uint8_t> dataForChecksum = nackPacket.serialize();
    nackPacket.checksum = bsdChecksum(dataForChecksum);

    return nackPacket;
}

Packet parsePacket(const std::vector<uint8_t>& data) {
    
    if (data.size() < 4) {  // checking length
        return std::nullopt;
    }

    // checking start bits
    uint16_t startBits = (data[0] << 8) | data[1];
    if (startBits != Packet::startBits) {
        return std::nullopt;
    }

    Packet packet;
    packet.identifier = data[2];

    // handling identifier
    switch (packet.identifier) {
        case 0x00: 
        case 0x01: 
        case 0x02: 
        case 0x04: 
        case 0x05: 
        case 0x06: 
            if (data.size() != 4) return std::nullopt; //if no payload, clear
            break;
        case 0x03: 
            if (data.size() != 5) return std::nullopt; // checking packet size
            packet.payload = std::vector<uint8_t>(data.begin() + 3, data.end() - 1);
            break;
        case 0x07:
            if (data.size() != 10) return std::nullopt; // checking packet size
            packet.payload = std::vector<uint8_t>(data.begin() + 3, data.end() - 1);
            break;
        default:
            return std::nullopt; // unknown identifier
    }

    packet.checksum = data.back();

    return packet;
}

//incomplete so u cant rly run it im sorry
int main() {
    SerialDriver driver;
    // std::vector<uint8_t> receivedData = ;
    // Packet packet = parsePacket(receivedData);
    // Packet response = driver.handlePacket(packet);
    return 0;
}