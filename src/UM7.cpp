#include <UM7.h>

UM7::UM7(HardwareSerial &serial) : _serial(serial) {
    _serial.begin(115200);

    _configuration = {};
    _readings      = {};
    _packet        = {};
    _packet.state  = UM7::Packet::HEADER_S;
}

HardwareSerial &UM7::getSerial() {
    return _serial;
}

bool UM7::checkIncomingPacket() {
    uint16_t sum = 's' + 'n' + 'p' + _packet.type + _packet.address;

    for (uint8_t i = 0; i < _packet.totalLen; i++) {
        sum += _packet.content[i / 4][i % 4];
    }

    return _packet.checksum == sum;
}

#define I8(n) ((int8_t) (_packet.content[i][n]))
#define U8(n) ((uint8_t) (_packet.content[i][n]))
#define I16(n)                                                                 \
    ((int16_t) (_packet.content[i][n] << 8 | _packet.content[i][n + 1]))
#define U16(n)                                                                 \
    ((uint16_t) (_packet.content[i][n] << 8 | _packet.content[i][n + 1]))

#define float4Byte                                                             \
    union {                                                                    \
        uint8_t b[4];                                                          \
        float   f;                                                             \
    }
#define FLOAT                                                                  \
    (punned.b[3] = _packet.content[i][0],                                      \
     punned.b[2] = _packet.content[i][1],                                      \
     punned.b[1] = _packet.content[i][2],                                      \
     punned.b[0] = _packet.content[i][3],                                      \
     punned.f)

void UM7::processIncomingPacket() {
    float4Byte punned;
    uint8_t    address = _packet.address;

    for (uint8_t i = 0; i < _packet.batchLen; i++, address++) {
        switch (address) {
            case COMM_SETTINGS:
                _configuration.baudRate    = (UM7::BaudRate)(U8(0) >> 4);
                _configuration.gpsBaud     = (UM7::BaudRate)(U8(0) & 0x0F);
                _configuration.transmitGps = U8(2) & 0x01;
                _configuration.transmitSat = U8(3) >> 4 & 0x01;
                break;

            case TELE_RATES_1:
                _configuration.teleRawAccel = U8(0);
                _configuration.teleRawGyro  = U8(1);
                _configuration.teleRawMag   = U8(2);
                break;

            case TELE_RATES_2:
                _configuration.teleTemp   = U8(0);
                _configuration.teleRawAll = U8(3);
                break;

            case TELE_RATES_3:
                _configuration.teleProcAccel = U8(0);
                _configuration.teleProcGyro  = U8(1);
                _configuration.teleProcMag   = U8(2);
                break;

            case TELE_RATES_4:
                _configuration.teleProcAll = U8(3);
                break;

            case TELE_RATES_5:
                _configuration.teleQuat  = U8(0);
                _configuration.teleEuler = U8(1);
                _configuration.telePos   = U8(2);
                _configuration.teleVel   = U8(3);
                break;

            case TELE_RATES_6:
                _configuration.telePose   = U8(0);
                _configuration.teleHealth = (UM7::HealthTeleRate)(U8(1) & 0x0F);
                _configuration.teleGyroBias = U8(2);
                break;

            case TELE_RATES_7:
                _configuration.teleNmeaHealth   = (UM7::TeleRate)(U8(0) >> 4);
                _configuration.teleNmeaPose     = (UM7::TeleRate)(U8(0) & 0x0F);
                _configuration.teleNmeaAttitude = (UM7::TeleRate)(U8(1) >> 4);
                _configuration.teleNmeaSensor   = (UM7::TeleRate)(U8(1) & 0x0F);
                _configuration.teleNmeaGpsRate  = (UM7::TeleRate)(U8(2) >> 4);
                _configuration.teleNmeaGpsPose  = (UM7::TeleRate)(U8(2) & 0x0F);
                _configuration.teleNmeaQuat     = (UM7::TeleRate)(U8(3) >> 4);
                break;

            case COMM_MISC_SETTINGS:
                _configuration.usePpsPin            = U8(2) & 0x01;
                _configuration.zeroGyroOnStart      = U8(3) >> 2 & 0x01;
                _configuration.useQuat              = U8(3) >> 1 & 0x01;
                _configuration.useMagInStateUpdates = U8(3) & 0x01;
                break;

            case HOME_NORTH:
                _configuration.home.north = FLOAT;
                break;

            case HOME_EAST:
                _configuration.home.east = FLOAT;
                break;

            case HOME_UP:
                _configuration.home.up = FLOAT;
                break;

            case GYRO_TRIM_X:
                _configuration.gyroTrim.x = FLOAT;
                break;

            case GYRO_TRIM_Y:
                _configuration.gyroTrim.y = FLOAT;
                break;

            case GYRO_TRIM_Z:
                _configuration.gyroTrim.z = FLOAT;
                break;

            case MAG_CALIB_1_1:
                _configuration.magCalib[0][0] = FLOAT;
                break;

            case MAG_CALIB_1_2:
                _configuration.magCalib[0][1] = FLOAT;
                break;

            case MAG_CALIB_1_3:
                _configuration.magCalib[0][2] = FLOAT;
                break;

            case MAG_CALIB_2_1:
                _configuration.magCalib[1][0] = FLOAT;
                break;

            case MAG_CALIB_2_2:
                _configuration.magCalib[1][1] = FLOAT;
                break;

            case MAG_CALIB_2_3:
                _configuration.magCalib[1][2] = FLOAT;
                break;

            case MAG_CALIB_3_1:
                _configuration.magCalib[2][0] = FLOAT;
                break;

            case MAG_CALIB_3_2:
                _configuration.magCalib[2][1] = FLOAT;
                break;

            case MAG_CALIB_3_3:
                _configuration.magCalib[2][2] = FLOAT;
                break;

            case MAG_BIAS_X:
                _configuration.magBias.x = FLOAT;
                break;

            case MAG_BIAS_Y:
                _configuration.magBias.y = FLOAT;
                break;

            case MAG_BIAS_Z:
                _configuration.magBias.z = FLOAT;
                break;

            case ACCEL_CALIB_1_1:
                _configuration.accelCalib[0][0] = FLOAT;
                break;

            case ACCEL_CALIB_1_2:
                _configuration.accelCalib[0][1] = FLOAT;
                break;

            case ACCEL_CALIB_1_3:
                _configuration.accelCalib[0][2] = FLOAT;
                break;

            case ACCEL_CALIB_2_1:
                _configuration.accelCalib[1][0] = FLOAT;
                break;

            case ACCEL_CALIB_2_2:
                _configuration.accelCalib[1][1] = FLOAT;
                break;

            case ACCEL_CALIB_2_3:
                _configuration.accelCalib[1][2] = FLOAT;
                break;

            case ACCEL_CALIB_3_1:
                _configuration.accelCalib[2][0] = FLOAT;
                break;

            case ACCEL_CALIB_3_2:
                _configuration.accelCalib[2][1] = FLOAT;
                break;

            case ACCEL_CALIB_3_3:
                _configuration.accelCalib[2][2] = FLOAT;
                break;

            case ACCEL_BIAS_X:
                _configuration.accelBias.x = FLOAT;
                break;

            case ACCEL_BIAS_Y:
                _configuration.accelBias.y = FLOAT;
                break;

            case ACCEL_BIAS_Z:
                _configuration.accelBias.z = FLOAT;
                break;

            case HEALTH:
                _readings.health.satsUsed     = U8(0) >> 2;
                _readings.health.hdop         = U16(0) & 0x03FF;
                _readings.health.satsInView   = U8(2) >> 2;
                _readings.health.overflow     = U8(2) & 0x01;
                _readings.health.badMagNorm   = U8(3) >> 5 & 0x01;
                _readings.health.badAccelNorm = U8(3) >> 4 & 0x01;
                _readings.health.badAccel     = U8(3) >> 3 & 0x01;
                _readings.health.badGyro      = U8(3) >> 2 & 0x01;
                _readings.health.badMag       = U8(3) >> 1 & 0x01;
                _readings.health.badGps       = U8(3) & 0x01;
                break;

            case RAW_GYRO_X_Y:
                _readings.rawGyro.x = I16(0);
                _readings.rawGyro.y = I16(2);
                break;

            case RAW_GYRO_Z:
                _readings.rawGyro.z = I16(0);
                break;

            case RAW_GYRO_TIME:
                _readings.rawGyroTime = FLOAT;
                break;

            case RAW_ACCEL_X_Y:
                _readings.rawAccel.x = I16(0);
                _readings.rawAccel.y = I16(2);
                break;

            case RAW_ACCEL_Z:
                _readings.rawAccel.z = I16(0);
                break;

            case RAW_ACCEL_TIME:
                _readings.rawAccelTime = FLOAT;
                break;

            case RAW_MAG_X_Y:
                _readings.rawMag.x = I16(0);
                _readings.rawMag.y = I16(2);
                break;

            case RAW_MAG_Z:
                _readings.rawMag.z = I16(0);
                break;

            case RAW_MAG_TIME:
                _readings.rawMagTime = FLOAT;
                break;

            case TEMP:
                _readings.temp = FLOAT;
                break;

            case TEMP_TIME:
                _readings.tempTime = FLOAT;
                break;

            case PROC_GYRO_X:
                _readings.procGyro.x = FLOAT;
                break;

            case PROC_GYRO_Y:
                _readings.procGyro.y = FLOAT;
                break;

            case PROC_GYRO_Z:
                _readings.procGyro.z = FLOAT;
                break;

            case PROC_GYRO_TIME:
                _readings.procGyroTime = FLOAT;
                break;

            case PROC_ACCEL_X:
                _readings.procAccel.x = FLOAT;
                break;

            case PROC_ACCEL_Y:
                _readings.procAccel.y = FLOAT;
                break;

            case PROC_ACCEL_Z:
                _readings.procAccel.z = FLOAT;
                break;

            case PROC_ACCEL_TIME:
                _readings.procAccelTime = FLOAT;
                break;

            case PROC_MAG_X:
                _readings.procMag.x = FLOAT;
                break;

            case PROC_MAG_Y:
                _readings.procMag.y = FLOAT;
                break;

            case PROC_MAG_Z:
                _readings.procMag.z = FLOAT;
                break;

            case PROC_MAG_TIME:
                _readings.procMagTime = FLOAT;
                break;

            case QUAT_AB:
                _readings.quat.a = (float) I16(0) / 29789.09091f;
                _readings.quat.b = (float) I16(2) / 29789.09091f;
                break;

            case QUAT_CD:
                _readings.quat.c = (float) I16(0) / 29789.09091f;
                _readings.quat.d = (float) I16(2) / 29789.09091f;
                break;

            case QUAT_TIME:
                _readings.quatTime = FLOAT;
                break;

            case EULER_PHI_THETA:
                _readings.euler.phi   = (float) I16(0) / 91.02222f;
                _readings.euler.theta = (float) I16(2) / 91.02222f;
                break;

            case EULER_PSI:
                _readings.euler.psi = (float) I16(0) / 91.02222f;
                break;

            case EULER_PHI_THETA_RATE:
                _readings.eulerRate.phi   = (float) I16(0) / 16.0f;
                _readings.eulerRate.theta = (float) I16(0) / 16.0f;
                break;

            case EULER_PSI_RATE:
                _readings.eulerRate.psi = (float) I16(0) / 16.0f;
                break;

            case EULER_TIME:
                _readings.eulerTime = FLOAT;
                break;

            case POS_NORTH:
                _readings.pos.north = FLOAT;
                break;

            case POS_EAST:
                _readings.pos.east = FLOAT;
                break;

            case POS_UP:
                _readings.pos.up = FLOAT;
                break;

            case POS_TIME:
                _readings.posTime = FLOAT;
                break;

            case VEL_NORTH:
                _readings.vel.north = FLOAT;
                break;

            case VEL_EAST:
                _readings.vel.east = FLOAT;
                break;

            case VEL_UP:
                _readings.vel.up = FLOAT;
                break;

            case VEL_TIME:
                _readings.velTime = FLOAT;
                break;

            case GPS_NORTH:
                _readings.gps.pos.north = FLOAT;
                break;

            case GPS_EAST:
                _readings.gps.pos.east = FLOAT;
                break;

            case GPS_UP:
                _readings.gps.pos.up = FLOAT;
                break;

            case GPS_COURSE:
                _readings.gps.course = FLOAT;
                break;

            case GPS_SPEED:
                _readings.gps.speed = FLOAT;
                break;

            case GPS_TIME:
                _readings.gpsTime = FLOAT;
                break;

            case SAT_1_2:
                _readings.sats[0].id      = U8(0);
                _readings.sats[0].snRatio = U8(1);
                _readings.sats[1].id      = U8(2);
                _readings.sats[1].snRatio = U8(3);
                break;

            case SAT_3_4:
                _readings.sats[2].id      = U8(0);
                _readings.sats[2].snRatio = U8(1);
                _readings.sats[3].id      = U8(2);
                _readings.sats[3].snRatio = U8(3);
                break;

            case SAT_5_6:
                _readings.sats[4].id      = U8(0);
                _readings.sats[4].snRatio = U8(1);
                _readings.sats[5].id      = U8(2);
                _readings.sats[5].snRatio = U8(3);
                break;

            case SAT_7_8:
                _readings.sats[6].id      = U8(0);
                _readings.sats[6].snRatio = U8(1);
                _readings.sats[7].id      = U8(2);
                _readings.sats[7].snRatio = U8(3);
                break;

            case SAT_9_10:
                _readings.sats[8].id      = U8(0);
                _readings.sats[8].snRatio = U8(1);
                _readings.sats[9].id      = U8(2);
                _readings.sats[9].snRatio = U8(3);
                break;

            case SAT_11_12:
                _readings.sats[10].id      = U8(0);
                _readings.sats[10].snRatio = U8(1);
                _readings.sats[11].id      = U8(2);
                _readings.sats[11].snRatio = U8(3);
                break;

            case GYRO_BIAS_X:
                _readings.gyroBias.x = FLOAT;
                break;

            case GYRO_BIAS_Y:
                _readings.gyroBias.y = FLOAT;
                break;

            case GYRO_BIAS_Z:
                _readings.gyroBias.z = FLOAT;
                break;

            default:
                break;
        }
    }
}

void UM7::parse(uint8_t in) {
    switch (_packet.state) {
        case UM7::Packet::HEADER_S:
            if (in == 's') {
                _packet.state = UM7::Packet::HEADER_N;
            }
            break;

        case UM7::Packet::HEADER_N:
            _packet.state =
                in == 'n' ? UM7::Packet::HEADER_P : UM7::Packet::HEADER_S;
            break;

        case UM7::Packet::HEADER_P:
            _packet.state =
                in == 'p' ? UM7::Packet::PACKET_TYPE : UM7::Packet::HEADER_S;
            break;

        case UM7::Packet::PACKET_TYPE:
            _packet.type     = in;
            _packet.hasData  = in >> 7;
            _packet.isBatch  = in >> 6 & 0x01;
            _packet.batchLen = in >> 2 & 0x0F;
            _packet.hidden   = in >> 1 & 0x01;
            _packet.cmdFail  = in & 0x01;
            _packet.state    = UM7::Packet::ADDRESS;
            break;

        case UM7::Packet::ADDRESS:
            _packet.address = (UM7::ReadableRegister) in;

            if (_packet.hasData) {
                _packet.totalLen = _packet.isBatch ? _packet.batchLen * 4 : 4;
                _packet.state    = UM7::Packet::DATA_BYTES;
            } else {
                _packet.totalLen = 0;
                _packet.state    = UM7::Packet::CHECKSUM_1;
            }
            break;

        case UM7::Packet::DATA_BYTES:
            _packet.content[_packet.pos / 4][_packet.pos % 4] = in;
            _packet.pos++;

            if (_packet.pos >= _packet.totalLen) {
                _packet.state = UM7::Packet::CHECKSUM_1;
                _packet.pos   = 0;
            }
            break;

        case UM7::Packet::CHECKSUM_1:
            _packet.checksum = in;
            _packet.checksum <<= 8;
            _packet.state = UM7::Packet::CHECKSUM_2;
            break;

        case UM7::Packet::CHECKSUM_2:
            _packet.checksum |= in;
            _packet.state = UM7::Packet::HEADER_S;

            if (checkIncomingPacket()) {
                processIncomingPacket();
            }
            break;

        default:
            _packet.state = UM7::Packet::HEADER_S;
            break;
    }
}

void UM7::readSerial() {
    while (_serial.available()) {
        parse(_serial.read());
    }
}

#define PACKET(regs)                                                           \
    uint8_t len                 = (regs);                                      \
    uint8_t packet[7 + len * 4] = {'s', 'n', 'p'}

#define PACKET_TYPE(hasData, isBatch, batchLen)                                \
    packet[3] = ((uint8_t) (hasData) << 7 | (uint8_t) (isBatch) << 6 |         \
                 (uint8_t) (batchLen) << 2)

#define PACKET_REGISTER(reg) packet[4] = (reg)

#define PACKET_DATA(i, a, b, c, d)                                             \
    packet[0 + 5 + i * 4] = (a);                                               \
    packet[1 + 5 + i * 4] = (b);                                               \
    packet[2 + 5 + i * 4] = (c);                                               \
    packet[3 + 5 + i * 4] = (d)

#define PACKET_DATA_FLOAT(i, val)                                              \
    float4Byte out##i;                                                         \
    out##i.f = (val);                                                          \
    PACKET_DATA(i, out##i.b[3], out##i.b[2], out##i.b[1], out##i.b[0])

#define CHECKSUM()                                                             \
    uint16_t checksum = 's' + 'n' + 'p' + packet[3] + packet[4];               \
    for (int i = 0; i < len; i++) {                                            \
        checksum += packet[0 + 5 + i * 4] + packet[1 + 5 + i * 4] +            \
                    packet[2 + 5 + i * 4] + packet[3 + 5 + i * 4];             \
    }                                                                          \
    packet[5 + len * 4] = (uint8_t) (checksum >> 8);                           \
    packet[6 + len * 4] = (uint8_t) (checksum & 0xFF)

#define SEND() _serial.write(packet, 7 + len * 4)

void UM7::requestRead(UM7::ReadableRegister reg) {
    PACKET(0);
    PACKET_TYPE(false, false, 0);
    PACKET_REGISTER(reg);
    CHECKSUM();
    SEND();
}

void UM7::requestReadBatch(UM7::ReadableRegister first,
                           UM7::ReadableRegister last) {
    if (first > last) {
        return;
    }

    PACKET(0);
    PACKET_TYPE(false, true, first - last + 1);
    PACKET_REGISTER(first);
    CHECKSUM();
    SEND();
}

void UM7::executeCommand(UM7::CommandRegister command) {
    PACKET(0);
    PACKET_TYPE(false, false, 0);
    PACKET_REGISTER(command);
    CHECKSUM();
    SEND();
}

UM7::BaudRate UM7::getBaudRate() {
    return _configuration.baudRate;
}

void UM7::setBaudRate(UM7::BaudRate rate) {
    _configuration.baudRate = rate;
}

UM7::BaudRate UM7::getGpsBaud() {
    return _configuration.gpsBaud;
}

void UM7::setGpsBaud(UM7::BaudRate rate) {
    _configuration.gpsBaud = rate;
}

uint8_t UM7::getTransmitGps() {
    return _configuration.transmitGps;
}

void UM7::setTransmitGps(bool val) {
    _configuration.transmitGps = val;
}

uint8_t UM7::getTransmitSat() {
    return _configuration.transmitSat;
}

void UM7::setTransmitSat(bool val) {
    _configuration.transmitSat = val;
}

void UM7::commitCommSettings() {
    PACKET(1);
    PACKET_TYPE(true, false, 0);
    PACKET_REGISTER(COMM_SETTINGS);
    PACKET_DATA(0,
                (_configuration.baudRate << 4) | _configuration.gpsBaud,
                0,
                _configuration.transmitGps,
                _configuration.transmitSat << 4);
    CHECKSUM();
    SEND();
}

uint8_t UM7::getTeleRawAccel() {
    return _configuration.teleRawAccel;
}

void UM7::setTeleRawAccel(uint8_t rate) {
    _configuration.teleRawAccel = rate;
}

uint8_t UM7::getTeleRawGyro() {
    return _configuration.teleRawGyro;
}

void UM7::setTeleRawGyro(uint8_t rate) {
    _configuration.teleRawGyro = rate;
}

uint8_t UM7::getTeleRawMag() {
    return _configuration.teleRawMag;
}

void UM7::setTeleRawMag(uint8_t rate) {
    _configuration.teleRawMag = rate;
}

uint8_t UM7::getTeleTemp() {
    return _configuration.teleTemp;
}

void UM7::setTeleTemp(uint8_t rate) {
    _configuration.teleTemp = rate;
}

uint8_t UM7::getTeleRawAll() {
    return _configuration.teleRawAll;
}

void UM7::setTeleRawAll(uint8_t rate) {
    _configuration.teleRawAll = rate;
}

uint8_t UM7::getTeleProcAccel() {
    return _configuration.teleProcAccel;
}

void UM7::setTeleProcAccel(uint8_t rate) {
    _configuration.teleProcAccel = rate;
}

uint8_t UM7::getTeleProcGyro() {
    return _configuration.teleProcGyro;
}

void UM7::setTeleProcGyro(uint8_t rate) {
    _configuration.teleProcGyro = rate;
}

uint8_t UM7::getTeleProcMag() {
    return _configuration.teleProcMag;
}

void UM7::setTeleProcMag(uint8_t rate) {
    _configuration.teleProcMag = rate;
}

uint8_t UM7::getTeleProcAll() {
    return _configuration.teleProcAll;
}

void UM7::setTeleProcAll(uint8_t rate) {
    _configuration.teleProcAll = rate;
}

uint8_t UM7::getTeleQuat() {
    return _configuration.teleQuat;
}

void UM7::setTeleQuat(uint8_t rate) {
    _configuration.teleQuat = rate;
}

uint8_t UM7::getTeleEuler() {
    return _configuration.teleEuler;
}

void UM7::setTeleEuler(uint8_t rate) {
    _configuration.teleEuler = rate;
}

uint8_t UM7::getTelePos() {
    return _configuration.telePos;
}

void UM7::setTelePos(uint8_t rate) {
    _configuration.telePos = rate;
}

uint8_t UM7::getTeleVel() {
    return _configuration.teleVel;
}

void UM7::setTeleVel(uint8_t rate) {
    _configuration.teleVel = rate;
}

uint8_t UM7::getTelePose() {
    return _configuration.telePose;
}

void UM7::setTelePose(uint8_t rate) {
    _configuration.telePose = rate;
}

UM7::HealthTeleRate UM7::getTeleHealth() {
    return _configuration.teleHealth;
}

void UM7::setTeleHealth(UM7::HealthTeleRate rate) {
    _configuration.teleHealth = rate;
}

uint8_t UM7::getTeleGyroBias() {
    return _configuration.teleGyroBias;
}

void UM7::setTeleGyroBias(uint8_t rate) {
    _configuration.teleGyroBias = rate;
}

void UM7::commitTeleSettings() {
    PACKET(6);
    PACKET_TYPE(true, true, 6);
    PACKET_REGISTER(TELE_RATES_1);
    PACKET_DATA(0,
                _configuration.teleRawAccel,
                _configuration.teleRawGyro,
                _configuration.teleRawMag,
                0);
    PACKET_DATA(1, _configuration.teleTemp, 0, 0, _configuration.teleRawAll);
    PACKET_DATA(2,
                _configuration.teleProcAccel,
                _configuration.teleProcGyro,
                _configuration.teleProcMag,
                0);
    PACKET_DATA(3, 0, 0, 0, _configuration.teleProcAll);
    PACKET_DATA(4,
                _configuration.teleQuat,
                _configuration.teleEuler,
                _configuration.telePos,
                _configuration.teleVel);
    PACKET_DATA(5,
                _configuration.telePose,
                _configuration.teleHealth,
                _configuration.teleGyroBias,
                0);
    CHECKSUM();
    SEND();
}

UM7::TeleRate UM7::getTeleNmeaHealth() {
    return _configuration.teleNmeaHealth;
}

void UM7::setTeleNmeaHealth(UM7::TeleRate rate) {
    _configuration.teleNmeaHealth = rate;
}

UM7::TeleRate UM7::getTeleNmeaPose() {
    return _configuration.teleNmeaPose;
}

void UM7::setTeleNmeaPose(UM7::TeleRate rate) {
    _configuration.teleNmeaPose = rate;
}

UM7::TeleRate UM7::getTeleNmeaAttitude() {
    return _configuration.teleNmeaAttitude;
}

void UM7::setTeleNmeaAttitude(UM7::TeleRate rate) {
    _configuration.teleNmeaAttitude = rate;
}

UM7::TeleRate UM7::getTeleNmeaSensor() {
    return _configuration.teleNmeaSensor;
}

void UM7::setTeleNmeaSensor(UM7::TeleRate rate) {
    _configuration.teleNmeaSensor = rate;
}

UM7::TeleRate UM7::getTeleNmeaGpsRate() {
    return _configuration.teleNmeaGpsRate;
}

void UM7::setTeleNmeaGpsRate(UM7::TeleRate rate) {
    _configuration.teleNmeaGpsRate = rate;
}

UM7::TeleRate UM7::getTeleNmeaGpsPose() {
    return _configuration.teleNmeaGpsPose;
}

void UM7::setTeleNmeaGpsPose(UM7::TeleRate rate) {
    _configuration.teleNmeaGpsPose = rate;
}

UM7::TeleRate UM7::getTeleNmeaQuat() {
    return _configuration.teleNmeaQuat;
}

void UM7::setTeleNmeaQuat(UM7::TeleRate rate) {
    _configuration.teleNmeaQuat = rate;
}

void UM7::commitTeleNmeaSettings() {
    PACKET(1);
    PACKET_TYPE(true, false, 0);
    PACKET_REGISTER(TELE_RATES_7);
    PACKET_DATA(
        0,
        (_configuration.teleNmeaHealth << 4) | _configuration.teleNmeaPose,
        (_configuration.teleNmeaAttitude << 4) | _configuration.teleNmeaSensor,
        (_configuration.teleNmeaGpsRate << 4) | _configuration.teleNmeaGpsPose,
        _configuration.teleNmeaQuat << 4);
    CHECKSUM();
    SEND();
}

bool UM7::getUsePpsPin() {
    return _configuration.usePpsPin;
}

void UM7::setUsePpsPin(bool val) {
    _configuration.usePpsPin = val;
}

bool UM7::getZeroGyroOnStart() {
    return _configuration.zeroGyroOnStart;
}

void UM7::setZeroGyroOnStart(bool val) {
    _configuration.zeroGyroOnStart = val;
}

bool UM7::getUseQuat() {
    return _configuration.useQuat;
}

void UM7::setUseQuat(bool val) {
    _configuration.useQuat = val;
}

bool UM7::getUseMagInStateUpdates() {
    return _configuration.useMagInStateUpdates;
}

void UM7::setUseMagInStateUpdates(bool val) {
    _configuration.useMagInStateUpdates = val;
}

void UM7::commitCommMiscSettings() {
    PACKET(1);
    PACKET_TYPE(true, false, 0);
    PACKET_REGISTER(COMM_MISC_SETTINGS);
    PACKET_DATA(0,
                0,
                0,
                _configuration.usePpsPin,
                (_configuration.zeroGyroOnStart << 2) |
                    (_configuration.useQuat << 1) |
                    _configuration.useMagInStateUpdates);
    CHECKSUM();
    SEND();
}

UM7::Geographic<float> UM7::getHome() {
    return _configuration.home;
}

void UM7::setHome(UM7::Geographic<float> val) {
    _configuration.home = val;
}

float UM7::getHomeNorth() {
    return _configuration.home.north;
}

void UM7::setHomeNorth(float val) {
    _configuration.home.north = val;
}

float UM7::getHomeEast() {
    return _configuration.home.east;
}

void UM7::setHomeEast(float val) {
    _configuration.home.east = val;
}

float UM7::getHomeUp() {
    return _configuration.home.up;
}

void UM7::setHomeUp(float val) {
    _configuration.home.up = val;
}

void UM7::commitHomeSettings() {
    PACKET(3);
    PACKET_TYPE(true, true, 3);
    PACKET_REGISTER(HOME_NORTH);
    PACKET_DATA_FLOAT(0, _configuration.home.north);
    PACKET_DATA_FLOAT(1, _configuration.home.east);
    PACKET_DATA_FLOAT(2, _configuration.home.up);
    CHECKSUM();
    SEND();
}

UM7::Cartesian<float> UM7::getGyroTrim() {
    return _configuration.gyroTrim;
}

void UM7::setGyroTrim(UM7::Cartesian<float> val) {
    _configuration.gyroTrim = val;
}

float UM7::getGyroTrimX() {
    return _configuration.gyroTrim.x;
}

void UM7::setGyroTrimX(float val) {
    _configuration.gyroTrim.x = val;
}

float UM7::getGyroTrimY() {
    return _configuration.gyroTrim.y;
}

void UM7::setGyroTrimY(float val) {
    _configuration.gyroTrim.x = val;
}

float UM7::getGyroTrimZ() {
    return _configuration.gyroTrim.z;
}

void UM7::setGyroTrimZ(float val) {
    _configuration.gyroTrim.x = val;
}

void UM7::commitGyroTrimSettings() {
    PACKET(3);
    PACKET_TYPE(true, true, 3);
    PACKET_REGISTER(GYRO_TRIM_X);
    PACKET_DATA_FLOAT(0, _configuration.gyroTrim.x);
    PACKET_DATA_FLOAT(1, _configuration.gyroTrim.y);
    PACKET_DATA_FLOAT(2, _configuration.gyroTrim.z);
    CHECKSUM();
    SEND();
}

float UM7::getMagCalib(uint8_t x, uint8_t y) {
    x = constrain(x, 1, 3) - 1;
    y = constrain(y, 1, 3) - 1;

    return _configuration.magCalib[x][y];
}

void UM7::setMagCalib(uint8_t x, uint8_t y, uint8_t val) {
    x = constrain(x, 1, 3) - 1;
    y = constrain(y, 1, 3) - 1;

    _configuration.magCalib[x][y] = val;
}

void UM7::commitMagCalibSettings() {
    PACKET(9);
    PACKET_TYPE(true, true, 9);
    PACKET_REGISTER(MAG_CALIB_1_1);
    PACKET_DATA_FLOAT(0, _configuration.magCalib[0][0]);
    PACKET_DATA_FLOAT(1, _configuration.magCalib[0][1]);
    PACKET_DATA_FLOAT(2, _configuration.magCalib[0][2]);
    PACKET_DATA_FLOAT(3, _configuration.magCalib[1][0]);
    PACKET_DATA_FLOAT(4, _configuration.magCalib[1][1]);
    PACKET_DATA_FLOAT(5, _configuration.magCalib[1][2]);
    PACKET_DATA_FLOAT(6, _configuration.magCalib[2][0]);
    PACKET_DATA_FLOAT(7, _configuration.magCalib[2][1]);
    PACKET_DATA_FLOAT(8, _configuration.magCalib[2][2]);
    CHECKSUM();
    SEND();
}

UM7::Cartesian<float> UM7::getMagBias() {
    return _configuration.magBias;
}

void UM7::setMagBias(UM7::Cartesian<float> val) {
    _configuration.magBias = val;
}

float UM7::getMagBiasX() {
    return _configuration.magBias.x;
}

void UM7::setMagBiasX(float val) {
    _configuration.magBias.x = val;
}

float UM7::getMagBiasY() {
    return _configuration.magBias.y;
}

void UM7::setMagBiasY(float val) {
    _configuration.magBias.y = val;
}

float UM7::getMagBiasZ() {
    return _configuration.magBias.z;
}

void UM7::setMagBiasZ(float val) {
    _configuration.magBias.z = val;
}

void UM7::commitMagBiasSettings() {
    PACKET(3);
    PACKET_TYPE(true, true, 3);
    PACKET_REGISTER(MAG_BIAS_X);
    PACKET_DATA_FLOAT(0, _configuration.magBias.x);
    PACKET_DATA_FLOAT(1, _configuration.magBias.y);
    PACKET_DATA_FLOAT(2, _configuration.magBias.z);
    CHECKSUM();
    SEND();
}

float UM7::getAccelCalib(uint8_t x, uint8_t y) {
    x = constrain(x, 1, 3) - 1;
    y = constrain(y, 1, 3) - 1;

    return _configuration.accelCalib[x][y];
}

void UM7::setAccelCalib(uint8_t x, uint8_t y, uint8_t val) {
    x = constrain(x, 1, 3) - 1;
    y = constrain(y, 1, 3) - 1;

    _configuration.accelCalib[x][y] = val;
}

void UM7::commitAccelCalibSettings() {
    PACKET(9);
    PACKET_TYPE(true, true, 9);
    PACKET_REGISTER(ACCEL_CALIB_1_1);
    PACKET_DATA_FLOAT(0, _configuration.accelCalib[0][0]);
    PACKET_DATA_FLOAT(1, _configuration.accelCalib[0][1]);
    PACKET_DATA_FLOAT(2, _configuration.accelCalib[0][2]);
    PACKET_DATA_FLOAT(3, _configuration.accelCalib[1][0]);
    PACKET_DATA_FLOAT(4, _configuration.accelCalib[1][1]);
    PACKET_DATA_FLOAT(5, _configuration.accelCalib[1][2]);
    PACKET_DATA_FLOAT(6, _configuration.accelCalib[2][0]);
    PACKET_DATA_FLOAT(7, _configuration.accelCalib[2][1]);
    PACKET_DATA_FLOAT(8, _configuration.accelCalib[2][2]);
    CHECKSUM();
    SEND();
}

UM7::Cartesian<float> UM7::getAccelBias() {
    return _configuration.accelBias;
}

void UM7::setAccelBias(UM7::Cartesian<float> val) {
    _configuration.accelBias = val;
}

float UM7::getAccelBiasX() {
    return _configuration.accelBias.x;
}

void UM7::setAccelBiasX(float val) {
    _configuration.accelBias.x = val;
}

float UM7::getAccelBiasY() {
    return _configuration.accelBias.y;
}

void UM7::setAccelBiasY(float val) {
    _configuration.accelBias.y = val;
}

float UM7::getAccelBiasZ() {
    return _configuration.accelBias.z;
}

void UM7::setAccelBiasZ(float val) {
    _configuration.accelBias.z = val;
}

void UM7::commitAccelBiasSettings() {
    PACKET(3);
    PACKET_TYPE(true, true, 3);
    PACKET_REGISTER(ACCEL_BIAS_X);
    PACKET_DATA_FLOAT(0, _configuration.accelBias.x);
    PACKET_DATA_FLOAT(1, _configuration.accelBias.y);
    PACKET_DATA_FLOAT(2, _configuration.accelBias.z);
    CHECKSUM();
    SEND();
}

UM7::Health UM7::getHealth() {
    return _readings.health;
}

uint8_t UM7::getHealthSatsUsed() {
    return _readings.health.satsUsed;
}

float UM7::getHealthHdop() {
    return _readings.health.hdop;
}

uint8_t UM7::getHealthSatsInView() {
    return _readings.health.satsInView;
}

bool UM7::getHealthOverflow() {
    return _readings.health.overflow;
}

bool UM7::getHealthBadAccelNorm() {
    return _readings.health.badAccelNorm;
}

bool UM7::getHealthBadMagNorm() {
    return _readings.health.badMagNorm;
}

bool UM7::getHealthBadAccel() {
    return _readings.health.badAccel;
}

bool UM7::getHealthBadGyro() {
    return _readings.health.badGyro;
}

bool UM7::getHealthBadMag() {
    return _readings.health.badMag;
}

bool UM7::getHealthBadGps() {
    return _readings.health.badGps;
}

UM7::Cartesian<int16_t> UM7::getRawGyro() {
    return _readings.rawGyro;
}

int16_t UM7::getRawGyroX() {
    return _readings.rawGyro.x;
}

int16_t UM7::getRawGyroY() {
    return _readings.rawGyro.y;
}

int16_t UM7::getRawGyroZ() {
    return _readings.rawGyro.z;
}

float UM7::getRawGyroTime() {
    return _readings.rawGyroTime;
}

UM7::Cartesian<int16_t> UM7::getRawAccel() {
    return _readings.rawAccel;
}

int16_t UM7::getRawAccelX() {
    return _readings.rawAccel.x;
}

int16_t UM7::getRawAccelY() {
    return _readings.rawAccel.y;
}

int16_t UM7::getRawAccelZ() {
    return _readings.rawAccel.z;
}

float UM7::getRawAccelTime() {
    return _readings.rawAccelTime;
}

UM7::Cartesian<int16_t> UM7::getRawMag() {
    return _readings.rawMag;
}

int16_t UM7::getRawMagX() {
    return _readings.rawMag.x;
}

int16_t UM7::getRawMagY() {
    return _readings.rawMag.y;
}

int16_t UM7::getRawMagZ() {
    return _readings.rawMag.z;
}

float UM7::getRawMagTime() {
    return _readings.rawMagTime;
}

float UM7::getTemp() {
    return _readings.temp;
}

float UM7::getTempTime() {
    return _readings.tempTime;
}

UM7::Cartesian<float> UM7::getProcAccel() {
    return _readings.procAccel;
}

UM7::Cartesian<float> UM7::getProcGyro() {
    return _readings.procGyro;
}

float UM7::getProcGyroX() {
    return _readings.procGyro.x;
}

float UM7::getProcGyroY() {
    return _readings.procGyro.y;
}

float UM7::getProcGyroZ() {
    return _readings.procGyro.z;
}

float UM7::getProcGyroTime() {
    return _readings.procGyroTime;
}

float UM7::getProcAccelX() {
    return _readings.procAccel.x;
}

float UM7::getProcAccelY() {
    return _readings.procAccel.y;
}

float UM7::getProcAccelZ() {
    return _readings.procAccel.z;
}

float UM7::getProcAccelTime() {
    return _readings.procAccelTime;
}

UM7::Cartesian<float> UM7::getProcMag() {
    return _readings.procMag;
}

float UM7::getProcMagX() {
    return _readings.procMag.x;
}

float UM7::getProcMagY() {
    return _readings.procMag.y;
}

float UM7::getProcMagZ() {
    return _readings.procMag.z;
}

float UM7::getProcMagTime() {
    return _readings.procMagTime;
}

UM7::Quaternion<float> UM7::getQuat() {
    return _readings.quat;
}

float UM7::getQuatA() {
    return _readings.quat.a;
}

float UM7::getQuatB() {
    return _readings.quat.a;
}

float UM7::getQuatC() {
    return _readings.quat.a;
}

float UM7::getQuatD() {
    return _readings.quat.a;
}

float UM7::getQuatTime() {
    return _readings.quatTime;
}

UM7::Euler<float> UM7::getEuler() {
    return _readings.euler;
}

float UM7::getEulerPhi() {
    return _readings.euler.phi;
}

float UM7::getEulerTheta() {
    return _readings.euler.theta;
}

float UM7::getEulerPsi() {
    return _readings.euler.psi;
}

UM7::Euler<float> UM7::getEulerRate() {
    return _readings.eulerRate;
}

float UM7::getEulerRatePhi() {
    return _readings.eulerRate.phi;
}

float UM7::getEulerRateTheta() {
    return _readings.eulerRate.theta;
}

float UM7::getEulerRatePsi() {
    return _readings.eulerRate.psi;
}

float UM7::getEulerTime() {
    return _readings.eulerTime;
}

UM7::Geographic<float> UM7::getPos() {
    return _readings.pos;
}

float UM7::getPosNorth() {
    return _readings.pos.north;
}

float UM7::getPosEast() {
    return _readings.pos.east;
}

float UM7::getPosUp() {
    return _readings.pos.up;
}

float UM7::getPosTime() {
    return _readings.posTime;
}

UM7::Geographic<float> UM7::getVel() {
    return _readings.vel;
}

float UM7::getVelNorth() {
    return _readings.vel.north;
}

float UM7::getVelEast() {
    return _readings.vel.east;
}

float UM7::getVelUp() {
    return _readings.vel.up;
}

float UM7::getVelTime() {
    return _readings.velTime;
}

UM7::GPS UM7::getGps() {
    return _readings.gps;
}

UM7::Geographic<float> UM7::getGpsPos() {
    return _readings.gps.pos;
}

float UM7::getGpsPosNorth() {
    return _readings.gps.pos.north;
}

float UM7::getGpsPosEast() {
    return _readings.gps.pos.east;
}

float UM7::getGpsPosUp() {
    return _readings.gps.pos.up;
}

float UM7::getGpsCourse() {
    return _readings.gps.course;
}

float UM7::getGpsSpeed() {
    return _readings.gps.speed;
}

float UM7::getGpsTime() {
    return _readings.gpsTime;
}

UM7::Satellite UM7::getSatellite(uint8_t index) {
    constrain(index, 0, 11);
    return _readings.sats[index];
}

uint8_t UM7::getSatelliteId(uint8_t index) {
    constrain(index, 0, 11);
    return _readings.sats[index].id;
}

uint8_t UM7::getSatelliteSnRatio(uint8_t index) {
    constrain(index, 0, 11);
    return _readings.sats[index].snRatio;
}

UM7::Cartesian<float> UM7::getGyroBias() {
    return _readings.gyroBias;
}

float UM7::getGyroBiasX() {
    return _readings.gyroBias.x;
}

float UM7::getGyroBiasY() {
    return _readings.gyroBias.y;
}

float UM7::getGyroBiasZ() {
    return _readings.gyroBias.z;
}
