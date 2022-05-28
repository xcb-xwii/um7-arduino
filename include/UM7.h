#ifndef UM7_H
#define UM7_H

#include <Arduino.h>

class UM7 {
    public:
    enum BaudRate : uint8_t {
        BAUD_9600   = 0,
        BAUD_14400  = 1,
        BAUD_19200  = 2,
        BAUD_38400  = 3,
        BAUD_57600  = 4,
        BAUD_115200 = 5,
        BAUD_128000 = 6,
        BAUD_153600 = 7,
        BAUD_230400 = 8,
        BAUD_256000 = 9,
        BAUD_460800 = 10,
        BAUD_921600 = 11
    };

    enum HealthTeleRate : uint8_t {
        H_TELE_OFF   = 0,
        H_TELE_0_125 = 1,
        H_TELE_0_25  = 2,
        H_TELE_0_5   = 3,
        H_TELE_1     = 4,
        H_TELE_2     = 5,
        H_TELE_4     = 6
    };

    enum TeleRate : uint8_t {
        TELE_OFF = 0,
        TELE_1   = 1,
        TELE_2   = 2,
        TELE_4   = 3,
        TELE_5   = 4,
        TELE_10  = 5,
        TELE_15  = 6,
        TELE_20  = 7,
        TELE_30  = 8,
        TELE_40  = 9,
        TELE_50  = 10,
        TELE_60  = 11,
        TELE_70  = 12,
        TELE_80  = 13,
        TELE_90  = 14,
        TELE_100 = 15
    };

    struct Health {
        uint8_t satsUsed;
        float   hdop;
        uint8_t satsInView;
        bool    overflow;
        bool    badAccelNorm;
        bool    badMagNorm;
        bool    badAccel;
        bool    badGyro;
        bool    badMag;
        bool    badGps;
    };

    template<typename T> struct Cartesian {
        T x;
        T y;
        T z;
    };

    template<typename T> struct Euler {
        T phi;
        T theta;
        T psi;
    };

    template<typename T> struct Quaternion {
        T a;
        T b;
        T c;
        T d;
    };

    template<typename T> struct Geographic {
        T north;
        T east;
        T up;
    };

    struct GPS {
        Geographic<float> pos;
        float             course;
        float             speed;
    };

    struct Satellite {
        uint8_t id;
        uint8_t snRatio;
    };

    enum ReadableRegister : uint8_t {
        COMM_SETTINGS        = 0x00,
        TELE_RATES_1         = 0x01,
        TELE_RATES_2         = 0x02,
        TELE_RATES_3         = 0x03,
        TELE_RATES_4         = 0x04,
        TELE_RATES_5         = 0x05,
        TELE_RATES_6         = 0x06,
        TELE_RATES_7         = 0x07,
        COMM_MISC_SETTINGS   = 0x08,
        HOME_NORTH           = 0x09,
        HOME_EAST            = 0x0A,
        HOME_UP              = 0x0B,
        GYRO_TRIM_X          = 0x0C,
        GYRO_TRIM_Y          = 0x0D,
        GYRO_TRIM_Z          = 0x0E,
        MAG_CALIB_1_1        = 0x0F,
        MAG_CALIB_1_2        = 0x10,
        MAG_CALIB_1_3        = 0x11,
        MAG_CALIB_2_1        = 0x12,
        MAG_CALIB_2_2        = 0x13,
        MAG_CALIB_2_3        = 0x14,
        MAG_CALIB_3_1        = 0x15,
        MAG_CALIB_3_2        = 0x16,
        MAG_CALIB_3_3        = 0x17,
        MAG_BIAS_X           = 0x18,
        MAG_BIAS_Y           = 0x19,
        MAG_BIAS_Z           = 0x1A,
        ACCEL_CALIB_1_1      = 0x1B,
        ACCEL_CALIB_1_2      = 0x1C,
        ACCEL_CALIB_1_3      = 0x1D,
        ACCEL_CALIB_2_1      = 0x1E,
        ACCEL_CALIB_2_2      = 0x1F,
        ACCEL_CALIB_2_3      = 0x20,
        ACCEL_CALIB_3_1      = 0x21,
        ACCEL_CALIB_3_2      = 0x22,
        ACCEL_CALIB_3_3      = 0x23,
        ACCEL_BIAS_X         = 0x24,
        ACCEL_BIAS_Y         = 0x25,
        ACCEL_BIAS_Z         = 0x26,
        HEALTH               = 0x55,
        RAW_GYRO_X_Y         = 0x56,
        RAW_GYRO_Z           = 0x57,
        RAW_GYRO_TIME        = 0x58,
        RAW_ACCEL_X_Y        = 0x59,
        RAW_ACCEL_Z          = 0x5A,
        RAW_ACCEL_TIME       = 0x5B,
        RAW_MAG_X_Y          = 0x5C,
        RAW_MAG_Z            = 0x5D,
        RAW_MAG_TIME         = 0x5E,
        TEMP                 = 0x5F,
        TEMP_TIME            = 0x60,
        PROC_GYRO_X          = 0x61,
        PROC_GYRO_Y          = 0x62,
        PROC_GYRO_Z          = 0x63,
        PROC_GYRO_TIME       = 0x64,
        PROC_ACCEL_X         = 0x65,
        PROC_ACCEL_Y         = 0x66,
        PROC_ACCEL_Z         = 0x67,
        PROC_ACCEL_TIME      = 0x68,
        PROC_MAG_X           = 0x69,
        PROC_MAG_Y           = 0x6A,
        PROC_MAG_Z           = 0x6B,
        PROC_MAG_TIME        = 0x6C,
        QUAT_AB              = 0x6D,
        QUAT_CD              = 0x6E,
        QUAT_TIME            = 0x6F,
        EULER_PHI_THETA      = 0x70,
        EULER_PSI            = 0x71,
        EULER_PHI_THETA_RATE = 0x72,
        EULER_PSI_RATE       = 0x73,
        EULER_TIME           = 0x74,
        POS_NORTH            = 0x75,
        POS_EAST             = 0x76,
        POS_UP               = 0x77,
        POS_TIME             = 0x78,
        VEL_NORTH            = 0x79,
        VEL_EAST             = 0x7A,
        VEL_UP               = 0x7B,
        VEL_TIME             = 0x7C,
        GPS_NORTH            = 0x7D,
        GPS_EAST             = 0x7E,
        GPS_UP               = 0x7F,
        GPS_COURSE           = 0x80,
        GPS_SPEED            = 0x81,
        GPS_TIME             = 0x82,
        SAT_1_2              = 0x83,
        SAT_3_4              = 0x84,
        SAT_5_6              = 0x85,
        SAT_7_8              = 0x86,
        SAT_9_10             = 0x87,
        SAT_11_12            = 0x88,
        GYRO_BIAS_X          = 0x89,
        GYRO_BIAS_Y          = 0x8A,
        GYRO_BIAS_Z          = 0x8B
    };

    enum CommandRegister : uint8_t {
        FIRMWARE_VER    = 0xAA,
        COMMIT_CONFIG   = 0xAB,
        FACTORY_RESET   = 0xAC,
        ZERO_GYROS      = 0xAD,
        SET_HOME_POS    = 0xAE,
        SET_MAG_REF     = 0xB0,
        CALIBRATE_ACCEL = 0xB1,
        RESET_EKF       = 0xB3
    };

    private:
    HardwareSerial &_serial;

    struct Configuration {
        BaudRate       baudRate;
        BaudRate       gpsBaud;
        bool           transmitGps;
        bool           transmitSat;
        uint8_t        teleRawAccel;
        uint8_t        teleRawGyro;
        uint8_t        teleRawMag;
        uint8_t        teleTemp;
        uint8_t        teleRawAll;
        uint8_t        teleProcAccel;
        uint8_t        teleProcGyro;
        uint8_t        teleProcMag;
        uint8_t        teleProcAll;
        uint8_t        teleQuat;
        uint8_t        teleEuler;
        uint8_t        telePos;
        uint8_t        teleVel;
        uint8_t        telePose;
        HealthTeleRate teleHealth;
        uint8_t        teleGyroBias;
        TeleRate       teleNmeaHealth;
        TeleRate       teleNmeaPose;
        TeleRate       teleNmeaAttitude;
        TeleRate       teleNmeaSensor;
        TeleRate       teleNmeaGpsRate;
        TeleRate       teleNmeaGpsPose;
        TeleRate       teleNmeaQuat;
        bool           usePpsPin;
        bool           zeroGyroOnStart;
        bool           useQuat;
        bool           useMagInStateUpdates;

        Geographic<float> home;
        Cartesian<float>  gyroTrim;
        float             magCalib[3][3];
        Cartesian<float>  magBias;
        float             accelCalib[3][3];
        Cartesian<float>  accelBias;
    } _configuration;

    struct Readings {
        Health             health;
        Cartesian<int16_t> rawAccel;
        float              rawAccelTime;
        Cartesian<int16_t> rawGyro;
        float              rawGyroTime;
        Cartesian<int16_t> rawMag;
        float              rawMagTime;
        float              temp;
        float              tempTime;
        Cartesian<float>   procAccel;
        float              procAccelTime;
        Cartesian<float>   procGyro;
        float              procGyroTime;
        Cartesian<float>   procMag;
        float              procMagTime;
        Quaternion<float>  quat;
        float              quatTime;
        Euler<float>       euler;
        Euler<float>       eulerRate;
        float              eulerTime;
        Geographic<float>  pos;
        float              posTime;
        Geographic<float>  vel;
        float              velTime;
        GPS                gps;
        float              gpsTime;
        Satellite          sats[12];
        Cartesian<float>   gyroBias;
    } _readings;

    struct Packet {
        enum State {
            HEADER_S = 0,
            HEADER_N,
            HEADER_P,
            PACKET_TYPE,
            ADDRESS,
            DATA_BYTES,
            CHECKSUM_1,
            CHECKSUM_2
        } state;

        uint8_t          type;
        bool             hasData;
        bool             isBatch;
        uint8_t          batchLen;
        bool             hidden;
        bool             cmdFail;
        ReadableRegister address;
        uint8_t          content[16][4];
        uint16_t         checksum;

        uint8_t totalLen;
        uint8_t pos;
    } _packet;

    bool checkIncomingPacket();
    void processIncomingPacket();

    public:
    UM7(HardwareSerial &serial);

    HardwareSerial &getSerial();

    void parse(uint8_t in);
    void readSerial();

    void requestRead(ReadableRegister reg);
    void requestReadBatch(ReadableRegister first, ReadableRegister last);
    void executeCommand(CommandRegister command);

    BaudRate getBaudRate();
    void     setBaudRate(BaudRate rate);
    BaudRate getGpsBaud();
    void     setGpsBaud(BaudRate rate);
    uint8_t  getTransmitGps();
    void     setTransmitGps(bool val);
    uint8_t  getTransmitSat();
    void     setTransmitSat(bool val);
    void     commitCommSettings();

    uint8_t        getTeleRawAccel();
    void           setTeleRawAccel(uint8_t rate);
    uint8_t        getTeleRawGyro();
    void           setTeleRawGyro(uint8_t rate);
    uint8_t        getTeleRawMag();
    void           setTeleRawMag(uint8_t rate);
    uint8_t        getTeleTemp();
    void           setTeleTemp(uint8_t rate);
    uint8_t        getTeleRawAll();
    void           setTeleRawAll(uint8_t rate);
    uint8_t        getTeleProcAccel();
    void           setTeleProcAccel(uint8_t rate);
    uint8_t        getTeleProcGyro();
    void           setTeleProcGyro(uint8_t rate);
    uint8_t        getTeleProcMag();
    void           setTeleProcMag(uint8_t rate);
    uint8_t        getTeleProcAll();
    void           setTeleProcAll(uint8_t rate);
    uint8_t        getTeleQuat();
    void           setTeleQuat(uint8_t rate);
    uint8_t        getTeleEuler();
    void           setTeleEuler(uint8_t rate);
    uint8_t        getTelePos();
    void           setTelePos(uint8_t rate);
    uint8_t        getTeleVel();
    void           setTeleVel(uint8_t rate);
    uint8_t        getTelePose();
    void           setTelePose(uint8_t rate);
    HealthTeleRate getTeleHealth();
    void           setTeleHealth(HealthTeleRate rate);
    uint8_t        getTeleGyroBias();
    void           setTeleGyroBias(uint8_t rate);
    void           commitTeleSettings();

    TeleRate getTeleNmeaHealth();
    void     setTeleNmeaHealth(TeleRate rate);
    TeleRate getTeleNmeaPose();
    void     setTeleNmeaPose(TeleRate rate);
    TeleRate getTeleNmeaAttitude();
    void     setTeleNmeaAttitude(TeleRate rate);
    TeleRate getTeleNmeaSensor();
    void     setTeleNmeaSensor(TeleRate rate);
    TeleRate getTeleNmeaGpsRate();
    void     setTeleNmeaGpsRate(TeleRate rate);
    TeleRate getTeleNmeaGpsPose();
    void     setTeleNmeaGpsPose(TeleRate rate);
    TeleRate getTeleNmeaQuat();
    void     setTeleNmeaQuat(TeleRate rate);
    void     commitTeleNmeaSettings();

    bool getUsePpsPin();
    void setUsePpsPin(bool val);
    bool getZeroGyroOnStart();
    void setZeroGyroOnStart(bool val);
    bool getUseQuat();
    void setUseQuat(bool val);
    bool getUseMagInStateUpdates();
    void setUseMagInStateUpdates(bool val);
    void commitCommMiscSettings();

    Geographic<float> getHome();
    void              setHome(Geographic<float> val);
    float             getHomeNorth();
    void              setHomeNorth(float val);
    float             getHomeEast();
    void              setHomeEast(float val);
    float             getHomeUp();
    void              setHomeUp(float val);
    void              commitHomeSettings();

    Cartesian<float> getGyroTrim();
    void             setGyroTrim(Cartesian<float> val);
    float            getGyroTrimX();
    void             setGyroTrimX(float val);
    float            getGyroTrimY();
    void             setGyroTrimY(float val);
    float            getGyroTrimZ();
    void             setGyroTrimZ(float val);
    void             commitGyroTrimSettings();

    float getMagCalib(uint8_t x, uint8_t y);
    void  setMagCalib(uint8_t x, uint8_t y, uint8_t val);
    void  commitMagCalibSettings();

    Cartesian<float> getMagBias();
    void             setMagBias(Cartesian<float> val);
    float            getMagBiasX();
    void             setMagBiasX(float val);
    float            getMagBiasY();
    void             setMagBiasY(float val);
    float            getMagBiasZ();
    void             setMagBiasZ(float val);
    void             commitMagBiasSettings();

    float getAccelCalib(uint8_t x, uint8_t y);
    void  setAccelCalib(uint8_t x, uint8_t y, uint8_t val);
    void  commitAccelCalibSettings();

    Cartesian<float> getAccelBias();
    void             setAccelBias(Cartesian<float> val);
    float            getAccelBiasX();
    void             setAccelBiasX(float val);
    float            getAccelBiasY();
    void             setAccelBiasY(float val);
    float            getAccelBiasZ();
    void             setAccelBiasZ(float val);
    void             commitAccelBiasSettings();

    Health  getHealth();
    uint8_t getHealthSatsUsed();
    float   getHealthHdop();
    uint8_t getHealthSatsInView();
    bool    getHealthOverflow();
    bool    getHealthBadAccelNorm();
    bool    getHealthBadMagNorm();
    bool    getHealthBadAccel();
    bool    getHealthBadGyro();
    bool    getHealthBadMag();
    bool    getHealthBadGps();

    Cartesian<int16_t> getRawAccel();
    int16_t            getRawAccelX();
    int16_t            getRawAccelY();
    int16_t            getRawAccelZ();
    float              getRawAccelTime();

    Cartesian<int16_t> getRawGyro();
    int16_t            getRawGyroX();
    int16_t            getRawGyroY();
    int16_t            getRawGyroZ();
    float              getRawGyroTime();

    Cartesian<int16_t> getRawMag();
    int16_t            getRawMagX();
    int16_t            getRawMagY();
    int16_t            getRawMagZ();
    float              getRawMagTime();

    float getTemp();
    float getTempTime();

    Cartesian<float> getProcAccel();
    float            getProcAccelX();
    float            getProcAccelY();
    float            getProcAccelZ();
    float            getProcAccelTime();

    Cartesian<float> getProcGyro();
    float            getProcGyroX();
    float            getProcGyroY();
    float            getProcGyroZ();
    float            getProcGyroTime();

    Cartesian<float> getProcMag();
    float            getProcMagX();
    float            getProcMagY();
    float            getProcMagZ();
    float            getProcMagTime();

    Quaternion<float> getQuat();
    float             getQuatA();
    float             getQuatB();
    float             getQuatC();
    float             getQuatD();
    float             getQuatTime();

    Euler<float> getEuler();
    float        getEulerPhi();
    float        getEulerTheta();
    float        getEulerPsi();
    Euler<float> getEulerRate();
    float        getEulerRatePhi();
    float        getEulerRateTheta();
    float        getEulerRatePsi();
    float        getEulerTime();

    Geographic<float> getPos();
    float             getPosNorth();
    float             getPosEast();
    float             getPosUp();
    float             getPosTime();

    Geographic<float> getVel();
    float             getVelNorth();
    float             getVelEast();
    float             getVelUp();
    float             getVelTime();

    GPS               getGps();
    Geographic<float> getGpsPos();
    float             getGpsPosNorth();
    float             getGpsPosEast();
    float             getGpsPosUp();
    float             getGpsCourse();
    float             getGpsSpeed();
    float             getGpsTime();

    Satellite getSatellite(uint8_t index);
    uint8_t   getSatelliteId(uint8_t index);
    uint8_t   getSatelliteSnRatio(uint8_t index);

    Cartesian<float> getGyroBias();
    float            getGyroBiasX();
    float            getGyroBiasY();
    float            getGyroBiasZ();
};

#endif
