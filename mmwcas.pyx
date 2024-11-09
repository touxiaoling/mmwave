#from libc.stdio cimport printf as DEBUG_PRINT
from libc.stdio cimport printf
from libc.stdint cimport uint8_t, int8_t,int16_t,uint16_t, int32_t, uint32_t
from libc.math cimport ceil

cdef extern from "ti/mmwave/mmwave.h":
    '''
    FILE* rls_traceF = NULL;
    void CloseTraceFile() {
    if (rls_traceF != NULL) {
        fclose(rls_traceF);
        rls_traceF = NULL;
    }
    }
    '''
    ctypedef struct rlProfileCfg_t:
        uint16_t profileId
        uint8_t pfVcoSelect
        uint8_t pfCalLutUpdate
        uint32_t startFreqConst
        uint32_t idleTimeConst
        uint32_t adcStartTimeConst
        uint32_t rampEndTime
        uint32_t txOutPowerBackoffCode
        uint32_t txPhaseShifter
        int16_t freqSlopeConst
        int16_t txStartTime
        uint16_t numAdcSamples
        uint16_t digOutSampleRate
        uint8_t hpfCornerFreq1
        uint8_t hpfCornerFreq2
        uint16_t txCalibEnCfg
        uint16_t rxGain
        uint16_t reserved

    ctypedef struct rlFrameCfg_t:
        uint16_t reserved0
        uint16_t chirpStartIdx
        uint16_t chirpEndIdx
        uint16_t numLoops
        uint16_t numFrames
        uint16_t numAdcSamples
        uint32_t framePeriodicity
        uint16_t triggerSelect
        uint16_t reserved1
        uint32_t frameTriggerDelay

    ctypedef struct rlChirpCfg_t:
        uint16_t chirpStartIdx
        uint16_t chirpEndIdx
        uint16_t profileId
        uint16_t reserved
        uint32_t startFreqVar
        uint16_t freqSlopeVar
        uint16_t idleTimeVar
        uint16_t adcStartTimeVar
        uint16_t txEnable
    
    ctypedef struct rlChanCfg_t:
        uint16_t rxChannelEn
        uint16_t txChannelEn
        uint16_t cascading
        uint16_t cascadingPinoutCfg
    
    ctypedef struct rlAdcBitFormat_t:
        uint32_t b2AdcBits
        uint32_t b6Reserved0
        uint32_t b8FullScaleReducFctr
        uint32_t b2AdcOutFmt
        uint32_t b14Reserved1

    ctypedef struct rlAdcOutCfg_t:
        rlAdcBitFormat_t fmt
        uint16_t reserved0
        uint16_t reserved1

    ctypedef struct rlDevDataFmtCfg_t:
        uint16_t rxChannelEn
        uint16_t adcBits
        uint16_t adcFmt
        uint8_t iqSwapSel
        uint8_t chInterleave
        uint32_t reserved
    
    ctypedef struct rlRfLdoBypassCfg_t:
        uint16_t ldoBypassEnable
        uint8_t supplyMonIrDrop
        uint8_t ioSupplyIndicator
    
    ctypedef struct rlLowPowerModeCfg_t:
        uint16_t reserved
        uint16_t lpAdcMode
    
    ctypedef struct rlRfMiscConf_t:
        uint32_t miscCtl
        uint32_t reserved
    
    ctypedef struct rlDevDataPathCfg_t:
        uint8_t intfSel
        uint8_t transferFmtPkt0
        uint8_t transferFmtPkt1
        uint8_t cqConfig
        uint8_t cq0TransSize
        uint8_t cq1TransSize
        uint8_t reserved

    ctypedef struct rlDevDataPathClkCfg_t:
        uint8_t laneClkCfg
        uint8_t dataRate
        uint16_t reserved

    ctypedef struct rlDevHsiClk_t:
        uint16_t hsiClk
        uint16_t reserved

    ctypedef struct rlDevCsi2Cfg_t:
        uint32_t lanePosPolSel
        uint8_t lineStartEndDis
        uint8_t reserved0
        uint16_t reserved0
    
    ctypedef struct rlTdaArmCfg_t:
        unsigned int framePeriodicity
        unsigned char* captureDirectory
        unsigned int numberOfFilesToAllocate
        unsigned int dataPacking
        unsigned int numberOfFramesToCapture

    int MMWL_chirpConfig(unsigned char deviceMap, rlChirpCfg_t chirpCfgArgs)
    unsigned int createDevMapFromDevId(unsigned char devId)
    int MMWL_DevicePowerUp(unsigned char deviceMap, uint32_t rlClientCbsTimeout, uint32_t sopTimeout)
    int MMWL_firmwareDownload(unsigned char deviceMap)
    int MMWL_setDeviceCrcType(unsigned char deviceMap)
    int MMWL_rfEnable(unsigned char deviceMap)
    int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade, rlChanCfg_t rfChanCfgArgs)
    int MMWL_adcOutConfig(unsigned char deviceMap, rlAdcOutCfg_t adcOutCfgArgs)
    int MMWL_RFDeviceConfig(unsigned char deviceMap)
    int MMWL_ldoBypassConfig(unsigned char deviceMap, rlRfLdoBypassCfg_t rfLdoBypassCfgArgs)
    int MMWL_dataFmtConfig(unsigned char deviceMap, rlDevDataFmtCfg_t dataFmtCfgArgs)
    int MMWL_lowPowerConfig(unsigned char deviceMap, rlLowPowerModeCfg_t rfLpModeCfgArgs)
    int MMWL_ApllSynthBwConfig(unsigned char deviceMap)
    int MMWL_setMiscConfig(unsigned char deviceMap, rlRfMiscConf_t miscCfg)
    int MMWL_rfInit(unsigned char deviceMap)
    int MMWL_dataPathConfig(unsigned char deviceMap, rlDevDataPathCfg_t datapathCfgArgs)
    int MMWL_hsiClockConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t datapathClkCfgArgs, rlDevHsiClk_t hisClkgs)
    int MMWL_CSI2LaneConfig(unsigned char deviceMap, rlDevCsi2Cfg_t CSI2LaneCfgArgs)
    int MMWL_profileConfig(unsigned char deviceMap, rlProfileCfg_t profileCfgArgs)
    int MMWL_frameConfig(unsigned char deviceMap, rlFrameCfg_t frameCfgArgs, rlChanCfg_t channelCfgArgs, rlAdcOutCfg_t adcOutCfgArgs, rlDevDataPathCfg_t datapathCfgArgs, rlProfileCfg_t profileCfgArgs)
    int MMWL_AssignDeviceMap(unsigned char deviceMap,uint8_t* masterMap,uint8_t* slavesMap)
    int MMWL_ArmingTDA(rlTdaArmCfg_t tdaArmCfgArgs)
    int MMWL_StartFrame(unsigned char deviceMap)
    int MMWL_StopFrame(unsigned char deviceMap)
    int MMWL_DeArmingTDA()
    int MMWL_TDAInit(unsigned char *ipAddr , unsigned int port,uint8_t deviceMap)




# 定义程序名称、版本等常量
PROG_NAME = "mmwcas"             # Name of the program
PROG_VERSION = "0.1"             # Program version
PROG_COPYRIGHT = "Copyright (C) 2024"
#DEBUG_PRINT = printf             # Debug print function

RL_RET_CODE_OK = 0               # Return code for success

# 开发环境标志和其他常量
DEV_ENV = 1
NUM_CHIRPS = 12

CRED=b"\e[0;31m"    # Terminal code for regular red text
CGREEN=b"\e[0;32m"    # Terminal code for regular greed text
CRESET=b"\e[0m"       # Clear reset terminal color

TRUE = 1


# 设备配置结构体
ctypedef struct devConfig_t:
    uint8_t deviceMap         # Device Map (1: Master, 2: Slave1, 4: Slave2, 8: Slave3)
    uint8_t masterMap         # Master device map (value: 1)
    uint8_t slavesMap         # Slave devices map (value: 14)

    rlFrameCfg_t frameCfg

    # Profile configuration
    rlProfileCfg_t profileCfg

    # Chirp configuration
    rlChirpCfg_t chirpCfg

    # Channel configuration
    rlChanCfg_t channelCfg

    # ADC output configuration
    rlAdcOutCfg_t adcOutCfg

    # Data format configuration
    rlDevDataFmtCfg_t dataFmtCfg

    # LDO Bypass configuration
    rlRfLdoBypassCfg_t ldoCfg

    # Low Power Mode configuration
    rlLowPowerModeCfg_t lpmCfg

    # Miscellaneous configuration
    rlRfMiscConf_t miscCfg

    # Datapath configuration
    rlDevDataPathCfg_t datapathCfg

    # Datapath clock configuration
    rlDevDataPathClkCfg_t datapathClkCfg

    # High Speed Clock configuration
    rlDevHsiClk_t hsClkCfg

    # CSI2 configuration
    rlDevCsi2Cfg_t csi2LaneCfg


cdef rlProfileCfg_t profileCfgArgs=rlProfileCfg_t(
    profileId = 0,
    pfVcoSelect = 0x02,
    startFreqConst = 1435384036,   # 77GHz | 1 LSB = 53.644 Hz
    freqSlopeConst = 311,          # 15.0148 Mhz/us | 1LSB = 48.279 kHz/uS
    idleTimeConst = 500,           # 5us  | 1LSB = 10ns
    adcStartTimeConst = 600,       # 6us  | 1LSB = 10ns
    rampEndTime = 4000,            # 40us | 1LSB = 10ns
    txOutPowerBackoffCode = 0x0,
    txPhaseShifter = 0x0,
    txStartTime = 0x0,             # 0us | 1LSB = 10ns
    numAdcSamples = 256,           # 256 ADC samples per chirp
    digOutSampleRate = 8000,      # 8000 ksps (8 MHz) | 1LSB = 1 ksps
    hpfCornerFreq1 = 0x0,          # 175kHz
    hpfCornerFreq2 = 0x0,          # 350kHz
    rxGain = 48,                   # 48 dB | 1LSB = 1dB
)

cdef rlFrameCfg_t frameCfgArgs=rlFrameCfg_t(
    chirpStartIdx = 0,
    chirpEndIdx = 11,
    numFrames = 0,                 # (0 for infinite)
    numLoops = 16,
    numAdcSamples = 2 * 256,       # Complex samples (for I and Q siganls)
    frameTriggerDelay = 0x0,
    framePeriodicity = 20000000,   # 100ms | 1LSB = 5ns
)

cdef rlChirpCfg_t chirpCfgArgs = rlChirpCfg_t(
    chirpStartIdx = 0,
    chirpEndIdx = 0,
    profileId = 0,
    txEnable = 0x00,
    adcStartTimeVar = 0,
    idleTimeVar = 0,
    startFreqVar = 0,
    freqSlopeVar = 0,
)

# Channel config */
cdef rlChanCfg_t channelCfgArgs = rlChanCfg_t(
    rxChannelEn = 0x0F,      # Enable all 4 RX Channels
    txChannelEn = 0x07,      # Enable all 3 TX Channels
    cascading = 0x02,        # Slave
)

cdef rlAdcBitFormat_t adcBitFmtArgs = rlAdcBitFormat_t(
    b2AdcBits = 2,           # 16-bit ADC
    b2AdcOutFmt = 1,         # Complex values
    b8FullScaleReducFctr = 0,
)
# ADC output config */
cdef rlAdcOutCfg_t adcOutCfgArgs = rlAdcOutCfg_t(
    fmt = adcBitFmtArgs,
)

# Data format config */
cdef rlDevDataFmtCfg_t dataFmtCfgArgs = rlDevDataFmtCfg_t(
    iqSwapSel = 0,           # I first
    chInterleave = 0,        # Interleaved mode
    rxChannelEn = 0xF,       # All RX antenna enabled
    adcFmt = 1,              # Complex
    adcBits = 2,             # 16-bit ADC
)

# LDO Bypass config */
cdef rlRfLdoBypassCfg_t ldoCfgArgs = rlRfLdoBypassCfg_t(
    ldoBypassEnable = 3,       # RF LDO disabled, PA LDO disabled
    ioSupplyIndicator = 0,
    supplyMonIrDrop = 0,
)

# Low Power Mode config */
cdef rlLowPowerModeCfg_t lpmCfgArgs = rlLowPowerModeCfg_t(
    lpAdcMode = 0,             # Regular ADC power mode
)

# Miscellaneous config */
cdef rlRfMiscConf_t miscCfgArgs = rlRfMiscConf_t(
    miscCtl = 1,               # Enable Per chirp phase shifter
)

# Datapath config */
cdef rlDevDataPathCfg_t datapathCfgArgs = rlDevDataPathCfg_t(
    intfSel = 0,               # CSI2 intrface
    transferFmtPkt0 = 1,       # ADC data only
    transferFmtPkt1 = 0,       # Suppress packet 1
)

# Datapath clock config */
cdef rlDevDataPathClkCfg_t datapathClkCfgArgs = rlDevDataPathClkCfg_t(
    laneClkCfg = 1,            # DDR Clock
    dataRate = 1,              # 600Mbps
)

# High speed clock config */
cdef rlDevHsiClk_t hsClkCfgArgs = rlDevHsiClk_t(
    hsiClk = 0x09,             # DDR 600Mbps
)

# CSI2 config */
cdef rlDevCsi2Cfg_t csi2LaneCfgArgs = rlDevCsi2Cfg_t(
    lineStartEndDis = 0,       # Enable
    lanePosPolSel = 0x35421,   # 0b 0011 0101 0100 0010 0001,
)

"""
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|       | Dev 1 | Dev 1 | Dev 1 | Dev 2 | Dev 2 | Dev 2 | Dev 3 | Dev 3 | Dev 3 | Dev 4 | Dev 4 | Dev 4 |
| Chirp |  TX0  |  TX1  |  TX2  |  TX 0 |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |
|     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |
|     2 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |
|     3 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |
|     4 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |
|     5 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |
|     6 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |
|     7 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     8 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     9 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    10 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    11 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
"""


cdef int8_t is_in_table(uint8_t value, uint8_t[:] table, uint8_t size):
    '''@brief Check if a value is in the table provided in argument
    #* @param value Value to look for in the table
    #* @param table Table defining the search context
    #* @param size Size of the table
    #* @return int8_t
    #* Return the index where the match has been found. -1 if not found
    '''
    cdef uint8_t i
    for i in range(size):
        if table[i] == value:
            return i
    return -1


cpdef uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t chirpCfg):
    """@brief MIMO Chirp configuration
    #* @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
    #* @param chirpCfg Initital chirp configuration
    #* @return uint32_t Configuration status
    """
    # 定义设备的 Tx 表
    cdef uint8_t[4][3] chripTxTable=[[11,10,9],[8,7,6],[5,4,3],[2,1,0]]
    
    # 定义状态变量
    cdef int status = 0
    cdef uint8_t i
    cdef int8_t txIdx
    
    for i in range(NUM_CHIRPS):
        txIdx = is_in_table(i, chripTxTable[devId], 3)

        # 更新 chirp 配置
        chirpCfg.chirpStartIdx = i
        chirpCfg.chirpEndIdx = i
        if txIdx < 0:
            chirpCfg.txEnable = 0x00
        else:
            chirpCfg.txEnable = (1 << txIdx)

        # 配置 chirp 并更新状态
        status += MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg)

        # 打印调试信息
        printf(b"[CHIRP CONFIG] dev %u, chirp idx %u, status: %d\n", devId, i, status)
        if status != 0:
            printf(b"Configuration of chirp %d failed!\n", i)
            break

    return status

cdef void check(int status, char* success_msg, char* error_msg,
                unsigned char deviceMap, uint8_t is_required):
    """@brief Check status and print error or success message
    @param status Status value returned by a function
    @param success_msg Success message to print when status is 0
    @param error_msg Error message to print in case of error
    @param deviceMap Device map the check if related to
    @param is_required Indicates if the checking stage is required. if so,the program exits in case of failure.
    @return uint32_t Configuration status

    @note: Status is considered successful when the status integer is 0.
    Any other value is considered a failure.
    """
    # 模拟 DEV_ENV 环境下的调试信息
    if DEV_ENV:
        printf(b"STATUS %4d | DEV MAP: %2u | ", status, deviceMap)

    # 检查状态
    if status == RL_RET_CODE_OK:
        if DEV_ENV:
            printf(CGREEN)
            printf(success_msg)
            printf(CRESET)
            printf("\n")
        return
    else:
        if DEV_ENV:
            printf(CRED)
            printf(error_msg)
            printf(CRESET)
            printf("\n")
        
        # 如果 is_required 为非零，则退出程序
        if is_required != 0:
            exit(status)


cdef int32_t initMaster(rlChanCfg_t channelCfg,rlAdcOutCfg_t adcOutCfg):
    cdef unsigned int masterId = 0
    cdef unsigned int masterMap = 1 << masterId
    cdef int status = 0
    channelCfg.cascading = 1
    status += MMWL_DevicePowerUp(masterMap, 1000, 1000)
    check(status,
        b"[MASTER] Power up successful!",
        b"[MASTER] Error: Failed to power up device!", masterMap, TRUE)

    status += MMWL_firmwareDownload(masterMap)
    check(status,
        b"[MASTER] Firmware successfully uploaded!",
        b"[MASTER] Error: Firmware upload failed!", masterMap, TRUE)

    status += MMWL_setDeviceCrcType(masterMap)
    check(status,
        b"[MASTER] CRC type has been set!",
        b"[MASTER] Error: Unable to set CRC type!", masterMap, TRUE)

    status += MMWL_rfEnable(masterMap)
    check(status,
        b"[MASTER] RF successfully enabled!",
        b"[MASTER] Error: Failed to enable master RF", masterMap, TRUE)

    status += MMWL_channelConfig(masterMap, channelCfg.cascading, channelCfg)
    check(status,
        b"[MASTER] Channels successfully configured!",
        b"[MASTER] Error: Channels configuration failed!", masterMap, TRUE)

    status += MMWL_adcOutConfig(masterMap, adcOutCfg)
    check(status,
        b"[MASTER] ADC output format successfully configured!",
        b"[MASTER] Error: ADC output format configuration failed!", masterMap, TRUE)

    check(status,
        b"[MASTER] Init completed with sucess",
        b"[MASTER] Init completed with error", masterMap, TRUE)
    return status

cdef int32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg):
    cdef int status = 0
    cdef uint8_t slavesMap = (1 << 1) | (1 << 2) | (1 << 3)
    cdef unsigned int slaveMap

    # slave chip
    channelCfg.cascading = 2

    for slaveId in range(1,4):
        slaveMap = 1 << slaveId

        status += MMWL_DevicePowerUp(slaveMap, 1000, 1000)
        check(status,
            b"[SLAVE] Power up successful!",
            b"[SLAVE] Error: Failed to power up device!", slaveMap, TRUE)

    #Config of all slaves together
    status += MMWL_firmwareDownload(slavesMap)
    check(status,
        b"[SLAVE] Firmware successfully uploaded!",
        b"[SLAVE] Error: Firmware upload failed!", slavesMap, TRUE)

    status += MMWL_setDeviceCrcType(slavesMap)
    check(status,
        b"[SLAVE] CRC type has been set!",
        b"[SLAVE] Error: Unable to set CRC type!", slavesMap, TRUE)

    status += MMWL_rfEnable(slavesMap)
    check(status,
        b"[SLAVE] RF successfully enabled!",
        b"[SLAVE] Error: Failed to enable master RF", slavesMap, TRUE)

    status += MMWL_channelConfig(slavesMap, channelCfg.cascading,channelCfg)
    check(status,
        b"[SLAVE] Channels successfully configured!",
        b"[SLAVE] Error: Channels configuration failed!", slavesMap, TRUE)

    status += MMWL_adcOutConfig(slavesMap, adcOutCfg)
    check(status,
        b"[SLAVE] ADC output format successfully configured!",
        b"[SLAVE] Error: ADC output format configuration failed!", slavesMap, TRUE)

    check(status,
        b"[SLAVE] Init completed with sucess",
        b"[SLAVE] Init completed with error", slavesMap, TRUE)
    return status

cdef uint32_t configure (devConfig_t config):
    cdef int status = 0
    cdef int devId = 0
    status += initMaster(config.channelCfg, config.adcOutCfg)
    status += initSlaves(config.channelCfg, config.adcOutCfg)

    status += MMWL_RFDeviceConfig(config.deviceMap)
    check(status,
        b"[ALL] RF deivce configured!",
        b"[ALL] RF device configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg)
    check(status,
        b"[ALL] LDO Bypass configuration successful!",
        b"[ALL] LDO Bypass configuration failed!", config.deviceMap, TRUE)

    status += MMWL_dataFmtConfig(config.deviceMap, config.dataFmtCfg)
    check(status,
        b"[ALL] Data format configuration successful!",
        b"[ALL] Data format configuration failed!", config.deviceMap, TRUE)

    status += MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg)
    check(status,
        b"[ALL] Low Power Mode configuration successful!",
        b"[ALL] Low Power Mode configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ApllSynthBwConfig(config.deviceMap)
    status += MMWL_setMiscConfig(config.deviceMap, config.miscCfg)
    status += MMWL_rfInit(config.deviceMap)
    check(status,
        b"[ALL] RF successfully initialized!",
        b"[ALL] RF init failed!", config.deviceMap, TRUE)

    status += MMWL_dataPathConfig(config.deviceMap, config.datapathCfg)
    status += MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg)
    status += MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg)
    check(status,
        b"[ALL] Datapath configuration successful!",
        b"[ALL] Datapath configuration failed!", config.deviceMap, TRUE)

    status += MMWL_profileConfig(config.deviceMap, config.profileCfg)
    check(status,
        b"[ALL] Profile configuration successful!",
        b"[ALL] Profile configuration failed!", config.deviceMap, TRUE)

    # MIMO Chirp configuration
    for devId in range(4):
        status += configureMimoChirp(devId, config.chirpCfg)

    check(status,
        b"[ALL] Chirp configuration successful!",
        b"[ALL] Chirp configuration failed!", config.deviceMap, TRUE)

    #Master frame config.
    status += MMWL_frameConfig(
        config.masterMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,
        config.datapathCfg,
        config.profileCfg
    )
    check(status,
        b"[MASTER] Frame configuration completed!",
        b"[MASTER] Frame configuration failed!", config.masterMap, TRUE)

    #Slaves frame config
    status += MMWL_frameConfig(
        config.slavesMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,  
        config.datapathCfg,
        config.profileCfg
    )
    check(status,
        b"[SLAVE] Frame configuration completed!",
        b"[SLAVE] Frame configuration failed!", config.slavesMap, TRUE)

    check(status,
        b"[MIMO] Configuration completed!",
        b"[MIMO] Configuration completed with error!", config.deviceMap, TRUE)

    return status

cdef devConfig_t config

cpdef mmw_set_config(dict configdict):
    global config
    config.deviceMap = 1|(1<<1)|(1<<2)|(1<<3)
    MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap)
    config.frameCfg = frameCfgArgs
    config.profileCfg = profileCfgArgs
    config.chirpCfg = chirpCfgArgs
    config.channelCfg = channelCfgArgs
    config.csi2LaneCfg = csi2LaneCfgArgs
    config.datapathCfg = datapathCfgArgs
    config.datapathClkCfg=datapathClkCfgArgs
    config.hsClkCfg = hsClkCfgArgs
    config.ldoCfg = ldoCfgArgs
    config.lpmCfg = lpmCfgArgs
    config.miscCfg = miscCfgArgs

    cdef dict mimo,profile,frame,channel
    if "mimo" in configdict:
        mimo = configdict["mimo"]
        if "profile" in mimo: # [PROFILE CONFIGURATION]
            profile = mimo["profile"]
            if "id" in profile:
                config.profileCfg.profileId = <uint16_t>(profile["id"])
            if "startFrequency" in profile: # Chirp start frequency in GHz
                config.profileCfg.startFreqConst = <uint32_t>(ceil(profile["startFrequency"]*1e9/53.644)) # 1LSB = 53.644 Hz
            if "frequencySlope" in profile: # Frequency slope in MHz/us
                config.profileCfg.freqSlopeConst = <int16_t>(ceil(profile["frequencySlope"]*1e3/48.279)) # 1LSB = 48.279 kHz/us
            if "idleTime" in profile:# Chrip Idle time in us
                config.profileCfg.idleTimeConst = <uint32_t>(ceil(profile["idleTime"]*1e2)) # 1LSB = 10ns
            if "adcStartTime" in profile:# ADC start time in us
                config.profileCfg.adcStartTimeConst = <uint32_t>(ceil(profile["adcStartTime"]*1e2)) # 1LSB = 10ns
            if "rampEndTime" in profile:# Chirp ramp end time in us
                config.profileCfg.rampEndTime = <uint32_t>(ceil(profile["rampEndTime"]*1e2)) # 1LSB = 10ns
            if "txStartTIme" in profile:# TX starttime in us
                config.profileCfg.txStartTime = <uint16_t>(ceil(profile["txStartTIme"]*1e2)) # 1LSB = 10ns
            if "numAdcSamples" in profile:# Number of ADC samples per chirp
                config.profileCfg.numAdcSamples = <uint16_t>(profile["numAdcSamples"])
            if "adcSamplingFrequency" in profile:# ADC sampling frequency in ksps
                config.profileCfg.digOutSampleRate = <uint16_t>(profile["adcSamplingFrequency"])
            if "rxGain" in profile:# rxGain in dB
                config.profileCfg.rxGain = <uint16_t>(profile["rxGain"])
            if "hpfCornerFreq1" in profile: # hpfCornerFreq1
                config.profileCfg.hpfCornerFreq1 = <uint8_t>(profile["hpfCornerFreq1"])
            if "hpfCornerFreq2" in profile: # hpfCornerFreq2
                config.profileCfg.hpfCornerFreq2 = <uint8_t>(profile["hpfCornerFreq2"])
            
        if "frame" in mimo: # [FRAME CONFIGURATION]
            frame = mimo["frame"]
            if "numFrames" in frame: # Number of frames to record
                config.frameCfg.numFrames = <uint16_t>(frame["numFrames"])
            if "numLoops" in frame: # Number of chirp loop per frame
                config.frameCfg.numLoops = <uint16_t>(frame["numLoops"])
            if "framePeriodicity" in frame: # Frame periodicity in ms
                config.frameCfg.framePeriodicity = <uint32_t>(ceil(frame["framePeriodicity"]*5e-6)) # 1LSB = 5ns
        if "channel" in mimo:# [CHANNEL CONFIGURATION]
            channel = mimo["channel"]
            if "rxChannelEn" in channel: # RX Channel configuration
                config.channelCfg.rxChannelEn = <uint16_t>(channel["rxChannelEn"])
            if "txChannelEn" in channel: # TX Channel configuration
                config.channelCfg.txChannelEn = <uint16_t>(channel["txChannelEn"])
        config.frameCfg.numAdcSamples = 2 * config.profileCfg.numAdcSamples
        config.dataFmtCfg.rxChannelEn = config.channelCfg.rxChannelEn
        
    config.dataFmtCfg.rxChannelEn = channelCfgArgs.rxChannelEn
    config.dataFmtCfg.adcBits = adcOutCfgArgs.fmt.b2AdcBits
    config.dataFmtCfg.adcFmt = adcOutCfgArgs.fmt.b2AdcOutFmt
    return 0

cpdef int mmw_init(
    str ip_addr="192.168.33.180",
    int port = 5001,
    ):
    cdef int status = 0
    cdef bytes ip_addr_bytes = ip_addr.encode('utf-8')
    status = MMWL_TDAInit(ip_addr_bytes,port,config.deviceMap)
    check(status,
        b"[ALL] TDA Init successful!",
        b"[ALL] TDA Init failed!", config.deviceMap, TRUE)

    configure(config) 
    return status

cpdef int mmw_arming_tda(str capture_path):
    cdef int status = 0
    cdef bytes capture_path_bytes = capture_path.encode('utf-8')
    cdef rlTdaArmCfg_t tdaCfg = rlTdaArmCfg_t(
        captureDirectory = capture_path_bytes,
        framePeriodicity = (frameCfgArgs.framePeriodicity * 5)//(1000 * 1000),
        numberOfFilesToAllocate = 0,
        numberOfFramesToCapture = 0, # config.frameCfg.numFrames,
        dataPacking = 0, # 0: 16-bit | 1: 12-bit
    )
    status = MMWL_ArmingTDA(tdaCfg)
    check(status,
        b"[ALL] TDA Arming successful!",
        b"[ALL] TDA Arming failed!", config.deviceMap, TRUE)
    return status

cpdef int mmw_start_frame():
    cdef int status = 0
    cdef int i
    for i in range(3,-1,-1):
        status += MMWL_StartFrame(1<<i)
        check(status,
            b"[ALL] RF successfully enabled!",
            b"[ALL] Error: Failed to enable master RF", 1<<i, TRUE)
    return status

cpdef int mmw_stop_frame():
    cdef int status = 0
    cdef int i
    for i in range(3,-1,-1):
        status += MMWL_StopFrame(1<<i)
        check(status, 
            b"[ALL] RF successfully disenabled!",
            b"[ALL] Error: Failed to disenable master RF", 1<<i, TRUE)
    return status

cpdef int mmw_dearming_tda():
    cdef int status = 0
    status = MMWL_DeArmingTDA()
    check(status,
      b"[MMWCAS-RF] Stop recording",
      b"[MMWCAS-RF] Failed to de-arm TDA board!", 32, TRUE)
    return status
