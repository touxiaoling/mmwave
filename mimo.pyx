from libc cimport string
from libc.stdio cimport printf,fprintf
from libc.stdint cimport uint8_t, int8_t,int16_t,uint16_t, int32_t, uint32_t
from libc.math cimport ceil
from libc.time cimport usleep

cdef extern from "ti/mmwave/mmwave.h":
    int MML_TDAInit(unsigned char *ipAddr , unsigned int port,uint8_t deviceMap)
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
        uint32_t acdBitFromat

    ctypedef struct rlAdcOutCfg_t:
        rlAdcBitFormat_t fmt
        uint16_t reserved0
        uint16_t reserved1

    ctypedef struct rlDevDataFmtCfg_t:
        uint16_t rxChannelEn
        uint16_t adcBits
        uint16_t adcFmt
        uint8_t iqSwapSel
        uint8_t chInterLeave
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
        uint8_t lanClkCfg
        uint8_t datarate
        uint16_t reserved

    ctypedef struct rlDevHsiClk_t:
        uint16_t hsiClk
        uint16_t reserved

    ctypedef struct rlDevCsi2Cfg_t:
        uint32_t lanPosPolSel
        uint8_t lineStartEndDis
        uint8_t reserved0
        uint16_t reserved1
    
    ctypedef struct rlTdaArmCfg_t:
        unsigned int framePeriodicity
        unsigned char* captureDirectory
        unsigned int numberOfFilesToAllocate
        unsigned int dataPacking
        unsigned int numberOfFramesToCapture

    cdef extern int MMWL_chirpConfig(unsigned char deviceMap, rlChirpCfg_t chirpCfgArgs)
    cdef extern unsigned int createDevMapFromDevId(unsigned char devId)
    cdef extern int MMWL_DevicePowerUp(unsigned char deviceMap, uint32_t rlClientCbsTimeout, uint32_t sopTimeout)
    cdef extern int MMWL_firmwareDownload(unsigned char deviceMap)
    cdef extern int MMWL_setDeviceCrcType(unsigned char deviceMap)
    cdef extern int MMWL_rfEnable(unsigned char deviceMap)
    cdef extern int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade, rlChanCfg_t rfChanCfgArgs)
    cdef extern int MMWL_adcOutConfig(unsigned char deviceMap, rlRfMiscConf_t miscCfg)
    cdef extern int MMWL_RFDeviceConfig(unsigned char deviceMap)
    cdef extern int MMWL_ldoBypassConfig(unsigned char deviceMap, rlRfLdoBypassCfg_t rfLdoBypassCfgArgs)
    cdef extern int MMWL_dataFmtConfig(unsigned char deviceMap, rlDevDataFmtCfg_t dataFmtCfgArgs)
    cdef extern int MMWL_lowPowerConfig(unsigned char deviceMap, rlLowPowerModeCfg_t rfLpModeCfgArgs)
    cdef extern int MMWL_ApllSynthBwConfig(unsigned char deviceMap)
    cdef extern int MMWL_setMiscConfig(unsigned char deviceMap, rlRfMiscConf_t miscCfg)
    cdef extern int MMWL_rfInit(unsigned char deviceMap)
    cdef extern int MMWL_dataPathConfig(unsigned char deviceMap, rlDevDataPathCfg_t datapathCfgArgs)
    cdef extern int MMWL_hsiClockConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t datapathClkCfgArgs, rlDevHsiClk_t hisClkgs)
    cdef extern int MMWL_CSI2LaneConfig(unsigned char deviceMap, rlDevCsi2Cfg_t CSI2LaneCfgArgs)
    cdef extern int MMWL_profileConfig(unsigned char deviceMap, rlProfileCfg_t profileCfgArgs)
    cdef extern int MMWL_frameConfig(unsigned char deviceMap, rlFrameCfg_t frameCfgArgs, rlChanCfg_t channelCfgArgs, rlAdcOutCfg_t adcOutCfgArgs, rlDevDataPathCfg_t datapathCfgArgs, rlProfileCfg_t profileCfgArgs)
    cdef extern int MMWL_AssignDeviceMap(unsigned char deviceMap,uint8_t* masterMap,uint8_t* slavesMap)
    cdef extern int MMWL_ArmingTDA(rlTdaArmCfg_t tdaArmCfgArgs)
    cdef extern int MMWL_StartFrame(unsigned char deviceMap)
    cdef extern int MMWL_StopFrame(unsigned char deviceMap)
    cdef extern int MMWL_DeArmingTDA()




# 定义程序名称、版本等常量
PROG_NAME = "mmwave"             # Name of the program
PROG_VERSION = "0.1"             # Program version
PROG_COPYRIGHT = "Copyright (C) 2022"
DEBUG_PRINT = printf             # Debug print function

RL_RET_CODE_OK = 0               # Return code for success

# 开发环境标志和其他常量
DEV_ENV = 1
NUM_CHIRPS = 12

CRED="\e[0;31m"    # Terminal code for regular red text
CGREEN="\e[0;32m"    # Terminal code for regular greed text
CRESET="\e[0m"       # Clear reset terminal color

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

# ADC output config */
cdef rlAdcOutCfg_t adcOutCfgArgs = rlAdcOutCfg_t(
    fmt = 2+1<<16+0<<18,
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

# 定义一个毫秒级睡眠的函数
cdef void msleep(int milliseconds):
    usleep(milliseconds * 1000)

#*
#* @brief MIMO Chirp configuration
#*
#* @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
#* @param chirpCfg Initital chirp configuration
#* @return uint32_t Configuration status
#*
cdef int8_t is_in_table(uint8_t value, uint8_t[:] table, uint8_t size):
    cdef uint8_t i
    for i in range(size):
        if table[i] == value:
            return i
    return -1


#*
#* @brief MIMO Chirp configuration
#*
#* @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
#* @param chirpCfg Initital chirp configuration
#* @return uint32_t Configuration status
#*
cpdef uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t* chirpCfg):
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
        status += MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg[0])

        # 打印调试信息
        fprintf(b"[CHIRP CONFIG] dev %u, chirp idx %u, status: %d\n", devId, i, status)
        if status != 0:
            DEBUG_PRINT(b"Configuration of chirp %d failed!\n", i)
            break

    return status

cdef void check(int status, const char* success_msg, const char* error_msg,
                unsigned char deviceMap, uint8_t is_required):
    # 模拟 DEV_ENV 环境下的调试信息
    if DEV_ENV:
        printf("STATUS %4d | DEV MAP: %2u | ", status, deviceMap)

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
    cdef unsigned int masterMap = 1<<masterId
    cdef int status = 0
    channelCfg.cascading =1
    status += MMWL_DevicePowerUp(masterMap, 1000, 1000)
    check(status,
    "[MASTER] Power up successful!",
    "[MASTER] Error: Failed to power up device!", masterMap, TRUE)

    status += MMWL_firmwareDownload(masterMap)
    check(status,
    "[MASTER] Firmware successfully uploaded!",
    "[MASTER] Error: Firmware upload failed!", masterMap, TRUE)

    status += MMWL_setDeviceCrcType(masterMap)
    check(status,
    "[MASTER] CRC type has been set!",
    "[MASTER] Error: Unable to set CRC type!", masterMap, TRUE)

    status += MMWL_rfEnable(masterMap)
    check(status,
    "[MASTER] RF successfully enabled!",
    "[MASTER] Error: Failed to enable master RF", masterMap, TRUE)

    status += MMWL_channelConfig(masterMap, channelCfg.cascading, channelCfg)
    check(status,
    "[MASTER] Channels successfully configured!",
    "[MASTER] Error: Channels configuration failed!", masterMap, TRUE)

    status += MMWL_adcOutConfig(masterMap, adcOutCfg)
    check(status,
    "[MASTER] ADC output format successfully configured!",
    "[MASTER] Error: ADC output format configuration failed!", masterMap, TRUE)

    check(status,
    "[MASTER] Init completed with sucess\n",
    "[MASTER] Init completed with error", masterMap, TRUE)
    return status

cdef int32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg):
    cdef int status = 0
    cdef uint8_t slavesMap = (1 << 1) | (1 << 2) | (1 << 3)

    # slave chip
    channelCfg.cascading = 2

    for slaveId in range(1,4):
        cdef unsigned int slaveMap = 1 << slaveId

        status += MMWL_DevicePowerUp(slaveMap, 1000, 1000)
        check(status,
            "[SLAVE] Power up successful!",
            "[SLAVE] Error: Failed to power up device!", slaveMap, TRUE)

    #Config of all slaves together
    status += MMWL_firmwareDownload(slavesMap)
    check(status,
        "[SLAVE] Firmware successfully uploaded!",
        "[SLAVE] Error: Firmware upload failed!", slavesMap, TRUE)

    status += MMWL_setDeviceCrcType(slavesMap)
    check(status,
        "[SLAVE] CRC type has been set!",
        "[SLAVE] Error: Unable to set CRC type!", slavesMap, TRUE)

    status += MMWL_rfEnable(slavesMap)
    check(status,
        "[SLAVE] RF successfully enabled!",
        "[SLAVE] Error: Failed to enable master RF", slavesMap, TRUE)

    status += MMWL_channelConfig(slavesMap, channelCfg.cascading,channelCfg)
    check(status,
        "[SLAVE] Channels successfully configured!",
        "[SLAVE] Error: Channels configuration failed!", slavesMap, TRUE)

    status += MMWL_adcOutConfig(slavesMap, adcOutCfg)
    check(status,
        "[SLAVE] ADC output format successfully configured!",
        "[SLAVE] Error: ADC output format configuration failed!", slavesMap, TRUE)

    check(status,
        "[SLAVE] Init completed with sucess\n",
        "[SLAVE] Init completed with error", slavesMap, TRUE)
    return status

cdef uint32_t configure (devConfig_t config):
    cdef int status = 0
    cdef int devId = 0
    status += initMaster(config.channelCfg, config.adcOutCfg)
    status += initSlaves(config.channelCfg, config.adcOutCfg)

    status += MMWL_RFDeviceConfig(config.deviceMap)
    check(status,
        "[ALL] RF deivce configured!",
        "[ALL] RF device configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg)
    check(status,
        "[ALL] LDO Bypass configuration successful!",
        "[ALL] LDO Bypass configuration failed!", config.deviceMap, TRUE)

    status += MMWL_dataFmtConfig(config.deviceMap, config.dataFmtCfg)
    check(status,
        "[ALL] Data format configuration successful!",
        "[ALL] Data format configuration failed!", config.deviceMap, TRUE)

    status += MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg)
    check(status,
        "[ALL] Low Power Mode configuration successful!",
        "[ALL] Low Power Mode configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ApllSynthBwConfig(config.deviceMap)
    status += MMWL_setMiscConfig(config.deviceMap, config.miscCfg)
    status += MMWL_rfInit(config.deviceMap)
    check(status,
        "[ALL] RF successfully initialized!",
        "[ALL] RF init failed!", config.deviceMap, TRUE)

    status += MMWL_dataPathConfig(config.deviceMap, config.datapathCfg)
    status += MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg)
    status += MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg)
    check(status,
        "[ALL] Datapath configuration successful!",
        "[ALL] Datapath configuration failed!", config.deviceMap, TRUE)

    status += MMWL_profileConfig(config.deviceMap, config.profileCfg)
    check(status,
        "[ALL] Profile configuration successful!",
        "[ALL] Profile configuration failed!", config.deviceMap, TRUE)

    # MIMO Chirp configuration
    for devId in range(4):
        status += configureMimoChirp(devId, config.chirpCfg)

    check(status,
        "[ALL] Chirp configuration successful!",
        "[ALL] Chirp configuration failed!", config.deviceMap, TRUE)

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
        "[MASTER] Frame configuration completed!",
        "[MASTER] Frame configuration failed!", config.masterMap, TRUE)

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
        "[SLAVE] Frame configuration completed!",
        "[SLAVE] Frame configuration failed!", config.slavesMap, TRUE)

    check(status,
        "[MIMO] Configuration completed!\n",
        "[MIMO] Configuration completed with error!", config.deviceMap, TRUE)

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
        if "profile" in mimo:
            profile = mimo["profile"]
            if "id" in profile:
                config.profileCfg.profileId = uint16_t(profile["id"])
            if "startFrequency" in profile:
                config.profileCfg.startFreqConst = uint32_t(ceil(profile["startFrequency"]*1e9/53.644))
            if "frequencySlope" in profile:
                config.profileCfg.freqSlopeConst = int16_t(ceil(profile["frequencySlope"]*1e3/48.279)) 
            if "idleTime" in profile:
                config.profileCfg.idleTimeConst = uint32_t(ceil(profile["idleTime"]*1e2))
            if "adcStartTime" in profile:
                config.profileCfg.adcStartTimeConst = uint32_t(ceil(profile["adcStartTime"]*1e2))
            if "rampEndTime" in profile:
                config.profileCfg.rampEndTime = uint32_t(ceil(profile["rampEndTime"]*1e2))
            if "txStartTIme" in profile:
                config.profileCfg.txStartTime = uint16_t(ceil(profile["txStartTIme"]*1e2))
            if "numAdcSamples" in profile:
                config.profileCfg.numAdcSamples = uint16_t(profile["numAdcSamples"])
            if "adcSamplingFrequency" in profile:
                config.profileCfg.digOutSampleRate = uint16_t(profile["adcSamplingFrequency"])
            if "rxGain" in profile:
                config.profileCfg.rxGain = uint16_t(profile["rxGain"])
            if "hpfCornerFreq1" in profile:
                config.profileCfg.hpfCornerFreq1 = uint8_t(profile["hpfCornerFreq1"])
            if "hpfCornerFreq2" in profile:
                config.profileCfg.hpfCornerFreq2 = uint8_t(profile["hpfCornerFreq2"])
            
        if "frame" in mimo:
            frame = mimo["frame"]
            if "numFrames" in frame:
                config.frameCfg.numFrames = uint16_t(frame["numFrames"])
            if "numLoops" in frame:
                config.frameCfg.numLoops = uint16_t(frame["numLoops"])
            if "framePeriodicity" in frame:
                config.frameCfg.framePeriodicity = uint32_t(ceil(frame["framePeriodicity"]*1e7))
        if "channel" in mimo:
            channel = mimo["channel"]
            if "rxChannelEn" in channel:
                config.channelCfg.rxChannelEn = uint16_t(channel["rxChannelEn"])
            if "txChannelEn" in channel:
                config.channelCfg.txChannelEn = uint16_t(channel["txChannelEn"])
        config.frameCfg.numAdcSamples = 2 * config.profileCfg.numAdcSamples
        config.dataFmtCfg.rxChannelEn = config.channelCfg.rxChannelEn
        
        config.dataFmtCfg.rxChannelEn = channelCfgArgs.rxChannelEn
        config.dataFmtCfg.adcBits = adcOutCfgArgs.fmt & 0x3
        config.dataFmtCfg.adcFmt = (adcOutCfgArgs.fmt >> 16) & 0x3

cpdef int mmw_init(
    bytes capture_directory,
    bytes ip_addr="192.168.30.180",
    int port = 1800,
    ):
    cdef int status = 0
    status = MML_TDAInit(ip_addr,port,config.deviceMap)
    check(status,
        "[ALL] TDA Init successful!",
        "[ALL] TDA Init failed!", config.deviceMap, TRUE)

    configure(config) 

    return status

cpdef int mmw_arming_tda(bytes capture_path):
    cdef int status = 0
    cdef rlTdaArmCfg_t tdaCfg=rlTdaArmCfg_t(
        captureDirectory = capture_path,
        framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000),
        numberOfFilesToAllocate = 0,
        numberOfFramesToCapture = 0, # config.frameCfg.numFrames,
        dataPacking = 0, # 0: 16-bit | 1: 12-bit
    )
    status = MMWL_ArmingTDA(tdaCfg)
    check(status,
        "[ALL] TDA Arming successful!",
        "[ALL] TDA Arming failed!", config.deviceMap, TRUE)
    
    return status

cpdef int mmw_start_frame():
    cdef int status = 0
    cdef int i
    for i in range(3,-1,-1):
        status += MMWL_StartFrame(1<<i)
        check(status,
            "[ALL] RF successfully enabled!",
            "[ALL] Error: Failed to enable master RF", 1<<i, TRUE)

cpdef int mmw_stop_frame():
    cdef int status = 0
    cdef int i
    for i in range(3,-1,-1):
        status += MMWL_StopFrame(1<<i)
        check(status, 
            "[ALL] RF successfully disenabled!",
            "[ALL] Error: Failed to disenable master RF", 1<<i, TRUE)

cpdef int mmw_dearming_tda():
    cdef int status = 0
    status = MMWL_DeArmingTDA(config.deviceMap)
    check(status,
      "[MMWCAS-RF] Stop recording",
      "[MMWCAS-RF] Failed to de-arm TDA board!\n", 32, TRUE)
    return status
