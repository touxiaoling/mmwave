from setuptools import setup, Extension
from Cython.Build import cythonize

# 定义项目的根目录
ROOT_DIR = "ti"

# 定义包含头文件和源文件的目录
MMWLINK_IDIR = f"{ROOT_DIR}/mmwavelink/src"
MMWLINK_H_IDIR = f"{ROOT_DIR}/mmwavelink/include"
MMWETH_IDIR = f"{ROOT_DIR}/ethernet/src"
MMWAVE_IDIR = f"{ROOT_DIR}/mmwave"
FIRMWARE_IDIR = f"{ROOT_DIR}/firmware"
#CLI_OPT_IDIR = "opt"
#TOML_CONFIG_IDIR = "toml"

# 定义需要编译的所有 `.c` 文件路径
sources = [
    "mmwcas.pyx",         # 包含 Cython 主文件
    f"{MMWLINK_IDIR}/rl_controller.c",
    f"{MMWLINK_IDIR}/rl_device.c",
    f"{MMWLINK_IDIR}/rl_driver.c",
    f"{MMWLINK_IDIR}/rl_monitoring.c",
    f"{MMWLINK_IDIR}/rl_sensor.c",
    f"{MMWETH_IDIR}/mmwl_port_ethernet.c",
    f"{MMWETH_IDIR}/mtime.c",
    f"{MMWAVE_IDIR}/crc_compute.c",
    f"{MMWAVE_IDIR}/mmwave.c",
    f"{MMWAVE_IDIR}/rls_osi.c",
#    f"{CLI_OPT_IDIR}/*.c",
#    f"{TOML_CONFIG_IDIR}/*.c"
]

# 创建扩展模块
extensions = Extension(
        name = "mmwcas",          # 输出模块的名称
        sources=sources,        # 所有源文件
        include_dirs=[
#            ".",                # 当前目录
            MMWLINK_IDIR,       # mmwlink 目录
            MMWLINK_H_IDIR,     # mmwlink 头文件目录
            MMWETH_IDIR,        # mmwethernet 目录
            MMWAVE_IDIR,        # mmwave 目录
            FIRMWARE_IDIR,
#            CLI_OPT_IDIR,       # cliopt 目录
#            TOML_CONFIG_IDIR    # tomlconfig 目录
        ],
        extra_compile_args=["-w"],  # 添加编译选项（如禁用警告）
        libraries=["pthread", "m"],  # 链接 pthread 和数学库
    )

# 编写 setup 配置
setup(
    name="mmwcas",
    ext_modules=cythonize(extensions,
                          #annotate=True,
                          compiler_directives={'language_level' : "3str"}),
)