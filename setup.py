from setuptools import setup, Extension
from Cython.Build import cythonize

# 定义项目的根目录
ROOT_DIR = "ti"

# 定义包含头文件和源文件的目录
MMWLINK_IDIR = f"{ROOT_DIR}/mmwavelink/src"
MMWETH_IDIR = f"{ROOT_DIR}/ethernet/src"
MMWAVE_IDIR = f"{ROOT_DIR}/mmwave"
#CLI_OPT_IDIR = "opt"
#TOML_CONFIG_IDIR = "toml"

# 定义需要编译的所有 `.c` 文件路径
sources = [
    "mimo.pyx",         # 包含 Cython 主文件
    f"{MMWLINK_IDIR}/*.c",
    f"{MMWETH_IDIR}/*.c",
    f"{MMWAVE_IDIR}/*.c",
#    f"{CLI_OPT_IDIR}/*.c",
#    f"{TOML_CONFIG_IDIR}/*.c"
]

# 创建扩展模块
extensions = [
    Extension(
        "mmwcas",          # 输出模块的名称
        sources=sources,        # 所有源文件
        include_dirs=[
            ".",                # 当前目录
            MMWLINK_IDIR,       # mmwlink 目录
            MMWETH_IDIR,        # mmwethernet 目录
            MMWAVE_IDIR,        # mmwave 目录
#            CLI_OPT_IDIR,       # cliopt 目录
#            TOML_CONFIG_IDIR    # tomlconfig 目录
        ],
        extra_compile_args=["-w"],  # 添加编译选项（如禁用警告）
        libraries=["pthread", "m"]  # 链接 pthread 和数学库
    )
]

# 编写 setup 配置
setup(
    name="mmwcas",
    ext_modules=cythonize(extensions),
)