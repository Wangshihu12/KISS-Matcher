# CMake最低版本要求：支持3.16到3.31之间的版本
cmake_minimum_required(VERSION 3.16...3.31)

# 项目定义：KISS-Matcher，版本0.3.0，使用C++语言
project(kiss_matcher VERSION 0.3.0 LANGUAGES CXX)

# 输出KISS-Matcher项目的ASCII艺术字LOGO
# 用于在CMake配置阶段显示项目标识
message("")
message("██╗  ██╗██╗███████╗███████╗      ")
message("██║ ██╔╝██║██╔════╝██╔════╝      ")
message("█████╔╝ ██║███████╗███████╗█████╗")
message("██╔═██╗ ██║╚════██║╚════██║╚════╝")
message("██║  ██╗██║███████║███████║      ")
message("╚═╝  ╚═╝╚═╝╚══════╝╚══════╝      ")
message("")
message("███╗   ███╗ █████╗ ████████╗ ██████╗██╗  ██╗███████╗██████╗ ")
message("████╗ ████║██╔══██╗╚══██╔══╝██╔════╝██║  ██║██╔════╝██╔══██╗")
message("██╔████╔██║███████║   ██║   ██║     ███████║█████╗  ██████╔╝")
message("██║╚██╔╝██║██╔══██║   ██║   ██║     ██╔══██║██╔══╝  ██╔══██╗")
message("██║ ╚═╝ ██║██║  ██║   ██║   ╚██████╗██║  ██║███████╗██║  ██║")
message("╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝")
message("")

# ==== 编译选项配置 ====
# 定义可选的编译选项，用户可以通过-D参数在命令行中开启或关闭

# 使用Ccache加速编译（如果系统中安装了Ccache），默认开启
option(USE_CCACHE "Build using Ccache if found on the path" ON)

# 使用系统预安装的Eigen3库，而非下载第三方版本，默认开启
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)

# 使用系统预安装的Intel oneAPI TBB（线程构建块）库，默认关闭
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" OFF)

# 使用系统预安装的ROBIN库（来自MIT SPARK实验室），默认开启
option(USE_SYSTEM_ROBIN "Use system pre-installed ROBIN from SPARK @ MIT" ON)

# 使用地址清理器（Address Sanitizer）进行内存错误检测，默认关闭
option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)

# ==== 构建类型设置 ====
# 如果用户未指定构建类型，设置默认构建类型为Release（发布版本）
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    # 强制设置构建类型为Release，适用于生产环境的优化构建
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    # 为cmake-gui或ccmake提供可选的构建类型列表
    # Debug：调试版本；Release：发布版本；MinSizeRel：最小体积版本；RelWithDebInfo：带调试信息的发布版本
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# ==== C++标准设置 ====
# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
# 要求必须支持C++17，如果编译器不支持则报错
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器特定的C++扩展，确保代码的可移植性
set(CMAKE_CXX_EXTENSIONS OFF)

# ==== 地址清理器配置 ====
# 如果启用了地址清理器选项，添加相应的编译和链接选项
if (USE_ADDRESS_SANITIZER)
    # 添加地址清理器的编译选项，用于检测内存错误（如越界访问、使用已释放内存等）
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    # 添加地址清理器的链接选项
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

# ==== 引入CMake模块和依赖项 ====
# 引入GNU标准安装目录定义（如CMAKE_INSTALL_LIBDIR、CMAKE_INSTALL_INCLUDEDIR等）
include(GNUInstallDirs)
# 引入第三方依赖项查找脚本
include(3rdparty/find_dependencies.cmake)
# 引入编译器选项配置
include(cmake/CompilerOptions.cmake)

# 查找LZ4压缩库（必需）
# 注意：如果没有这一行，Python绑定的KISS-Matcher将无法正常工作
find_library(LZ4_LIBRARY lz4 REQUIRED)

# ==== OpenMP并行计算配置 ====
# 查找OpenMP库（用于多线程并行计算）
# 这是在所有平台上正确设置OpenMP编译器选项的最佳方式
include(FindOpenMP)

if (OPENMP_FOUND)
    # 如果找到OpenMP，将其编译和链接选项添加到对应的FLAGS中
    # 为C语言添加OpenMP编译选项
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    # 为C++语言添加OpenMP编译选项
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    # 为可执行文件链接添加OpenMP链接选项
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    # 如果未找到OpenMP，输出错误信息
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)

# 设置Release模式下的C++编译选项
# -O3：最高级别的优化
# -Wall：开启所有警告
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

# 添加核心代码目录到包含路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/core)

# ==== 库目标定义 ====
# 设置目标库名称
set(TARGET_NAME kiss_matcher_core)

# 创建静态库目标
# 注意：必须使用STATIC类型，否则在Python绑定时会导致ImportError
# 错误信息："libkiss_matcher_core.so: cannot open shared object file: NO such file or directory ..."
# 详见Issue #37 (https://github.com/MIT-SPARK/KISS-Matcher/issues/37)
add_library(${TARGET_NAME} STATIC
    # ROBIN匹配算法实现
    core/kiss_matcher/ROBINMatching.cpp
    # Faster PFH (点特征直方图) 特征提取实现
    core/kiss_matcher/FasterPFH.cpp
    # KISS-Matcher主算法实现
    core/kiss_matcher/KISSMatcher.cpp
    # GNC (Graduated Non-Convexity) 求解器实现
    core/kiss_matcher/GncSolver.cpp
)

# 为库目标创建命名空间别名，便于在其他CMake项目中引用
add_library(kiss_matcher::kiss_matcher_core ALIAS kiss_matcher_core)

# ==== 目标链接库配置 ====
# 为目标库链接所需的依赖库
target_link_libraries(${TARGET_NAME}
    PUBLIC 
    Eigen3::Eigen      # Eigen3线性代数库
    robin::robin       # ROBIN匹配库
    ${OpenMP_LIBS}     # OpenMP并行计算库
    ${EIGEN3_LIBS}     # Eigen3相关库
    TBB::tbb           # Intel TBB线程构建块库
    ${LZ4_LIBRARY}     # LZ4压缩库
)

# 设置全局目标属性，使kiss_matcher::core在Python绑定中全局可用
set_global_target_properties(${TARGET_NAME})

# ==== 目标包含目录配置 ====
# 为目标库指定头文件包含目录
target_include_directories(${TARGET_NAME}
    PRIVATE
    # 私有包含目录：仅在编译本库时使用
    ${CMAKE_CURRENT_SOURCE_DIR}/core/
    PUBLIC
    # 公共包含目录：在编译本库和使用本库的项目中都可用
    # BUILD_INTERFACE：构建时使用的包含目录
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    # INSTALL_INTERFACE：安装后使用的包含目录
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ==== 安装目标配置 ====
# 安装编译好的库文件
install(TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}-export
    # 共享库安装目录
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    # 静态库安装目录
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# ==== 导出目标配置 ====
# 安装导出的目标配置文件，以便其他CMake项目可以找到并使用本库
install(EXPORT ${PROJECT_NAME}-export
    # 安装到CMake模块目录
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    # 使用命名空间，避免与其他库冲突
    NAMESPACE ${PROJECT_NAME}::
    # 导出文件名
    FILE ${PROJECT_NAME}Targets.cmake
)

# ==== 安装头文件 ====
# 将kiss_matcher头文件目录安装到系统包含目录
install(DIRECTORY core/kiss_matcher DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ==== 导出构建树中的目标 ====
# 在构建树中导出目标，无需安装即可在其他项目中使用
export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE ${PROJECT_NAME}-exports.cmake)

# ==== 安装CMake配置文件 ====
# 安装项目的CMake配置文件，供find_package()使用
install(FILES cmake/${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
