@echo off
REM 这是简化版本 - 直接通过Python运行 每个脚本在新的cmd窗口中
setlocal enabledelayedexpansion

set "PROJECT_ROOT=%~dp0"
set "PYTHON_EXE=%PROJECT_ROOT%.venv\Scripts\python.exe"

if not exist "%PYTHON_EXE%" (
    echo 错误：虚拟环境不存在，请先运行: python -m venv .venv
    pause
    exit /b 1
)

echo.
echo ========================================
echo 启动项目脚本
echo ========================================
echo.

REM 先运行主程序（在新窗口中）
echo 启动: 主程序
start "Main Program" "%PYTHON_EXE%" "%PROJECT_ROOT%run.py"

REM 运行test_scripts中的各个脚本
echo 启动: 记录数据
start "Record Data" "%PYTHON_EXE%" "%PROJECT_ROOT%test_scripts\record_data.py"

echo 启动: 分析数据
start "Analyze Data" "%PYTHON_EXE%" "%PROJECT_ROOT%test_scripts\analyze_recorded_data.py"

echo 启动: 离线路径
start "Offline Path" "%PYTHON_EXE%" "%PROJECT_ROOT%test_scripts\offline_path.py"

REM echo 启动: 绘图
REM start "Plot" "%PYTHON_EXE%" "%PROJECT_ROOT%test_scripts\plot.py"

echo.
echo ========================================
echo 所有脚本已启动！
echo ========================================
echo.

