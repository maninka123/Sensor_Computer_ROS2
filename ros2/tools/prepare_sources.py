#!/usr/bin/env python3
"""Fetch pinned official drivers and apply documented Jazzy build adaptations."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / 'vendor'
PINS = {
    'livox': ('https://github.com/Livox-SDK/livox_ros2_driver.git',
              '1565b976ab1905f911cd2a757fc32433422b701c'),
    'flir': ('https://github.com/ros-drivers/flir_camera_driver.git',
             '986172dbd8dab9ac5a365d0fcbd785508803e960'),
}


def main():
    VENDOR.mkdir(exist_ok=True)
    for name, (url, revision) in PINS.items():
        checkout = VENDOR / name
        if not (checkout / '.git').exists():
            subprocess.run(['git', 'clone', url, str(checkout)], check=True)
        subprocess.run(['git', '-C', str(checkout), 'fetch', 'origin', revision], check=True)
        # Only the dedicated ignored vendor checkout is reset; project changes are untouched.
        subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', revision], check=True)
    livox_cmake = VENDOR/'livox/livox_sdk_vendor/CMakeLists.txt'
    text = livox_cmake.read_text().replace('set(livox_sdk_REV "v2.3.1")',
        'set(livox_sdk_REV "9306596a2bf15c1343bc023b497465ed0a32909d")')
    if 'PATCH_COMMAND' not in text:
        text = text.replace('    TIMEOUT 60', '    PATCH_COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/patch_sdk.py <SOURCE_DIR>\n    TIMEOUT 60')
    livox_cmake.write_text(text)
    shutil.copy2(ROOT/'tools/patch_livox_sdk.py', VENDOR/'livox/livox_sdk_vendor/patch_sdk.py')
    flir_cmake = VENDOR/'flir/spinnaker_camera_driver/CMakeLists.txt'
    text = flir_cmake.read_text().replace(
        '  include(cmake/DownloadSpinnaker.cmake)\n  download_spinnaker(SPINNAKER_LIBRARIES SPINNAKER_INCLUDE_DIRS)',
        '  message(FATAL_ERROR "Install the supplied Noble ARM64 Spinnaker SDK; automatic fallback downloads are disabled")')
    flir_cmake.write_text(text)
    # Exclude packages outside the production camera driver dependency graph.
    for name in ['flir_camera_description', 'spinnaker_synchronized_camera_driver']:
        (VENDOR/'flir'/name/'COLCON_IGNORE').touch()
    print('Pinned official driver sources prepared; local changes are limited to SDK pin/download policy.')


if __name__ == '__main__':
    main()
