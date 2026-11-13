#!/usr/bin/env python3
"""GCC 13 build fixes for pinned original Livox SDK; no packet/geometry changes."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
header = root/'sdk_core/src/base/thread_base.h'
text = header.read_text()
if '#include <memory>' not in text:
    text = '#include <memory>\n'+text
header.write_text(text)
# Legacy bundled fmt declares char8_t in C++11 and emits GCC 13's C++20
# compatibility warning. Keep diagnostics visible without treating that one
# warning as an error; this dependency still builds as C++11.
for cmake in root.rglob('CMakeLists.txt'):
    text = cmake.read_text()
    if '-Werror' in text and '-Wno-error=c++20-compat' not in text:
        cmake.write_text(text.replace('-Werror', '-Werror -Wno-error=c++20-compat'))
