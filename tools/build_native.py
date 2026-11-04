"""Build portable LTE tests using the installed MSVC or a native g++."""
import os
from pathlib import Path
import subprocess
import shutil

ROOT = Path(__file__).resolve().parents[1]


def build():
    dest = ROOT / 'build/native-lte'
    dest.parent.mkdir(parents=True,exist_ok=True)
    sources = ['tests/native_lte.cpp', 'src/LteTimeDecoder.cpp', 'src/LteRfDecoder.cpp']
    if os.name == 'nt':
        dest = dest.with_suffix('.exe')
        vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        installation = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-requires',
            'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
        setup = Path(installation) / 'VC/Auxiliary/Build/vcvars64.bat'
        # Fixed tool path, no user-controlled command interpolation. No file operations.
        raw = subprocess.check_output(f'"{setup}" >nul && set', shell=True, text=True)
        env = dict(os.environ)
        env.update(line.split('=', 1) for line in raw.splitlines() if '=' in line and not line.startswith('='))
        compiler = shutil.which('cl', path=env.get('Path', env.get('PATH')))
        subprocess.run([compiler, '/nologo', '/O2', '/W4', '/std:c++14', '/EHsc', '/Isrc', *sources,
                        '/Fe:' + str(dest), '/Fo:build/'], cwd=ROOT, env=env, check=True)
    else:
        subprocess.run(['g++', '-O2', '-std=c++14', '-Wall', '-Wextra', '-Isrc', *sources, '-o', str(dest)], cwd=ROOT, check=True)
    return dest


if __name__ == '__main__':
    print(build())
