#!/usr/bin/env python3
"""Link existing host CUDA runtime libraries into a private, bounded lib dir.

Only links NVIDIA shared objects. The host originals are never modified.
This is a laptop diagnostic convenience, not a Xavier deployment recipe.
"""
from pathlib import Path
import os

HERE = Path(__file__).resolve().parent
SOURCE = Path.home() / '.local/opt/resolve/libs'
DESTINATION = HERE / '.cuda_libs'
GLOBS = (
    'libcudart.so*', 'libcublas.so*', 'libcublasLt.so*',
    'libcudnn*.so*', 'libcufft.so*', 'libcurand.so*',
    'libcusparse.so*', 'libnvrtc*.so*',
)


def main():
    if not SOURCE.is_dir():
        raise RuntimeError('Optional host CUDA libraries missing: ' + str(SOURCE))
    DESTINATION.mkdir(exist_ok=True)
    created = 0
    for pattern in GLOBS:
        for source in SOURCE.glob(pattern):
            if not source.is_file():
                continue
            target = DESTINATION / source.name
            if target.is_symlink() and target.resolve() == source.resolve():
                continue
            if target.exists() or target.is_symlink():
                raise RuntimeError('Refusing to overwrite local CUDA link: ' + str(target))
            target.symlink_to(source.resolve())
            created += 1
    print(f'Private CUDA links: {created} created; {len(list(DESTINATION.iterdir()))} total')


if __name__ == '__main__':
    main()
