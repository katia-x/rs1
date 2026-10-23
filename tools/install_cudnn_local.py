"""Extract the pinned native cuDNN wheel into a standalone C++ library prefix."""
import argparse
from pathlib import Path
import shutil
import tempfile
import zipfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('wheel', type=Path)
parser.add_argument('--prefix', type=Path, default=Path.home()/'opt/cudnn-9.7.1')
args=parser.parse_args()
if not args.wheel.name.startswith('nvidia_cudnn_cu12-9.7.1.26-'):
    parser.error('Expected the nvidia-cudnn-cu12 9.7.1.26 wheel')
with tempfile.TemporaryDirectory() as directory:
    with zipfile.ZipFile(args.wheel) as archive:
        archive.extractall(directory)
    source=Path(directory)/'nvidia/cudnn'
    for folder, pattern in [('include','*.h'),('lib','*.so*')]:
        destination=args.prefix/folder
        destination.mkdir(parents=True,exist_ok=True)
        for path in (source/folder).glob(pattern):
            shutil.copy2(path,destination/path.name)
    for path in Path(directory).rglob('LICENSE*'):
        shutil.copy2(path,args.prefix/path.name)
    link=args.prefix/'lib/libcudnn.so'
    if not link.exists(): link.symlink_to('libcudnn.so.9')
assert (args.prefix/'include/cudnn.h').is_file()
assert (args.prefix/'lib/libcudnn.so.9').is_file()
print(args.prefix)
