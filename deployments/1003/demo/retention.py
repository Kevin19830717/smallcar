"""Disk-only rolling diagnostic storage. Never traverses outside its root."""
import re,shutil
from pathlib import Path
LIMIT=2_000_000_000
RESERVE=1_000_000_000
SESSION_LIMIT=256_000_000
def size(root):
    return sum(p.stat().st_size for p in Path(root).rglob('*') if p.is_file() and not p.is_symlink())
def remove(root,path):
    root=Path(root).resolve();path=Path(path)
    if path.is_symlink() or path.resolve().parent!=root:raise ValueError('Unsafe diagnostic cleanup target')
    if not re.fullmatch(r'\d{8}T\d{6}Z_[a-f0-9]{8}(?:\.tar\.(?:gz|tmp))?',path.name):raise ValueError('Unrecognized diagnostic filename')
    if path.is_dir():shutil.rmtree(path)
    elif path.is_file():path.unlink()
def prune(root,budget=LIMIT,protected=()):
    root=Path(root);protected={Path(p).name for p in protected if p is not None}
    candidates=sorted((p for p in root.iterdir() if not p.name.startswith('.') and p.name not in protected),key=lambda p:p.stat().st_mtime)
    used=size(root)
    for p in candidates:
        if used<=budget:break
        amount=size(p) if p.is_dir() else p.stat().st_size
        remove(root,p);used-=amount
    return used
