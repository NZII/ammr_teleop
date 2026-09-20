#!/usr/bin/env python3
"""Export a manifest-selected source bundle; never touches a remote device."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import fnmatch

ROOT=Path(__file__).resolve().parents[1]
SKIP={'.git','__pycache__','build','install','log','logs','migration'}

def selected(root,role):
    spec=json.loads((root/'deploy'/f'{role}.manifest').read_text())
    files={}
    for pattern in spec['include']:
        matches=list(root.glob(pattern))
        if not matches:raise FileNotFoundError(f'Manifest source missing: {pattern}')
        for match in matches:
            candidates=match.rglob('*') if match.is_dir() else [match]
            for path in candidates:
                rel=path.relative_to(root).as_posix()
                if set(path.relative_to(root).parts)&SKIP or path.suffix=='.pyc':continue
                if any(fnmatch.fnmatch(rel,item) for item in spec.get('exclude',[])):continue
                if path.is_dir():continue
                if path.is_symlink():
                    raise ValueError(f'Symlink requires an explicit deployment decision: {rel}')
                files[rel]=path
    return files

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=['server','agx'],required=True)
    p.add_argument('--output',type=Path)
    p.add_argument('--list',action='store_true')
    args=p.parse_args()
    files=selected(ROOT,args.role)
    if args.list:
        for rel in sorted(files):print(rel)
        print(f'{len(files)} files, {sum(x.stat().st_size for x in files.values())} bytes')
        return
    if args.output is None:p.error('--output is required unless --list')
    output=args.output.expanduser().resolve()
    if output==ROOT or ROOT in output.parents:p.error('Export outside the source project')
    if output.exists():p.error('Output must be a new directory; existing bundles are never overwritten')
    output.mkdir(parents=True)
    hashes={}
    for rel,path in sorted(files.items()):
        destination=output/rel
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,destination)
        hashes[rel]=hashlib.sha256(destination.read_bytes()).hexdigest()
    (output/'deployment.json').write_text(json.dumps({'role':args.role,'sha256':hashes},indent=2)+'\n')
    print(f'Exported {args.role}: {len(files)} files -> {output}')

if __name__=='__main__':main()
