#!/usr/bin/env python3
"""Versioned source bundles, staged installs, explicit activation and rollback.

No command starts a mission or hardware. Remote upload requires explicit push.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path,PurePosixPath
import re
import shlex
import socket
import subprocess
import sys
import tarfile
import time
import tempfile

ROOT=Path(__file__).resolve().parents[1]
SOURCE_DIRS=('payload_catch','launch','config','resource','tools','deploy')
SOURCE_FILES=('package.xml','setup.py','setup.cfg','README.md','AGENTS.md')


def sha(data):return hashlib.sha256(data).hexdigest()


def snapshot(root):
    files={}
    for directory in SOURCE_DIRS:
        if (root/directory).is_symlink():raise ValueError('source directory is symlink')
        for path in (root/directory).rglob('*'):
            if path.is_symlink():raise ValueError('source symlinks are not bundled: '+str(path))
            if path.is_file() and not any(x in ('__pycache__','.git') for x in path.parts) and path.suffix not in ('.pyc','.pyo'):
                files[path.relative_to(root).as_posix()]=path.read_bytes()
    for name in SOURCE_FILES:
        files[name]=(root/name).read_bytes()
    return files


def make_manifest(files):
    hashes={name:sha(value) for name,value in sorted(files.items())}
    release_id=sha(json.dumps(hashes,sort_keys=True,separators=(',',':')).encode())
    return dict(schema=1,release_id=release_id,files=hashes)


def bundle(root,target):
    files=snapshot(root);manifest=make_manifest(files)
    # Include the current working tree, including untracked safety/PWM changes.
    with tarfile.open(target,'x:gz') as archive:
        for name,value in {**files,'FLEET_MANIFEST.json':json.dumps(manifest,sort_keys=True).encode()}.items():
            member=tarfile.TarInfo(name);member.size=len(value);member.mode=0o644
            archive.addfile(member,io.BytesIO(value))
    return manifest['release_id']


def read_bundle(path):
    files={}
    with tarfile.open(path,'r:gz') as archive:
        total=0
        for member in archive:
            name=PurePosixPath(member.name)
            if (not member.isfile() or name.is_absolute() or '..' in name.parts
                    or str(name)!=member.name or member.name in files or member.size>20_000_000):
                raise ValueError('unsafe / duplicate bundle member: '+member.name)
            total+=member.size
            if total>100_000_000:raise ValueError('bundle too large')
            files[member.name]=archive.extractfile(member).read()
    manifest=json.loads(files.pop('FLEET_MANIFEST.json'))
    if make_manifest(files)!=manifest:raise ValueError('bundle integrity mismatch')
    for name in files:
        if name.split('/')[0] not in SOURCE_DIRS+SOURCE_FILES:
            raise ValueError('unexpected bundle path: '+name)
    return files,manifest


def verify(root):
    manifest=json.loads((root/'FLEET_MANIFEST.json').read_text())
    files=snapshot(root)
    if any((root/name).is_symlink() for name in files) or make_manifest(files)!=manifest:
        raise ValueError('installed source differs from bundle')
    return manifest


def base_dir(path):
    base=Path(path).expanduser().resolve()
    if base in (Path('/'),Path.home(),ROOT) or len(base.parts)<3:
        raise ValueError('use a dedicated deployment directory')
    base.mkdir(parents=True,exist_ok=True)
    return base


def running_lock(base):
    import fcntl
    handle=open(base/'operation.lock','a')
    try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BaseException:
        handle.close();raise
    return handle


def ensure_stopped(base):
    import fcntl
    handle=open(base/'mission.lock','a')
    try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BaseException:
        handle.close();raise
    return handle


def install(args):
    base=base_dir(args.base)
    with running_lock(base),ensure_stopped(base):
        files,manifest=read_bundle(args.bundle)
        role_file=base/'ROLE'
        if role_file.exists() and role_file.read_text().strip()!=args.role:
            raise ValueError('deployment directory already assigned to another role')
        release=base/'releases'/manifest['release_id']
        source=release/'src'
        if release.exists():
            if verify(source)!=manifest or (release/'ROLE').read_text().strip()!=args.role:
                raise ValueError('existing staging differs; preserve for inspection')
        else:
            release.mkdir(parents=True)
            source.mkdir()
            for name,value in files.items():
                path=source/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(value)
            (source/'FLEET_MANIFEST.json').write_text(json.dumps(manifest,sort_keys=True))
            (release/'ROLE').write_text(args.role+'\n')
        role_file.write_text(args.role+'\n')
        if not args.no_build:
            if os.environ.get('ROS_DISTRO')!='humble':raise ValueError('source Humble and px4_msgs overlay before install')
            subprocess.run([sys.executable,'-c','import rclpy,px4_msgs,numpy,scipy,yaml'],check=True)
            (release/'BUILT').unlink(missing_ok=True)
            workspace=release/'ws';(workspace/'src').mkdir(parents=True,exist_ok=True)
            package=workspace/'src/payload_catch'
            if package.is_symlink():
                if package.resolve()!=source:raise ValueError('workspace points to another source')
            else:package.symlink_to(source,target_is_directory=True)
            subprocess.run(['colcon','build','--symlink-install','--executor','sequential',
                            '--packages-select','payload_catch'],cwd=workspace,check=True)
            verify(source)
            (release/'BUILT').write_text(json.dumps(dict(host=socket.gethostname(),architecture=os.uname().machine,
                                                       built_at=time.time())))
        print('Staged only:',release,'(not activated, no mission started)')


def atomic_link(base,name,target):
    temporary=base/(name+'.next')
    if temporary.exists() or temporary.is_symlink():temporary.unlink()
    temporary.symlink_to(target,target_is_directory=True)
    os.replace(temporary,base/name)


def select(base,release_id):
    if not re.fullmatch(r'[0-9a-f]{64}',release_id):raise ValueError('invalid release ID')
    release=base/'releases'/release_id
    if release.is_symlink() or release.resolve().parent!=base/'releases':raise ValueError('invalid release directory')
    if (release/'ROLE').read_text()!=(base/'ROLE').read_text():raise ValueError('role mismatch')
    built=json.loads((release/'BUILT').read_text())
    if built['host']!=socket.gethostname() or built['architecture']!=os.uname().machine:
        raise ValueError('must build on this target host and architecture')
    verify(release/'src')
    return release


def activate(args):
    base=base_dir(args.base)
    with running_lock(base),ensure_stopped(base):
        if args.command=='rollback':
            previous=(base/'previous').resolve(strict=True)
            if previous.parent!=base/'releases':raise ValueError('invalid rollback target')
            target=select(base,previous.name)
        else:target=select(base,args.release)
        current=base/'current'
        if current.exists():
            old=current.resolve()
            if old!=target:
                if old.parent!=base/'releases':raise ValueError('foreign current pointer')
                atomic_link(base,'previous',old)
        atomic_link(base,'current',target)
        print('Selected:',target,'(no mission started)')


def push(args):
    # No remote extraction, execution, sudo, service changes or automatic restart.
    files,_=read_bundle(args.bundle)
    if not re.fullmatch(r'[A-Za-z0-9_.@-]+',args.host) or args.host.startswith('-'):
        raise ValueError('use an SSH alias or user@host')
    name='payload-catch-'+sha(Path(args.bundle).read_bytes())[:16]+'.tar.gz'
    subprocess.run(['scp','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',str(Path(args.bundle).resolve()),
                    args.host+':'+name],check=True)
    with tempfile.TemporaryDirectory() as directory:
        installer=Path(directory)/'payload-catch-installer.py'
        installer.write_bytes(files['tools/fleet_deploy.py'])
        subprocess.run(['scp','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',str(installer),
                        args.host+':payload-catch-installer.py'],check=True)
    print('Uploaded to remote home:',name,'and payload-catch-installer.py; install/activation remain explicit')


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    b=sub.add_parser('bundle');b.add_argument('--out',required=True)
    v=sub.add_parser('verify');v.add_argument('source')
    t=sub.add_parser('push');t.add_argument('--host',required=True);t.add_argument('--bundle',required=True)
    i=sub.add_parser('install');i.add_argument('--bundle',required=True);i.add_argument('--role',choices=['a','b'],required=True)
    i.add_argument('--base',default='~/payload_catch_deploy');i.add_argument('--no-build',action='store_true',help='stage only; cannot activate')
    for name in ('activate','rollback'):
        a=sub.add_parser(name);a.add_argument('--base',default='~/payload_catch_deploy')
        a.add_argument('--ground-confirmed',action='store_true',required=True,help='grounded, disarmed and all role processes stopped')
        if name=='activate':a.add_argument('--release',required=True)
    a=p.parse_args()
    if a.command=='bundle':print(bundle(ROOT,a.out))
    elif a.command=='verify':print(verify(Path(a.source).resolve())['release_id'])
    elif a.command=='push':push(a)
    elif a.command=='install':install(a)
    else:activate(a)


if __name__=='__main__':main()
