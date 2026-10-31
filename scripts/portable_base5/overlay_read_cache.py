#!/usr/bin/env python3
"""Session-local cache of original validated layout overlay JSON.

Every eligible call reads actual file bytes for SHA256. A new (path, SHA) is
accepted only by the unmodified original read_overlay, followed by another real
SHA check. Only JSON parsing/content-validation work is reused. No native QA,
source pins, geometry checks or capture policy are replaced.

Cached values are recursively immutable dict/list subclasses. deepcopy returns
ordinary mutable containers for the existing recovery/finalization writers.
The bounded LRU retains at most 16 values by default; evicted values remain
immutable if callers retain them. Exit checking audits retained values only,
not an unbounded historical file inventory. Fixed-source boundary validation
remains the caller's separate responsibility.
"""
from collections import OrderedDict
from contextlib import contextmanager
import copy
import hashlib
import json
import os
from pathlib import Path
import stat


class OverlayCacheContractError(RuntimeError):
    """The original file/function or cached immutable data changed unexpectedly."""


def require(condition, message):
    if not condition:
        raise OverlayCacheContractError(message)


def _immutable(*args, **kwargs):
    raise TypeError('Cached overlay is immutable; use copy.deepcopy before editing')


class FrozenDict(dict):
    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = __ior__ = _immutable

    def __new__(cls,*args,**kwargs):
        result=dict.__new__(cls)
        dict.update(result,*args,**kwargs)
        return result

    def __init__(self,*args,**kwargs):
        # Construction happened in __new__; reinitialization cannot mutate it.
        pass

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        result = {}
        memo[id(self)] = result
        result.update((copy.deepcopy(key,memo),copy.deepcopy(value,memo)) for key,value in self.items())
        return result


class FrozenList(list):
    __setitem__ = __delitem__ = append = clear = extend = insert = pop = remove = reverse = sort = __iadd__ = __imul__ = _immutable

    def __new__(cls,values=()):
        result=list.__new__(cls)
        list.extend(result,values)
        return result

    def __init__(self,*args,**kwargs):
        pass

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        result = []
        memo[id(self)] = result
        result.extend(copy.deepcopy(value,memo) for value in self)
        return result


def freeze(value):
    if isinstance(value,dict):
        return FrozenDict({key:freeze(item) for key,item in value.items()})
    if isinstance(value,list):
        return FrozenList(freeze(item) for item in value)
    require(value is None or type(value) in (str,bool,int,float), 'Only original JSON values may be cached')
    return value


def canonical(path):
    path = Path(path).expanduser().absolute()
    require('..' not in path.parts, 'Overlay path must not contain parent traversal')
    return path


def _signature(info):
    return (info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns,info.st_ctime_ns,
            info.st_mode,info.st_uid,info.st_gid)


def actual_sha(path):
    """Fresh physical bytes only: no stat-only digest memoization."""
    path = canonical(path)
    for ancestor in (*reversed(path.parents),path):
        require(not ancestor.is_symlink(), 'Overlay path contains a symlink: '+str(ancestor))
    fd = os.open(path,os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before=os.fstat(fd)
        require(stat.S_ISREG(before.st_mode), 'Overlay must be a regular file')
        digest=hashlib.sha256()
        while True:
            block=os.read(fd,1024*1024)
            if not block:
                break
            digest.update(block)
        after=os.fstat(fd)
        require(_signature(before) == _signature(after) == _signature(path.lstat()),
                'Overlay changed while its actual bytes were being hashed')
        return digest.hexdigest()
    finally:
        os.close(fd)


def memory_sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
        allow_nan=False).encode()).hexdigest()


class ReadOverlayCache:
    def __init__(self,original,*,max_entries=16,exclude_roots=()):
        require(callable(original) and type(max_entries) is int and 1 <= max_entries <= 128,
                'Callable original reader and bounded 1..128 entry LRU required')
        self.original=original
        self.max_entries=max_entries
        self.exclude_roots=tuple(canonical(root) for root in exclude_roots)
        self.cache=OrderedDict()
        self.stats=dict(calls=0,actual_hash_reads=0,hits=0,misses=0,passthrough=0,
            evictions=0,path_revisions=0,retained_verified=0)

    def _sha(self,path):
        result=actual_sha(path)
        self.stats['actual_hash_reads']+=1
        return result

    def _check_memory(self,record):
        require(memory_sha(record['value']) == record['memory_sha256'],
                'Cached overlay memory changed despite immutable API')

    def read(self,path):
        self.stats['calls']+=1
        path=canonical(path)
        if any(path == root or root in path.parents for root in self.exclude_roots):
            self.stats['passthrough']+=1
            return self.original(path)
        checksum=self._sha(path)
        key=(str(path),checksum)
        if key in self.cache:
            self.stats['hits']+=1
            self.cache.move_to_end(key)
            return self.cache[key]['value']
        self.stats['misses']+=1
        result=self.original(path)  # All original schema/digest/group checks.
        require(self._sha(path) == checksum,'Overlay changed during original validation')
        require(isinstance(result,dict),'Original overlay reader must return a JSON mapping')
        value=freeze(result)
        record=dict(value=value,memory_sha256=memory_sha(value))
        # Keep only the latest validated content for a path. The enclosing
        # fixed-source boundary independently decides whether changes are legal.
        for old in [other for other in self.cache if other[0] == str(path)]:
            self._check_memory(self.cache[old]);self.cache.pop(old)
            self.stats['path_revisions']+=1
        self.cache[key]=record
        while len(self.cache)>self.max_entries:
            oldest=next(iter(self.cache));self._check_memory(self.cache[oldest])
            self.cache.pop(oldest);self.stats['evictions']+=1
        return value

    def verify(self):
        for (path,checksum),record in self.cache.items():
            require(self._sha(path) == checksum,'Retained overlay bytes changed before scope exit: '+path)
            self._check_memory(record)
            self.stats['retained_verified']+=1
        return dict(self.stats,retained_entries=len(self.cache))


@contextmanager
def installed(layout_module=None,*,max_entries=16,exclude_roots=()):
    """Temporarily replace only read_overlay; restore even on failure."""
    if layout_module is None:
        import base5_layout_split as layout_module
    original=layout_module.read_overlay
    require(not getattr(original,'_base5_overlay_cache',False),'Overlay cache is already installed')
    cache=ReadOverlayCache(original,max_entries=max_entries,exclude_roots=exclude_roots)
    def read(path):
        return cache.read(path)
    read._base5_overlay_cache=True
    layout_module.read_overlay=read
    try:
        yield cache
    finally:
        changed=layout_module.read_overlay is not read
        layout_module.read_overlay=original
        require(not changed,'Scoped overlay reader ownership changed')
        cache.verify()
