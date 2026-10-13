#!/usr/bin/env python3
"""Fresh-environment dashboard that follows its verified completion lineage.

Read-only observation; never starts a collector or changes suite/capture files.
Legacy 8075 dashboard stays unmodified. Completion already includes the parent;
do NOT add its accepted count to the parent's accepted count.
"""
from pathlib import Path
import threading
import json
import re
import base5_capture_dashboard as ui

NativeDashboard=ui.Dashboard


def lineage_header(path):
    # Native producer writes contract/source/baseline FIRST. Never parse the
    # full 5K baseline audit array on every browser refresh (tens of MiB).
    try:
        with Path(path).open() as handle:prefix=handle.read(8192)
        if not re.match(r'\s*\{\s*"contract"\s*:\s*\{',prefix):return {}
        result={}
        for key in ('source_sha256','baseline'):
            match=re.search('"'+key+'"\\s*:\\s*',prefix)
            if not match:return {}
            value,_=json.JSONDecoder().raw_decode(prefix[match.end():])
            if not isinstance(value,str):return {}
            result[key]=value
        return result
    except (OSError,ValueError):return {}


def completion_root(parent,manifest_sha):
    parent=Path(parent);candidate=parent.parent/(parent.name+'_completed')
    contract=lineage_header(candidate/'state.json')
    status=ui.read_optional(candidate/'status.json') or {}
    if (contract.get('source_sha256')==manifest_sha and contract.get('baseline')==str(parent.resolve())
            and status.get('flexible_completion') and (candidate/'selected_ledger.jsonl').exists()):
        return candidate
    return parent


class FollowingDashboard:
    def __init__(self,root,manifest=None,suite=None,history_file=None,storage_roots=()):
        self.root=Path(root);self.manifest=manifest;self.suite=suite;self.history_file=history_file
        self.storage_roots=storage_roots;self.parent=NativeDashboard(root,manifest,suite,history_file,storage_roots)
        self.child=None;self.lock=threading.RLock()

    def current(self):
        with self.lock:
            selected=completion_root(self.root,self.parent.manifest_sha256)
            if selected==self.root:return self.parent
            if self.child is None:
                self.child=NativeDashboard(selected,self.manifest,self.suite,self.history_file,
                    (self.root,*self.storage_roots))
                self.child.history.extend(self.parent.history)
            return self.child

    def sample_storage(self):self.current().sample_storage()

    def payload(self,now=None):
        current=self.current();status=current.payload(now)
        status['canonical_selection_followed']=current is not self.parent
        status['parent_capture_root']=str(self.root.resolve())
        if self.parent.suite:
            progress=[]
            for env in self.parent.suite['environments']:
                root=completion_root(env['capture_output'],env['manifest_sha256'])
                item=status if root.resolve()==current.root.resolve() else ui.read_optional(root/'status.json') or {}
                progress.append(dict(environment=env['environment'],stage=item.get('stage','not_started'),
                    requested=env['expected_frames'],accepted=item.get('accepted_candidate',0),
                    finished=item.get('finished',0),review=item.get('review',0)))
            status['suite_progress']=progress;status['suite_accepted']=sum(e['accepted'] for e in progress)
        return status


if __name__=='__main__':
    ui.Dashboard=FollowingDashboard
    ui.main()
