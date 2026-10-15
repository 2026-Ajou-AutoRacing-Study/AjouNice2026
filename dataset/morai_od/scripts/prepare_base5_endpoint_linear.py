#!/usr/bin/env python3
"""Fresh two-endpoint candidates; short person paths borrow bounded saved endpoint heights, pending live QA."""
import argparse
import copy
import statistics
from pathlib import Path

import generate_base5_independent as gen
import prepare_base5_intersection_ground_pilot as pilot
from base5_capture_safety import atomic_json,sha256
from base5_intersection_ground import height,load_lookup
from base5_intersection_placement import ENDPOINT_MODE


def at(profile,s):
    c,v=profile['center'],profile['axis']
    return height(profile,c['x']+s*v[0],c['y']+s*v[1])


def endpoint_profile(profile,donor,donor_key):
    """Each endpoint lies within its recorded source. Cross-asset extension is a candidate, not certification."""
    first,last=-11.9,12.
    own_last=profile['knots'][-1]['s']
    lower=at(profile,first);offset=0.
    borrowed=own_last<last
    if borrowed:
        # Avoid the unchanged provisional first seed. Compare actual overlapping
        # editor heights; preserve the measured local pivot difference only.
        comparisons=[at(profile,s)['z']-at(donor,s)['z'] for s in (-6.,-3.,0.)]
        if max(map(abs,comparisons))>.05:raise ValueError('Endpoint donor has incompatible overlap heights')
        offset=statistics.median(comparisons)
        upper=at(donor,last);upper['z']+=offset
    else:upper=at(profile,last)
    result=copy.deepcopy(profile)
    result.update(knots=[dict(lower,l=0.,x=lower['x'],y=lower['y']),
                         dict(upper,l=0.,x=upper['x'],y=upper['y'])],
                  s_bounds_m=[first,last],waypoint_mode=ENDPOINT_MODE,contact_verified=False,
                  live_operational_qa_passed=False,
                  transfer_verified_within_live_scope=False,
                  endpoint_derivation=dict(source_mode='saved-endpoint-heights-linear-interior',
                    borrowed_upper=borrowed,donor_profile=donor_key if borrowed else None,
                    overlap_median_offset_m=offset,
                    limitation='Interior is deliberately a straight chord like sample motion, not terrain following. Donor extension requires fresh asset/direction live QA.'))
    return result


def prepare(lookup_file,prepared_file,output):
    if output.exists():raise FileExistsError('Fresh endpoint diagnostic output required')
    old=load_lookup(lookup_file)
    if old.get('status')!='LIVE_VALIDATED':raise ValueError('Archived live source required')
    evidence=old['live_evidence']
    if sha256(evidence['path'])!=evidence['sha256']:raise ValueError('Source report changed')
    data=copy.deepcopy(old);data.pop('live_evidence',None)
    data['status']='ENDPOINT_LINEAR_PENDING_LIVE';data['profiles']={}
    data['sources']+= [dict(path=str(lookup_file.resolve()),sha256=sha256(lookup_file)),
                      dict(path=evidence['path'],sha256=evidence['sha256'])]
    prepared=gen.geo.load_json(prepared_file);templates={r['crosswalk']['crosswalk_id']:r for r in prepared['scenes'] if r['asset']=='Woman2'}
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    assets={a['name']:a for pool in ctx['pools'].values() for a in pool}
    rows=[]
    for key,profile in sorted(old['profiles'].items()):
        crossing=profile['crosswalk_id'];donor_key=crossing+'/Ped_Pet1'
        data['profiles'][key]=endpoint_profile(profile,old['profiles'][donor_key],donor_key)
        asset=assets[profile['asset']];row=copy.deepcopy(templates[crossing])
        row.update(number=200+len(rows),asset=asset['name'],data_id=asset['data_id'],source_list=asset['source_list'])
        rows.append(row)
    output.mkdir(parents=True)
    lookup=output/'height_lookup_pending.json';atomic_json(lookup,data)
    prepared.update(scenes=rows,count=len(rows));source=output/'endpoint_prepared.json';atomic_json(source,prepared)
    return pilot.prepare(lookup,source,output/'pilot',fast_people=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lookup',type=Path,required=True);p.add_argument('--prepared-manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(prepare(a.lookup,a.prepared_manifest,a.output))
