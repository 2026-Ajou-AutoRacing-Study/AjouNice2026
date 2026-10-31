"""Read-only original-MGeo release geometry; no ROS/native API or map mutation.

The movable Lanelet ref_line remains the stopping target. This module only
retains mission identity until the ordered route crosses its original paint.
"""
import math


def _number(value):
    return type(value) in (int,float) and math.isfinite(value)


def parse_judging_lines(rows):
    """Return valid unique IDs and diagnostics without refusing node startup."""
    lines,errors={},[]
    if rows is None:
        return lines,errors
    if not isinstance(rows,list) or len(rows)>32:
        return {},['signal_judging_lines must be a list of at most32 rows']
    seen=set()
    for index,row in enumerate(rows):
        gid=row.get('regulatory_id') if isinstance(row,dict) else None
        try:
            if type(gid) is not int or gid<=0:
                raise ValueError('positive integer regulatory_id required')
            if gid in seen:
                raise ValueError('duplicate regulatory_id')
            seen.add(gid)
            coordinates,direction=row.get('line_xy'),row.get('approach_xy')
            if (not isinstance(coordinates,list) or not 4<=len(coordinates)<=32 or len(coordinates)%2
                    or not all(_number(value) for value in coordinates)):
                raise ValueError('finite flattened line_xy required')
            if (not isinstance(direction,list) or len(direction)!=2
                    or not all(_number(value) for value in direction)
                    or not .99<=math.hypot(*direction)<=1.01):
                raise ValueError('unit approach_xy required')
            points=list(zip(coordinates[::2],coordinates[1::2]))
            for a,b in zip(points,points[1:]):
                dx,dy=b[0]-a[0],b[1]-a[1]
                length=math.hypot(dx,dy)
                if length<.1 or abs(dx*direction[1]-dy*direction[0])/length<.5:
                    raise ValueError('degenerate or approach-parallel line')
            lines[gid]=dict(points=points,approach=tuple(direction))
        except (ValueError,TypeError) as error:
            if type(gid) is int:
                lines.pop(gid,None)
            errors.append('index={} reg={}: {}'.format(index,gid,error))
    return lines,errors


def route_judging_progress(line,route_geometry,virtual_progress_m):
    """Intersect finite original paint with the ordered route occurrence.

    Unlike projecting its midpoint onto the associated lane, intersection may
    lie on a successor lane beyond the moved ref_line. Opposite-direction and
    remote/repeated route occurrences are never used as a convenient fallback.
    """
    hits=[]
    for _index,start,preferred_length,alternatives in route_geometry:
        for _lane_id,points,length in alternatives:
            offset=0.
            for a,b in zip(points,points[1:]):
                vx,vy=b[0]-a[0],b[1]-a[1]
                size=math.hypot(vx,vy)
                if size<1e-9:
                    continue
                forward=(vx*line['approach'][0]+vy*line['approach'][1])/size
                if forward>.1:
                    for c,d in zip(line['points'],line['points'][1:]):
                        wx,wy=d[0]-c[0],d[1]-c[1]
                        denominator=vx*wy-vy*wx
                        if abs(denominator)<1e-9:
                            continue
                        dx,dy=c[0]-a[0],c[1]-a[1]
                        t=(dx*wy-dy*wx)/denominator
                        u=(dx*vy-dy*vx)/denominator
                        if -1e-9<=t<=1.+1e-9 and -1e-9<=u<=1.+1e-9:
                            progress=start+preferred_length*(offset+max(0.,min(1.,t))*size)/length
                            if virtual_progress_m-1.5<=progress<=virtual_progress_m+50.:
                                hits.append(progress)
                offset+=size
    return min(hits,key=lambda progress:abs(progress-virtual_progress_m)) if hits else None
