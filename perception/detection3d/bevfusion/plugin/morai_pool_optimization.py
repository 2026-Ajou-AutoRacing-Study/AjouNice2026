"""Opt-in experimental cache A / separated pool B; no production defaults change.

Use enable_pool_optimization(view_transform, 'cache'|'separated'). No parameters
are added or changed. Training/grad-enabled calls never reuse a geometry cache.
The legacy truncating .long() boundary convention and [B,C,X,Y] pool order remain.
"""
import hashlib
import math
from pathlib import Path
import types

import torch


_EXT = None


def extension():
    global _EXT
    if _EXT is None:
        from torch.utils.cpp_extension import load
        source = Path(__file__).with_name('separated_bev_pool.cu')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:10]
        _EXT = load(name='morai_separated_pool_' + digest, sources=[str(source)],
                    extra_cuda_cflags=['-O3', '--fmad=false'], verbose=False)
    return _EXT


def build_plan(view, geom, compact_indices=False):
    """Make compact sorted frustum indices, without an [frustum,C] feature tensor."""
    B, N, D, H, W, _ = geom.shape
    # 201.6 / .2 is 1007.9999999999999 in Python. Truncation silently
    # removes a padding column even when stride-2 output shapes still match.
    # Follow config-owned integer geometry, as restore_view_geometry does.
    counts = [(b[1]-b[0])/b[2] for b in
              (view.xbound, view.ybound, view.zbound)]
    nx = tuple(round(n) for n in counts)
    if any(abs(n-rounded)>1e-6 or rounded<=0 for n,rounded in zip(counts,nx)):
        raise ValueError('Pooling bounds must define an integral positive grid')
    if compact_indices and (B*N*D*H*W>=2**31 or math.prod(nx)*B>=2**31):
        raise ValueError('Compact pooling plan exceeds int32 index range')
    # Integer truncation and rank ordering stay identical for the supported
    # domain; fewer bytes are moved and radix-sorted in every training batch.
    values=(geom-(view.bx-view.dx/2))/view.dx
    coords = (values.int() if compact_indices else values.long()).reshape(-1,3)
    valid = ((coords >= 0) & (coords < coords.new_tensor(nx))).all(dim=1)
    rd = torch.where(valid)[0]
    if compact_indices:rd=rd.int()
    xyz = coords[rd]
    batch = rd // (N*D*H*W)
    # Existing sort order x,y,z,b (not output-contiguous b,z,x,y).
    rank = xyz[:, 0]*(nx[1]*nx[2]*B)+xyz[:, 1]*(nx[2]*B)+xyz[:, 2]*B+batch
    order = rank.argsort()
    rd, xyz, batch, rank = rd[order], xyz[order], batch[order], rank[order]
    coords4 = torch.cat((xyz, batch[:, None]), dim=1).int().contiguous()
    rf = ((rd//(D*H*W))*H*W + rd%(H*W)).int().contiguous()
    rb = (((batch*nx[2]+xyz[:, 2])*nx[0]+xyz[:, 0])*nx[1]+xyz[:, 1]).int().contiguous()
    if rd.numel():
        keep = torch.ones_like(rank, dtype=torch.bool)
        keep[1:] = rank[1:] != rank[:-1]
        starts = torch.where(keep)[0].int()
        lengths = torch.diff(torch.cat((starts, starts.new_tensor([rd.numel()]))))
    else:
        starts = rd.new_empty(0, dtype=torch.int32)
        lengths = starts.clone()
    return dict(rd=rd.int().contiguous(), rf=rf, rb=rb, starts=starts.contiguous(),
                lengths=lengths.contiguous(), coords=coords4,
                shape=(B,nx[2],nx[0],nx[1]), input_shape=(B,N,D,H,W))


def separated_reference(depth, feat, plan):
    """Autograd-capable PyTorch reference, CPU or CUDA, used by unit tests."""
    rd,rf,rb=(plan[k].long() for k in ('rd','rf','rb'))
    values=depth.reshape(-1)[rd,None]*feat.reshape(-1,feat.shape[-1])[rf]
    out=feat.new_zeros((math.prod(plan['shape']),feat.shape[-1]))
    return out.index_add(0,rb,values).reshape(*plan['shape'],feat.shape[-1])


def batched_geometry(view, camera2lidar_rots, camera2lidar_trans, intrins,
                     post_rots, post_trans, repair_grid_boundaries=False, **kwargs):
    """Experimental algebra-preserving layout, without repeated 3x3 matrices.

    BMM kernels can round differently near voxel boundaries. This is NOT the
    default; qualification must compare cell indices, not only XYZ tolerance.
    """
    B,N,_=camera2lidar_trans.shape
    shape=(B,N,*view.frustum.shape)
    def apply(matrix,points):
        values=points.reshape(B*N,-1,3).transpose(1,2)
        return torch.bmm(matrix.reshape(B*N,3,3),values).transpose(1,2).reshape(shape)
    inverse_post=torch.inverse(post_rots)
    combine=camera2lidar_rots.matmul(torch.inverse(intrins))
    points=view.frustum-post_trans.view(B,N,1,1,1,3)
    points=apply(inverse_post,points)
    points=torch.cat((points[...,:2]*points[...,2:3],points[...,2:3]),dim=-1)
    points=apply(combine,points)
    points+=camera2lidar_trans.view(B,N,1,1,1,3)
    if 'extra_rots' in kwargs:
        points=apply(kwargs['extra_rots'][:,None].expand(B,N,3,3),points)
    if 'extra_trans' in kwargs:
        points+=kwargs['extra_trans'].view(B,1,1,1,1,3)
    if repair_grid_boundaries:
        # Different GEMM layouts round by a few micrometres. Re-evaluate only
        # rays within 1mm of a possible grid boundary using the ORIGINAL 3x3
        # matrix-vector layout, including truncation at the negative boundary.
        scaled=(points-(view.bx-view.dx/2))/view.dx
        possible=((scaled>=-1.01)&(scaled<=view.nx+1)).all(dim=-1)
        near=((scaled-scaled.round()).abs()<=.001/view.dx).any(dim=-1)
        selected=torch.where((possible&near).reshape(-1))[0]
        if hasattr(view,'_pool_optimization'):
            view._pool_optimization['boundary_rays_recomputed']=selected.numel()
        rays=view.frustum.numel()//3
        camera=selected//rays;batch=camera//N
        fixed=view.frustum.reshape(-1,3)[selected%rays]-post_trans.reshape(-1,3)[camera]
        fixed=inverse_post.reshape(-1,3,3)[camera].matmul(fixed.unsqueeze(-1))
        fixed=torch.cat((fixed[:,:2]*fixed[:,2:3],fixed[:,2:3]),dim=1)
        fixed=combine.reshape(-1,3,3)[camera].matmul(fixed).squeeze(-1)
        fixed+=camera2lidar_trans.reshape(-1,3)[camera]
        if 'extra_rots' in kwargs:
            fixed=kwargs['extra_rots'][batch].matmul(fixed.unsqueeze(-1)).squeeze(-1)
        if 'extra_trans' in kwargs:fixed+=kwargs['extra_trans'][batch]
        points=points.contiguous()
        points.reshape(-1,3)[selected]=fixed
    return points


class SeparatedPool(torch.autograd.Function):
    @staticmethod
    def forward(ctx, depth, feat, plan, backward_mode='legacy'):
        if backward_mode not in ('legacy', 'warp', 'grouped'):
            raise ValueError('Unknown separated pooling backward mode')
        depth,feat=depth.contiguous().float(),feat.contiguous().float()
        out=extension().forward(depth,feat,*(plan[k] for k in
                 ('rd','rf','rb','starts','lengths')),math.prod(plan['shape']))
        indices=[plan['rd'],plan['rf'],plan['rb']]
        if backward_mode=='grouped' and any(ctx.needs_input_grad[:2]):
            # Only training needs this second ordering. Never cache/reuse a
            # training geometry plan across independently augmented batches.
            order=plan['rf'].argsort()
            indices=[index[order].contiguous() for index in indices]
            if order.numel():
                keep=torch.ones_like(indices[1],dtype=torch.bool)
                keep[1:]=indices[1][1:]!=indices[1][:-1]
                starts=torch.where(keep)[0].int()
                lengths=torch.diff(torch.cat((starts,starts.new_tensor([order.numel()]))))
            else:
                starts=plan['starts'].new_empty(0);lengths=starts.clone()
            indices.extend((starts.contiguous(),lengths.contiguous()))
        ctx.save_for_backward(depth,feat,*indices)
        ctx.shape=plan['shape']
        ctx.backward_mode=backward_mode
        return out.reshape(*plan['shape'],feat.shape[-1])

    @staticmethod
    def backward(ctx, grad):
        depth,feat,*indices=ctx.saved_tensors
        grad=grad.contiguous().reshape(-1,feat.shape[-1])
        if ctx.backward_mode=='grouped':
            gd,gf=extension().backward_grouped(grad,depth,feat,*indices)
        else:
            gd,gf=extension().backward(grad,depth,feat,*indices,ctx.backward_mode=='warp')
        return (gd,gf)+tuple(None for _ in ctx.needs_input_grad[2:])


def signature(view, args, kwargs):
    """Value-based calibration key. A small GPU->CPU sync is included in timings."""
    h=hashlib.sha256()
    # Cache tensors belong to the execution stream that created them. Do not
    # reuse an asynchronous allocation on an unrelated stream without events.
    h.update(str(torch.cuda.current_stream(view.frustum.device).cuda_stream
                 if view.frustum.is_cuda else 'cpu').encode())
    for t in list(args)+[kwargs[k] for k in sorted(kwargs)]:
        if not torch.is_tensor(t):
            raise TypeError('Only tensor geometry arguments are supported')
        if t.requires_grad:
            raise ValueError('Calibration gradients require the uncached path')
        h.update(str((t.shape,t.dtype,t.device)).encode())
        h.update(t.detach().contiguous().cpu().numpy().tobytes())
    h.update(str((view.image_size,view.feature_size,view.xbound,view.ybound,
                  view.zbound,view.dbound,view.frustum.device,view.frustum.dtype)).encode())
    for name in ('dx','bx','nx','frustum'):
        t=getattr(view,name)
        h.update(str((t.data_ptr(),t._version)).encode())
    return h.digest()


def enable_pool_optimization(view, mode='cache', cache_limit_bytes=256*2**20,
                             rectangular=True, backward_mode='legacy',compact_indices=False,
                             geometry_mode='legacy'):
    """Attach reversible, instance-only methods; default model remains untouched."""
    if mode not in ('cache','separated') or hasattr(view,'_pool_optimization'):
        raise ValueError('Invalid mode or already optimized')
    if backward_mode not in ('legacy','warp','grouped'):
        raise ValueError('Unknown separated pooling backward mode')
    if geometry_mode not in ('legacy','batched_checked'):
        raise ValueError('Unknown geometry execution mode')
    originals={k:getattr(view,k) for k in ('get_geometry','get_cam_feats','bev_pool')}
    state=dict(mode=mode,entries={},hits=0,misses=0,bypasses=0,
               bytes=0,plan=None,limit=int(cache_limit_bytes),originals=originals,
               backward_mode=backward_mode,compact_indices=bool(compact_indices),geometry_mode=geometry_mode,
               fast_geometry_calls=0,legacy_geometry_calls=0)
    view._pool_optimization=state

    def geometry(self,*args,**kwargs):
        cacheable=not self.training and not torch.is_grad_enabled()
        if cacheable:
            key=signature(self,args,kwargs)
            if key in state['entries']:
                state['hits']+=1
                geom,plan=state['entries'][key];state['plan']=plan
                return geom
        else:
            state['bypasses']+=1
        # The 1mm boundary guard is qualified for FP32, not TF32/FP16 GEMM.
        # A future precision change must not silently use this fast path.
        precise=self.frustum.dtype==torch.float32 and all(
            t.dtype==torch.float32 for t in (*args,*kwargs.values()) if torch.is_tensor(t)) and not (self.frustum.is_cuda and (
            torch.backends.cuda.matmul.allow_tf32 or (torch.is_autocast_enabled('cuda') and
                torch.get_autocast_dtype('cuda')!=torch.float32)))
        state['geometry_guard']=dict(training=self.training,tf32=torch.backends.cuda.matmul.allow_tf32,
            autocast=torch.is_autocast_enabled('cuda'),autocast_dtype=str(torch.get_autocast_dtype('cuda')),
            calibration_grad=any(
                t.requires_grad for t in (*args,*kwargs.values()) if torch.is_tensor(t)))
        if geometry_mode=='batched_checked' and self.training and precise and not any(
                t.requires_grad for t in (*args,*kwargs.values()) if torch.is_tensor(t)):
            geom=batched_geometry(self,*args,repair_grid_boundaries=True,**kwargs)
            state['fast_geometry_calls']+=1
        else:
            geom=originals['get_geometry'](*args,**kwargs)
            state['legacy_geometry_calls']+=1
        plan=build_plan(self,geom,compact_indices=state['compact_indices']);state['plan']=plan
        state['misses']+=1
        if cacheable:
            size=geom.numel()*geom.element_size()+sum(v.numel()*v.element_size()
                     for v in plan.values() if torch.is_tensor(v))
            state['entries'].clear();state['bytes']=0
            if size<=state['limit']:
                state['entries'][key]=(geom,plan);state['bytes']=size
        return geom

    def cam_feats(self,img,depth_image):
        if mode=='cache':
            return originals['get_cam_feats'](img,depth_image)
        B,N,C,H,W=img.shape
        d=self.dtransform(depth_image.reshape(B*N,*depth_image.shape[2:]))
        pred=self.depthnet(torch.cat((d,img.reshape(B*N,C,H,W)),dim=1))
        depth=pred[:,:self.D].softmax(dim=1).reshape(B,N,self.D,H,W)
        feat=pred[:,self.D:self.D+self.C].reshape(B,N,self.C,H,W)
        return depth,feat.permute(0,1,3,4,2).contiguous()

    def pool(self,geom,x):
        plan=state['plan']
        if plan is None:
            raise RuntimeError('Geometry must be computed before pooling')
        if mode=='cache' and (self.training or torch.is_grad_enabled()):
            return originals['bev_pool'](geom,x)
        if mode=='separated':
            depth,feat=x
            if tuple(depth.shape)!=plan['input_shape'] or tuple(feat.shape[:-1])!=(
                    *plan['input_shape'][:2],*plan['input_shape'][3:]):
                raise ValueError('Depth/feature dimensions differ from geometry plan')
            out=SeparatedPool.apply(depth,feat,plan,state['backward_mode'])
        else:
            # The upstream legacy materialized CUDA pool launches on the
            # default stream. Cache A is deliberately restricted to that
            # contract; separated B uses the current PyTorch stream.
            if geom.is_cuda and torch.cuda.current_stream(geom.device)!=torch.cuda.default_stream(geom.device):
                raise RuntimeError('Cache-only legacy pool requires the default CUDA stream')
            from projects.BEVFusion.bevfusion.ops.bev_pool import bev_pool_ext
            B,Z,X,Y=plan['shape']
            if plan['rd'].numel():
                out=bev_pool_ext.bev_pool_forward(
                    x.reshape(-1,x.shape[-1])[plan['rd'].long()].contiguous(),
                    plan['coords'],plan['lengths'],plan['starts'],B,Z,X,Y)
            else:
                out=x.new_zeros((B,Z,X,Y,x.shape[-1]))
        out=out.permute(0,4,1,2,3).contiguous()
        out=torch.cat(out.unbind(dim=2),dim=1)
        return out.transpose(-1,-2).contiguous() if rectangular else out

    for name,method in [('get_geometry',geometry),('get_cam_feats',cam_feats),('bev_pool',pool)]:
        setattr(view,name,types.MethodType(method,view))
    return state


def disable_pool_optimization(view):
    for name,method in view._pool_optimization['originals'].items():
        setattr(view,name,method)
    del view._pool_optimization
