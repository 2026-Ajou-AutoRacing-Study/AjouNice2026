#!/usr/bin/env python3
"""CPU/CUDA correctness, calibration invalidation and nondefault-stream tests."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import unittest

import torch
from projects.BEVFusion.bevfusion.depth_lss import DepthLSSTransform
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import MoraiRectangularDepthLSSTransform
from perception.detection3d.bevfusion.plugin.morai_pool_optimization import (
    build_plan, separated_reference, SeparatedPool, enable_pool_optimization,
    disable_pool_optimization, extension,batched_geometry)


class PoolTests(unittest.TestCase):
    device='cpu'

    def setup_inputs(self,empty=False):
        torch.manual_seed(17)
        v=DepthLSSTransform(8,4,(32,32),(4,4),(-2,2,1),(-3,3,1),(-1,1,1),(1,4,1)).to(self.device)
        geom=torch.randn(2,3,3,4,4,3,device=self.device)*2
        if empty:geom.fill_(100)
        return v,geom

    def test_forward_and_gradient(self):
        for empty in (False,True):
            v,g=self.setup_inputs(empty);p=build_plan(v,g)
            d=torch.rand(2,3,3,4,4,device=self.device,requires_grad=True)
            f=torch.randn(2,3,4,4,4,device=self.device,requires_grad=True)
            ref=separated_reference(d,f,p)
            if self.device=='cuda':
                actual=SeparatedPool.apply(d,f,p)
                torch.testing.assert_close(actual,ref,atol=2e-6,rtol=2e-6)
                grad=torch.randn_like(ref)
                expected=torch.autograd.grad(ref,(d,f),grad,retain_graph=True)
                observed=torch.autograd.grad(actual,(d,f),grad)
                for a,b in zip(observed,expected):
                    torch.testing.assert_close(a,b,atol=3e-6,rtol=3e-6)
            else:
                self.assertTrue(torch.autograd.gradcheck(
                    lambda x,y:separated_reference(x,y,p),(d.double(),f.double())))

    def test_legacy_boundary_and_batch(self):
        v,g=self.setup_inputs();g.fill_(100)
        # Preserve .long() truncation of -0.25, not a geometry-policy change.
        g[0,0,0,0,0]=g.new_tensor([-2.25,-3.25,-1.25])
        g[1,0,0,0,0]=g.new_tensor([1.2,2.2,.2])
        p=build_plan(v,g)
        self.assertEqual(sorted(p['rd'].cpu().tolist()),[0,144])
        self.assertEqual(sorted(p['rb'].cpu().tolist()),[0,95])

    def test_warp_backward_empty_and_channel_tails(self):
        if self.device!='cuda':self.skipTest('CUDA-only')
        for empty in (False,True):
            v,g=self.setup_inputs(empty);p=build_plan(v,g)
            for channels in (1,4,31,32,80,81):
                d=torch.rand(2,3,3,4,4,device='cuda',requires_grad=True)
                f=torch.randn(2,3,4,4,channels,device='cuda',requires_grad=True)
                ref=separated_reference(d,f,p)
                actual=SeparatedPool.apply(d,f,p,'warp')
                torch.testing.assert_close(actual,ref,atol=2e-6,rtol=2e-6)
                grad=torch.randn_like(ref)
                expected=torch.autograd.grad(ref,(d,f),grad)
                observed=torch.autograd.grad(actual,(d,f),grad)
                for a,b in zip(observed,expected):
                    torch.testing.assert_close(a,b,atol=2e-5,rtol=2e-5)

    def test_invalid_backward_mode(self):
        v,g=self.setup_inputs()
        with self.assertRaisesRegex(ValueError,'backward mode'):
            enable_pool_optimization(v,'separated',backward_mode='unknown')

    def test_grouped_backward_empty_and_channel_tails(self):
        if self.device!='cuda':self.skipTest('CUDA-only')
        for empty in (False,True):
            v,g=self.setup_inputs(empty);p=build_plan(v,g,compact_indices=True)
            for channels in (1,4,31,32,80,81):
                d=torch.rand(2,3,3,4,4,device='cuda',requires_grad=True)
                f=torch.randn(2,3,4,4,channels,device='cuda',requires_grad=True)
                ref=separated_reference(d,f,p);actual=SeparatedPool.apply(d,f,p,'grouped')
                torch.testing.assert_close(actual,ref,atol=2e-6,rtol=2e-6)
                grad=torch.randn_like(ref)
                expected=torch.autograd.grad(ref,(d,f),grad)
                observed=torch.autograd.grad(actual,(d,f),grad)
                for a,b in zip(observed,expected):torch.testing.assert_close(a,b,atol=2e-5,rtol=2e-5)

    def test_compact_plan_matches_legacy(self):
        for empty in (False,True):
            v,g=self.setup_inputs(empty)
            a=build_plan(v,g);b=build_plan(v,g,compact_indices=True)
            for key in a:
                if torch.is_tensor(a[key]):
                    torch.testing.assert_close(a[key],b[key],rtol=0,atol=0)
                else:self.assertEqual(a[key],b[key])

    def test_compact_plan_rejects_index_overflow(self):
        v,g=self.setup_inputs();v.xbound=(-2,2,1e-9)
        with self.assertRaisesRegex(ValueError,'int32 index range'):
            build_plan(v,g,compact_indices=True)

    def test_batched_geometry_nonsquare_and_extra_transforms(self):
        v=DepthLSSTransform(8,4,(32,48),(4,6),(-2,2,1),(-3,3,1),
                            (-1,1,1),(1,4,1)).to(self.device)
        matrices=torch.eye(3,device=self.device)[None,None].repeat(2,3,1,1)
        matrices[...,0,0]=1.13;matrices[...,1,1]=.91;matrices[...,0,1]=.12
        translation=torch.randn(2,3,3,device=self.device)*.1
        args=(matrices,translation,matrices,matrices,translation)
        kwargs=dict(extra_rots=matrices[:,0],extra_trans=translation[:,0])
        expected=v.get_geometry(*args,**kwargs)
        actual=batched_geometry(v,*args,**kwargs)
        torch.testing.assert_close(expected,actual,atol=2e-6,rtol=2e-6)

    def test_repaired_geometry_preserves_boundary_and_empty_plans(self):
        v,_=self.setup_inputs()
        for shift in (0.,100.):
            identity=torch.eye(3,device=self.device)[None,None].repeat(2,3,1,1)
            translation=torch.full((2,3,3),shift,device=self.device)
            args=(identity,translation,identity,identity,torch.zeros_like(translation))
            original=v.get_geometry(*args)
            actual=batched_geometry(v,*args,repair_grid_boundaries=True)
            a=build_plan(v,original);b=build_plan(v,actual,compact_indices=True)
            for key in ('rd','rf','rb','starts','lengths','coords'):
                torch.testing.assert_close(a[key],b[key],rtol=0,atol=0)

    def test_fast_geometry_training_only_and_calibration_gradient_fallback(self):
        v,_=self.setup_inputs()
        state=enable_pool_optimization(v,'separated',geometry_mode='batched_checked')
        identity=torch.eye(3,device=self.device)[None,None]
        translation=torch.zeros(1,1,3,device=self.device)
        args=(identity,translation,identity,identity,translation)
        v.train();v.get_geometry(*args)
        self.assertEqual(state['fast_geometry_calls'],1)
        gradient_translation=translation.clone().requires_grad_()
        actual=v.get_geometry(identity,gradient_translation,identity,identity,translation)
        self.assertEqual(state['legacy_geometry_calls'],1)
        actual.sum().backward();self.assertIsNotNone(gradient_translation.grad)
        v.eval()
        with torch.no_grad():v.get_geometry(*args)
        self.assertEqual(state['legacy_geometry_calls'],2)

    def test_fast_geometry_falls_back_with_double_calibration(self):
        v,_=self.setup_inputs();v.double();v.train()
        state=enable_pool_optimization(v,'separated',geometry_mode='batched_checked')
        identity=torch.eye(3,device=self.device,dtype=torch.float64)[None,None]
        translation=torch.zeros(1,1,3,device=self.device,dtype=torch.float64)
        v.get_geometry(identity,translation,identity,identity,translation)
        self.assertEqual(state['fast_geometry_calls'],0)
        self.assertEqual(state['legacy_geometry_calls'],1)

    def test_fast_geometry_falls_back_with_tf32(self):
        if self.device!='cuda':self.skipTest('CUDA-only precision guard')
        v,_=self.setup_inputs();v.train()
        state=enable_pool_optimization(v,'separated',geometry_mode='batched_checked')
        identity=torch.eye(3,device='cuda')[None,None];translation=torch.zeros(1,1,3,device='cuda')
        previous=torch.backends.cuda.matmul.allow_tf32
        try:
            torch.backends.cuda.matmul.allow_tf32=True
            v.get_geometry(identity,translation,identity,identity,translation)
        finally:torch.backends.cuda.matmul.allow_tf32=previous
        self.assertEqual(state['fast_geometry_calls'],0)
        self.assertEqual(state['legacy_geometry_calls'],1)

    def test_fast_geometry_accepts_float32_autocast_but_not_float16(self):
        if self.device!='cuda':self.skipTest('CUDA-only precision guard')
        v,_=self.setup_inputs();v.train()
        state=enable_pool_optimization(v,'separated',geometry_mode='batched_checked')
        identity=torch.eye(3,device='cuda')[None,None];translation=torch.zeros(1,1,3,device='cuda')
        args=(identity,translation,identity,identity,translation)
        with torch.autocast('cuda',dtype=torch.float32):v.get_geometry(*args)
        self.assertEqual(state['fast_geometry_calls'],1)
        with torch.autocast('cuda',dtype=torch.float16):v.get_geometry(*args)
        self.assertEqual(state['legacy_geometry_calls'],1)

    def test_fractional_pitch_does_not_truncate_last_padding_column(self):
        for pitch,expected in ((.2,1008),(.3,672),(.4,504)):
            v=DepthLSSTransform(8,4,(32,32),(4,4),(-100.8,100.8,pitch),
                (-33.6,33.6,pitch),(-10,10,20),(1,4,1)).to(self.device)
            g=torch.full((1,1,3,4,4,3),200.,device=self.device)
            g[0,0,0,0,0]=g.new_tensor([100.8-pitch/2,0,0])
            p=build_plan(v,g)
            self.assertEqual(p['shape'][2],expected)
            self.assertEqual(p['rd'].cpu().tolist(),[0])
            self.assertEqual(int(p['coords'][0,0]),expected-1)
        v.xbound=(-100.8,100.79,.4)
        with self.assertRaisesRegex(ValueError,'integral'):build_plan(v,g)

    def test_fresh_rectangular_training_has_rounded_grid_parameters(self):
        for pitch,nx in ((.2,1008),(.3,672),(.4,504)):
            view=MoraiRectangularDepthLSSTransform(8,4,(32,32),(4,4),
                (-100.8,100.8,pitch),(-33.6,33.6,pitch),(-10,10,20),(1,4,1)).to(self.device)
            self.assertEqual(int(view.nx[0]),nx)
            self.assertEqual(int(view.nx[1]),round(67.2/pitch))

    def test_cache_hit_invalidation_and_training_bypass(self):
        v,_=self.setup_inputs();v.eval()
        state=enable_pool_optimization(v,'cache',rectangular=False)
        identity=torch.eye(3,device=self.device)[None,None].repeat(1,2,1,1)
        translation=torch.zeros(1,2,3,device=self.device)
        args=[identity,translation,identity,identity,translation]
        with torch.no_grad():
            first=v.get_geometry(*args);second=v.get_geometry(*args)
            self.assertIs(first,second);self.assertEqual(state['hits'],1)
            args[1]=translation.clone();args[1][0,0,0]=.25
            self.assertFalse(torch.equal(v.get_geometry(*args),first))
            self.assertEqual(len(state['entries']),1)
            v.frustum.add_(.1);v.get_geometry(*args)
            self.assertEqual(state['misses'],3)
        v.train();v.get_geometry(*args);self.assertEqual(state['bypasses'],1)
        disable_pool_optimization(v);self.assertFalse(hasattr(v,'_pool_optimization'))

    def test_zero_cache_budget(self):
        v,_=self.setup_inputs();v.eval();s=enable_pool_optimization(v,cache_limit_bytes=0)
        identity=torch.eye(3,device=self.device)[None,None]
        tr=torch.zeros(1,1,3,device=self.device)
        with torch.no_grad():
            for _ in range(2):v.get_geometry(identity,tr,identity,identity,tr)
        self.assertEqual(s['hits'],0);self.assertEqual(s['bytes'],0)

    def test_nondefault_stream(self):
        if self.device!='cuda':self.skipTest('CUDA-only')
        stream=torch.cuda.Stream()
        with torch.cuda.stream(stream):
            v,g=self.setup_inputs();p=build_plan(v,g)
            d=torch.rand(2,3,3,4,4,device='cuda',requires_grad=True)
            f=torch.randn(2,3,4,4,4,device='cuda',requires_grad=True)
            ref=separated_reference(d,f,p);actual=SeparatedPool.apply(d,f,p)
            actual.sum().backward()
            stream.synchronize()
            torch.testing.assert_close(actual,ref,atol=2e-6,rtol=2e-6)
            self.assertTrue(torch.isfinite(f.grad).all())

    def test_warp_backward_nondefault_stream(self):
        if self.device!='cuda':self.skipTest('CUDA-only')
        stream=torch.cuda.Stream()
        with torch.cuda.stream(stream):
            v,g=self.setup_inputs();p=build_plan(v,g)
            d=torch.rand(2,3,3,4,4,device='cuda',requires_grad=True)
            f=torch.randn(2,3,4,4,80,device='cuda',requires_grad=True)
            ref=separated_reference(d,f,p)
            actual=SeparatedPool.apply(d,f,p,'warp')
            grad=torch.randn_like(ref)
            expected=torch.autograd.grad(ref,(d,f),grad)
            observed=torch.autograd.grad(actual,(d,f),grad)
        stream.synchronize()
        for a,b in zip(observed,expected):torch.testing.assert_close(a,b,atol=2e-5,rtol=2e-5)

    def test_real_depthnet_forward_backward_and_shape_guard(self):
        if self.device!='cuda':self.skipTest('Legacy pool CUDA-only')
        torch.manual_seed(23)
        v=MoraiRectangularDepthLSSTransform(8,4,(32,32),(4,4),(-8,8,1),
            (-6,6,1),(-8,8,16),(1,4,1)).cuda().train()
        other=deepcopy(v);state=enable_pool_optimization(other,'separated',backward_mode='grouped',compact_indices=True)
        identity=torch.eye(3,device='cuda')[None,None].repeat(1,2,1,1)
        tr=torch.zeros(1,2,3,device='cuda');args=[identity,tr,identity,identity,tr]
        img=torch.randn(1,2,8,4,4,device='cuda',requires_grad=True)
        depth_image=torch.rand(1,2,1,32,32,device='cuda',requires_grad=True)
        g=v.get_geometry(*args);reference=v.bev_pool(g,v.get_cam_feats(img,depth_image))
        actual=other.bev_pool(other.get_geometry(*args),other.get_cam_feats(img,depth_image))
        torch.testing.assert_close(actual,reference,atol=2e-6,rtol=2e-6)
        grad=torch.randn_like(reference)
        expected=torch.autograd.grad(reference,(img,depth_image,*v.depthnet.parameters()),grad)
        observed=torch.autograd.grad(actual,(img,depth_image,*other.depthnet.parameters()),grad)
        for a,b in zip(expected,observed):torch.testing.assert_close(a,b,atol=5e-5,rtol=5e-5)
        self.assertEqual(state['bypasses'],1)
        with self.assertRaisesRegex(ValueError,'dimensions'):
            other.bev_pool(g,(torch.ones(1,2,3,3,4,device='cuda'),torch.ones(1,2,4,4,4,device='cuda')))

    def test_stream_specific_cache_and_legacy_guard(self):
        if self.device!='cuda':self.skipTest('CUDA-only')
        v,_=self.setup_inputs();v.eval();s=enable_pool_optimization(v,'separated',rectangular=False)
        identity=torch.eye(3,device='cuda')[None,None];tr=torch.zeros(1,1,3,device='cuda')
        args=[identity,tr,identity,identity,tr]
        with torch.no_grad():v.get_geometry(*args)
        torch.cuda.synchronize();stream=torch.cuda.Stream()
        with torch.cuda.stream(stream),torch.no_grad():
            geom=v.get_geometry(*args);v.get_geometry(*args)
            self.assertEqual(s['misses'],2);self.assertEqual(s['hits'],1)
            p=s['plan'];d=torch.rand(*p['input_shape'],device='cuda')
            f=torch.randn(*p['input_shape'][:2],*p['input_shape'][3:],4,device='cuda')
            actual=v.bev_pool(geom,(d,f))
            ref=separated_reference(d,f,p).permute(0,4,1,2,3)
            ref=torch.cat(ref.unbind(dim=2),dim=1)
            torch.testing.assert_close(actual,ref,atol=2e-6,rtol=2e-6)
        stream.synchronize();disable_pool_optimization(v)
        enable_pool_optimization(v,'cache',rectangular=False)
        with torch.cuda.stream(stream),torch.no_grad():
            geom=v.get_geometry(*args)
            with self.assertRaisesRegex(RuntimeError,'default CUDA stream'):
                v.bev_pool(geom,torch.randn(1,1,3,4,4,4,device='cuda'))
        stream.synchronize()


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);args=a.parse_args()
    results={}
    for device in ('cpu','cuda'):
        PoolTests.device=device
        if device=='cuda':extension()
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(PoolTests))
        results[device]=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                             skipped=len(result.skipped),pass_all=result.wasSuccessful())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(results,indent=2)+'\n')
    if not all(r['pass_all'] for r in results.values()):raise SystemExit(1)
