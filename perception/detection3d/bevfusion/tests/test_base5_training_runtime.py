"""Single-GPU scalar logging must preserve loss values and gradients."""
import unittest

import torch
from projects.BEVFusion.bevfusion.bevfusion import BEVFusion
from perception.detection3d.bevfusion.plugin.morai_training_runtime import MoraiTrainingRuntimeBEVFusion


class TrainingRuntimeTests(unittest.TestCase):
    def test_logs_and_gradients_match(self):
        for device in ('cpu', 'cuda'):
            if device=='cuda' and not torch.cuda.is_available():continue
            for dtype in (torch.float32,torch.float16,torch.float64):
                x=torch.tensor([.1,.2,.3],device=device,dtype=dtype,requires_grad=True)
                losses=dict(loss_a=x.square(),loss_b=[x*.2,x*.3],matched_iou=x.mean())
                baseline,logs=BEVFusion.parse_losses(None,losses)
                actual,batched=MoraiTrainingRuntimeBEVFusion.parse_losses(None,losses)
                self.assertEqual(logs,batched)
                torch.testing.assert_close(actual,baseline,rtol=0,atol=0)
                a=torch.autograd.grad(actual,x,retain_graph=True)[0]
                b=torch.autograd.grad(baseline,x)[0]
                torch.testing.assert_close(a,b,rtol=0,atol=0)

    def test_invalid_value(self):
        with self.assertRaises(TypeError):
            MoraiTrainingRuntimeBEVFusion.parse_losses(None,dict(loss_bad=3.))

    @unittest.skipUnless(torch.cuda.is_available(),'CUDA-only fused optimizer')
    def test_fused_adamw_and_state_resume(self):
        torch.manual_seed(21)
        a=torch.randn(33,71,device='cuda',requires_grad=True)
        b=a.detach().clone().requires_grad_()
        baseline=torch.optim.AdamW([a],lr=1e-4,weight_decay=.01,foreach=True)
        fused=torch.optim.AdamW([b],lr=1e-4,weight_decay=.01,fused=True,foreach=False)
        for _ in range(6):
            gradient=torch.randn_like(a)
            a.grad=gradient.clone();b.grad=gradient.clone()
            baseline.step();fused.step()
            torch.testing.assert_close(a,b,rtol=2e-6,atol=2e-6)
            for name in ('exp_avg','exp_avg_sq'):
                torch.testing.assert_close(baseline.state[a][name],fused.state[b][name],
                                           rtol=2e-6,atol=2e-6)
        c=b.detach().clone().requires_grad_()
        resumed=torch.optim.AdamW([c],lr=1e-4,weight_decay=.01,fused=True,foreach=False)
        # Serialize instead of sharing state tensors with the live optimizer.
        import io
        buffer=io.BytesIO();torch.save(fused.state_dict(),buffer);buffer.seek(0)
        resumed.load_state_dict(torch.load(buffer,weights_only=True))
        gradient=torch.randn_like(b);b.grad=gradient.clone();c.grad=gradient.clone()
        fused.step();resumed.step()
        torch.testing.assert_close(b,c,rtol=0,atol=0)


if __name__=='__main__':unittest.main()
