"""Bounded, serial ROS service calls with observable timeout and transport cleanup."""
import sys
import threading
import time
import traceback


class BoundedServiceCaller:
    def __init__(self,factory,report):
        self.factory=factory;self.report=report;self.worker=None

    def call(self,request,timeout,phase):
        if self.worker is not None and self.worker.is_alive():
            raise RuntimeError('Prior EventCmd worker still alive; no new request sent')
        proxy=self.factory();done=threading.Event();result=[]
        def invoke():
            try:result.append((True,proxy(request)))
            except Exception as exc:result.append((False,exc))
            finally:done.set()
        self.worker=threading.Thread(target=invoke,daemon=True)
        started=time.monotonic();record=dict(phase=phase,timeout_sec=timeout)
        self.worker.start()
        try:
            if not done.wait(timeout):
                frame=sys._current_frames().get(self.worker.ident)
                record['timeout_stack']=traceback.format_stack(frame) if frame is not None else []
                record['response_verified']=False
                raise RuntimeError('MORAI EventCmd response timeout')
            if not result[0][0]:raise RuntimeError(str(result[0][1]))
            record['response_verified']=True
            return result[0][1]
        except BaseException as exc:
            record['error']=str(exc) or type(exc).__name__;raise
        finally:
            # A fresh persistent proxy exposes its transport, unlike rospy's
            # nonpersistent proxy. Close every call; never reuse across reloads.
            # Closing TCP does not undo a command already received by MORAI.
            try:proxy.close()
            except Exception as exc:record['close_error']=str(exc)
            self.worker.join(timeout=.25)
            record.update(elapsed_sec=time.monotonic()-started,worker_still_alive=self.worker.is_alive())
            self.report(record)
