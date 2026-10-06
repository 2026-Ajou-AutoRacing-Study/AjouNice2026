import sys
import threading
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_event_cmd import BoundedServiceCaller


class EventCmdTest(unittest.TestCase):
    def test_success_closes_fresh_proxy_and_reports(self):
        proxies=[];records=[]
        class Proxy:
            closed=False
            def __call__(self,request):return request
            def close(self):self.closed=True
        def factory():
            proxy=Proxy();proxies.append(proxy);return proxy
        caller=BoundedServiceCaller(factory,records.append)
        self.assertEqual(caller.call(1,.1,'park'),1)
        self.assertEqual(caller.call(2,.1,'park'),2)
        self.assertTrue(all(p.closed for p in proxies));self.assertEqual(len(proxies),2)
        self.assertTrue(all(r['response_verified'] and not r['worker_still_alive'] for r in records))

    def test_timeout_closes_transport_and_stops_worker(self):
        ended=threading.Event();records=[]
        class Proxy:
            def __call__(self,request):ended.wait();raise RuntimeError('closed')
            def close(self):ended.set()
        caller=BoundedServiceCaller(Proxy,records.append)
        with self.assertRaisesRegex(RuntimeError,'timeout'):caller.call(None,.01,'park')
        self.assertFalse(records[-1]['worker_still_alive']);self.assertTrue(records[-1]['timeout_stack'])

    def test_unstoppable_worker_prevents_overlapping_retry(self):
        release=threading.Event();records=[];calls=[]
        class Proxy:
            def __call__(self,request):calls.append(request);release.wait();return request
            def close(self):pass
        caller=BoundedServiceCaller(Proxy,records.append)
        try:
            with self.assertRaisesRegex(RuntimeError,'timeout'):caller.call(1,.01,'park')
            with self.assertRaisesRegex(RuntimeError,'no new request sent'):caller.call(2,.01,'park')
            self.assertEqual(calls,[1]);self.assertTrue(records[-1]['worker_still_alive'])
        finally:release.set();caller.worker.join(timeout=1)


if __name__=='__main__':unittest.main()
