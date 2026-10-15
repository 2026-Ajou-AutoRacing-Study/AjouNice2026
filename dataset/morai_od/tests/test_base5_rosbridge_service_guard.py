import sys
import threading
import types
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_rosbridge_service_guard import install


def fixture():
    class Handler:
        responses={}
        id_counter=1
        def __init__(self,*args):self.lock=threading.Lock()
        def next_id(self):
            value=self.id_counter;self.id_counter+=1;return value
    class Capability:
        def advertise_service(self,message):self.events.append('original')
    return types.SimpleNamespace(AdvertisedServiceHandler=Handler,AdvertiseService=Capability)


class ServiceGuardTest(unittest.TestCase):
    def test_response_stores_and_ids_isolated_between_generations(self):
        module=fixture();install(module)
        old=module.AdvertisedServiceHandler();new=module.AdvertisedServiceHandler()
        self.assertIsNot(old.responses,new.responses)
        old.responses['id']='old';self.assertNotIn('id',new.responses)
        self.assertNotEqual(old.next_id(),new.next_id())

    def test_duplicate_retires_old_waiter_before_overwrite(self):
        module=fixture();install(module);cap=module.AdvertiseService();cap.events=[]
        old=types.SimpleNamespace(graceful_shutdown=lambda timeout:cap.events.append('retired'))
        cap.protocol=types.SimpleNamespace(external_service_list={'/Service_MoraiEventCmd':old})
        cap.advertise_service(dict(service='/Service_MoraiEventCmd'))
        self.assertEqual(cap.events,['retired','original'])

    def test_install_idempotent_and_concurrent_request_ids_unique(self):
        module=fixture();install(module);initialize=module.AdvertisedServiceHandler.__init__
        install(module);self.assertIs(initialize,module.AdvertisedServiceHandler.__init__)
        handler=module.AdvertisedServiceHandler();values=[]
        threads=[threading.Thread(target=lambda:values.append(handler.next_id())) for _ in range(50)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        self.assertEqual(len(set(values)),50)


if __name__=='__main__':unittest.main()
