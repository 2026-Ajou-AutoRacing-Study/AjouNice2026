"""Opt-in rosbridge service generation isolation; installed ROS files stay untouched."""
import uuid


def install(module):
    handler=module.AdvertisedServiceHandler
    if getattr(handler,'_base5_generation_guard',False):return
    original_init=handler.__init__
    original_advertise=module.AdvertiseService.advertise_service
    def initialize(self,*args,**kwargs):
        self.responses={}
        self._base5_generation=uuid.uuid4().hex
        self.id_counter=0
        original_init(self,*args,**kwargs)
    def next_id(self):
        with self.lock:
            self.id_counter+=1
            return self._base5_generation+':'+str(self.id_counter)
    def advertise(self,message):
        # Duplicate advertise previously shut down only the ROS registration,
        # leaving old handle_request response-wait loops alive.
        prior=self.protocol.external_service_list.get(message.get('service'))
        if prior is not None:prior.graceful_shutdown(timeout=.05)
        return original_advertise(self,message)
    handler.__init__=initialize;handler.next_id=next_id
    module.AdvertiseService.advertise_service=advertise
    handler._base5_generation_guard=True


def reproduce(module):
    """Deterministic installed-code reproduction; fake transport, no ROS requests."""
    import threading
    from unittest.mock import patch
    sent=threading.Event();messages=[];result=[]
    class Protocol:
        def send(self,message):messages.append(message);sent.set()
        def log(self,*args):pass
    with patch.object(module.rospy,'Service'),patch.object(module,'get_service_class',return_value=object),patch.object(module.message_conversion,'extract_values',return_value={}):
        old=module.AdvertisedServiceHandler('/probe','probe',Protocol())
        new=module.AdvertisedServiceHandler('/probe','probe',Protocol())
        worker=threading.Thread(target=lambda:result.append(old.handle_request(object())),daemon=True)
        worker.start()
        try:
            if not sent.wait(1):raise RuntimeError('Probe worker did not start')
            old_id=messages[0]['id'];new_id='service_request:/probe:'+str(new.next_id())
            new.responses[new_id]='new_generation_response'
            worker.join(timeout=.1)
            return dict(ids_equal=old_id==new_id,shared_response_store=old.responses is new.responses,
                old_consumed_new_response=result==['new_generation_response'],old_id=old_id,new_id=new_id,
                scene_load=False,network_request=False)
        finally:
            with old.lock:old.shutdown_requested=True
            worker.join(timeout=1)
            old.responses.pop(old_id,None);new.responses.pop(new_id,None)
