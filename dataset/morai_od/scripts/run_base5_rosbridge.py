#!/usr/bin/env python3
"""Base5-only guarded rosbridge entry point; do not run alongside another bridge."""
from pathlib import Path
import runpy
import socket
from base5_rosbridge_service_guard import install


if __name__=='__main__':
    with socket.socket() as connection:
        if connection.connect_ex(('127.0.0.1',9090))==0:
            raise RuntimeError('Port9090 already in use: stop the existing rosbridge terminal first')
    import rosbridge_library.capabilities.advertise_service as module
    install(module)
    import rospkg
    package=Path(rospkg.RosPack().get_path('rosbridge_server'))
    script=package.parents[1]/'lib/rosbridge_server/rosbridge_websocket'
    if not script.is_file():raise FileNotFoundError('Installed rosbridge entry point unavailable: '+str(script))
    print('Base5 rosbridge service guard: isolated response store, generation-unique IDs, retired duplicate handlers',flush=True)
    runpy.run_path(str(script),run_name='__main__')
