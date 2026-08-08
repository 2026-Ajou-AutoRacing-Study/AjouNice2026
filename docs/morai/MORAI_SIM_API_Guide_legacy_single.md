# MORAI SIM API Guide — single document

> 로컬에 저장된 MORAI 공식 HTML을 검색하기 쉬운 단일 Markdown으로 기계 변환한 문서다. 표·코드·목록은 최대한 보존했지만, 정확한 UI 배치와 이미지는 원본 HTML 또는 온라인 공식 문서를 함께 확인한다.

> **프로젝트 버전 주의:** 이 자료의 주 문서는 2026년 26.R1 기준이지만 대회 런처는 2025년 빌드다. 대회 런처는 2024~2026 문서 사이의 기능 상태일 수 있으므로, 여기의 기능·토픽·필드를 그대로 지원한다고 가정하지 않는다. 실제 대회 런처에서 관측한 UI, 토픽, 메시지 정의와 실행 결과를 최종 기준으로 삼는다.

- 범위: 별도 SIM API Guide 미러; 현재 버전과 다를 수 있어 보조 자료로 사용
- 기준 URL: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/
- 포함 페이지: 10개
- 로컬 변환일: 2026-08-06

## 포함 페이지

- MORAI SIM: API 소개
- MORAI Simulator Control API
- MORAI Simulator Control API 사용하기
- MORAI Simulator Control API 명세서
- [예제]MSC API 및 ROS로 시뮬레이터 제어하기
- [예제]MSC API 및 UDP로 시뮬레이터 제어하기
- OpenSCENARIO API
- gRPC API
- gRPC API 사용하기
- gRPC API 명세서

---

## MORAI SIM: API 소개

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-sim-api-26

MORAI SIM: API는 MORAI SIM의 UI 기반이 아닌, 코드 레벨에서 직접 MORAI SIM의 시뮬레이터 기능을 사용할 수 있도록 MORAI SIM과 사용자 코드 간의 주고받는 인터페이스이다.

MORAI SIM: API는 현재 UDP와 gRPC 통신을 지원하며, 파이썬과 같은 사용자 코드의 클래스에서 각 통신 환경에 맞는 MORAI SIM: API를 호출하여 시뮬레이터 기능을 사용할 수 있다. 예를 들어, 시뮬레이터에서 제공하는 맵을 로드하거나 NPC 차량을 제어할 수 있으며, 시뮬레이터 상에 사용자가 원하는 테스트 시나리오를 구성할 수 있다.

본 문서에서 다루는 MORAI SIM: API의 종류는 아래와 같다.

- MORAI Simulator Control(MSC) API: ROS 및 UDP 통신을 사용하여 MORAI SIM을 제어할 수 있는 API
- gRPC API: gRPC 통신을 사용하여 MORAI SIM을 제어할 수 있는 API
- OpenSCENARIO API: gRPC 통신을 사용하여 MORAI Scenario Runner를 제어할 수 있는 API

MORAI: SIM API를 사용하기에 앞서, 필요한 환경 및 전제 사항은 아래와 같다.

- Python: MORAI SIM:API를 호출하는 테스트 코드 실행에 필요
- MORAI SIM: 호출한 API에 맞게 시뮬레이터가 동작하는지 테스트 실행에 필요

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

Python과 MORAI SIM의 버전은 각 통신 별 API 테스트 코드 및 OS 환경에 따라 다르기 때문에 각 API 사용 방법을 참고한다.

이후 섹션부터는 위의 MORAI SIM:API별 예제 코드를 활용하여 각 API를 실행하는 방법과 각 예제 코드에서 API가 동작하는 방식에 대해서 다룬다.

×


---

## MORAI Simulator Control API

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-26

MORAI Simulator Control(MSC) API는 UDP 통신을 사용하여 사용자 코드 단에서 직접 MORAI SIM을 제어할 수 있도록 사용자 측과 MORAI SIM 간의 명령 및 응답 데이터를 전달하는 인터페이스이다. 기존의 MORAI SIM의 UI 환경에서는 다양한 시나리오에 대한 테스트를 수행하기까지 여러 단계의 설정 과정이 필요하지만, MSC API를 이용하면 보다 빠르고 간편한 설정으로 시나리오에 대한 시뮬레이션 테스트를 자동화할 수 있다.

본 섹션에서는 UDP 통신 기반의 MSC API로 MORAI Simulator를 실행하는 방법에 대해서 설명한다.

또한 MORAI Simulator 실행 이후, ROS와 UDP 통신을 활용하여 시뮬레이터를 제어하는 방법까지 다루어 본다.

---

MORAI Simulator Control API 사용하기

MORAI Simulator Control API 명세서

[예제]MSC API 및 ROS로 시뮬레이터 제어하기

[예제]MSC API 및 UDP로 시뮬레이터 제어하기

×


---

## MORAI Simulator Control API 사용하기

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-2-26

본 섹션에서는 예제 코드를 활용하여 MSC API를 요청하고, MORAI Simulator를 실행하는 방법에 대하여 설명한다.

---

### 사전 요구 사항

MSC API를 사용하기 위하여 필요한 설치 프로그램 정보는 아래와 같다.

- Python 3.7.3 이상
- MORAI SIM(버전 상관 없음)

### MSC API 사용 절차

MSC API를 사용하는 절차는 아래와 같다.

#### Step 1: MORAI Launcher 실행

1] 다운로드한 MORAI Launcher 실행 파일(예: MoraiLauncher_Win.exe)을 클릭하여 MORAI Launcher를 실행한다.

MORAI Launcher 실행이 완료되면 아래와 같이 MORAI Launcher의 로그인 화면이 나타난다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20221213-044528.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**MORAI Launcher 로그인 화면**

2] MORAI SIM Launcher 실행 후, ‘MoraiLauncher_Win_Data > SaveFile’ 경로에서 `LinkConnection.txt` 파일을 확인한다.

LinkConnection.txt는 MORAI SIM Launcher 및 Simulator 측 IP와 Port 정보를 포함하는 파일이다.

현재 Launcher 및 Simulator 환경에 맞추어 기입해야 하는 각 IP와 Port 정보는 아래와 같다.

- Receive Host IP : API를 통해 Command를 받을 Launcher / Simulator 의 IP
- Receive Host PORT : API를 통해 Command를 받을 Launcher / Simulator 의 Port
- Request Destination IP : Launcher / Simulator에서 정보를 보낼 목적지 IP
- Request Destination PORT : Launcher / Simulator에서 정보를 보낼 목적지 Port

위 Step 2에서 정의한 params.txt의 파라미터와 LinkConnection.txt의 파라미터 간의 연결 관계는 아래와 같다.

예를 들어서, LinkConnection.txt의 `Receive Host IP` (MORAI SIM Launcher 및 Simulator 측 IP 주소)와 params.txt의 `request_dst_ip`(사용자 측에서 정의한 보내는 목적지 IP 주소)는 같다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/2.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

**params.txt의 파라미터와 LinkConnection.txt 파라미터 간의 연결 관계**

#### Step 2: 예제 코드 다운로드

MSC API를 요청하기 위하여 아래 경로의 예제 코드를 다운로드한다.

- [https://github.com/MORAI-Autonomous/MSC](https://github.com/MORAI-Autonomous/MSC)

다운로드한 예제 코드의 주요 파일 구성은 아래와 같다.

- api.py : params.txt 에 입력한 정보를 읽어 MORAI Simulator를 실행
- params.txt : 파이썬 테스트 코드를 실행하는 사용자 환경에서 시뮬레이터와 통신하기 위해 필요한 파라미터 정보를 포함한다. 사용자 환경에 맞추어 기입해야 하는 각 파라미터의 정보는 아래와 같다.
  - receive_user_ip: MORAI SIM Launcher 및 Simulator에서 보내는 정보( Data, Command result 등)를 받는 사용자 측 IP 주소
  - receive_user_port : MORAI SIM Launcher 및 Simulator에서 보내는 정보를 받는 측 사용자 측 Port 번호
  - request_dst_ip: 사용자 측에서 보내는 제어 명령을 받는 MORAI SIM Launcher 및 Simulator 측 IP 주소
  - request_dst_port: 사용자 측에서 보내는 제어 명령을 받는 MORAI SIM Launcher 및 Simulator 측 IP 번호
  - user_id: User의 ID
  - user_pw: User의 PASSWORD
  - version: 실행할 Simulator의 Version
  - map: Simulator에서 Play할 Map 명칭(대소문자 구분)
  - vehicle : Simulator에서 Play할 Vehicle 명칭(대소문자 구분)
  - network_file: 시뮬레이터 내 네트워크 설정과 관련한 스크립트 파일('MoraiLauncher_Win_Data > SaveFile > Network' 경로에 존재)
  - sensor_file: 시뮬레이터 내 센서 설정과 관련한 스크립트 파일('MoraiLauncher_Win_Data > SaveFile > Sensor' 경로에 존재)
  - Scenario_file: 시뮬레이터 내 시나리오 설정과 관련한 스크립트 파일('MoraiLauncher_Win_Data > SaveFile > Scenario' 경로에 존재)![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20220302-071435.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)‘params.txt’ 파일의 파라미터 정보

#### Step 3: 예제 코드로 MORAI Simulator 실행

##### 1] API 사용 가능 확인

예제 코드를 실행하기에 앞서, 아래와 같이 MORAI Launcher 및 Simulator 화면에 나타나는 메시지로 MSC API를 사용할 수 있는지 확인할 수 있다.

- 'MSC is Connected’ (Green): MSC API 사용 가능 상태
- ‘MSC is Disconnected’ (Red): MSC API 사용 불가능 상태

**MORAI Launcher 로그인 화면:**좌측 상단에서 MSC API 사용 가능 여부를 확인

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20210805-080555.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

---

**MORAI Simulator 실행 화면**: 좌측 하단에서 MSC API 사용 가능 여부를 확인

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20210805-082630.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20210805-080635.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

---

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/6.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/tip-macro-icon--29936453c365cfc0f714.svg)

Morai Simulator Control API 사용이 불가능한 상태에서는 IP 및 PORT 번호 확인이 필요하다. 특히 MORAI Launcher의 로그인 화면에서 MSC API 사용이 불가능하다면 이미 다른 Launcher가 실행 중인지 확인한다.

##### 2] 예제 코드 실행

다운로드한 예제 코드 중 params.txt 의 파라미터를 사용자 환경에 맞추어 입력한 후, `api.py` 테스트 코드로 MORAI 시뮬레이터를 실행한다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/tip-macro-icon--29936453c365cfc0f714.svg)

MORAI Launcher 실행 후 ‘Step 3’ 단계부터는 `api.py` 테스트 코드로 ‘로그인’하여 MORAI 시뮬레이터를 실행할 수 있다.

### 예제 코드 설명

예제 코드로 시뮬레이터 실행 명령을 수행하는 순서는 아래와 같다.

1] api.py에서 `launcher_start()` 클래스를 호출하여 launcher_start_api.py 실행

2] launcher_start_api.py에서 controller.py의 controller class method인 commander( ) 로 시뮬레이터 실행 명령 수행

3] 시뮬레이터 실행 명령은 define.py의 Command_list Class에 미리 정의함

PY

```py
##launcher_start_api.py
if self.controller.is_befor_login():                                        
    self.controller.commander(Command.LOGIN, f'{user_id}/{user_pw}')#Login명령
    
if self.controller.is_after_login() or self.controller.is_after_sim_quit_to_launcher():  # is_after_sim_quit_to_launcher : Simulator에서 quit 명령 후 Launcher 복귀 상태 확인

    if not self.controller.is_version_selected():
        self.controller.commander(Command.SELECT_VER, version)#version 선택명령
        
        if self.controller.is_not_find_version(): #선택한 버전이 없는 버전인지 확인.                    
            break
    
    if self.controller.is_can_execute_sim():
        self.controller.commander(Command.EXECUTE_SIM,'') #Simulator 실행 명령
                                
    if self.controller.is_sim_not_install():
        self.controller.commander(Command.INSTALL_SIM,'') #Simulator 설치 명령

if self.controller.is_sim_lobby():
    self.controller.commander(Command.MAP_VEHICLE_SELECT, f'{map}/{vehicle}')#시뮬레이션/옵션 변경 실행 명령

if self.controller.is_sim_playing():
    if network_file :
        self.controller.commander(Command.NET_SETTING, network_file) #시뮬레이터 네트워크 세팅 명령
    if sensor_file:
        self.controller.commander(Command.SEN_SETTING, sensor_file) #시뮬레이터 센서 세팅 명령
    if scenario_file:
        self.controller.commander(Command.SCEN_SETTING, f'{scenario_file}/30') #시뮬레이터 시나리오 세팅 명령
    '''
        Scenario Setting Option
        0x01 : Delete All
        0x02 : Ego Vehicle
        0x04 : Surround Vehicle
        0x08 : Pedestrian
        0x10 : Obstacle
        0x20 : Pause (scenario load 후 시뮬레이터 정지(직접 esc키 또는 play command로 해제 가능))
        원하는 option을 or 연산 해서 사용
        e.g) Ego vihcle  Surrond Vehicle, Pedestrain, Obstacle을 load하려면
             0x02 | 0x04 | 0x08 | 0x10  -> 0b0010 | 0b0100 | 0b1000 | 0b0001 0000 -> 30 (0b0001 1110, 0x1E)
    '''
    break

##define.py
class Command_list(Enum):
    #Launcher Command
    LOGIN           = (Platform.LUANCHER, cmd.LOGIN         , user_id+'/'+user_pw)
    SELECT_VER      = (Platform.LUANCHER, cmd.SELECT_VER    , version)
    INSTALL_SIM     = (Platform.LUANCHER, cmd.INSTALL_SIM   , "")
    EXECUTE_SIM     = (Platform.LUANCHER, cmd.EXECUTE_SIM   , "")
    QUIT_LAUNCHER   = (Platform.LUANCHER, cmd.QUIT_LAUNCHER , "")
    LOGOUT          = (Platform.LUANCHER, cmd.LOGOUT        , "")

    #Simulator Command
    MAP_VEHICLE_SELECT      = (Platform.SIMULATOR, cmd.MAP_VEHICLE_SELECT   , map+'/'+vehicle)
    SIM_PAUSE               = (Platform.SIMULATOR, cmd.SIM_PAUSE            , "")
    SIM_PLAY                = (Platform.SIMULATOR, cmd.SIM_PLAY             , "")
    NET_SETTING             = (Platform.SIMULATOR, cmd.NET_SETTING          , network_file)
    NET_SAVE                = (Platform.SIMULATOR, cmd.NET_SAVE             , )
    SEN_SETTING             = (Platform.SIMULATOR, cmd.SEN_SETTING          , sensor_file)
    SEN_SAVE                = (Platform.SIMULATOR, cmd.SEN_SAVE             , "")
    SCEN_SETTING            = (Platform.SIMULATOR, cmd.SCEN_SETTING         , scenario_file+"/62")
    SCEN_SAVE               = (Platform.SIMULATOR, cmd.SCEN_SAVE            , "")
    QUIT_SIM                = (Platform.SIMULATOR, cmd.QUIT_SIM             , "")   
```

예제 코드 실행 흐름은 아래와 같다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236879885/image-20221215-120309.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

### 사용 예제

위에서 설명한 순서대로 MORAI Launcher 실행 후 `api.py` 테스트 코드로 MORAI 시뮬레이터를 실행하는 예제는 아래와 같다.

×


---

## MORAI Simulator Control API 명세서

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-1-26

MSC API를 구성하는 명령어 및 결과 데이터와 같은 정보는 아래 MSC API 명세서를 확인한다.

- [https://docs.google.com/spreadsheets/d/1jHbR_JoZFYfxMirwSp-48peWkJf1xUmMZyFIwetxcZM/edit#gid=0](https://docs.google.com/spreadsheets/d/1jHbR_JoZFYfxMirwSp-48peWkJf1xUmMZyFIwetxcZM/edit#gid=0)

×


---

## [예제]MSC API 및 ROS로 시뮬레이터 제어하기

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/ros-msc-api-26

본 섹션에서는 MSC API와 ROS 통신으로 MORAI SIM을 제어하는 방법에 대해서 설명한다.

[MSC API를 사용](https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-2-26.html)하여 MORAI Simulator를 실행 후, ROS 통신을 적용한 예제 코드를 통해 차량 주행 테스트까지 진행한다.

---

### 사전 요구 사항

ROS 통신을 적용한 MSC API를 사용하기 위하여 필요한 설치 프로그램 정보는 아래와 같다.

- Python 3.7.5 이상
- MORAI SIM(버전 상관 없음)

### 예제 진행 절차

본 섹션에서 다루는 예제를 아래의 절차대로 진행한다.

#### Step 1: MORAI Launcher 실행

[MORAI Simulator Control API | Step-1:-MORAI-Launcher-실행](https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-26.html#Step-1%3A-MORAI-Launcher-%EC%8B%A4%ED%96%89) 방법과 동일하다.

#### Step 2: 예제 코드 다운로드

아래 경로의 예제 코드를 다운로드한다.

- ROS1: [https://github.com/MORAI-Autonomous/MSC/tree/ros1_ex](https://github.com/MORAI-Autonomous/MSC/tree/ros1_ex)
- ROS2: [https://github.com/MORAI-Autonomous/MSC/tree/ros2_ex](https://github.com/MORAI-Autonomous/MSC/tree/ros2_ex)

다운로드한 예제 코드의 주요 파일 구성은 아래와 같다.

- network_example: 네트워크 예제 파일 포함, ‘MoraiLauncher_Win_Data > SaveFile > Network > (version)’ 경로에 복사
- msc_ros: msc_ros_example 파일 포함
- params.txt: 예제 코드를 실행하는 사용자 환경에서 MORAI SIM와 통신하기 위해 필요한 파라미터 정보를 포함

#### Step 3: 예제 코드로 MORAI Simulator 실행

##### 1] 파라미터 정보 수정

아래의 사항을 고려하여 다운로드한 예제 코드의 파라미터를 수정한다.

- 시뮬레이터 및 Code 실행이 다른 환경(IP 가 다른 환경)에서 진행 되면 params.txt , params.json 의 ip를 수정해야 한다.
- 시뮬레이터 실행과 예제 코드를 실행하는 사용자 환경이 다른 경우(IP가 다른 환경), params.txt와 params.json 파일 내 관련 IP 정보를 수정해야 한다.
- params.txt 내 id, pw, version , map, vehicle 과 같은 파라미터를 사용자 환경에 맞게 기입한다(대소문자 구분). 예: receive_user_ip : 127.0.0.1 receive_user_port : 10326 request_dst_ip : 127.0.0.1 request_dst_port : 10508 user_id : id user_pw : pw version : v.R1.240407.0 map : R_KR_PG_K-City vehicle : 2017_Kia_Niro(HEV) network_file : ros_network Sensor_file : Scenario_file :

##### 2] Simulator Network 설정

Simulator의 Edit > Network Setting 메뉴에서 해당 테스트 Scenario를 저장할 때 필요한 네트워크 정보를 아래와 같이 입력한다.

예:

- Bridge IP: 127.0.0.1
- Bridge PORT: 9090

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/233767016/20210901_194559.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/233767016/20210901_194619.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

##### 3] 예제 코드 실행

msc_ros.launch를 실행하면 parmas.txt 내 id, pw ,version, map, vehicle, network_file 정보에 맞추어 MORAI 시뮬레이터를 실행 및 주행 테스트를 진행할 수 있다.

### 예제 코드 설명

ROS 통신을 적용한 예제 코드에 관한 설명은 아래와 같다.

- `class launcher_start`: params.txt 내 파라미터 정보를 읽어 MSC API로 시뮬레이션을 시작하는 class

PY

```py
import time,os
from lib.define import *
from lib.read_text import *
from lib.controller import *

class launcher_start(controller):

    def __init__(self):
        self.controller = controller()

    def launcher_start(self):

        while True:
            if self.controller.update(): #data update
                
                self.controller.is_downloading() #check downloading status

                if self.controller.is_befor_login():
                    self.controller.commander(Command.LOGIN, f'{user_id}/{user_pw}')#Login명령
                        
                if self.controller.is_after_login() or self.controller.is_after_sim_quit_to_launcher():  # is_after_sim_quit_to_launcher : Simulator에서 quit 명령 후 Launcher 복귀 상태 확인
                                        
                    if not self.controller.is_version_selected():
                        self.controller.commander(Command.SELECT_VER, version)#version 선택명령
                        
                        if self.controller.is_not_find_version(): #선택한 버전이 없는 버전인지 확인.                    
                            break
                    
                    if self.controller.is_can_execute_sim():
                        self.controller.commander(Command.EXECUTE_SIM,'') #Simulator 실행 명령
                                                
                    if self.controller.is_sim_not_install():
                        self.controller.commander(Command.INSTALL_SIM,'') #Simulator 설치 명령

                if self.controller.is_sim_lobby():
                    self.controller.commander(Command.MAP_VEHICLE_SELECT, f'{map}/{vehicle}')#시뮬레이션/옵션 변경 실행 명령

                if self.controller.is_sim_playing():
                    if network_file :
                        self.controller.commander(Command.NET_SETTING, network_file) #시뮬레이터 네트워크 세팅 명령
                    if sensor_file:
                        self.controller.commander(Command.SEN_SETTING, sensor_file) #시뮬레이터 센서 세팅 명령
                    if scenario_file:
                        self.controller.commander(Command.SCEN_SETTING, f'{scenario_file}/30') #시뮬레이터 시나리오 세팅 명령
                    '''
                        Scenario Setting Option
                        0x01 : Delete All
                        0x02 : Ego Vehicle
                        0x04 : Surround Vehicle
                        0x08 : Pedestrian
                        0x10 : Obstacle
                        0x20 : Pause (scenario load 후 시뮬레이터 정지(직접 esc키 또는 play command로 해제 가능))
                        원하는 option을 or 연산 해서 사용
                        e.g) Ego vihcle  Surrond Vehicle, Pedestrain, Obstacle을 load하려면
                             0x02 | 0x04 | 0x08 | 0x10  -> 0b0010 | 0b0100 | 0b1000 | 0b0001 0000 -> 30 (0b0001 1110, 0x1E)
                    '''
                    break

            else :                
                print("[NO Simulator Control Data]",end='\r')                
```

×


---

## [예제]MSC API 및 UDP로 시뮬레이터 제어하기

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/udp-msc-api-26

본 섹션에서는 MSC API와 UDP 통신으로 MORAI Simulator를 제어하는 방법에 대해서 설명한다.

[MSC API를 사용](https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-2-26.html)하여 MORAI Simulator를 실행 후, UDP 통신을 적용한 예제 코드를 통해 차량 주행 테스트까지 진행한다.

---

### 사전 요구 사항

UDP 통신을 적용한 MSC API를 사용하기 위하여 필요한 설치 프로그램 정보는 아래와 같다.

- Python 3.7.5 이상
- MORAI SIM(버전 상관 없음)

### 예제 진행 절차

본 섹션에서 다루는 예제를 아래의 절차대로 진행한다.

#### Step 1: MORAI Launcher 실행

[MORAI Simulator Control API | Step-1:-MORAI-Launcher-실행](https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/morai-simulator-control-api-26.html#Step-1%3A-MORAI-Launcher-%EC%8B%A4%ED%96%89) 방법과 동일하다.

#### Step 2: 예제 코드 다운로드

아래 경로의 예제 코드를 다운로드한다.

- [https://github.com/MORAI-Autonomous/MSC/tree/udp_ex](https://github.com/MORAI-Autonomous/MSC/tree/udp_ex)

#### Step 3: 예제 코드로 MORAI Simulator 실행

##### 1] 파라미터 정보 수정

아래의 사항을 고려하여 다운로드한 예제 코드의 파라미터를 수정한다.

- 시뮬레이터 실행과 예제 코드를 실행하는 사용자 환경이 다른 경우(IP가 다른 환경), params.txt 파일 내 관련 IP 정보를 수정해야 한다.
- params.txt 내 id, pw, version , map, vehicle 과 같은 파라미터를 사용자 환경에 맞게 기입한다(대소문자 구분). 예: receive_user_ip : 127.0.0.1 receive_user_port : 10326 request_dst_ip : 127.0.0.1 request_dst_port : 10508 user_id : id user_pw : pw version : v.R1.240407.0 map : R_KR_PG_K-City vehicle : 2017_Kia_Niro(HEV) network_file : udp_network Sensor_file : Scenario_file :

##### 2] Simulator Network setting

- Network Setting은 Scenario를 저장할 때의 네트워크 정보를 적용.
- ‘network_example’ 폴더의 udp_network.json 파일을 ‘MoraiLauncher_Win_Data > SaveFile > Network > (version)’ 경로에 복사한다.

##### 3] 예제 코드 실행

api.py를 실행하면 parmas.txt 내 id, pw ,version, map, vehicle, network_file 정보에 맞추어 MORAI 시뮬레이터를 실행 및 주행 테스트를 진행할 수 있다.

### 예제 코드 설명

UDP 통신을 적용한 예제 코드에 관한 설명은 아래와 같다.

- `class launcher_start`: params.txt 내 파라미터 정보를 읽어 MSC API로 시뮬레이션을 시작하는 class

PY

```py
import time,os
from lib.define import *
from lib.read_text import *
from lib.controller import *

class launcher_start(controller):

    def __init__(self):
        self.controller = controller()

    def launcher_start(self):

        while True:
            if self.controller.update(): #data update
                
                self.controller.is_downloading() #check downloading status

                if self.controller.is_befor_login():
                    self.controller.commander(Command.LOGIN, f'{user_id}/{user_pw}')#Login명령
                        
                if self.controller.is_after_login() or self.controller.is_after_sim_quit_to_launcher():  # is_after_sim_quit_to_launcher : Simulator에서 quit 명령 후 Launcher 복귀 상태 확인
                                        
                    if not self.controller.is_version_selected():
                        self.controller.commander(Command.SELECT_VER, version)#version 선택명령
                        
                        if self.controller.is_not_find_version(): #선택한 버전이 없는 버전인지 확인.                    
                            break
                    
                    if self.controller.is_can_execute_sim():
                        self.controller.commander(Command.EXECUTE_SIM,'') #Simulator 실행 명령
                                                
                    if self.controller.is_sim_not_install():
                        self.controller.commander(Command.INSTALL_SIM,'') #Simulator 설치 명령

                if self.controller.is_sim_lobby():
                    self.controller.commander(Command.MAP_VEHICLE_SELECT, f'{map}/{vehicle}')#시뮬레이션/옵션 변경 실행 명령

                if self.controller.is_sim_playing():
                    if network_file :
                        self.controller.commander(Command.NET_SETTING, network_file) #시뮬레이터 네트워크 세팅 명령
                    if sensor_file:
                        self.controller.commander(Command.SEN_SETTING, sensor_file) #시뮬레이터 센서 세팅 명령
                    if scenario_file:
                        self.controller.commander(Command.SCEN_SETTING, f'{scenario_file}/30') #시뮬레이터 시나리오 세팅 명령
                    '''
                        Scenario Setting Option
                        0x01 : Delete All
                        0x02 : Ego Vehicle
                        0x04 : Surround Vehicle
                        0x08 : Pedestrian
                        0x10 : Obstacle
                        0x20 : Pause (scenario load 후 시뮬레이터 정지(직접 esc키 또는 play command로 해제 가능))
                        원하는 option을 or 연산 해서 사용
                        e.g) Ego vihcle  Surrond Vehicle, Pedestrain, Obstacle을 load하려면
                             0x02 | 0x04 | 0x08 | 0x10  -> 0b0010 | 0b0100 | 0b1000 | 0b0001 0000 -> 30 (0b0001 1110, 0x1E)
                    '''
                    break

            else :                
                print("[NO Simulator Control Data]",end='\r')                
```

×


---

## OpenSCENARIO API

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/openscenario-api-26

본 섹션에서는 MORAI OpenSCENARIO API를 사용하는 방법에 대하여 설명한다.

OpenSCENARIO API는 사용자의 Python 코드 단에서 Scenario Runner의 OpenSCENARIO 파일 Import 및 Simulation 기능을 직접 호출하여 사용할 수 있도록 필요한 명령 및 데이터를 전달하는 인터페이스이다.

---

### 사전 요구 사항

OpenSCENARIO API를 사용하기 위하여 필요한 설치 프로그램 정보는 아래와 같다.

- Python 및 추가 설치 모듈
  - numpy (1.19.1)
  - PyQt5
  - matplotlib
  - scipy
  - eventhandler
  - pyproj
  - grpcio(1.39.0)
  - grpcio-tools (1.39.0)
  - pyconcrete - 22.R2와 호환되는 API를 사용하는 경우 설치 필요 - 패키지 설치 전 `set PYCONCRETE_PASSPHRASE={your passphrase}`명령어로 암호문 설정 필요
  - sourcedefender - 22.R3 부터는 pyconcrete 대신 sourcedefender 설치 필요
- MORAI SIM
- 예제 코드

아래와 같이 MORAI SIM 버전 및 OS에 맞는 Python 버전 및 예제 코드를 사용해야 한다.

| **MORAI SIM 버전** | **Python 버전** | **예제 코드** | **비고** |
| --- | --- | --- | --- |
| 22.R2 | Window: python 3.7.3 | Window [OpenSCENARIO_API.zip](https://help-morai-sim.scrollhelp.site/__attachments/233767428/OpenSCENARIO_API.zip?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678) | pyconcrete로 만든 API로 OS마다 다른 예제 코드 제공 |
| Linux(ubuntu 18.04): python 3.7.13 | Linux [OpenSCENARIO_API_Linux.zip](https://help-morai-sim.scrollhelp.site/__attachments/233767428/OpenSCENARIO_API_Linux.zip?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678) |  |  |
| 22.R3 | python 3.7.3 | [OpenSCENARIO_API_22.R3.zip](https://help-morai-sim.scrollhelp.site/__attachments/233767428/OpenSCENARIO_API_22.R3.zip?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678) | sourcedefender를 사용한 버전으로 운영체제는 호환되면 python 버전은 3.7.3 버전을 사용해야 함 |

### OpenSCENARIO API 사용 순서

아래의 순서로 사용자 측 예제 코드에서 OpenSCENARIO API를 호출하여 Scenario Runner 기능을 사용할 수 있다.

1] MORAI Launcher 실행 및 로그인

2] 특정 시뮬레이터 버전 설치 및 실행

3] 시뮬레이터의 맵 선택 및 차량 선택 화면에 위치한 상태에서 main.py 스크립트 실행

4] 5개의 시나리오가 연속적으로 실행되는 것을 확인

### 예제 코드 설명

#### 파일 구조

- main.py
- open_scenario_client_wrapper.py
- open_scenario_importer_wrapper.py
- data 폴더
  - OpenSCENARIO 예제 파일
- lib 폴더
  - 암호화하여 제공된 MORAI OpenSCENARIO 라이브러리

#### 예제 코드 동작 방식

main 함수에서 시나리오 파일에 대해 차례대로 `OpenScenarioImporterWrapper` 클래스를 사용하여 시나리오 파일을 Import 하고, `OpenScenarioClientWrapper` 클래스를 통해 시나리오 파일 Simulation을 시작한다.

`OpenScenarioImporterWrapper` , `OpenScenarioClientWrapper` 클래스는 MORAI OpenSCENARIO 라이브러리 사용 예시를 보여주기 위한 클래스로 사용자가 임의로 수정 가능하다.

- 현재 시뮬레이터와 gRPC 통신을 하기 위해서는 아래와 같은 IP, Port를 사용
  - `localhost:7789`

main.py

PY

```py
import os
import sys
import numpy as np
import math
import time

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

import sourcedefender

from open_scenario_importer_wrapper import OpenScenarioImporterWrapper
from open_scenario_client_wrapper import OpenScenarioClientWrapper

if __name__ == '__main__': 
    # 실행할 시나리오 리스트
    scenario_list = [
        '.\data\openscenario\R_KR_PG_K-City\Scenario_CCRB_1.xosc',
        '.\data\openscenario\R_KR_PG_K-City\Scenario_CCRB_2.xosc',
        '.\data\openscenario\R_KR_PG_K-City\Scenario_CCRB_3.xosc',
        '.\data\openscenario\V_RHT_Suburb_03\Scenario_Animal_Deer.xosc',
        '.\data\openscenario\V_RHT_Suburb_03\Scenario_UTurn.xosc'
    ]
    
    client_wrapper = OpenScenarioClientWrapper('127.0.0.1', '7789')
    for scenario_file_path in scenario_list:
        # OpenSCENARIO 파일 Import
        importer_wrapper = OpenScenarioImporterWrapper()

        abs_scenario_file_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), scenario_file_path))
        importer_wrapper.import_open_scenario(abs_scenario_file_path)
        
        client_wrapper.set_open_scenario_importer(importer_wrapper.scenario_importer)
        
        # 시나리오 시작
        client_wrapper.start_scenario()

        # 시나리오가 종료될 때까지 대기
        # 시나리오가 종료되면 자동으로 OpenScenarioClientWrapper에서 Stop Scenario 호출
        while client_wrapper.get_stop_status():
            time.sleep(1)
```

open_scenario_client_wrapper.py

CODE

```
import os
import sys
import threading

from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIntValidator
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject

from lib.openscenario.client.open_scenario_client import OpenScenarioClient

class OpenScenarioClientWrapper(QObject):   
    def __init__(self, ip, port):
        super().__init__()

        self.client = OpenScenarioClient(ip, port, self) 
        self.open_scenario_importer = None

    def set_open_scenario_importer(self, open_scenario_importer):
        """ open_scenario_importer instance 설정 """
        self.client.set_open_scenario_importer(open_scenario_importer)
        
    def start_scenario(self):
        self.client.start_scenario()

    def stop_scenario(self):
        self.client.stop_scenario()

    def get_stop_status(self):
        return self.client.is_start

    @pyqtSlot(bool, str)
    def start_scenario_callback(self, result, desc):
        """scenario runner에서 사용 시 callback 함수를 등록해줘야 하기 때문에 에러나지 않게 하기 위해 정의"""
        pass

    @pyqtSlot(bool, str) 
    def stop_scenario_callback(self, result, desc):
        """scenario runner에서 사용 시 callback 함수를 등록해줘야 하기 때문에 에러나지 않게 하기 위해 정의"""
        pass
```

open_scenario_importer_wrapper.py

CODE

```
import os
import sys

from lib.openscenario.class_defs.enumerations import *
from lib.openscenario.open_scenario_importer import OpenScenarioImporter
from lib.mgeo.class_defs.mgeo import MGeo

class OpenScenarioImporterWrapper():
    """ OpenScenario 파일 Import 클래스 """
    def __init__(self):
        self.scenario_importer = OpenScenarioImporter()
        self.scenario_data = None
        self.mgeo_folder_path = None

    def import_open_scenario(self, scenario_file_path):        
        self.scenario_importer.import_open_scenario(scenario_file_path)
        
        self.mgeo_folder_path = self.scenario_importer.get_mgeo_folder_path()
        self.mgeo_planner_map = MGeo.create_instance_from_json(self.mgeo_folder_path)

        self.scenario_importer.set_mgeo(self.mgeo_planner_map)
        self.scenario_importer.update_scenario_data()

    def clear(self):
        self.scenario_importer.clear()
```

×


---

## gRPC API

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/grpc-api-26

gRPC는 Google에서 개발한 오픈소스 RPC(Remote Procedure Call) 프레임워크에서 제공하는 API이다.

gRPC API는 HTTP/2 프로토콜이 결합된 RPC(Remote Procedure Call) 통신을 통해 클라이언트와 서버 간의 다양한 요청 및 응답을 전달한다.

gRPC API는 플랫폼 및 프로그래밍 언어에 제약이 없는 Proto 파일을 단위로 사용할 수 있다. 즉 클라이언트에서는 이 파일을 컴파일하여 서버 내 특정 프로시저를 호출하고 결과값을 반환 받는다.

따라서 클라이언트는 서버에서만 가능했던 동작을 gRPC API를 요청하여 클라이언트 측에서 바로 활용하고 다양한 기능을 구현할 수 있다.

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/233767126/image-20221215-101551.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

gRPC API 동작 방식 [출처: [https://grpc.io](https://grpc.io/)]

본 섹션에서는 gRPC API을 사용하여 MORAI Simulator를 제어하는 방법을 설명하며, gRPC API를 구성하는 Proto 파일을 단위로, 서비스와 메시지와 같은 세부 명세 항목에 대해서 다룬다.

---

gRPC API 사용하기

gRPC API 명세서

×


---

## gRPC API 사용하기

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/grpc-api-1-26

본 섹션에서는 gRPC API를 사용하여 MORAI Simulator를 제어하는 방법에 대하여 설명한다.

사용자는 gRPC API를 통해 UI를 사용하지 않아도, 시뮬레이터에서 맵 로드, NPC 차량 생성 및 제어 등을 수행할 수 있고, 사용자가 원하는 테스트 시나리오 환경을 구성할 수 있다.

---

### 사전 요구 사항

#### Example code 다운로드

- 24.R2 / 26.R1 버전과 호환되는 예제 코드

[https://github.com/MORAI-Autonomous/MORAI-DriveExample_GRPC/tree/grpc-py](https://github.com/MORAI-Autonomous/MORAI-DriveExample_GRPC/tree/grpc-py)

- Protocol Documentation

[https://github.com/MORAI-Autonomous/grpc-docs](https://github.com/MORAI-Autonomous/grpc-docs)

#### Python 환경 설정

- python 버전
  - example code는 python 3.7.3에서 작성 및 테스트 진행
- 설치 필요 python 모듈
  - grpcio : 1.44.0
  - grpcio-tools : 1.44.0

### gRPC API 사용 순서

1] Morai Launcher 실행 후 로그인

2] Morai Launcher에서 특정 시뮬레이터 버전 설치 및 실행

3] 시뮬레이터의 맵 선택 및 차량 선택 화면에 위치한 상태에서 example.py 스크립트 실행

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__attachments/236519468/image-20220328-065934.png?inst-v=fe18267d-8fe9-479e-ba7e-da30a93f7678)

4] 결과

- 맵이 로드되고 Ego 차량이 Built-In 모드로 주행하는 것을 확인
- NPC 1, 2 차량이 생성되어 원을 그리며 주행하는 것을 확인
- 장애물이 2개 생성된 것을 확인

### 예제 코드 설명

#### 파일 구조

- example 코드
  - example.py
- gRPC 통신에 사용되는 proto 파일
  - lib/grpc/src/define.py
  - lib/grpc/src/api
  - lib/grpc/src/proto

#### 예제 코드 동작 방식

main 함수에서 GRPCClient 클래스를 생성하여 해당 클래스를 통해 gRPC API를 호출하여 시뮬레이터와 gRPC 통신을 수행

- 현재 시뮬레이터와 gRPC 통신을 하기 위해서는 아래와 같은 IP, Port를 사용
  - `localhost:7789`

프로그램은 바로 종료되지 않기 위해 무한 루프를 돌다가 ‘Ctrl + C’를 누르면 프로그램 종료

예제 코드의 동작 순서는 아래와 같다.

1. client 생성 후 connect 하여 gRPC 통신 시작
2. start simulation 함수를 호출하여 사용자가 원하는 맵과 차량 로드, ego 차량을 원하는 위치로 이동
3. spawn vehicle 함수를 호출하여 npc 차량 생성
4. spawn obstacle 함수를 호출하여 장애물 생성
5. multi ego ctrl 함수를 thread로 생성하여 npc 제어 명령 주기적으로 송신
6. get ego status 함수를 호출하여 차량 상태 정보 수신

PY

```py
import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, 'lib\grpc\src')))
sys.path.append(os.path.normpath(os.path.join(current_path, 'lib\grpc\src\proto')))

import time
import threading

from lib.grpc.src.defines import CLIENT_KEY, MORAI_SIM_ADDRESS, MORAI_SIM_PORT, TEST_VEHICLE_MODEL, map_and_vehicle, vehicle_transform_001, vehicle_transform_002, vehicle_transform_003, cruise_on_link, sync_mode_realtime
from lib.grpc.src.api.morai_sim_client import MoraiSimClient
from lib.grpc.src.proto.morai.common.type_pb2 import Transform
from proto.morai.environment.environment_enum_pb2 import WeatherType

class GRPCClient():
    def __init__(self):
        self.grpc_client = MoraiSimClient(CLIENT_KEY)
        self.grpc_client.connect(MORAI_SIM_ADDRESS, MORAI_SIM_PORT)
        self.grpc_client.start_simulation(map_and_vehicle, vehicle_transform_001, cruise_on_link, sync_mode_realtime)

        world = self.grpc_client.get_simulation_world()

        world.set_time(13)
        world.set_weather(WeatherType.WEATHER_TYPE_SUNNY)

        self.ego = world.get_ego()
        self.vehicle1 = world.spawn_vehicle(vehicle_transform_002, TEST_VEHICLE_MODEL, 'vehicle1', velocity=30, multi_ego=True)
        self.vehicle2 = world.spawn_vehicle(vehicle_transform_003, TEST_VEHICLE_MODEL, 'vehicle2', velocity=30, multi_ego=True)

        transform1 = Transform()
        transform1.location.x = 208.68639761359265
        transform1.location.y = 1661.4038564971388
        transform1.location.z = 0.0
        transform1.rotation.x = 0.0
        transform1.rotation.y = 0.0
        transform1.rotation.z = 0.0
        world.spawn_obstacle(transform1, 'CargoBox', 'obstacle_1')

        transform2 = Transform()
        transform2.location.x = 212.4813728324434
        transform2.location.y = 1605.9207673354679
        transform2.location.z = 0.0
        transform2.rotation.x = 0.0
        transform2.rotation.x = 0.0
        transform2.rotation.x = 0.0
        world.spawn_obstacle(transform2, 'WoodBox', 'obstacle_2')

        world.pause()
        world.resume()

    def send_multi_ego_ctrl_cmd(self):
        while True:
            self.vehicle1.control(1, 0.9, 0.0, -1.0, 0.0, 0.0, 0)
            self.vehicle2.control(2, 0.0, 0.0, -1.0, 30.0, 0.0, 0)

    def get_ego_status(self):
        return self.ego.get_actor_state()

    def finalize(self):
        self.grpc_client.finalize()

if __name__ == "__main__":
    print('start example.')
    print('press ctrl + c for exit.')

    client = GRPCClient()

    ctrl_npc = threading.Thread(target=client.send_multi_ego_ctrl_cmd)
    ctrl_npc.start()
    
    try:
        while True:
            ego_status = client.get_ego_status()
            print(ego_status)
            time.sleep(1.0)
    except KeyboardInterrupt:
        client.finalize()
```

×


---

## gRPC API 명세서

> 원문: https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/grpc-api-docs-26

본 섹션에서는 MORAI SIM에서는 사용하는 gRPC API의 세부 항목을 설명하는 API 명세서에 대하여 다룬다.

[→ gRPC API 명세서 바로 가기](https://morai-autonomous.github.io/grpc-docs/)

### **22.R4.0 gRPC API 개선 사항**

22.R4.0에서 배포한**MORAI Standard Proto**는 MORAI Simulator의 서비스 카테고리 별로 RPC와 Message를 가지는 구조를 가진다.

**MORAI Standard Proto**는 아래와 같은 서비스 타입 별로 구성된다.

| **이름** | **설명** |
| --- | --- |
| Simulator | 시뮬레이터의 조작과 관련된 rpc 그룹 |
| Simulation | 시뮬레이션의 기본 조작과 관련된 rpc |
| Map | Map 과 관련된 rpc |
| Enviroment | 환경설정과 관련된 rpc |
| Infrastructure | 신호등, 교차로 등과 관련된 rpc |
| Scenario | 시나리오와 관련된 rpc |
| Actor | Actor(Object)의 제어에 대한 rpc |
| Sensor | Sensor 와 관련된 rpc |
| Util | 기타 기능 |
| Legacy | 미 분류 |

![MORAI documentation image](https://help-morai-sim.scrollhelp.site/__theme/images/common/info-macro-icon--39985156a8a940b9a79d.svg)

**22.R4.0 gRPC API은**아래와 같은 특징을 갖는 **MORAI Standard Proto**로 통합함

- MORAI Simulator의 서비스 카테고리 별로 RPC와 Message를 분류함
- Standard Proto 통합과 함께 RPC의 체계적인 관리를 위한 재구조화
- 메세지는 객체 단위로 분리
  - 확장성을 고려하여 message들을 각각의 성격에 부합하는 객체 단위로 분리

### 22.R4.0 gRPC Proto 구조

gRPC API는 아래와 같은 Proto 파일을 단위로 구성된다.

각 Proto 파일은 ‘Service > Procedure(RPC) > Message’ 순서의 계층적 구조를 갖는다.

Procedure 항목에서는 사용자가 호출할 수 있는 Remote Procedure에 대해서 카테고리를 나누어서 설명하고, Message 항목에서는 Procedure 정의 시 사용하는 Message들에 대해서 카테고리를 나누어서 설명한다.

CODE

```
syntax = "proto3";
package sample_msgs;
service SampleService {
    rpc FindPersonUnary (Person) returns (Person) { }
    rpc FindPersonStream (stream Person) returns (stream Person) { }
}

message Person {
    required string user_name        = 1;
    optional int64  favourite_number = 2;
    repeated string interests        = 3;
}

enum StatusCode {
	SUCCESS = 0;
	UNKNOWN_FAILURE = 1;  
}
```

- syntax
  - 위 예제에서는 proto3를 지정하여 version3의 규약을 따르겠다고 선언했습니다. 이를 명시하지 않으면 기본으로는 version2 문법을 따르게 됩니다. 버전에 따라 지원 언어와 문법에도 차이가 나타납니다. proto3가 지원 언어가 더 많고 새로운 기능 지원을 위해 proto3를 사용할 것을 권장합니다.
- message와 field
  - Proto파일은 주고 받는 데이터를 message라고 정의합니다. 그리고 이 메시지는 여러 타입의 field로 구성되며, 각 field는 고유한 번호를 가지게 됩니다. field 번호는 최소 1, 최대 536,870,911로 지정 가능하며, 19000~19999는 프로토콜 버퍼 구현을 위해 예약된 값이므로 사용할 수 없습니다.
  - 네이밍 이름은 PascalCase, 필드 이름은 snake_case로 사용할 것을 권장하고 있으며, 필드 이름은 숫자로 시작할 수 없습니다.
- package
  - package는 message 이름을 중첩 없이 구분할 때 사용합니다.
- service
  - service는 RPC를 통해 서버가 클라이언트에게 제공할 함수를 정의합니다. 기본적으로는 단방향(클라이언트 요청 서버 응답)으로 동작하지만 stream 옵션을 줘서 스트리밍을 구현할 수 있습니다.
- enum
  - enum은 상수를 저장하는 열거형 데이터입니다.

×


---
