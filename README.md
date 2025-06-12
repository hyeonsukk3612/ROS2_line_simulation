***

line tracer 패키지를 구현하고 동영상을 이용하여 시뮬레이션을 수행
하라 아래 그림처럼 카메라 대신에 동영상을 입력 받아 처리할 것
동영상은 https :://github com/ 2 sungryul/simulation 에서 다운로드
완료 후 강사에게 확인 받을 것 토픽주기 30 hz, 노드 3 개 토픽 2 개
패키지의 소스코드를 깃허브에 업로드할 것
실행결과를 동영상 (robot view) 으로 촬영하여 유튜브에 업로드할 것


![image](https://github.com/user-attachments/assets/b9e46fe1-01aa-4ba3-93cc-7702345ceb1a)


***

설정 및 빌드

***

colcon build --packages-select line_follow

source install/setup.bash

***

젝슨보드

***

ros2 run camera sub

ros2 run camera sub_jetson

***

윈도우

***

창1

ros2 run line_follow video_publisher

창2

ros2 run line_follow line_tracer
***

카메라 실제 구동 영상입니다

***

[https://youtu.be/n8J1Hp8aWLo](https://youtu.be/62VgNZelYME)
