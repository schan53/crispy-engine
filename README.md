# crispy-engine

폴더 PATH의 목록입니다.
볼륨 일련 번호는 8A67-0F10입니다.
C:.
│  buoy_autostart.desktop  # 라즈베리파이 부팅 시 자동 실행 설정 파일
│  
└─autostart
폴더 PATH의 목록입니다.
볼륨 일련 번호는 8A67-0F10입니다.
C:.
    integrated_controll.ino.ino  # 아두이노 통합 제어 펌웨어
    
폴더 PATH의 목록입니다.
볼륨 일련 번호는 8A67-0F10입니다.
C:.
│  gps_center.py
│  gps_test.py
│  
├─export_model
│  │  export_model.py
│  │  export_model_pose.py
│  │  export_OpenThermalPose2.py
│  │  yolo11n-pose.pt
│  │  yolo11n-pose.torchscript
│  │  yolo11s-pose.pt
│  │  yolo11s-pose.torchscript
│  │  
│  ├─yolo11n-pose_ncnn_320
│  │      metadata.yaml
│  │      model.ncnn.bin
│  │      model.ncnn.param
│  │      model_ncnn.py
│  │      
│  └─yolo11s-pose_ncnn_320
│          metadata.yaml
│          model.ncnn.bin
│          model.ncnn.param
│          model_ncnn.py
│          
├─image_processing
│  │  buoy_client.py  ## GPS 데이터 수집 및 서버 전송 클라이언트
│  │  dual_camera_autodrive_lux_system.py  ### 메인 제어 스크립트 (비전, AI, 모터 제어)
│  │  sb_main.sh  # 메인 스크립트 실행 및 재시작 관리 셸 스크립트
│  │  
│  ├─CSI
│  │  │  image_processing.py
│  │  │  image_processing_pose.py
│  │  │  realtime_handup.py
│  │  │  RGB_autodrive_system.py
│  │  │  
│  │  ├─yolo11n-pose_ncnn_model
│  │  │      metadata.yaml
│  │  │      model.ncnn.bin
│  │  │      model.ncnn.param
│  │  │      model_ncnn.py
│  │  │      
│  │  ├─yolo11n_ncnn_model  ### YOLO 모델 (야간 열화상용)
│  │  │      metadata.yaml
│  │  │      model.ncnn.bin
│  │  │      model.ncnn.param
│  │  │      model_ncnn.py
│  │  │      
│  │  └─yolo11s-pose_ncnn_model  ## YOLO 모델 (주간 RGB용)
│  │          metadata.yaml
│  │          model.ncnn.bin
│  │          model.ncnn.param
│  │          model_ncnn.py
│  │          
│  ├─thermal
│  │  │  cap_test.py
│  │  │  metadata.yaml
│  │  │  model.ncnn.bin
│  │  │  model.ncnn.param
│  │  │  model_ncnn.py
│  │  │  test.py
│  │  │  thermal_final_pose.py
│  │  │  thermal_pose.py
│  │  │  Thermal_Pose_Realtime.py
│  │  │  thermal_temp.py
│  │  │  thermal_test2.py
│  │  │  yolo11n-pose.torchscript
│  │  │  yolo11s-pose.torchscript
│  │  │  
│  │  ├─New
│  │  ├─yolo11n-pose_ncnn_416
│  │  │      metadata.yaml
│  │  │      model.ncnn.bin
│  │  │      model.ncnn.param
│  │  │      model_ncnn.py
│  │  │      
│  │  ├─yolo11n-pose_ncnn_model
│  │  │      metadata.yaml
│  │  │      model.ncnn.bin
│  │  │      model.ncnn.param
│  │  │      model_ncnn.py
│  │  │      
│  │  ├─yolo11s-pose_ncnn_416
│  │  │      metadata.yaml
│  │  │      model.ncnn.bin
│  │  │      model.ncnn.param
│  │  │      model_ncnn.py
│  │  │      
│  │  └─yolo11s-pose_ncnn_model
│  │          metadata.yaml
│  │          model.ncnn.bin
│  │          model.ncnn.param
│  │          model_ncnn.py
│  │          
│  ├─yolo11n-pose_ncnn_model
│  │      metadata.yaml
│  │      model.ncnn.bin
│  │      model.ncnn.param
│  │      model_ncnn.py
│  │      
│  └─yolo11s-pose_ncnn_model
│          metadata.yaml
│          model.ncnn.bin
│          model.ncnn.param
│          model_ncnn.py
│          
└─yolo_env  ## Python 가상환경 폴더
