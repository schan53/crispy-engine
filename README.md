# crispy-engine

/smart-buoy/
├── yolo_env/                      # Python 가상환경 폴더
└── image_processing/
    ├── dual_camera_autodrive_lux_system.py  # 메인 제어 스크립트 (비전, AI, 모터 제어)
    ├── buoy_client.py             # GPS 데이터 수집 및 서버 전송 클라이언트
    ├── sb_main.sh                 # 메인 스크립트 실행 및 재시작 관리 셸 스크립트
    ├── yolo11s-pose_ncnn_model/   # YOLO 모델 (주간 RGB용)
    └── yolo11n-pose_ncnn_model/   # YOLO 모델 (야간 열화상용)

/Arduino/
└── integrated_controll.ino        # 아두이노 통합 제어 펌웨어

/home/pi/.config/autostart/
└── buoy_autostart.desktop         # 라즈베리파이 부팅 시 자동 실행 설정 파일
