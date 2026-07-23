# 주의: 아래 숫자 중 kick_vel_target, kick_quality_threshold, reward_scales의 kicking_ball_vel/
# kick_contact/kick_hold 값은 검증된 출처가 없는 "합리적 시작값"입니다. 처음 몇 번의 학습 곡선을
# 보고 반드시 튜닝하세요 (특히 kick_vel_target: 너무 낮으면 살살 밀기만 해도 만점, 너무 높으면
# 초반에 신호가 거의 안 잡힘).

from typing import Union
from params_proto import Meta
from dribblebot.envs.base.legged_robot_config import Cfg


def config_go2_kick(Cnfg: Union[Cfg, Meta]):
    _ = Cnfg.robot
    _.name = "go2"

    _ = Cnfg.init_state
    _.pos = [0.0, 0.0, 0.42]  # unitree_rl_gym go2_config.py 실측값 (Go1: 0.34m)
    _.default_joint_angles = {  # Go1과 동일한 규약 (unitree_rl_gym에서 확인)
        'FL_hip_joint': 0.1, 'RL_hip_joint': 0.1, 'FR_hip_joint': -0.1, 'RR_hip_joint': -0.1,
        'FL_thigh_joint': 0.8, 'RL_thigh_joint': 1.0, 'FR_thigh_joint': 0.8, 'RR_thigh_joint': 1.0,
        'FL_calf_joint': -1.5, 'RL_calf_joint': -1.5, 'FR_calf_joint': -1.5, 'RR_calf_joint': -1.5,
    }

    _ = Cnfg.control
    _.control_type = 'P'
    # Su et al. 2025 (CoRL, Walk/Dribble/Kick 공용 정책)이 실제로 쓴 값: Kp=35, Kd=0.5.
    # DribbleBot/unitree_rl_gym 기본값(Kp=20)보다 높습니다 - 타격 파워 확보 목적으로 보입니다.
    _.stiffness = {'joint': 35.}
    _.damping = {'joint': 0.5}
    _.action_scale = 0.25
    _.hip_scale_reduction = 0.5
    _.decimation = 4

    # _ = Cnfg.asset
    # _.file = '/root/Desktop/workspace/expo/unitree_rl_gym/resources/robots/go2/urdf/go2.urdf'
    # _.foot_name = "foot"
    # _.penalize_contacts_on = ["thigh", "calf"]
    # _.terminate_after_contacts_on = ["base"]
    # _.self_collisions = 1  # unitree_rl_gym 공식값 (Go1 DribbleBot 기본은 0) - URDF 임포트 후 재확인 권장
    # _.flip_visual_attachments = False
    # _.fix_base_link = False
    _ = Cnfg.asset
    _.file = '/root/Desktop/workspace/expo/unitree_rl_gym/resources/robots/go2/urdf/go2.urdf'
    _.foot_name = "foot"
    _.penalize_contacts_on = ["thigh", "calf"]
    _.terminate_after_contacts_on = ["base", "thigh", "calf", "Head_upper", "Head_lower"]  # 무릎/종아리/허벅지/머리 바닥 닿으면 즉시 실패 리셋 (발바닥 서기 강제)
    _.self_collisions = 1  # unitree_rl_gym 공식값 (0: 활성화 시 링크 침범으로 인한 튀김/미끄러짐 유발 방지)
    _.collapse_fixed_joints = False      # Fixed joint 병합 비활성화 (충돌체 노출 & 찌그러짐 해결)
    _.flip_visual_attachments = True     # 또는 False로 조정하며 메쉬 뒤집힘 확인
    _.fix_base_link = False

    _ = Cnfg.ball
    _.ball_init_pos = [0.8, 0.0, 0.11]   # 로봇 정면 0.8m (5호 축구공 반지름 0.11m)
    _.mass = 0.43                        # 공식 5호 축구공 질량 (430g)
    _.radius = 0.11                      # 공식 5호 축구공 반지름 (11cm, 지름 22cm)
    _.init_pos_range = [0.40, 0.30, 0.0]   # 전방 x in [0.4m, 1.2m], y in [-0.3m, 0.3m] 범위 무작위 스폰 (접근 보행 + 킥 풀 시퀀스 학습 지원)
    _.init_vel_range = [0.0, 0.0, 0.0]
    _.pos_reset_prob = 0.0
    _.vel_reset_prob = 0.0

    _ = Cnfg.rewards
    _.reward_container_name = "KickRewards"
    _.only_positive_rewards = False
    # legged_robot_config.py에 이미 있던 옵션 - 이름 그대로 Ji et al. 2022 스타일 클리핑.
    # 짧은 접촉 순간의 보상이 다른 항들에 묻히지 않게 해줘서 킥 태스크에 유리할 걸로 예상합니다.
    _.only_positive_rewards_ji22_style = True
    _.sigma_rew_neg = 5
    _.tracking_sigma = 0.25
    _.kick_quality_threshold = 0.6   # r_kick(0~1)이 이 값 넘으면 Pursue&Strike -> Hold 전환 (초반 학습 가속)
    _.kick_vel_target = 2.0          # m/s. 초반 크레딧 할당 신호 포착용 목표 공 속도
    _.soft_dof_pos_limit = 0.9
    _.base_height_target = 0.34
    
    # 주저앉음/넘어짐 상태 편법 수백 스텝 유지 방지 (Check Termination 안전장치)
    _.use_terminal_body_height = True
    _.terminal_body_height = 0.22    # 몸통 높이가 0.22m 미만으로 주저앉으면 즉시 실패 에피소드 리셋
    _.use_terminal_roll_pitch = True
    _.terminal_body_ori = 0.50       # 롤/피치 기울기가 45도 이상 넘어지면 즉시 실패 에피소드 리셋

    _ = Cnfg.reward_scales
    _.base_height = -2.0             # 고개 쳐박기/몸통 쳐짐 방지 (목표 높이 0.34m 유지)
    _.pitch_forward_penalty = -5.0   # 상체 전방 숙임/꼬꾸라짐 억제 (무게중심 뒤쪽 유지)
    _.stance_legs_support = 3.0        # 킥 다리를 제외한 3개 다리(반대쪽 앞다리+뒷다리2개) 단단한 지지 보상 (허우적거림 방지)
    _.lin_vel_z = -2.0               # 상하 요동 억제
    _.ang_vel_xy = -0.05             # 롤/피치 흔들림 억제
    _.ang_vel_z = -0.1               # 무의미한 제자리 팽이 회전 억제
    _.feet_slip = -0.08              # 지지 발 바닥 미끄러짐 억제
    _.action_smoothness_2 = -0.002   # 액션 떨림/갑작스러운 튐 부드럽게 억제
    _.feet_air_time = 0.0
    # 정규화/안정성 항 - go1_config.py 기본값 그대로
    _.torques = -0.0001
    _.action_rate = -0.05            # 액션 변화율 감점 상향 (다리 4개 허우적거림 억제)
    _.dof_acc = -5.0e-7              # 관절 급격한 가속도/발작 억제 (모터열화 방지 및 부드러운 스윙)
    _.dof_pos = -0.01                # 기본 자세 이탈 약한 페널티 (킥 시 다리 들기를 억제하지 않도록 약하게 설정)
    _.dof_pos_limits = -10.0
    _.orientation = -5.0
    _.collision = -5.0
    # 접근 단계 - SoccerRewards에 이미 있는 검증된 함수 재사용
    # (스케일은 train_dribbling.py에서 실제로 쓴 값 그대로 가져옴)
    _.dribbling_robot_ball_pos = 4.0
    _.dribbling_robot_ball_yaw = 4.0
    _.dribbling_robot_ball_vel = 0.5
    # 킥 전용 신규 항 (kick_rewards.py)
    _.kicking_ball_vel = 3.0
    _.kick_contact = 3.0
    _.kick_hold = 2.0                # 킥 임팩트 성공 직후 자리에 멈춰 자세를 안정화(Hold)하는 2단계 보상

    # 접근 보행 시 정갈한 4족 교차 보행(Trot Gait) 형성 보상 (soccer_rewards.py 내장 함수)
    _.tracking_contacts_shaped_force = 1.0
    _.tracking_contacts_shaped_vel = 1.0
    _.gait_force_sigma = 100.0
    _.gait_vel_sigma = 10.0

    # 드리블 태스크와 동일하게 base 속도 직접 트래킹은 끔 (로코모션은 부수적으로만 발생)
    _.tracking_lin_vel = 0.0
    _.tracking_ang_vel = 0.0

    _ = Cnfg.env
    _.add_balls = True
    _.num_envs = 4096
    _.episode_length_s = 8.0  # 드리블(20s)보다 짧게 - 한 에피소드에 킥 시도를 여러 번 압축
    _.num_privileged_obs = 6
    _.num_observations = 75

    _ = Cnfg.commands
    _.num_commands = 15
    _.resampling_time = 6.0
    _.heading_command = False
    _.distributional_commands = True

    # 킥 방향(commands[:,:2])이 이 range에서 랜덤 샘플링됩니다 - 우리가 실제로 쓰는 값
    _.lin_vel_x = [-1.5, 1.5]
    _.lin_vel_y = [-1.5, 1.5]
    _.num_bins_vel_x = 30
    _.num_bins_vel_y = 30

    # 아래부터는 gait/body-shape 관련 - KickRewards가 안 쓰지만 _step_contact_targets가
    # 무조건 호출하는 값들이라 있어야 함. train_dribbling.py 값 그대로 고정.
    _.ang_vel_yaw = [-0.0, 0.0]
    _.body_height_cmd = [-0.05, 0.05]
    _.gait_frequency_cmd_range = [3.0, 3.0]
    _.gait_phase_cmd_range = [0.5, 0.5]
    _.gait_offset_cmd_range = [0.0, 0.0]
    _.gait_bound_cmd_range = [0.0, 0.0]
    _.gait_duration_cmd_range = [0.5, 0.5]
    _.footswing_height_range = [0.09, 0.09]
    _.body_pitch_range = [-0.0, 0.0]
    _.body_roll_range = [-0.0, 0.0]
    _.stance_width_range = [0.0, 0.1]
    _.stance_length_range = [0.0, 0.1]
    _.exclusive_phase_offset = False
    _.pacing_offset = False
    _.balance_gait_distribution = False
    _.binary_phases = False
    _.gaitwise_curricula = False

    _.limit_vel_x = [-1.5, 1.5]
    _.limit_vel_y = [-1.5, 1.5]
    _.limit_vel_yaw = [-0.0, 0.0]
    _.limit_body_height = [-0.05, 0.05]
    _.limit_gait_frequency = [3.0, 3.0]
    _.limit_gait_phase = [0.5, 0.5]
    _.limit_gait_offset = [0.0, 0.0]
    _.limit_gait_bound = [0.0, 0.0]
    _.limit_gait_duration = [0.5, 0.5]
    _.limit_footswing_height = [0.09, 0.09]
    _.limit_body_pitch = [-0.0, 0.0]
    _.limit_body_roll = [-0.0, 0.0]
    _.limit_stance_width = [0.0, 0.1]
    _.limit_stance_length = [0.0, 0.1]

    _.num_bins_vel_yaw = 1
    _.num_bins_body_height = 1
    _.num_bins_gait_frequency = 1
    _.num_bins_gait_phase = 1
    _.num_bins_gait_offset = 1
    _.num_bins_gait_bound = 1
    _.num_bins_gait_duration = 1
    _.num_bins_footswing_height = 1
    _.num_bins_body_roll = 1
    _.num_bins_body_pitch = 1
    _.num_bins_stance_width = 1
    _.num_bins_stance_length = 1

    _ = Cnfg.terrain
    _.mesh_type = 'plane'
    _.curriculum = False
    _.teleport_robots = False
    _.x_init_range = 0.0                # 로봇 초기 x 스폰 노이즈 제거 (공과 밀착되는 현상 방지)
    _.y_init_range = 0.0                # 로봇 초기 y 스폰 노이즈 제거
    _.yaw_init_range = 0.0              # 로봇 초기 yaw 스폰 노이즈 제거

    _ = Cnfg.sim
    _.physx.max_gpu_contact_pairs = 2 ** 25       # 4096개 병렬 환경 및 로봇-공-지면 접촉쌍 메모리 확보 (33,554,432)
    _.physx.default_buffer_size_multiplier = 32   # 4096개 환경에서 Patch Buffer Overflow 경고 없는 쾌적한 버퍼 세팅

    _ = Cnfg.domain_rand
    _.randomize_friction = True
    _.friction_range = [0.3, 1.6]        # RoboNaldo 도메인 랜덤화 범위 참고 (지면 마찰)
    _.randomize_restitution = True
    _.restitution_range = [0.0, 0.95]    # 공 반발계수까지 포함해서 넓게 (마른/미끄러운 표면 대비)
    _.randomize_base_mass = True
    _.added_mass_range = [-1.0, 3.0]
    _.randomize_motor_strength = True
    _.motor_strength_range = [0.9, 1.1]
    _.push_robots = False  # v0에서는 끄고, hold 단계가 안정되면 켜서 강건성 추가

    _ = Cnfg.sensors
    _.sensor_names = [
        "ObjectSensor",        # <- 공 위치, 이게 없으면 정책이 공을 못 봄
        "OrientationSensor",
        "RCSensor",
        "JointPositionSensor",
        "JointVelocitySensor",
        "ActionSensor",
        "LastActionSensor",
        "ClockSensor",
        "YawSensor",
        "TimingSensor",
    ]
    _.sensor_args = {
        "ObjectSensor": {},
        "OrientationSensor": {},
        "RCSensor": {},
        "JointPositionSensor": {},
        "JointVelocitySensor": {},
        "ActionSensor": {},
        "LastActionSensor": {"delay": 1},
        "ClockSensor": {},
        "YawSensor": {},
        "TimingSensor": {},
    }
    _.privileged_sensor_names = {
        "BodyVelocitySensor": {},
        "ObjectVelocitySensor": {},   # <- 제 kick_rewards.py의 object_lin_vel이 바로 이 privileged 센서 출력입니다
    }
    _.privileged_sensor_args = {
        "BodyVelocitySensor": {},
        "ObjectVelocitySensor": {},
    }