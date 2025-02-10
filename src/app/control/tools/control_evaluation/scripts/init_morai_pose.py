#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------------
# @file           init_morai_pose.py 
# @brief          init vehicle pose in morai simulator
#
# @authors        Seounghoon Park (sunghoon8585@gmail.com)
#
# @date           2024-12-31      Created by Seounghoon Park
# -----------------------------------------------------------------------------

import sys
import json
from pathlib import Path

target_path = Path(__file__).resolve().parents[5] / 'bsw' / 'simulator' / 'morai_udp_networkmodule'
sys.path.append(str(target_path))

# udp network module
from lib.network.UDP import Sender
from lib.define.MultiEgoSetting import MultiEgoSetting


class InitMoraiPose(object):
    def __init__(self, init_pose_json_path, udp_ip, udp_port, speed_kph):
        """
        Morai Simulator에 초기 위치 정보를 전송하는 클래스
        """
        # Read ROS params
        self.init_pose_json_path    = Path(__file__).resolve().parents[0] / 'init_pose.json'
        self.udp_ip                 = ""

        # init UDP sender
        self.multi_ego_sender = Sender(udp_ip, int(udp_port))

        # load json init pose
        self.init_pos_x = 0.0
        self.init_pos_y = 0.0
        self.init_pos_z = 0.0
        self.init_roll_deg = 0.0
        self.init_pitch_deg = 0.0
        self.init_yaw_deg = 0.0
        self.init_speed_kph = speed_kph
        self.load_init_pose(init_pose_json_path)

        print("[Morai Pose Intiator] init_pose_json_path: %s", init_pose_json_path)
        print("[Morai Pose Intiator] udp_ip: %s", udp_ip)
        print("[Morai Pose Intiator] udp_port: %d", udp_port)

        self.run()



    def load_init_pose(self, init_pose_json_path):
        """
        JSON 파일에서 초기 위치와 자세 로드
        """
        if not init_pose_json_path:
            print("No JSON file path provided. Using default [0,0,0,0,0,0].")
            return

        try:
            with open(init_pose_json_path, 'r') as f:
                data = json.load(f)
            self.init_pos_x = data["m_InitPos"]["x"]
            self.init_pos_y = data["m_InitPos"]["y"]
            self.init_pos_z = data["m_InitPos"]["z"]
            self.init_roll_deg = data["m_InitRot"]["x"]
            self.init_pitch_deg = data["m_InitRot"]["y"]
            self.init_yaw_deg = data["m_InitRot"]["z"]
        except Exception as e:
            print("Failed to read or parse JSON: %s", e)
            print("Using default pose [0,0,0,0,0,0].")



    def run(self):
        """
        init ego pose by sending UDP to MORAI
        """
        multi_ego = MultiEgoSetting()
        multi_ego.Num_of_Ego = 1
        multi_ego.Cam_index = 0

        multi_ego.data[0].ego_index   = 0
        multi_ego.data[0].position_x  = self.init_pos_x
        multi_ego.data[0].position_y  = self.init_pos_y
        multi_ego.data[0].position_z  = self.init_pos_z
        multi_ego.data[0].roll        = self.init_roll_deg          
        multi_ego.data[0].pitch       = self.init_pitch_deg
        multi_ego.data[0].yaw         = self.init_yaw_deg
        multi_ego.data[0].velocity    = self.init_speed_kph 
        multi_ego.data[0].gear        = 4           # 1: Parking    2: Rear    3: Neutral   4:Drive    
        multi_ego.data[0].ctrl_mode   = 2           # 1: Keyboard   2:Automode

        self.multi_ego_sender.send(multi_ego)