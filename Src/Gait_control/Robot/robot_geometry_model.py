import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[3] 
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))


import numpy as np
from typing import List
from Src.Gait_control.Robot import config as cfg
import threading
import warnings

class Spider_robot:
    def __init__(self):
        self.legs = {
            "L1": Leg("L1"), 
            "L2": Leg("L2"),
            "L3": Leg("L3"),
            "R1": Leg("R1"),
            "R2": Leg("R2"),
            "R3": Leg("R3")
        }
        pass

    def write_all_ends_coordinate(self, ends_coordinate: dict):
        '''
        param shoud be:
        ends_coordinate = {
            "L1": [xxx, xxx, xxx], 
            "L2": [xxx, xxx, xxx],
            ......
        }
        '''
        try:
            self.legs["L1"].write_end_coordinate(ends_coordinate["L1"])
            self.legs["L2"].write_end_coordinate(ends_coordinate["L2"])
            self.legs["L3"].write_end_coordinate(ends_coordinate["L3"])
            self.legs["L4"].write_end_coordinate(ends_coordinate["L4"])
            self.legs["L5"].write_end_coordinate(ends_coordinate["L5"])
            self.legs["L6"].write_end_coordinate(ends_coordinate["L6"])
        except Exception:
            pass
        pass

    def read_all_ends_coordinate(self) -> dict:
        try:
            ends_coordinate = {
                "L1": self.legs["L1"].read_end_coordinate(),
                "L2": self.legs["L2"].read_end_coordinate(),
                "L3": self.legs["L3"].read_end_coordinate(),
                "L4": self.legs["L4"].read_end_coordinate(),
                "L5": self.legs["L5"].read_end_coordinate(),
                "L6": self.legs["L6"].read_end_coordinate(),
            }
            return ends_coordinate 
        except Exception:
            pass           

    def _write_all_joints_angle(self):
        pass
    
    def read_all_joints_angle(self) -> dict:        
        """
        return:
        { "L1_coxa": ..., "L1_femur": ..., "L1_tibia": ..., "R3_tibia": ... }
        """
        all_joint_angle = {}
        try:
            for name, leg in self.legs.items():
                try:
                    joint_dict = leg.read_all_joints_angle()
                    if isinstance(joint_dict, dict):
                        for joint_name, angle in joint_dict.items():
                            all_joint_angle[f"{name}_{joint_name}"] = angle
                except Exception:
                    continue
            return all_joint_angle
        except Exception:
            return {}

    def read_servo_outputs(self) -> dict:
        """
        return:
        { "L1_coxa": val, "L1_femur": val, ... }
        """
        outputs = {}
        try:
            for name, leg in self.legs.items():
                try:
                    so = leg.read_servo_output()
                    if isinstance(so, dict):
                        outputs.update(so)
                except Exception:
                    continue
            return outputs
        except Exception:
            return {}

    def publish_joints_angle(self):
        pass
    
    def publish_to_servo(self):
        pass
        


class Leg:
    def __init__(self, name: str,
                 Coxa_length: float = cfg.COXA_LENGTH, 
                 Femur_length: float = cfg.FEMUR_LENGTH, 
                 Tibia_length: float = cfg.TIBIA_LENGTH,
                 Link_ground_length: float = cfg.LINK_GROUND_LENGTH,
                 Link_crank_length: float = cfg.LINK_CRANK_LENGTH,
                 Link_coupler_length: float = cfg.LINK_COUPLER_LENGTH,
                 Link_rocker_length: float = cfg.LINK_ROCKER_LENGTH,
                 Default_Coxa_Angle: float = cfg.DEFAULT_COXA_ANGLE,
                 Default_Femur_Angle: float = cfg.DEFAULT_FEMUR_ANGLE,
                 Default_Tibia_Angle: float = cfg.DEFAULT_TIBIA_ANGLE,
                 Default_Coxa_Servo_Output: float = cfg.DEFAULT_COXA_SERVO_OUTPUT,
                 Default_Femur_Servo_Output: float = cfg.DEFAULT_FEMUR_SERVO_OUTPUT,
                 Default_Tibia_Servo_Output: float = cfg.DEFAULT_TIBIA_SERVO_OUTPUT,
                 Max_coxa_servo_output: dict = cfg.MAX_COXA_SERVO_OUTPUT,
                 Min_coxa_servo_output: dict = cfg.MIN_COXA_SERVO_OUTPUT,
                 Max_femur_servo_output: dict = cfg.MAX_FEMUR_SERVO_OUTPUT,
                 Min_femur_servo_output: dict = cfg.MIN_FEMUR_SERVO_OUTPUT,
                 Max_tibia_servo_output: dict = cfg.MAX_TIBIA_SERVO_OUTPUT,
                 Min_tibia_servo_output: dict = cfg.MIN_TIBIA_SERVO_OUTPUT):
        
        self.name = name     
        if 'L' in self.name:
            self.side = "left"
        elif 'R' in self.name:
            self.side = "right"
        else:
            raise Exception
        
        self._lock = threading.RLock()

        self.Coxa_length = Coxa_length
        self.Femur_length = Femur_length
        self.Tibia_length = Tibia_length

        self.Link_ground_length = Link_ground_length
        self.Link_crank_length = Link_crank_length
        self.Link_coupler_length = Link_coupler_length
        self.Link_rocker_length = Link_rocker_length

        self.__joint = {
            "coxa": Default_Coxa_Angle,
            "femur": Default_Femur_Angle,
            "tibia": Default_Tibia_Angle
        }

        self.__servo_output = {
            self.name + "_coxa": Default_Coxa_Servo_Output,
            self.name + "_femur": Default_Femur_Servo_Output,
            self.name + "_tibia": Default_Tibia_Servo_Output
        }

        self.__servo_output_limitation = {
            "max_coxa": Max_coxa_servo_output[self.side],
            "min_coxa": Min_coxa_servo_output[self.side],
            "max_femur": Max_femur_servo_output[self.side],
            "min_femur": Min_femur_servo_output[self.side],
            "max_tibia": Max_tibia_servo_output[self.side],
            "min_tibia": Min_tibia_servo_output[self.side],
        }

        self.coxa_servo_output_offset = self.calculate_coxa_servo_output_offset()
        self.calculate_servo_output_limitation()      

        self.__end_coordinate = self.forward_kinematic()
        self.write_end_coordinate(self.__end_coordinate)



    def read_end_coordinate(self) -> List[float]:
        with self._lock:
            #print(self.__end_coordinate)
            return self.__end_coordinate

    def read_joint_angle(self, name: str) -> float:
        try:
            with self._lock:
                #print(self.__joint[name])
                return self.__joint[name]
        except Exception:
            pass

    def read_all_joints_angle(self) -> dict:
        with self._lock:
            #print(self.__joint)
            return self.__joint

    def read_servo_output(self) -> dict:
        with self._lock:
            #print(self.__servo_output)
            return self.__servo_output
    
    def write_end_coordinate(self, end_coordinate: List[float]):
        # update all data
        # conduct safty check in servo_output_convert()
        with self._lock:
            self.__end_coordinate = end_coordinate
            try:
                self.__joint["coxa"], self.__joint["femur"], self.__joint["tibia"] = self.inverse_kinematic()
                self.__servo_output[self.name + "_coxa"], self.__servo_output[self.name + "_femur"], self.__servo_output[self.name + "_tibia"] = self.servo_output_convert()
            except Exception as e:
                print("e")
                exit(1)

    def _write_joint_angle(self, name: str, angle: float):
        pass

    def _write_all_joints_angle(self, joints_anlge: dict):
        pass

    def inverse_kinematic(self) -> List[float]:
        x, y, z = self.read_end_coordinate()
        
        #theta_1
        theta_1 = np.arctan2(y, x)
        
        #theta_2
        R = np.hypot(x, y)
        r = R - self.Coxa_length
        Distance = np.hypot(r, z)       # Distance from end_coordinate to Femur joint
        
        if Distance > self.Femur_length + self.Tibia_length or Distance < np.abs(self.Femur_length - self.Tibia_length):
            raise Exception(f"{self.name} target coordinate has no inverse kinematic solution.")
        
        alpha = np.arctan2(r, -z)
        beta = np.arccos( (Distance**2 + self.Femur_length**2 - self.Tibia_length**2) / (2*Distance*self.Femur_length) )
        theta_2 = np.pi - beta - alpha
        
        #theta_3
        theta_3 = np.arccos( (self.Tibia_length**2 + self.Femur_length**2 - Distance**2)  / (2*self.Tibia_length*self.Femur_length) ) - theta_2

        return np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3)

    def forward_kinematic(self) -> List[float]:
        r = self.Femur_length * np.sin(np.deg2rad(self.read_joint_angle("femur"))) + self.Tibia_length * np.sin(np.deg2rad(self.read_joint_angle("tibia")))
        R = self.Coxa_length + r

        x = R * np.cos(np.deg2rad(self.read_joint_angle("coxa")))
        y = R * np.sin(np.deg2rad(self.read_joint_angle("coxa")))
        z = self.Femur_length * np.cos(np.deg2rad(self.read_joint_angle("femur"))) - self.Tibia_length * np.cos(np.deg2rad(self.read_joint_angle("tibia")))
        
        return x, y, z

    def servo_output_convert(self) -> List[float]:
        '''
        check servo output limitation 
        '''
        
        if self.side == "left":
            coxa_servo_output = self.read_joint_angle("coxa")
            femur_servo_output = self.read_joint_angle("femur")
            tibia_servo_output = self.link_kinematic()
        elif self.side == "right":
            coxa_servo_output = self.read_joint_angle("coxa")
            femur_servo_output = 180.0 - self.read_joint_angle("femur")
            tibia_servo_output = self.link_kinematic()
        else:
            pass

        ALLOWABLE_WARNING = 2.0
        
        # check angular limitation, raise error if exceed the limit, raise warning if close to the limit(2°)
        if not (self.__servo_output_limitation["min_coxa"] <= coxa_servo_output <= self.__servo_output_limitation["max_coxa"]):
            raise Exception(f"{self.name} coxa servo output exceed limitation!")
        elif not (self.__servo_output_limitation["min_coxa"] + ALLOWABLE_WARNING <= coxa_servo_output <= self.__servo_output_limitation["max_coxa"] - ALLOWABLE_WARNING):
            warnings.warn(f"{self.name} coxa servo output close to limitation!", UserWarning)

        if not (self.__servo_output_limitation["min_femur"] <= femur_servo_output <= self.__servo_output_limitation["max_femur"]):
            raise Exception(f"{self.name} femur servo output exceed limitation!")
        elif not (self.__servo_output_limitation["min_femur"] + ALLOWABLE_WARNING <= femur_servo_output <= self.__servo_output_limitation["max_femur"] - ALLOWABLE_WARNING):
            warnings.warn(f"{self.name} femur servo output close to limitation!", UserWarning)
        
        if not (self.__servo_output_limitation["min_tibia"] <= tibia_servo_output <= self.__servo_output_limitation["max_tibia"]):
            raise Exception(f"{self.name} tibia servo output exceed limitation!")
        elif not (self.__servo_output_limitation["min_tibia"] + ALLOWABLE_WARNING <= tibia_servo_output <= self.__servo_output_limitation["max_tibia"] - ALLOWABLE_WARNING):
            warnings.warn(f"{self.name} tibia servo output close to limitation!", UserWarning)

        return coxa_servo_output, femur_servo_output, tibia_servo_output

    def calculate_coxa_servo_output_offset(self) -> float:
        # unit degree
        Distance = np.hypot(self.Link_rocker_length, self.Link_ground_length)
        alpha = np.arccos(self.Link_ground_length / Distance)
        beta = np.arccos((Distance**2 + self.Link_crank_length**2 - self.Link_coupler_length**2) / (2 * Distance * self.Link_crank_length))
        return np.rad2deg(np.pi/2 - alpha - beta)

    def calculate_servo_output_limitation(self):
        '''
        compare with default angular limitation
        '''
        # tibia limitation
        with self._lock:
            if self.side == "left":
                phi_0 = self.coxa_servo_output_offset       # degree
                max_theta_s = np.rad2deg(np.arccos((self.Link_ground_length**2 + self.Link_crank_length**2 - (self.Link_rocker_length + self.Link_coupler_length)**2) / (2 * self.Link_ground_length * self.Link_crank_length))) + phi_0
                min_theta_s = np.rad2deg(np.arccos((self.Link_ground_length**2 + (self.Link_crank_length + self.Link_coupler_length)**2 - self.Link_rocker_length**2) / (2 * self.Link_ground_length * (self.Link_crank_length + self.Link_coupler_length)))) + phi_0
                self.__servo_output_limitation["max_coxa"] = min(self.__servo_output_limitation["max_coxa"], max_theta_s)
                self.__servo_output_limitation["min_coxa"] = max(self.__servo_output_limitation["min_coxa"], min_theta_s)

            elif self.side == "right":
                phi_0 = self.coxa_servo_output_offset       # degree
                max_theta_s = np.rad2deg(np.pi - np.arccos((self.Link_ground_length**2 + (self.Link_crank_length + self.Link_coupler_length)**2 - self.Link_rocker_length**2) / (2 * self.Link_ground_length * (self.Link_crank_length + self.Link_coupler_length)))) - phi_0
                min_theta_s = np.rad2deg(np.pi - np.arccos((self.Link_ground_length**2 + self.Link_crank_length**2 - (self.Link_coupler_length + self.Link_rocker_length)**2) / (2 * self.Link_ground_length * self.Link_crank_length))) - phi_0
                self.__servo_output_limitation["max_coxa"] = min(self.__servo_output_limitation["max_coxa"], max_theta_s)
                self.__servo_output_limitation["min_coxa"] = max(self.__servo_output_limitation["min_coxa"], min_theta_s)
            else:
                pass

    def link_kinematic(self) -> float:
        # input unit: degree
        # output unit: degree       
        phi_0 = np.deg2rad(self.coxa_servo_output_offset)     # rad
        if self.side == "left":
            theta_3l = np.deg2rad(self.read_joint_angle("femur") + self.read_joint_angle("tibia"))
            Distance = np.sqrt(self.Link_rocker_length**2 + self.Link_ground_length**2 - 2 * self.Link_rocker_length * self.Link_ground_length * np.cos(np.pi - theta_3l))
            alpha = np.arccos((self.Link_ground_length**2 + Distance**2 - self.Link_rocker_length**2) / (2 * self.Link_ground_length * Distance))
            beta = np.arccos((self.Link_crank_length**2 + Distance**2 - self.Link_coupler_length**2) / (2 * Distance * self.Link_crank_length))
            return np.rad2deg(alpha + beta + phi_0)
        elif self.side == "right":
            theta_3r = np.pi - np.deg2rad(self.read_joint_angle("femur") + self.read_joint_angle("tibia"))
            Distance = np.sqrt(self.Link_rocker_length**2 + self.Link_ground_length**2 - 2 * self.Link_rocker_length * self.Link_ground_length * np.cos(theta_3r))
            alpha = np.arccos((self.Link_ground_length**2 + Distance**2 - self.Link_rocker_length**2) / (2 * self.Link_ground_length * Distance))
            beta = np.arccos((self.Link_crank_length**2 + Distance**2 - self.Link_coupler_length**2) / (2 * Distance * self.Link_crank_length))
            return np.rad2deg(np.pi - alpha - beta - phi_0)
        else:
            pass
        pass
    
if __name__ == "__main__":
    robot = Spider_robot()
    leg = robot.legs["L1"]
    #angle = leg.inverse_kinematic()
    
    while True:
        end_coordinate = [10.0, 91.0, -152.0]
        leg.write_end_coordinate(end_coordinate)
        #print(leg.read_all_joints_angle())
        print(leg.read_servo_output())

        pass
