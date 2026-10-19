#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math
import random
import rospy
from sensor_msgs.msg import LaserScan


class ScanDisturbanceInjector(object):
    """
    ROS1 Noetic 激光雷达干扰注入节点。

    功能：
    1. 订阅原始 LaserScan：默认 /scan_raw
    2. 按干扰等级向扫描数据中注入：
       - 团状沙尘散点
       - 带状沙尘散点
       - 牛体/障碍物遮挡或 NaN 遮挡
    3. 发布带干扰 LaserScan：默认 /scan

    推荐使用方式：
    rosbag play clean.bag --clock /scan:=/scan_raw
    rosrun fence_channel_local_planner scan_disturbance_injector.py _input_topic:=/scan_raw _output_topic:=/scan _level:=light
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scan_raw")
        self.output_topic = rospy.get_param("~output_topic", "/scan")
        self.level = rospy.get_param("~level", "light").strip().lower()

        # 是否一直处于干扰状态。
        # false：周期性出现干扰，更适合统计“丢失—重捕获时间”。
        # true：全程持续干扰，更适合做极限鲁棒性测试。
        self.always_active = bool(rospy.get_param("~always_active", False))

        # 程序启动后是否立即进入第一段干扰。
        self.initial_event_immediate = bool(rospy.get_param("~initial_event_immediate", True))

        # 固定随机种子，保证同一 bag 重复播放时干扰模式基本可复现。
        self.random_seed = int(rospy.get_param("~random_seed", 20260430))
        self.rng = random.Random(self.random_seed)

        # 只在右侧围栏候选区域注入干扰，默认角度为 -85° 到 -5°。
        # 若你的雷达坐标系右侧不是负角度，请修改这两个参数。
        self.sector_min_deg = float(rospy.get_param("~sector_min_deg", -85.0))
        self.sector_max_deg = float(rospy.get_param("~sector_max_deg", -5.0))

        # 注入点的极坐标距离范围。
        self.dust_range_min_m = float(rospy.get_param("~dust_range_min_m", 0.35))
        self.dust_range_max_m = float(rospy.get_param("~dust_range_max_m", 4.50))

        # 牛体/遮挡物模拟距离范围。
        self.occlusion_range_min_m = float(rospy.get_param("~occlusion_range_min_m", 0.60))
        self.occlusion_range_max_m = float(rospy.get_param("~occlusion_range_max_m", 2.20))

        # 遮挡模式：
        # cow_body：用一段较近的平滑弧形点模拟牛体或障碍物轮廓；
        # nan：把局部角域置为 NaN，模拟围栏完全丢失；
        # inf：把局部角域置为 inf；
        # max_range：把局部角域置为 scan.range_max。
        self.occlusion_mode = rospy.get_param("~occlusion_mode", "cow_body").strip().lower()

        # 是否只保留更近的虚假回波。
        # true：符合激光雷达“最近回波”的常见现象；
        # false：直接覆盖原 range。
        self.use_nearest_return = bool(rospy.get_param("~use_nearest_return", True))

        # 是否保留原始 LaserScan 的 range_min/range_max 约束。
        self.clamp_to_scan_range = bool(rospy.get_param("~clamp_to_scan_range", True))

        # 日志输出频率，避免刷屏。
        self.print_hz = float(rospy.get_param("~print_hz", 0.2))
        self.last_print_time = 0.0

        self.cfg = self.load_level_config(self.level)
        self.override_config_by_rosparam()

        self.active_event = None
        self.next_event_start_time = None
        self.last_msg_time = None
        self.event_id = 0
        self.received_count = 0
        self.published_count = 0

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback, queue_size=10)

        rospy.loginfo("[scan_disturbance_injector] started: input_topic=%s, output_topic=%s, level=%s, always_active=%s, seed=%d",
                      self.input_topic, self.output_topic, self.level, str(self.always_active), self.random_seed)
        rospy.loginfo("[scan_disturbance_injector] sector=[%.1f, %.1f] deg, occlusion_mode=%s",
                      self.sector_min_deg, self.sector_max_deg, self.occlusion_mode)

    def load_level_config(self, level):
        """
        不同干扰等级的默认参数。
        你可以通过 rosrun 后面的 _参数名:=数值 单独覆盖。
        """
        configs = {
            "none": {
                "enabled": False,
                "event_duration_min_s": 999999.0,
                "event_duration_max_s": 999999.0,
                "event_gap_min_s": 999999.0,
                "event_gap_max_s": 999999.0,
                "cluster_count": 0,
                "cluster_sigma_deg": 0.0,
                "cluster_density": 0.0,
                "cluster_range_noise_m": 0.0,
                "band_count": 0,
                "band_width_deg": 0.0,
                "band_density": 0.0,
                "band_range_noise_m": 0.0,
                "occlusion_count": 0,
                "occlusion_ratio": 0.0,
                "occlusion_density": 0.0,
                "occlusion_range_noise_m": 0.0,
                "max_drift_deg_per_s": 0.0
            },
            "light": {
                "enabled": True,
                "event_duration_min_s": 2.0,
                "event_duration_max_s": 3.0,
                "event_gap_min_s": 4.0,
                "event_gap_max_s": 7.0,
                "cluster_count": 2,
                "cluster_sigma_deg": 1.8,
                "cluster_density": 0.28,
                "cluster_range_noise_m": 0.045,
                "band_count": 1,
                "band_width_deg": 5.0,
                "band_density": 0.12,
                "band_range_noise_m": 0.060,
                "occlusion_count": 1,
                "occlusion_ratio": 0.08,
                "occlusion_density": 0.50,
                "occlusion_range_noise_m": 0.030,
                "max_drift_deg_per_s": 0.20
            },
            "medium": {
                "enabled": True,
                "event_duration_min_s": 3.0,
                "event_duration_max_s": 6.0,
                "event_gap_min_s": 3.0,
                "event_gap_max_s": 5.0,
                "cluster_count": 4,
                "cluster_sigma_deg": 2.8,
                "cluster_density": 0.42,
                "cluster_range_noise_m": 0.070,
                "band_count": 2,
                "band_width_deg": 8.0,
                "band_density": 0.25,
                "band_range_noise_m": 0.090,
                "occlusion_count": 2,
                "occlusion_ratio": 0.22,
                "occlusion_density": 0.70,
                "occlusion_range_noise_m": 0.050,
                "max_drift_deg_per_s": 0.35
            },
            "heavy": {
                "enabled": True,
                "event_duration_min_s": 6.0,
                "event_duration_max_s": 10.0,
                "event_gap_min_s": 1.0,
                "event_gap_max_s": 3.0,
                "cluster_count": 7,
                "cluster_sigma_deg": 4.0,
                "cluster_density": 0.60,
                "cluster_range_noise_m": 0.110,
                "band_count": 3,
                "band_width_deg": 12.0,
                "band_density": 0.42,
                "band_range_noise_m": 0.140,
                "occlusion_count": 3,
                "occlusion_ratio": 0.45,
                "occlusion_density": 0.88,
                "occlusion_range_noise_m": 0.070,
                "max_drift_deg_per_s": 0.55
            }
        }

        if level not in configs:
            rospy.logwarn("[scan_disturbance_injector] unknown level=%s, use medium instead.", level)
            level = "medium"

        return copy.deepcopy(configs[level])

    def override_config_by_rosparam(self):
        """
        允许通过 rosparam 覆盖等级参数，便于消融实验和调参。
        例如：
        rosrun ... _level:=medium _cluster_count:=0
        就可以关闭团状沙尘，只测试带状散点和遮挡。
        """
        for key in list(self.cfg.keys()):
            private_name = "~" + key
            if rospy.has_param(private_name):
                value = rospy.get_param(private_name)
                if isinstance(self.cfg[key], bool):
                    self.cfg[key] = bool(value)
                elif isinstance(self.cfg[key], int):
                    self.cfg[key] = int(value)
                else:
                    self.cfg[key] = float(value)

        self.cfg["event_duration_min_s"] = max(0.1, float(self.cfg["event_duration_min_s"]))
        self.cfg["event_duration_max_s"] = max(self.cfg["event_duration_min_s"], float(self.cfg["event_duration_max_s"]))
        self.cfg["event_gap_min_s"] = max(0.0, float(self.cfg["event_gap_min_s"]))
        self.cfg["event_gap_max_s"] = max(self.cfg["event_gap_min_s"], float(self.cfg["event_gap_max_s"]))
        self.cfg["cluster_count"] = max(0, int(self.cfg["cluster_count"]))
        self.cfg["band_count"] = max(0, int(self.cfg["band_count"]))
        self.cfg["occlusion_count"] = max(0, int(self.cfg["occlusion_count"]))
        self.cfg["occlusion_ratio"] = min(0.95, max(0.0, float(self.cfg["occlusion_ratio"])))
        self.cfg["cluster_density"] = min(1.0, max(0.0, float(self.cfg["cluster_density"])))
        self.cfg["band_density"] = min(1.0, max(0.0, float(self.cfg["band_density"])))
        self.cfg["occlusion_density"] = min(1.0, max(0.0, float(self.cfg["occlusion_density"])))

    @staticmethod
    def deg2rad(deg):
        return deg * math.pi / 180.0

    @staticmethod
    def rad2deg(rad):
        return rad * 180.0 / math.pi

    def get_msg_time(self, msg):
        if msg.header.stamp is not None and msg.header.stamp.to_sec() > 0.0:
            return msg.header.stamp.to_sec()
        return rospy.Time.now().to_sec()

    def is_time_reset(self, now):
        if self.last_msg_time is None:
            return False
        return now + 0.5 < self.last_msg_time

    def uniform_sector_angle(self):
        a = min(self.sector_min_deg, self.sector_max_deg)
        b = max(self.sector_min_deg, self.sector_max_deg)
        return self.rng.uniform(a, b)

    def clip_angle_to_sector(self, angle_deg):
        a = min(self.sector_min_deg, self.sector_max_deg)
        b = max(self.sector_min_deg, self.sector_max_deg)
        return min(b, max(a, angle_deg))

    def create_new_event(self, now):
        self.event_id += 1

        duration = self.rng.uniform(self.cfg["event_duration_min_s"], self.cfg["event_duration_max_s"])
        sector_width = abs(self.sector_max_deg - self.sector_min_deg)

        event = {
            "id": self.event_id,
            "start": now,
            "end": now + duration,
            "clusters": [],
            "bands": [],
            "occlusions": []
        }

        for _ in range(self.cfg["cluster_count"]):
            c = {
                "angle_deg": self.uniform_sector_angle(),
                "range_m": self.rng.uniform(self.dust_range_min_m, self.dust_range_max_m),
                "sigma_deg": max(0.1, float(self.cfg["cluster_sigma_deg"]) * self.rng.uniform(0.75, 1.25)),
                "density": float(self.cfg["cluster_density"]) * self.rng.uniform(0.80, 1.20),
                "range_noise_m": float(self.cfg["cluster_range_noise_m"]),
                "drift_deg_per_s": self.rng.uniform(-float(self.cfg["max_drift_deg_per_s"]), float(self.cfg["max_drift_deg_per_s"]))
            }
            c["density"] = min(1.0, max(0.0, c["density"]))
            event["clusters"].append(c)

        for _ in range(self.cfg["band_count"]):
            width = max(0.5, float(self.cfg["band_width_deg"]) * self.rng.uniform(0.75, 1.25))
            center = self.uniform_sector_angle()
            b = {
                "center_deg": center,
                "width_deg": width,
                "range_min_m": self.rng.uniform(self.dust_range_min_m, max(self.dust_range_min_m + 0.1, self.dust_range_max_m * 0.65)),
                "range_max_m": self.rng.uniform(max(self.dust_range_min_m + 0.2, self.dust_range_max_m * 0.65), self.dust_range_max_m),
                "density": min(1.0, max(0.0, float(self.cfg["band_density"]) * self.rng.uniform(0.80, 1.20))),
                "range_noise_m": float(self.cfg["band_range_noise_m"]),
                "drift_deg_per_s": self.rng.uniform(-float(self.cfg["max_drift_deg_per_s"]), float(self.cfg["max_drift_deg_per_s"]))
            }
            if b["range_max_m"] < b["range_min_m"]:
                b["range_min_m"], b["range_max_m"] = b["range_max_m"], b["range_min_m"]
            event["bands"].append(b)

        if self.cfg["occlusion_count"] > 0 and self.cfg["occlusion_ratio"] > 0.0:
            total_occlusion_width = sector_width * float(self.cfg["occlusion_ratio"])
            each_width = max(1.0, total_occlusion_width / float(self.cfg["occlusion_count"]))

            for _ in range(self.cfg["occlusion_count"]):
                center = self.uniform_sector_angle()
                o = {
                    "center_deg": center,
                    "width_deg": each_width * self.rng.uniform(0.75, 1.25),
                    "range_m": self.rng.uniform(self.occlusion_range_min_m, self.occlusion_range_max_m),
                    "density": min(1.0, max(0.0, float(self.cfg["occlusion_density"]) * self.rng.uniform(0.85, 1.10))),
                    "range_noise_m": float(self.cfg["occlusion_range_noise_m"]),
                    "drift_deg_per_s": self.rng.uniform(-float(self.cfg["max_drift_deg_per_s"]), float(self.cfg["max_drift_deg_per_s"]))
                }
                event["occlusions"].append(o)

        rospy.loginfo("[scan_disturbance_injector] new event id=%d, level=%s, duration=%.2fs, clusters=%d, bands=%d, occlusions=%d",
                      event["id"], self.level, duration, len(event["clusters"]), len(event["bands"]), len(event["occlusions"]))

        return event

    def update_event_state(self, now):
        if not self.cfg["enabled"]:
            self.active_event = None
            self.next_event_start_time = None
            return None

        if self.always_active:
            if self.active_event is None or now > self.active_event["end"]:
                self.active_event = self.create_new_event(now)
            return self.active_event

        if self.active_event is not None:
            if now <= self.active_event["end"]:
                return self.active_event

            gap = self.rng.uniform(self.cfg["event_gap_min_s"], self.cfg["event_gap_max_s"])
            self.next_event_start_time = now + gap
            rospy.loginfo("[scan_disturbance_injector] event id=%d ended, next event after %.2fs",
                          self.active_event["id"], gap)
            self.active_event = None
            return None

        if self.next_event_start_time is None:
            if self.initial_event_immediate:
                self.next_event_start_time = now
            else:
                self.next_event_start_time = now + self.rng.uniform(self.cfg["event_gap_min_s"], self.cfg["event_gap_max_s"])

        if now >= self.next_event_start_time:
            self.active_event = self.create_new_event(now)
            return self.active_event

        return None

    def angle_to_index(self, scan_msg, angle_deg):
        angle_rad = self.deg2rad(angle_deg)
        if abs(scan_msg.angle_increment) < 1e-12:
            return None
        idx = int(round((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        if idx < 0 or idx >= len(scan_msg.ranges):
            return None
        return idx

    def index_to_angle_deg(self, scan_msg, idx):
        return self.rad2deg(scan_msg.angle_min + float(idx) * scan_msg.angle_increment)

    def get_indices_in_angle_interval(self, scan_msg, center_deg, half_width_deg):
        low = center_deg - half_width_deg
        high = center_deg + half_width_deg

        indices = []
        n = len(scan_msg.ranges)
        for i in range(n):
            a = self.index_to_angle_deg(scan_msg, i)
            if low <= a <= high:
                indices.append(i)
        return indices

    def valid_range_value(self, scan_msg, r):
        if r is None:
            return False
        if not math.isfinite(r):
            return False

        r_min = scan_msg.range_min if scan_msg.range_min > 0.0 else 0.01
        r_max = scan_msg.range_max if scan_msg.range_max > r_min else 100.0
        return r_min <= r <= r_max

    def clamp_range(self, scan_msg, r):
        if not self.clamp_to_scan_range:
            return r

        r_min = scan_msg.range_min if scan_msg.range_min > 0.0 else 0.01
        r_max = scan_msg.range_max if scan_msg.range_max > r_min else 100.0

        if not math.isfinite(r):
            return r
        return min(r_max, max(r_min, r))

    def write_range(self, scan_msg, ranges, idx, new_r):
        if idx < 0 or idx >= len(ranges):
            return

        if math.isfinite(new_r):
            new_r = self.clamp_range(scan_msg, new_r)

        old_r = ranges[idx]

        if not self.use_nearest_return:
            ranges[idx] = new_r
            return

        if not self.valid_range_value(scan_msg, old_r):
            ranges[idx] = new_r
            return

        if not math.isfinite(new_r):
            ranges[idx] = new_r
            return

        if new_r < old_r:
            ranges[idx] = new_r

    def apply_cluster(self, scan_msg, ranges, cluster, now):
        dt = max(0.0, now - self.active_event["start"])
        center_deg = self.clip_angle_to_sector(cluster["angle_deg"] + cluster["drift_deg_per_s"] * dt)
        sigma_deg = max(0.1, cluster["sigma_deg"])
        half_width = 3.0 * sigma_deg
        indices = self.get_indices_in_angle_interval(scan_msg, center_deg, half_width)

        for idx in indices:
            a = self.index_to_angle_deg(scan_msg, idx)
            da = a - center_deg
            gaussian = math.exp(-0.5 * (da / sigma_deg) ** 2)
            p = cluster["density"] * gaussian

            if self.rng.random() > p:
                continue

            noise = self.rng.gauss(0.0, cluster["range_noise_m"])
            new_r = cluster["range_m"] + noise
            self.write_range(scan_msg, ranges, idx, new_r)

    def apply_band(self, scan_msg, ranges, band, now):
        dt = max(0.0, now - self.active_event["start"])
        center_deg = self.clip_angle_to_sector(band["center_deg"] + band["drift_deg_per_s"] * dt)
        half_width = max(0.1, 0.5 * band["width_deg"])
        indices = self.get_indices_in_angle_interval(scan_msg, center_deg, half_width)

        for idx in indices:
            if self.rng.random() > band["density"]:
                continue

            base_r = self.rng.uniform(band["range_min_m"], band["range_max_m"])
            noise = self.rng.gauss(0.0, band["range_noise_m"])
            new_r = base_r + noise
            self.write_range(scan_msg, ranges, idx, new_r)

    def apply_occlusion(self, scan_msg, ranges, occlusion, now):
        dt = max(0.0, now - self.active_event["start"])
        center_deg = self.clip_angle_to_sector(occlusion["center_deg"] + occlusion["drift_deg_per_s"] * dt)
        half_width = max(0.1, 0.5 * occlusion["width_deg"])
        indices = self.get_indices_in_angle_interval(scan_msg, center_deg, half_width)

        for idx in indices:
            if self.rng.random() > occlusion["density"]:
                continue

            a = self.index_to_angle_deg(scan_msg, idx)
            normalized = 0.0
            if half_width > 1e-6:
                normalized = (a - center_deg) / half_width

            if self.occlusion_mode == "nan":
                self.write_range(scan_msg, ranges, idx, float("nan"))

            elif self.occlusion_mode == "inf":
                self.write_range(scan_msg, ranges, idx, float("inf"))

            elif self.occlusion_mode == "max_range":
                self.write_range(scan_msg, ranges, idx, scan_msg.range_max)

            else:
                # cow_body：构造一段平滑凸起轮廓，模拟牛体、工具、障碍物遮挡围栏。
                curvature = 0.18 * (normalized ** 2)
                ripple = 0.025 * math.sin(5.0 * normalized * math.pi)
                noise = self.rng.gauss(0.0, occlusion["range_noise_m"])
                new_r = occlusion["range_m"] + curvature + ripple + noise
                self.write_range(scan_msg, ranges, idx, new_r)

    def apply_event(self, scan_msg, event, now):
        out = copy.deepcopy(scan_msg)
        ranges = list(out.ranges)

        for cluster in event["clusters"]:
            self.apply_cluster(out, ranges, cluster, now)

        for band in event["bands"]:
            self.apply_band(out, ranges, band, now)

        for occlusion in event["occlusions"]:
            self.apply_occlusion(out, ranges, occlusion, now)

        out.ranges = ranges
        return out

    def print_status_throttled(self, now, event):
        if self.print_hz <= 0.0:
            return

        period = 1.0 / self.print_hz
        if now - self.last_print_time < period:
            return

        self.last_print_time = now

        if event is None:
            next_info = -1.0
            if self.next_event_start_time is not None:
                next_info = max(0.0, self.next_event_start_time - now)

            rospy.loginfo("[scan_disturbance_injector] level=%s, inactive, next_event_in=%.2fs, received=%d, published=%d",
                          self.level, next_info, self.received_count, self.published_count)
        else:
            remain = max(0.0, event["end"] - now)
            rospy.loginfo("[scan_disturbance_injector] level=%s, active_event=%d, remain=%.2fs, received=%d, published=%d",
                          self.level, event["id"], remain, self.received_count, self.published_count)

    def scan_callback(self, msg):
        self.received_count += 1
        now = self.get_msg_time(msg)

        if self.is_time_reset(now):
            rospy.logwarn("[scan_disturbance_injector] time moved backward, reset event state.")
            self.active_event = None
            self.next_event_start_time = None
            self.event_id = 0
            self.rng = random.Random(self.random_seed)

        self.last_msg_time = now

        event = self.update_event_state(now)

        if event is None:
            out = copy.deepcopy(msg)
        else:
            out = self.apply_event(msg, event, now)

        self.pub.publish(out)
        self.published_count += 1
        self.print_status_throttled(now, event)


def main():
    rospy.init_node("scan_disturbance_injector", anonymous=False)
    ScanDisturbanceInjector()
    rospy.spin()


if __name__ == "__main__":
    main()