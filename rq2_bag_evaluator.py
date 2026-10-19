#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import glob
import math
import os
import re
import statistics
import sys

try:
    import rosbag
except Exception as e:
    print("[ERROR] 无法导入 rosbag。请确认已经 source ROS 环境：source /opt/ros/noetic/setup.bash")
    print("[ERROR] 详细错误：{}".format(e))
    sys.exit(1)


def is_finite_number(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(float(x))


def safe_float(x, default=float("nan")):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
        return default
    except Exception:
        return default


def mean_or_nan(values):
    valid = [float(v) for v in values if is_finite_number(v)]
    if len(valid) == 0:
        return float("nan")
    return sum(valid) / float(len(valid))


def std_or_nan(values):
    valid = [float(v) for v in values if is_finite_number(v)]
    if len(valid) <= 1:
        return 0.0 if len(valid) == 1 else float("nan")
    return statistics.stdev(valid)


def format_number(x, digits=4):
    if not is_finite_number(x):
        return ""
    return ("{:.%df}" % digits).format(float(x))


def infer_level_from_filename(path):
    name = os.path.basename(path).lower()

    level_patterns = [
        ("none", ["none", "no", "clean", "无干扰"]),
        ("light", ["light", "mild", "轻度"]),
        ("medium", ["medium", "mid", "moderate", "中度"]),
        ("heavy", ["heavy", "severe", "严重", "重度"])
    ]

    for level, keys in level_patterns:
        for k in keys:
            if k in name:
                return level

    m = re.search(r"rq2[_\-]?([a-zA-Z]+)", name)
    if m:
        return m.group(1).lower()

    return "unknown"


def expand_bag_patterns(patterns):
    bags = []
    for p in patterns:
        matches = sorted(glob.glob(p))
        if len(matches) > 0:
            bags.extend(matches)
        else:
            bags.append(p)

    unique = []
    seen = set()
    for b in bags:
        ab = os.path.abspath(b)
        if ab not in seen:
            unique.append(ab)
            seen.add(ab)

    return unique


class RQ2BagEvaluator(object):
    def __init__(self, args):
        self.args = args

    def get_bag_window(self, bag):
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()

        if self.args.start_offset is None:
            eval_start = bag_start
        else:
            eval_start = bag_start + float(self.args.start_offset)

        if self.args.end_offset is None:
            eval_end = bag_end
        else:
            eval_end = bag_start + float(self.args.end_offset)

        eval_start = max(bag_start, eval_start)
        eval_end = min(bag_end, eval_end)

        if eval_end <= eval_start:
            raise RuntimeError("评价时间窗口无效：eval_start={:.3f}, eval_end={:.3f}".format(eval_start, eval_end))

        return bag_start, bag_end, eval_start, eval_end

    def get_topic_set(self, bag):
        topic_info = bag.get_type_and_topic_info()[1]
        return set(topic_info.keys())

    def msg_data_to_int(self, msg, default=None):
        if hasattr(msg, "data"):
            try:
                return int(msg.data)
            except Exception:
                return default
        return default

    def get_cmd_values(self, msg):
        vx = 0.0
        wz = 0.0

        try:
            vx = float(msg.linear.x)
        except Exception:
            vx = 0.0

        try:
            wz = float(msg.angular.z)
        except Exception:
            wz = 0.0

        return vx, wz

    def is_path_valid(self, msg):
        if not hasattr(msg, "poses"):
            return False
        try:
            return len(msg.poses) >= int(self.args.min_path_poses)
        except Exception:
            return False

    def should_count_stop(self, active, current_status):
        if not active:
            return False

        if current_status == int(self.args.paused_status):
            return False

        return True

    def evaluate_one_bag(self, bag_path):
        level = self.args.level
        if level == "auto":
            level = infer_level_from_filename(bag_path)

        result = {
            "bag": os.path.basename(bag_path),
            "bag_path": os.path.abspath(bag_path),
            "level": level,
            "run_id": self.extract_run_id(bag_path),
            "bag_duration_s": 0.0,
            "eval_start_offset_s": 0.0,
            "eval_end_offset_s": 0.0,
            "eval_duration_s": 0.0,
            "active_duration_s": 0.0,
            "total_path_frames": 0,
            "valid_path_frames": 0,
            "path_valid_rate_percent": float("nan"),
            "lost_count": 0,
            "false_stop_count": 0,
            "reacquire_event_count": 0,
            "mean_reacquire_time_s": float("nan"),
            "max_reacquire_time_s": float("nan"),
            "recover_failed_count": 0,
            "status_msg_count": 0,
            "cmd_msg_count": 0,
            "path_msg_count": 0,
            "start_msg_count": 0,
            "feedback_msg_count": 0,
            "warning": ""
        }

        event_rows = []

        if not os.path.exists(bag_path):
            result["warning"] = "bag 文件不存在"
            return result, event_rows

        needed_topics = [
            self.args.path_topic,
            self.args.status_topic,
            self.args.cmd_topic,
            self.args.start_topic,
            self.args.feedback_topic
        ]

        bag = None
        try:
            bag = rosbag.Bag(bag_path, "r")
            bag_start, bag_end, eval_start, eval_end = self.get_bag_window(bag)
            topic_set = self.get_topic_set(bag)

            result["bag_duration_s"] = bag_end - bag_start
            result["eval_start_offset_s"] = eval_start - bag_start
            result["eval_end_offset_s"] = eval_end - bag_start
            result["eval_duration_s"] = eval_end - eval_start

            has_active_topic = (self.args.start_topic in topic_set) or (self.args.feedback_topic in topic_set)

            if self.args.force_full_bag or not has_active_topic:
                active = True
            else:
                active = False

            start_cmd_value = None
            feedback_value = None
            current_status = None
            previous_status = None

            last_msg_time = None

            last_path_valid = False
            last_path_time = None

            last_lost_count_time = -1e30
            open_lost_event = None
            event_id = 0

            recover_candidate_start = None

            stop_segment_active = False
            stop_segment_start = None

            reacquire_times = []
            recovered_events = 0
            failed_recover_events = 0

            def recompute_active():
                if self.args.force_full_bag or not has_active_topic:
                    return True
                a = False
                if start_cmd_value == 1:
                    a = True
                if feedback_value == 1:
                    a = True
                return a

            def close_stop_segment(close_time):
                nonlocal stop_segment_active, stop_segment_start
                if not stop_segment_active:
                    return 0

                duration = close_time - stop_segment_start
                count = 1 if duration >= float(self.args.stop_min_duration) else 0

                stop_segment_active = False
                stop_segment_start = None
                return count

            def try_update_recovery(t_now):
                nonlocal open_lost_event, recover_candidate_start, recovered_events, failed_recover_events

                if open_lost_event is None:
                    recover_candidate_start = None
                    return

                if not active:
                    recover_candidate_start = None
                    return

                stable_now = (current_status == int(self.args.running_status)) and last_path_valid

                if stable_now:
                    if recover_candidate_start is None:
                        recover_candidate_start = t_now

                    if t_now - recover_candidate_start >= float(self.args.recover_stable_sec):
                        recover_time = recover_candidate_start
                        reacquire_time = recover_time - open_lost_event["lost_start_time"]

                        if reacquire_time >= 0.0:
                            open_lost_event["recover_time"] = recover_time
                            open_lost_event["recover_confirm_time"] = t_now
                            open_lost_event["reacquire_time_s"] = reacquire_time
                            open_lost_event["recovered"] = 1
                            reacquire_times.append(reacquire_time)
                            recovered_events += 1

                            event_rows.append({
                                "bag": result["bag"],
                                "level": result["level"],
                                "run_id": result["run_id"],
                                "event_id": open_lost_event["event_id"],
                                "lost_start_offset_s": open_lost_event["lost_start_time"] - bag_start,
                                "recover_start_offset_s": recover_time - bag_start,
                                "recover_confirm_offset_s": t_now - bag_start,
                                "reacquire_time_s": reacquire_time,
                                "recovered": 1
                            })

                        open_lost_event = None
                        recover_candidate_start = None
                else:
                    recover_candidate_start = None

            for topic, msg, stamp in bag.read_messages(topics=needed_topics):
                t = stamp.to_sec()

                if last_msg_time is not None:
                    seg_start = max(last_msg_time, eval_start)
                    seg_end = min(t, eval_end)
                    if seg_end > seg_start and active:
                        result["active_duration_s"] += seg_end - seg_start

                in_window = (t >= eval_start) and (t <= eval_end)

                if topic == self.args.start_topic:
                    result["start_msg_count"] += 1
                    start_cmd_value = self.msg_data_to_int(msg, default=start_cmd_value)
                    active = recompute_active()

                elif topic == self.args.feedback_topic:
                    result["feedback_msg_count"] += 1
                    feedback_value = self.msg_data_to_int(msg, default=feedback_value)
                    active = recompute_active()

                elif topic == self.args.status_topic:
                    result["status_msg_count"] += 1
                    previous_status = current_status
                    current_status = self.msg_data_to_int(msg, default=current_status)

                    if in_window and active:
                        if current_status == int(self.args.lost_status) and previous_status != int(self.args.lost_status):
                            if t - last_lost_count_time >= float(self.args.lost_merge_gap):
                                result["lost_count"] += 1
                                last_lost_count_time = t
                                event_id += 1

                                if open_lost_event is not None:
                                    open_lost_event["recovered"] = 0
                                    open_lost_event["reacquire_time_s"] = float("nan")
                                    failed_recover_events += 1
                                    event_rows.append({
                                        "bag": result["bag"],
                                        "level": result["level"],
                                        "run_id": result["run_id"],
                                        "event_id": open_lost_event["event_id"],
                                        "lost_start_offset_s": open_lost_event["lost_start_time"] - bag_start,
                                        "recover_start_offset_s": "",
                                        "recover_confirm_offset_s": "",
                                        "reacquire_time_s": "",
                                        "recovered": 0
                                    })

                                open_lost_event = {
                                    "event_id": event_id,
                                    "lost_start_time": t,
                                    "recover_time": None,
                                    "recover_confirm_time": None,
                                    "reacquire_time_s": float("nan"),
                                    "recovered": 0
                                }
                                recover_candidate_start = None

                    if in_window:
                        try_update_recovery(t)

                elif topic == self.args.path_topic:
                    result["path_msg_count"] += 1
                    path_valid = self.is_path_valid(msg)
                    last_path_valid = path_valid
                    last_path_time = t

                    if in_window and active:
                        result["total_path_frames"] += 1
                        if path_valid:
                            result["valid_path_frames"] += 1

                    if in_window:
                        try_update_recovery(t)

                elif topic == self.args.cmd_topic:
                    result["cmd_msg_count"] += 1
                    vx, wz = self.get_cmd_values(msg)
                    is_stop = (abs(vx) < float(self.args.stop_v_thresh)) and (abs(wz) < float(self.args.stop_w_thresh))
                    allow_stop_count = self.should_count_stop(active, current_status)

                    if in_window and allow_stop_count and is_stop:
                        if not stop_segment_active:
                            stop_segment_active = True
                            stop_segment_start = t
                    else:
                        if in_window:
                            result["false_stop_count"] += close_stop_segment(t)

                    if in_window:
                        try_update_recovery(t)

                last_msg_time = t

            if last_msg_time is not None:
                seg_start = max(last_msg_time, eval_start)
                seg_end = eval_end
                if seg_end > seg_start and active:
                    result["active_duration_s"] += seg_end - seg_start

            result["false_stop_count"] += close_stop_segment(eval_end)

            if open_lost_event is not None:
                open_lost_event["recovered"] = 0
                open_lost_event["reacquire_time_s"] = float("nan")
                failed_recover_events += 1
                event_rows.append({
                    "bag": result["bag"],
                    "level": result["level"],
                    "run_id": result["run_id"],
                    "event_id": open_lost_event["event_id"],
                    "lost_start_offset_s": open_lost_event["lost_start_time"] - bag_start,
                    "recover_start_offset_s": "",
                    "recover_confirm_offset_s": "",
                    "reacquire_time_s": "",
                    "recovered": 0
                })

            if result["total_path_frames"] > 0:
                result["path_valid_rate_percent"] = 100.0 * float(result["valid_path_frames"]) / float(result["total_path_frames"])

            result["reacquire_event_count"] = len(reacquire_times)
            result["recover_failed_count"] = failed_recover_events

            if result["lost_count"] == 0:
                result["mean_reacquire_time_s"] = 0.0
                result["max_reacquire_time_s"] = 0.0
            elif len(reacquire_times) > 0:
                result["mean_reacquire_time_s"] = mean_or_nan(reacquire_times)
                result["max_reacquire_time_s"] = max(reacquire_times)
            else:
                result["mean_reacquire_time_s"] = float("nan")
                result["max_reacquire_time_s"] = float("nan")

            missing_topics = []
            for tp in needed_topics:
                if tp not in topic_set:
                    missing_topics.append(tp)

            if len(missing_topics) > 0:
                result["warning"] = "缺少话题：" + ";".join(missing_topics)

            if result["total_path_frames"] == 0:
                if result["warning"]:
                    result["warning"] += "；"
                result["warning"] += "有效评价窗口内没有 path 帧，检查 /start_capture 是否为 1，或使用 --force-full-bag"

        except Exception as e:
            result["warning"] = "评价失败：{}".format(e)

        finally:
            if bag is not None:
                bag.close()

        return result, event_rows

    def extract_run_id(self, bag_path):
        name = os.path.basename(bag_path)
        m = re.search(r"(\d+)(?=\.bag$)", name)
        if m:
            return m.group(1)
        return ""


def write_csv(path, rows, fieldnames):
    folder = os.path.dirname(os.path.abspath(path))
    if folder and not os.path.exists(folder):
        os.makedirs(folder)

    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            out = {}
            for k in fieldnames:
                v = r.get(k, "")
                if isinstance(v, float):
                    out[k] = format_number(v, 6)
                else:
                    out[k] = v
            writer.writerow(out)


def make_summary(rows):
    levels = []
    for r in rows:
        lv = r.get("level", "unknown")
        if lv not in levels:
            levels.append(lv)

    summary_rows = []
    for lv in levels:
        group = [r for r in rows if r.get("level", "unknown") == lv]

        path_rates = [r.get("path_valid_rate_percent", float("nan")) for r in group]
        lost_counts = [r.get("lost_count", float("nan")) for r in group]
        false_stops = [r.get("false_stop_count", float("nan")) for r in group]
        reacquire_times = [r.get("mean_reacquire_time_s", float("nan")) for r in group]
        failed_counts = [r.get("recover_failed_count", 0) for r in group]

        summary_rows.append({
            "level": lv,
            "n": len(group),
            "path_valid_rate_mean_percent": mean_or_nan(path_rates),
            "path_valid_rate_std_percent": std_or_nan(path_rates),
            "lost_count_mean": mean_or_nan(lost_counts),
            "lost_count_std": std_or_nan(lost_counts),
            "false_stop_count_mean": mean_or_nan(false_stops),
            "false_stop_count_std": std_or_nan(false_stops),
            "mean_reacquire_time_mean_s": mean_or_nan(reacquire_times),
            "mean_reacquire_time_std_s": std_or_nan(reacquire_times),
            "recover_failed_count_sum": sum([int(x) for x in failed_counts if is_finite_number(x)])
        })

    return summary_rows


def print_result_table(rows):
    print("")
    print("===== RQ2 单个 bag 评价结果 =====")
    header = "{:<22s} {:<8s} {:>12s} {:>10s} {:>10s} {:>14s} {:>10s}".format(
        "bag", "level", "path_eff/%", "lost", "falseStop", "reacq/s", "failed"
    )
    print(header)
    print("-" * len(header))

    for r in rows:
        line = "{:<22s} {:<8s} {:>12s} {:>10d} {:>10d} {:>14s} {:>10d}".format(
            str(r.get("bag", ""))[:22],
            str(r.get("level", ""))[:8],
            format_number(r.get("path_valid_rate_percent", float("nan")), 3),
            int(r.get("lost_count", 0)),
            int(r.get("false_stop_count", 0)),
            format_number(r.get("mean_reacquire_time_s", float("nan")), 3),
            int(r.get("recover_failed_count", 0))
        )
        print(line)

    print("")
    print("===== 按干扰等级汇总结果 =====")
    summary = make_summary(rows)
    header2 = "{:<8s} {:>4s} {:>18s} {:>14s} {:>16s} {:>18s} {:>14s}".format(
        "level", "n", "path_mean/%", "lost_mean", "falseStop_mean", "reacq_mean/s", "failed_sum"
    )
    print(header2)
    print("-" * len(header2))

    for r in summary:
        line = "{:<8s} {:>4d} {:>18s} {:>14s} {:>16s} {:>18s} {:>14d}".format(
            str(r.get("level", ""))[:8],
            int(r.get("n", 0)),
            format_number(r.get("path_valid_rate_mean_percent", float("nan")), 3),
            format_number(r.get("lost_count_mean", float("nan")), 3),
            format_number(r.get("false_stop_count_mean", float("nan")), 3),
            format_number(r.get("mean_reacquire_time_mean_s", float("nan")), 3),
            int(r.get("recover_failed_count_sum", 0))
        )
        print(line)


def parse_args():
    parser = argparse.ArgumentParser(description="RQ2 沙尘与遮挡鲁棒性 rosbag 离线评价脚本")

    parser.add_argument("--bags", nargs="+", required=True, help="输入 bag 文件，可使用通配符，例如 /home/m/bags/rq2_light_*.bag")
    parser.add_argument("--output", default="/home/m/bags/rq2_eval_results.csv", help="单个 bag 评价结果 CSV 输出路径")
    parser.add_argument("--summary-output", default="", help="按干扰等级汇总 CSV 输出路径，默认自动生成")
    parser.add_argument("--events-output", default="", help="丢失-重捕获事件明细 CSV 输出路径，默认自动生成")
    parser.add_argument("--level", default="auto", help="干扰等级。auto 表示从文件名推断，可选 none/light/medium/heavy")

    parser.add_argument("--path-topic", default="/corridor_parallel_right_selector/right_local_path", help="路径话题 nav_msgs/Path")
    parser.add_argument("--status-topic", default="/fence_follow/status", help="围栏状态话题 std_msgs/Int32")
    parser.add_argument("--cmd-topic", default="/cmd_vel", help="速度话题 geometry_msgs/Twist")
    parser.add_argument("--start-topic", default="/start_capture", help="围栏捕捉开始话题")
    parser.add_argument("--feedback-topic", default="/start_capture_feedback", help="围栏捕捉反馈话题")

    parser.add_argument("--min-path-poses", type=int, default=2, help="Path.poses 数量大于等于该值时认为路径有效")
    parser.add_argument("--running-status", type=int, default=1, help="围栏正常运行状态值")
    parser.add_argument("--paused-status", type=int, default=2, help="暂停状态值")
    parser.add_argument("--lost-status", type=int, default=3, help="围栏丢失状态值")
    parser.add_argument("--failed-status", type=int, default=4, help="失败状态值")
    parser.add_argument("--lost-merge-gap", type=float, default=1.0, help="两次 lost 间隔小于该值时合并为一次，单位 s")
    parser.add_argument("--recover-stable-sec", type=float, default=0.5, help="重新检测到有效路径并保持该时长后认为重捕获成功，单位 s")

    parser.add_argument("--stop-v-thresh", type=float, default=0.03, help="线速度绝对值小于该值认为接近停车，单位 m/s")
    parser.add_argument("--stop-w-thresh", type=float, default=0.03, help="角速度绝对值小于该值认为接近停车，单位 rad/s")
    parser.add_argument("--stop-min-duration", type=float, default=0.5, help="连续停车超过该时长才计为一次误停车，单位 s")

    parser.add_argument("--start-offset", type=float, default=None, help="评价窗口起点，相对 bag 起点，单位 s。例如 60")
    parser.add_argument("--end-offset", type=float, default=None, help="评价窗口终点，相对 bag 起点，单位 s。例如 200")
    parser.add_argument("--force-full-bag", action="store_true", help="忽略 /start_capture 和 /start_capture_feedback，全 bag 都参与统计")

    return parser.parse_args()


def main():
    args = parse_args()

    bags = expand_bag_patterns(args.bags)
    if len(bags) == 0:
        print("[ERROR] 没有找到任何 bag 文件")
        sys.exit(1)

    evaluator = RQ2BagEvaluator(args)

    rows = []
    event_rows = []

    for bag_path in bags:
        print("[INFO] evaluating bag: {}".format(bag_path))
        result, events = evaluator.evaluate_one_bag(bag_path)
        rows.append(result)
        event_rows.extend(events)

    output = os.path.abspath(args.output)

    if args.summary_output:
        summary_output = os.path.abspath(args.summary_output)
    else:
        root, ext = os.path.splitext(output)
        summary_output = root + "_summary.csv"

    if args.events_output:
        events_output = os.path.abspath(args.events_output)
    else:
        root, ext = os.path.splitext(output)
        events_output = root + "_events.csv"

    result_fields = [
        "bag",
        "bag_path",
        "level",
        "run_id",
        "bag_duration_s",
        "eval_start_offset_s",
        "eval_end_offset_s",
        "eval_duration_s",
        "active_duration_s",
        "total_path_frames",
        "valid_path_frames",
        "path_valid_rate_percent",
        "lost_count",
        "false_stop_count",
        "reacquire_event_count",
        "mean_reacquire_time_s",
        "max_reacquire_time_s",
        "recover_failed_count",
        "status_msg_count",
        "cmd_msg_count",
        "path_msg_count",
        "start_msg_count",
        "feedback_msg_count",
        "warning"
    ]

    summary_fields = [
        "level",
        "n",
        "path_valid_rate_mean_percent",
        "path_valid_rate_std_percent",
        "lost_count_mean",
        "lost_count_std",
        "false_stop_count_mean",
        "false_stop_count_std",
        "mean_reacquire_time_mean_s",
        "mean_reacquire_time_std_s",
        "recover_failed_count_sum"
    ]

    event_fields = [
        "bag",
        "level",
        "run_id",
        "event_id",
        "lost_start_offset_s",
        "recover_start_offset_s",
        "recover_confirm_offset_s",
        "reacquire_time_s",
        "recovered"
    ]

    write_csv(output, rows, result_fields)
    write_csv(summary_output, make_summary(rows), summary_fields)
    write_csv(events_output, event_rows, event_fields)

    print_result_table(rows)
    print("")
    print("[DONE] 单个 bag 评价结果：{}".format(output))
    print("[DONE] 干扰等级汇总结果：{}".format(summary_output))
    print("[DONE] 丢失-重捕获事件明细：{}".format(events_output))


if __name__ == "__main__":
    main()