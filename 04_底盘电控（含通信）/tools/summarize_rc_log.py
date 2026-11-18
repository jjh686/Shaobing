#!/usr/bin/env python3
"""Summarize saved STOP logs only; never open USB or access an MCU."""
import argparse
from collections import Counter
import json
from pathlib import Path


def summarize(path):
    samples = []
    for line in path.read_text().splitlines():
        try:
            sample = json.loads(line)
        except ValueError:
            continue
        if isinstance(sample, dict) and all(type(sample.get(k)) is int
                                            for k in ('uptime', 'status', 'fault')):
            samples.append(sample)
    if not samples:
        raise ValueError('未找到底盘反馈JSON；请检查同名日志中的设备或通信错误')
    counts = Counter(s['fault'] for s in samples)
    permitted = [not (s['fault'] & 2 or s['status'] & (4 | 32)) for s in samples]
    transitions = [(s['uptime'], s['fault']) for i, s in enumerate(samples)
                   if i == 0 or s['fault'] != samples[i-1]['fault']]
    active = sum(bool(s['status'] & 2) for s in samples)
    max_gap = max(((b['uptime']-a['uptime']) & 0xffffffff
                   for a, b in zip(samples, samples[1:])), default=0)
    duration = sum((b['uptime']-a['uptime']) & 0xffffffff
                   for a, b in zip(samples, samples[1:])) / 1000
    print(f'日志：{path}')
    print(f'底盘反馈：{len(samples)}帧；反馈覆盖：{duration:.2f}秒；最大相邻反馈间隔：{max_gap}ms')
    print('fault统计：' + ', '.join(f'{fault}: {n}帧' for fault, n in sorted(counts.items())))
    print(f'遥控/导航未许可（fault含2）：{sum(bool(s["fault"] & 2) for s in samples)}帧')
    print(f'四电机未全在线（fault含4）：{sum(bool(s["fault"] & 4) for s in samples)}帧')
    print(f'遥控许可状态：{sum(permitted)}/{len(samples)}帧；这不等于电机允许使能')
    print(f'ACTIVE帧：{active}；STOP测试应为0')
    print('fault变化（底盘上电毫秒:值，最多20项）：' + ', '.join(
        f'{uptime}:{fault}' for uptime, fault in transitions[:20]))
    if len(transitions) > 20:
        print(f'另有{len(transitions)-20}次变化，见原始日志。')
    print('结论仅针对已保存的反馈；同时核对zero_test报告中的passed和新鲜ACK统计。')
    print('源端故障代数、SBUS错误类别和恢复原因需另读云台/底盘变量；本工具不证明未采样期间无故障。')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', type=Path, nargs='+')
    args = parser.parse_args()
    try:
        for path in args.logs:
            summarize(path)
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc))
