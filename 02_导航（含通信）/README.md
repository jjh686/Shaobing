# 当前入口：固定云台，实时建图与自主探索

新测试步骤见 [实时建图与自主探索测试](docs/实时建图与自主探索测试-20261121.md)，安装数据填写方式见 [雷达安装测量表](docs/雷达安装测量表-20261121.md)。

```bash
bash scripts/run_live_exploration.sh build
bash scripts/run_live_exploration.sh mapping rviz:=true
```

`mapping` 不打开底盘 USB。自动探索使用 `navigation PORT`，但当前 `local/live_mounting.yaml` 缺少已确认的安装数据，入口会拒绝启动导航。不要仅把确认标志改为 true；先完成实测。旧 `measured_mounting.yaml` 的水平安装、20 cm 高度已不适用，已撤销其确认状态。

本轮上位机改动不需要重新生成或烧录 C 板固件。固件仍按根目录 AGENTS.md 归属：云台在 `05_云台/`，底盘在 `04_底盘电控（含通信）/chassis_driver/basic_framework/`。
