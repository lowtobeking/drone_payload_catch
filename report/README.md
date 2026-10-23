# 实验记录与本地归档

正式源码在 payload_catch、config、launch、deploy、tools；硬件说明和验证结论在 docs 及项目根目录。report 保留历次实验、部署与排错证据。

## Git 保存范围

- 保留已有跟踪文件、根目录报告、figures，以及每个实验目录顶层的说明、复现脚本、配置与 JSON 结果。
- 含现场 Wi-Fi 凭据的历史联网辅助脚本仅留本地，不加入 Git。
- 原始采样、进程输出、参数读回子目录、构建副本、部署包、diagnostic_helpers 等深层内容保留本地，按项目 .gitignore 排除。
- workfiles 是从 Windows 用户目录迁回的历史资料，workspace_cleanup_20261023 保存本次整理清单、源码快照和核验日志，均保留本地。
- 运行中的 SSH 会话登记和 capture_state 不进入版本控制；它们不是下次连接或采集的有效凭据。

[ARTIFACTS_20261023.json](ARTIFACTS_20261023.json) 保存整理前实验文件的相对路径、大小、SHA256 与归档类型。Git 中的索引不能代替原始文件备份；重新克隆仓库不会获得 local_archive 文件。后续复制工程时应同时保留本地归档。

## 2026-10-23 最后状态

动捕 XY/航向和 XYZ/航向静态融合已验证。人工抬高仅完成基准，动态验证尚未完成。测试参数已恢复，当前没有常驻动捕融合，不代表允许飞行。详见 docs 当日报告与 session_close_20261023 的继续入口。

## 原有实验目录

下表按整理前 Git 未跟踪文件统计，不包含原本已忽略的构建文件与缓存；大小单位 MiB。

| 目录 | 文件数 | 大小 |
| --- | ---: | ---: |
| [a_deployment_20261015](a_deployment_20261015/) | 10 | 1.35 |
| [a_full_mission_20261016](a_full_mission_20261016/) | 12 | 0.06 |
| [a_ground_debug_20261016](a_ground_debug_20261016/) | 26 | 0.07 |
| [a_ground_entry_20261016](a_ground_entry_20261016/) | 19 | 0.08 |
| [a_onboard_sim_20261016](a_onboard_sim_20261016/) | 60 | 2.67 |
| [a_solo_deployment_20261019](a_solo_deployment_20261019/) | 40 | 0.90 |
| [a_solo_gazebo_20261019](a_solo_gazebo_20261019/) | 275 | 17.32 |
| [a_solo_validation_20261019](a_solo_validation_20261019/) | 69 | 0.23 |
| [a_task_readiness_20261016](a_task_readiness_20261016/) | 23 | 0.12 |
| [ab_optimization_20261016](ab_optimization_20261016/) | 8 | 0.04 |
| [direct_release_20261015](direct_release_20261015/) | 6 | 0.03 |
| [fixed_reference_20261023](fixed_reference_20261023/) | 486 | 45.08 |
| [fleet_integration_20261015](fleet_integration_20261015/) | 4 | 0.35 |
| [humble_migration_20261014](humble_migration_20261014/) | 37 | 0.36 |
| [jetson_ownership_20261016](jetson_ownership_20261016/) | 107 | 1.73 |
| [jetson_reuse_20261016](jetson_reuse_20261016/) | 59 | 0.86 |
| [mocap_connection_20261019](mocap_connection_20261019/) | 409 | 10.04 |
| [mocap_fusion_20261023](mocap_fusion_20261023/) | 497 | 86.87 |
| [mocap_lift_20261023](mocap_lift_20261023/) | 123 | 51.24 |
| [mocap_z_20261023](mocap_z_20261023/) | 128 | 23.94 |
| [old_project_audit_20261016](old_project_audit_20261016/) | 4 | 0.03 |
| [pwm_angle_es08mdii_20261015](pwm_angle_es08mdii_20261015/) | 4 | 0.14 |
| [pwm_gui_20261015](pwm_gui_20261015/) | 2 | 0.03 |
| [pwm_integration_20261014](pwm_integration_20261014/) | 2 | 0.03 |
| [pwm_link_fix_20261015](pwm_link_fix_20261015/) | 13 | 0.10 |
| [pwm_pulse_ui_20261015](pwm_pulse_ui_20261015/) | 2 | 0.03 |
| [release_delay_20261015](release_delay_20261015/) | 7 | 0.06 |
| [safety_followup_plan.md](safety_followup_plan.md/) | 1 | 0.01 |
| [session_close_20261023](session_close_20261023/) | 4 | 0.96 |
| [timed_release_20261016](timed_release_20261016/) | 24 | 0.26 |
| [timing_alignment_20261015](timing_alignment_20261015/) | 8 | 0.06 |
| [workfiles](workfiles/) | 487 | 873.30 |
