# 圆形托盘（真机末端）SITL 难度扫描结果

模型：`FUNNEL_TYPE=tray`（x500_tray + payload_100g）；盘内径30cm/围边5cm/泡棉e=0.15；
eff_r=0.12m，v_retain=6.60 m/s（a_dive 自动=0，悬停接）。
每档重启 gz + 2×PX4，运行 70s。

| 试验 | 捕获 | 水平误差 | 接触相对速度 | 对正 rel_xy | min A-B | 双机落地 |
|---|---|---|---|---|---|---|
===== TR0_nominal  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[] =====
| TR0_nominal | ✅ | horiz=0.046m | rel_v=2.719m/s | rel_xy=0.055m | 1.071 m | ✅✅ |
  captured=1 horiz=0.046m rel_v=2.719m/s rel_xy=0.055m min_relA=1.071 land0=1 land1=1 ready=1
===== TR1_err0.10_track  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[release_xy_sigma:=0.10] =====
| TR1_err0.10_track | ✅ | horiz=0.029m | rel_v=2.653m/s | rel_xy=0.013m | 1.087 m | ✅✅ |
  captured=1 horiz=0.029m rel_v=2.653m/s rel_xy=0.013m min_relA=1.087 land0=1 land1=1 ready=1
===== TR2_relnav_heavy_lpf  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08] =====
| TR2_relnav_heavy_lpf | ✅ | horiz=0.020m | rel_v=0.014m/s | rel_xy=0.087m | 0.964 m | ✅✅ |
  captured=1 horiz=0.020m rel_v=0.014m/s rel_xy=0.087m min_relA=0.964 land0=1 land1=1 ready=1
===== TR3_relnav_heavy_nolpf  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08 est_lpf_alpha:=1.0] =====
| TR3_relnav_heavy_nolpf | ❌ | — | — | — | 0.767 m | — |
  captured=0    min_relA=0.767 land0=0 land1=0 ready=1
===== TR4_gap1.2  A=0.0,0.0,-4.7 B=0.0,0.0,-3.5 extra=[] =====
| TR4_gap1.2 | ✅ | horiz=0.052m | rel_v=3.117m/s | rel_xy=0.071m | 1.166 m | ✅✅ |
  captured=1 horiz=0.052m rel_v=3.117m/s rel_xy=0.071m min_relA=1.166 land0=1 land1=1 ready=1
===== TR5_dive5  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[a_dive:=5.0] =====
| TR5_dive5 | ✅ | horiz=0.033m | rel_v=2.719m/s | rel_xy=0.013m | 1.1 m | ✅✅ |
  captured=1 horiz=0.033m rel_v=2.719m/s rel_xy=0.013m min_relA=1.1 land0=1 land1=1 ready=1
===== TR6_err0.20_track  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[release_xy_sigma:=0.20] =====
| TR6_err0.20_track | ✅ | horiz=0.055m | rel_v=2.838m/s | rel_xy=0.052m | 1.113 m | ✅✅ |
  captured=1 horiz=0.055m rel_v=2.838m/s rel_xy=0.052m min_relA=1.113 land0=1 land1=1 ready=1
===== TR7_err0.20_notrack  A=0.0,0.0,-4.5 B=0.0,0.0,-3.5 extra=[release_xy_sigma:=0.20 track_payload:=false] =====
| TR7_err0.20_notrack | ✅ | horiz=0.037m | rel_v=2.880m/s | rel_xy=0.052m | 1.104 m | ✅✅ |
  captured=1 horiz=0.037m rel_v=2.880m/s rel_xy=0.052m min_relA=1.104 land0=1 land1=1 ready=1
SWEEP-DONE
