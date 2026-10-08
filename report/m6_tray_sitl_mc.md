# 圆形托盘（真机末端）SITL 蒙特卡洛结果（每档 N=3，逐次换种子）

模型：`FUNNEL_TYPE=tray`（x500_tray + payload_100g）；eff_r=0.12m，v_retain=6.60 m/s。

| 配置 | 成功率 | 水平误差(成功均值) | 接触速度(成功均值) | min A-B 最低 |
|---|---|---|---|---|
  rep1: captured=1 ready=1 hz=0.034 rel_v=0.070 min_relA=1.08
  rep2: captured=1 ready=1 hz=0.021 rel_v=2.837 min_relA=1.118
  rep3: captured=1 ready=1 hz=0.014 rel_v=2.876 min_relA=1.113
| C0_nominal | 3/3 | 0.023 m | 1.928 m/s | 1.080 m |
== C0_nominal: 3/3 mean_hz=0.023 mean_rv=1.928 min_minrelA=1.080
  rep1: captured=1 ready=1 hz=0.045 rel_v=2.837 min_relA=0.936
  rep2: captured=1 ready=1 hz=0.051 rel_v=2.802 min_relA=0.963
  rep3: captured=1 ready=1 hz=0.034 rel_v=2.603 min_relA=0.956
| C1_relnav_heavy_lpf | 3/3 | 0.043 m | 2.747 m/s | 0.936 m |
== C1_relnav_heavy_lpf: 3/3 mean_hz=0.043 mean_rv=2.747 min_minrelA=0.936
  rep1: captured=0 ready=1 hz= rel_v= min_relA=1.118
  rep2: captured=1 ready=1 hz=0.101 rel_v=0.066 min_relA=1.107
  rep3: captured=0 ready=1 hz= rel_v= min_relA=1.107
| C2_err0.20_track | 1/3 | 0.101 m | 0.066 m/s | 1.107 m |
== C2_err0.20_track: 1/3 mean_hz=0.101 mean_rv=0.066 min_minrelA=1.107
MC-DONE
