# 协同协议 SITL 验证

> 由 `python3 tools/validate_coord.py` 生成。检查协同释放协议的不变量与安全性。

| 配置 | 捕获 | 就绪先于释放 | ack 在就绪后 | 单次释放 | min\|A−B\| | 异常 | Failsafe |
|---|---|---|---|---|---|---|---|
| H_stack_nominal | ✅ | ✅ | ✅ | ✅ | 0.990 | 0 | 0 |
| H_stack_noise+safety | ✅ | ✅ | ✅ | ✅ | 1.034 | 0 | 0 |
| H_formation | ✅ | ✅ | ✅ | ✅ | 1.667 | 0 | 0 |

**总判定：全部通过 ✅**

> 注：`min\|A−B\|` 由 B 的**带噪估计**得到，是近似值；闸值为 0.5m（< `min_ab_gap`=0.8 为保守校验）。
> 不变量定义：A 收到 B 的 ready 必须先于 payload 释放；B 收到 A 的 release ack 必须在收到 ready 之后；
> 单次释放 = A 只发起一次释放（ready/释放事件各 1 次）。
