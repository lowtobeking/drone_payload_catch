#!/usr/bin/env python3
"""纯算法层"无 ROS 依赖"守卫测试。

参考工程把 `frame_convert.py` 单独成模块以便脱离 ROS 测试；本测试守住同一
设计约定：**核心算法层永远不偷偷 import rclpy/px4_msgs/...**，否则离线可验证、
可移植、可上 CI 的性质就悄悄丢掉了。

    python3 tools/test_purity.py
"""
import importlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

BLOCKED_ROOTS = {
    'rclpy', 'px4_msgs', 'std_msgs', 'geometry_msgs', 'sensor_msgs',
    'nav_msgs', 'tf2_ros', 'rosidl_runtime_py', 'builtin_interfaces',
    'rcl_interfaces', 'rosgraph_msgs',
}

# 约定：以下模块必须能在"完全没有 ROS"的环境导入
PURE_MODULES = [
    'payload_model', 'rendezvous', 'sim_core', 'payload_filter', 'stack_drop',
    'relnav', 'contact_detect', 'uncertainty', 'dynamics', 'impact',
    'perception', 'stats', 'coord_cert', 'keepout', 'safety_logic',
]

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


class RosBlocker:
    """把 ROS 相关顶层包挡在 import 之外。"""

    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in BLOCKED_ROOTS:
            raise ImportError(f'blocked ROS module: {fullname}')
        return None


# 连带清掉任何已被导入的 ROS 模块，保证拦截器必定生效
for mod in [m for m in list(sys.modules) if m.split('.')[0] in BLOCKED_ROOTS]:
    del sys.modules[mod]
sys.meta_path.insert(0, RosBlocker())

print('=== 拦截器自检 ===')
blocked = False
try:
    importlib.import_module('rclpy')
except ImportError as e:
    blocked = 'blocked ROS module' in str(e)
check('ROS 拦截器生效（rclpy 被挡）', blocked)

print('\n=== 纯模块必须无 ROS 依赖 ===')
for m in PURE_MODULES:
    try:
        importlib.import_module(f'payload_catch.{m}')
        check(f'{m} 无 ROS 依赖', True)
    except ImportError as e:
        if 'blocked ROS module' in str(e):
            check(f'{m} 无 ROS 依赖', False, str(e))
        else:
            print(f'  ⏭  {m} 跳过（缺少非 ROS 依赖：{e}）')
    except Exception as e:                                    # noqa: BLE001
        check(f'{m} 可导入', False, repr(e))

print('\n=== 反向检查：ROS 节点确实会用到 rclpy（拦截器没被绕过）===')
try:
    importlib.import_module('payload_catch.px4_iface')
    check('px4_iface 在无 ROS 时应导入失败', False, '竟然导入成功了')
except ImportError as e:
    check('px4_iface 在无 ROS 时导入失败', 'blocked ROS module' in str(e), str(e))

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ test_purity 全部通过')
