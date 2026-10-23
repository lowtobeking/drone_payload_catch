from pathlib import Path
import json
import math
import sys
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root))
from payload_catch.mocap_transform import load_config
work=Path(__file__).resolve().parent
_, transform=load_config(root/'config/mocap_a.preview.yaml')
cases={}
for name,source in (
    ('forward_before','planeA_forward_result_20261021.json'),
    ('yaw_before','planeA_yaw_result_20261021.json'),
    ('yaw_after','planeA_yaw_result_20261021.json'),
    ('forward_after_turn','planeA_cross_forward_result_20261021.json')):
    data=json.loads((work/source).read_text(encoding='utf-8'))
    raw=data['before' if name in ('forward_before','yaw_before') else 'after']
    position,q=transform.convert(raw['position'],raw['quaternion_xyzw'])
    matrix=Rotation.from_quat(q).as_matrix()
    cases[name]=dict(position_lab_frd_m=position.tolist(),
        quaternion_body_to_lab_frd_xyzw=q.tolist(),
        yaw_lab_frd_deg=math.degrees(math.atan2(matrix[1,0],matrix[0,0])),
        body_down_in_lab_frd=matrix[:,2].tolist())
    if matrix[2,2]<.98: raise AssertionError('level body down does not align with lab down')
result=dict(passed=True,preview_only=True,candidate_mounting_not_calibrated=True,cases=cases)
(work/'preview_measured_conversion.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
