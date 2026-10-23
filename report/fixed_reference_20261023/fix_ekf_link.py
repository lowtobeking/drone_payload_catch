from pathlib import Path
import ast
WORK=Path(__file__).resolve().parent
p=WORK/"run_local_ekf.py"
s=p.read_text(encoding="utf-8")
s=s.replace('extras=[B/"src/lib/geo/libgeo.a",B/"src/lib/world_magnetic_model/libworld_magnetic_model.a"]',
'''extras=[B/"src/lib/geo/libgeo.a",B/"src/lib/world_magnetic_model/libworld_magnetic_model.a",
        B/"src/lib/lat_lon_alt/liblat_lon_alt.a"]
objects=[]
for folder in ("bias_estimator","output_predictor","yaw_estimator"):
    objects.extend((B/"src/modules/ekf2/EKF"/folder).rglob("*.o"))
assert objects, "existing EKF submodule objects missing"''',1)
s=s.replace('str(src),str(lib),\n             *map(str,extras)',
            'str(src),"-Wl,--start-group",str(lib),*map(str,objects),\n             *map(str,extras),"-Wl,--end-group"',1)
ast.parse(s)
p.write_text(s,encoding="utf-8")
