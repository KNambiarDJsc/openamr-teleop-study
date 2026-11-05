import json, sys
for f in sys.argv[1:]:
    d = json.load(open(f))
    for r in d["runs"]:
        s = r["samples"]; z0 = r["tip_start"][2]
        brk = next((x for x in s if abs(x[4] - z0) > 0.005), None)
        if r["mode"] == "world":
            print(f'{r["label"]:13s} hold broke at lift {brk[1]*1000:.0f} mm (t={brk[0]:.1f}s); final tip error {r["tip_world_displacement_m"]*1000:.0f} mm; '
                  f'IK failed {r["ik_failures"]}/{r["ticks"]} ticks; arm joint travel {r["arm_joint_travel_rad"]:.2f} rad unattended; peak joint speed {r["peak_joint_speed_fraction_of_limit"]*100:.1f}% of limit')
        else:
            print(f'{r["label"]:13s} tip carried {r["tip_world_displacement_m"]*1000:.0f} mm in {r["stroke_s"]:.1f}s, arm joint travel {r["arm_joint_travel_rad"]:.2f} rad')
