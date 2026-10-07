"""Writes notebooks/check_your_actuator.ipynb (cells below). Run: python build_notebook.py"""
import os
import nbformat as nbf

nb = nbf.v4.new_notebook()
C = []
md = lambda s: C.append(nbf.v4.new_markdown_cell(s.strip("\n")))
code = lambda s: C.append(nbf.v4.new_code_cell(s.strip("\n")))

md("""
# Check your actuator in 2 minutes

[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/artificialmotion/am-sdk/blob/main/notebooks/check_your_actuator.ipynb)

Enter a datasheet's rated torque, peak torque, mass and gear ratio. The notebook builds an actuator model from them,
tells you which values it had to assume, runs a duty profile (a long hold, squat cycles) and plots the torque limit,
the winding temperature and the derate and trip events. Optionally it puts the actuator in a MuJoCo leg.

**Everything here is a simulation.** The thermal model is first-order and lumped, calibrated to reproduce the rating
under its stated conditions. When a datasheet does not state its conditions (most do not), the model assumes them and
says so. Nothing below is a measurement, a guarantee or a ranking of products. Vendor numbers are shown as the seller
published them; for the catalogue example the rating conditions are unstated. The model is open:
[docs/ACTUATOR_MODEL.md](https://github.com/artificialmotion/am-sdk/blob/main/docs/ACTUATOR_MODEL.md).
""")

code("""
import os, sys, math, importlib.util
if "google.colab" in sys.modules:
    os.environ["MUJOCO_GL"] = "egl"          # offscreen rendering on Colab; must be set before importing mujoco
if importlib.util.find_spec("am_sdk") is None:
    %pip install -q "am-sdk[batch] @ git+https://github.com/artificialmotion/am-sdk#subdirectory=python" mujoco
""")

code("""
import numpy as np
import matplotlib.pyplot as plt
import am_sdk
from am_sdk.actuator_model import ActuatorModel, load_spec, spec_from_datasheet

print("am_sdk from:", os.path.dirname(am_sdk.__file__))
""")

md("""
## 1. Your actuator

Edit the numbers. The defaults are a catalogue example: a listing that gives 40 N m rated, 120 N m peak, 1.42 kg and
a 9:1 ratio. This is the **seller's rating, conditions unstated** (ambient, cooling, mounting and temperature rise are
not given), so the model assumes them. It is here as an example of a typical datasheet, not as a statement about that
product. For comparison the notebook also loads the AM-M spec shipped with the SDK (a design target, simulated, not
built, conditions stated).
""")

code("""
#@title Datasheet values { run: "auto" }
name = "Catalogue example (seller rating, conditions unstated)" #@param {type:"string"}
rated_nm = 40.0     #@param {type:"number"}   # "rated" or "continuous" torque, N m
peak_nm = 120.0     #@param {type:"number"}   # peak torque, N m
mass_kg = 1.42      #@param {type:"number"}
ratio = 9.0         #@param {type:"number"}   # gear ratio (motor turns per output turn)
bus_v = 48.0        #@param {type:"number"}
ambient_c = 40.0    #@param {type:"number"}   # the ambient YOU want to test at (inside a robot torso, say)
""")

code("""
yours = spec_from_datasheet("your-actuator", rated_nm=rated_nm, peak_nm=peak_nm, mass_kg=mass_kg, ratio=ratio,
                            bus_v=bus_v, vendor="", model=name,
                            status="built from datasheet values; rating conditions unstated")
am_m = load_spec("AM-M")
ACTS = [(name, yours), ("AM-M (design target, simulated)", am_m)]

for label, sp in ACTS:
    print("=" * 78)
    print(label)
    print("  rating conditions:", sp.conditions.get("text") or "-", "| stated:", sp.conditions["stated"])
    print("  %d values are assumed (not from the source):" % len(sp.assumptions))
    for a in sp.assumptions:
        print("    %-42s %-12s %s" % (a["field"], str(a["value"])[:12], a["basis"][:70]))
""")

md("""
## 2. Duty profiles

* **Long hold:** the joint holds a static torque (speed 0) for 15 minutes, e.g. a crouch with a load.
* **Squat cycles:** 3 s cycles, torque from 15 % to 100 % of the peak value you pick, speed up to half the rated speed.

For each actuator the plots show the commanded torque, the torque limit the model enforces (speed, temperature and
peak-budget dependent), the winding temperature, and the events: peak budget empty, derate start, trip.
The derate law is an assumption (see the assumption list above).
""")

code("""
hold_frac = 1.0        #@param {type:"number"}   # hold torque as a fraction of the rated torque
squat_peak_frac = 1.8  #@param {type:"number"}   # squat peak torque as a fraction of the rated torque
seconds = 900          #@param {type:"integer"}
DT = 0.002

def run_profile(spec, kind, ambient, seconds=seconds, dt=DT):
    act = ActuatorModel(spec, dt=dt, ambient_c=ambient)
    r = spec.continuous_nm
    w0 = spec.continuous_speed or 2.0
    stride = int(round(1.0 / dt))
    rows = []
    for k in range(int(seconds / dt)):
        t = k * dt
        if kind == "hold":
            cmd, w = hold_frac * r, 0.0
        else:
            cmd = squat_peak_frac * r * (0.15 + 0.85 * math.sin(math.pi * t / 3.0) ** 2)
            w = 0.5 * w0 * math.cos(2.0 * math.pi * t / 3.0)
        act.step(cmd, w)
        if k % stride == 0:
            s = act.state
            lim = s.limit_pos_nm if cmd >= 0 else s.limit_neg_nm
            rows.append((t, cmd, s.tau_nm, lim, s.winding_c, s.derate))
    return act, np.array(rows)

def show(kind, title):
    fig, axs = plt.subplots(2, len(ACTS), figsize=(11, 5.5), sharex=True)
    summary = []
    for j, (label, sp) in enumerate(ACTS):
        act, R = run_profile(sp, kind, ambient_c)
        t = R[:, 0]
        ax = axs[0, j]
        ax.plot(t, R[:, 1], color="0.6", lw=1, label="commanded")
        ax.plot(t, R[:, 3], color="k", lw=1.4, label="torque limit")
        ax.set_ylabel("N m")
        ax.set_title(label, fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        ax = axs[1, j]
        ax.plot(t, R[:, 4], color="k")
        ax.axhline(sp.thermal["derate_c"], color="0.5", ls="--", lw=0.8)
        ax.text(t[0], sp.thermal["derate_c"] + 1, "derate starts", fontsize=7, color="0.4")
        ax.axhline(sp.thermal["trip_c"], color="firebrick", ls=":", lw=0.8)
        ax.set_ylabel("winding, C")
        ax.set_xlabel("time, s (simulated)")
        first = {}
        for e in act.events:
            if e.kind not in first:
                first[e.kind] = e.t
                if e.kind in ("derate_start", "trip", "peak_budget_empty"):
                    ax.axvline(e.t, color="firebrick" if e.kind != "peak_budget_empty" else "0.6", lw=0.8)
                    ax.text(e.t, ax.get_ylim()[0] + 2, " " + e.kind, rotation=90, fontsize=7, va="bottom")
        summary.append((label, R[:, 4].max(), first.get("derate_start"), first.get("trip"),
                        100.0 * (1.0 - np.sum(R[:, 2]) / max(1e-9, np.sum(R[:, 1]))), act.event_counts))
    fig.suptitle(title + "  |  SIMULATED, first-order thermal model, ambient %.0f C" % ambient_c, fontsize=10)
    fig.tight_layout()
    plt.show()
    for label, tmax, td, tt, short, ev in summary:
        print("%-45s max winding %5.1f C | derate at %s | trip at %s | torque shortfall %.1f %% | %s"
              % (label, tmax, "-" if td is None else "%.0f s" % td, "-" if tt is None else "%.0f s" % tt, short,
                 {k: v for k, v in ev.items() if v}))
""")

code("""
show("hold", "Long hold at %.0f %% of rated torque, %d s" % (100 * hold_frac, seconds))
""")

code("""
show("squat", "Squat cycles, peak %.0f %% of rated torque, %d s" % (100 * squat_peak_frac, seconds))
""")

md("""
How to read it: if the torque limit falls below the commanded torque, the joint can no longer deliver what you asked.
"Torque shortfall" is the commanded torque that was not delivered, summed over the run (sampled once per second). A hold at
the rated torque sits right at the edge by design, so it tests the rating's conditions as much as the actuator.
Change `ambient_c`, `hold_frac` and the datasheet numbers above and re-run.
""")

md("""
## 3. Optional: put it in a MuJoCo leg

A fixed hip, a thigh hanging down and a shank held horizontally, with a mass at the foot sized so that the knee holds
`hold_frac` of the rated torque. The knee uses your actuator model. The hip is a stand-in sized large so that it does
not limit anything. Time runs 1:1 in the model; the frames show the start, the middle and the end. The knee is coloured
by winding temperature. If offscreen rendering is not available the cell falls back to plots only.
""")

code("""
import mujoco
from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank, mjcf_defaults_block, mjcf_snippet

HIP = spec_from_datasheet("hip-standin", rated_nm=400.0, peak_nm=800.0, mass_kg=5.0, ratio=9.0,
                          status="oversized stand-in so the hip does not limit")
G, SHANK_KG = 9.81, 2.0
payload_kg = max(0.5, (hold_frac * yours.continuous_nm - SHANK_KG * G * 0.2) / (G * 0.4))
Q_KNEE = -math.pi / 2

def leg_xml():
    hip, knee = mjcf_snippet(HIP, "hip"), mjcf_snippet(yours, "knee")
    return '''<mujoco model="leg_hold"><option timestep="%g"/>
  %s
  <visual><global offwidth="480" offheight="480"/><headlight ambient="0.55 0.55 0.55" diffuse="0.6 0.6 0.6"/></visual>
  <asset><texture type="skybox" builtin="flat" rgb1="0.957 0.953 0.933" rgb2="0.957 0.953 0.933" width="8" height="8"/></asset>
  <worldbody><body name="base" pos="0 0 1.0">
    <body name="thigh"><joint name="hip" type="hinge" axis="0 1 0" %s/>
      <geom contype="0" conaffinity="0" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04" mass="4" rgba="0.09 0.09 0.09 1"/>
      <body name="shank" pos="0 0 -0.4"><joint name="knee" type="hinge" axis="0 1 0" %s/>
        <geom contype="0" conaffinity="0" name="motor" type="cylinder" euler="90 0 0" size="0.06 0.05" mass="0.5" rgba="0.3 0.3 0.28 1"/>
        <geom contype="0" conaffinity="0" name="shank_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.032" mass="%g" rgba="0.09 0.09 0.09 1"/>
        <geom contype="0" conaffinity="0" type="sphere" pos="0 0 -0.4" size="0.075" mass="%g" rgba="0.09 0.09 0.09 1"/>
      </body></body></body></worldbody>
  <actuator>%s %s</actuator></mujoco>''' % (DT, mjcf_defaults_block([HIP, yours]), hip["joint_attrs"], knee["joint_attrs"],
                                            SHANK_KG, payload_kg, hip["actuator"], knee["actuator"])

model = mujoco.MjModel.from_xml_string(leg_xml())
data = mujoco.MjData(model)
data.qpos[:] = [0.0, Q_KNEE]
mujoco.mj_forward(model, data)
bank = MujocoActuatorBank(model, data, {"hip": HIP, "knee": yours}, apply_plant_params=False, ambient_c=ambient_c)
SIM_S = 300
snap_t = [0.0, SIM_S / 2, SIM_S - 1.0]
snaps, hist = {}, []
for k in range(int(SIM_S / DT)):
    t = k * DT
    bank.step({"hip": 1500.0 * (0.0 - data.qpos[0]) - 50.0 * data.qvel[0],
               "knee": 1500.0 * (Q_KNEE - data.qpos[1]) - 50.0 * data.qvel[1]})
    if k % 500 == 0:
        st = bank["knee"].state
        hist.append((t, abs(st.tau_nm), st.limit_pos_nm if st.tau_nm >= 0 else st.limit_neg_nm, st.winding_c, data.qpos[1]))
    for ts in snap_t:
        if ts not in snaps and t >= ts:
            snaps[ts] = (data.qpos.copy(), bank["knee"].state.winding_c)
H = np.array(hist)
print("payload %.1f kg at the foot; knee torque %.1f N m, winding %.1f -> %.1f C, knee angle %.3f -> %.3f rad, events %s"
      % (payload_kg, H[5, 1], H[0, 3], H[-1, 3], H[0, 4], H[-1, 4], dict(bank["knee"].event_counts)))
""")

code("""
fig = plt.figure(figsize=(11, 4.2))
frames = None
try:
    renderer = mujoco.Renderer(model, 480, 480)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.22, 0.0, 0.78); cam.distance, cam.azimuth, cam.elevation = 1.75, 90.0, -5.0
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "shank_geom")
    base = model.geom_rgba[gid].copy()
    frames = []
    for ts in snap_t:
        qpos, tw = snaps[ts]
        data.qpos[:] = qpos; mujoco.mj_forward(model, data)
        f = min(1.0, max(0.0, (tw - ambient_c) / (yours.thermal["trip_c"] - ambient_c)))
        model.geom_rgba[gid] = (1 - f) * base + f * np.array([0.70, 0.15, 0.10, 1.0])
        renderer.update_scene(data, cam)
        frames.append((ts, tw, renderer.render().copy()))
except Exception as e:
    print("Offscreen rendering is not available here (%s); showing plots only." % type(e).__name__)
    frames = None
if frames:
    for i, (ts, tw, img) in enumerate(frames):
        ax = fig.add_subplot(1, 4, i + 1)
        ax.imshow(img); ax.axis("off"); ax.set_title("t = %d s, winding %.0f C" % (ts, tw), fontsize=8)
    ax = fig.add_subplot(1, 4, 4)
else:
    ax = fig.add_subplot(1, 1, 1)
ax.plot(H[:, 0], H[:, 3], color="k"); ax.set_xlabel("time, s (simulated)"); ax.set_ylabel("knee winding, C")
ax.set_title("SIMULATED", fontsize=8)
fig.tight_layout(); plt.show()
""")

md("""
## What this does and does not tell you

* It is a simulation with a first-order (1 or 2 node) thermal network, calibrated so the model reproduces the datasheet
  rating under the conditions it states, or under the assumed conditions listed above when none are stated.
* Values the datasheet does not give (inertia, friction, thermal time constants, derate and trip temperatures,
  efficiency, peak duration) are assumptions with a stated basis. Replace them with measured values in the spec JSON
  when you have them: see "Fit your own actuator" in `docs/ACTUATOR_MODEL.md` and `python/examples/08_*.py`.
* It does not say one product is better than another. It shows what a given set of datasheet numbers implies for a duty
  profile you choose, so you can ask the seller the right questions (conditions, duration, temperature limit).
* Artificial Motion units shown here are design targets for evaluation units shipping in Q1 2027.

Apache-2.0, [github.com/artificialmotion/am-sdk](https://github.com/artificialmotion/am-sdk).
""")

nb["cells"] = C
nb["metadata"] = {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
                  "language_info": {"name": "python"}, "colab": {"provenance": []}}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "notebooks", "check_your_actuator.ipynb")
nbf.write(nb, out)
print("wrote", os.path.normpath(out))
