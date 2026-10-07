"""Clip B: a knee holding a load for a long time; winding temperature rises and the torque limit derates. SIMULATED.

MuJoCo leg (hip + knee, thigh down, shank horizontal, 7.8 kg mass at the foot), each joint through an
am_sdk.actuator_model.ActuatorModel. The knee is an AM-M (design-target spec, not built). The room is 55 C
(warm robot interior, an assumption). 600 s of simulated time are shown as a x60 time-lapse.
Output: docs/media/actuator_thermal_hold.{mp4,gif}. Usage: python clip_actuator.py [--sim-only]
"""
import math
import os
import sys
import tempfile

from common import *  # noqa: F401,F403
import mujoco
from am_sdk.actuator_model.mujoco_bridge import MujocoActuatorBank, mjcf_defaults_block, mjcf_snippet

HIP, KNEE = "AM-H-T", "AM-M"
AMBIENT_C, SECONDS, PAYLOAD_KG = 58.0, 600.0, 8.2
Q_KNEE = -math.pi / 2


def leg_xml():
    hip, knee = mjcf_snippet(HIP, "hip"), mjcf_snippet(KNEE, "knee")
    return """<mujoco model="am_leg_hold">
  <option timestep="0.001"/>
  %s
  <visual><global offwidth="720" offheight="720"/><headlight ambient="0.55 0.55 0.55" diffuse="0.6 0.6 0.6"/>
          <quality shadowsize="0"/></visual>
  <asset><texture type="skybox" builtin="flat" rgb1="0.957 0.953 0.933" rgb2="0.957 0.953 0.933" width="8" height="8"/></asset>
  <worldbody>
    <body name="base" pos="0 0 1.0">
      <geom contype="0" conaffinity="0" name="mount" type="box" size="0.05 0.12 0.06" pos="-0.06 0 0.06" rgba="0.55 0.55 0.52 1"/>
      <body name="thigh">
        <joint name="hip" type="hinge" axis="0 1 0" %s/>
        <geom contype="0" conaffinity="0" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04" mass="4" rgba="0.09 0.09 0.09 1"/>
        <geom contype="0" conaffinity="0" type="cylinder" pos="0 0 0" euler="90 0 0" size="0.06 0.05" mass="0.5" rgba="0.3 0.3 0.28 1"/>
        <body name="shank" pos="0 0 -0.4">
          <joint name="knee" type="hinge" axis="0 1 0" %s/>
          <geom contype="0" conaffinity="0" name="knee_motor" type="cylinder" euler="90 0 0" size="0.06 0.05" mass="0.5" rgba="0.3 0.3 0.28 1"/>
          <geom contype="0" conaffinity="0" name="shank_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.032" mass="2" rgba="0.09 0.09 0.09 1"/>
          <geom contype="0" conaffinity="0" name="load" type="sphere" pos="0 0 -0.4" size="0.075" mass="%g" rgba="0.09 0.09 0.09 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    %s
    %s
  </actuator>
</mujoco>""" % (mjcf_defaults_block([HIP, KNEE]), hip["joint_attrs"], knee["joint_attrs"], PAYLOAD_KG,
                 hip["actuator"], knee["actuator"])


def simulate(cache):
    model = mujoco.MjModel.from_xml_string(leg_xml())
    data = mujoco.MjData(model)
    data.qpos[0], data.qpos[1] = 0.0, Q_KNEE
    mujoco.mj_forward(model, data)
    bank = MujocoActuatorBank(model, data, {"hip": HIP, "knee": KNEE}, apply_plant_params=False, ambient_c=AMBIENT_C)
    dt = model.opt.timestep
    rec = {k: [] for k in ("t", "q", "tau", "lim", "tw", "derate", "hip_tw")}
    samples = []
    for k in range(int(SECONDS / dt)):
        cmd = {"hip": 1500 * (0.0 - data.qpos[0]) - 50 * data.qvel[0],
               "knee": 1500 * (Q_KNEE - data.qpos[1]) - 50 * data.qvel[1]}
        bank.step(cmd)
        if k % 1000 == 0:
            s, h = bank["knee"].state, bank["hip"].state
            lim = s.limit_pos_nm if s.tau_nm >= 0 else s.limit_neg_nm
            for key, v in (("t", k * dt), ("q", data.qpos[1]), ("tau", abs(s.tau_nm)), ("lim", lim),
                           ("tw", s.winding_c), ("derate", s.derate), ("hip_tw", h.winding_c)):
                rec[key].append(v)
            samples.append(data.qpos.copy())
    out = {k: np.array(v) for k, v in rec.items()}
    out["qpos"] = np.array(samples)
    out["events"] = str(bank["knee"].event_counts)
    np.savez(cache, **out)
    return out


if __name__ == "__main__":
    cache = os.path.join(tempfile.gettempdir(), "am_showcase_actuator_hold.npz")
    if os.path.exists(cache) and "--resim" not in sys.argv:
        D = dict(np.load(cache, allow_pickle=True))
    else:
        D = simulate(cache)
    print("knee tau %.1f -> %.1f N*m, limit %.1f -> %.1f, winding %.1f -> %.1f C, hip winding max %.1f, derate min %.3f, q %.3f -> %.3f, events %s"
          % (D["tau"][5], D["tau"][-1], D["lim"][5], D["lim"][-1], D["tw"][0], D["tw"][-1], D["hip_tw"].max(),
             D["derate"].min(), D["q"][0], D["q"][-1], D["events"]))
    for i in range(0, 600, 50):
        print(i, round(D["tau"][i], 1), round(D["lim"][i], 1), round(D["tw"][i], 1), round(D["derate"][i], 3), round(D["q"][i], 3))
    if "--sim-only" in sys.argv:
        sys.exit(0)

    # ---------------------------------------------------------------- render
    model = mujoco.MjModel.from_xml_string(leg_xml())
    data = mujoco.MjData(model)
    rw = 640   # render size (square), drawn into the left panel
    renderer = mujoco.Renderer(model, rw, rw)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.22, 0.0, 0.78)
    cam.distance, cam.azimuth, cam.elevation = 1.75, 90.0, -5.0
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "shank_geom")
    gid2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "knee_motor")
    base_rgba = model.geom_rgba[gid].copy()
    N_FR, FPS = 250, 25
    t, tau, lim, tw = D["t"], D["tau"], D["lim"], D["tw"]
    T_END = t[-1]

    fig = new_fig()
    title(fig, "Holding a load, a long time", "%s knee, %.1f kg at the foot, %.0f C ambient (assumed). Spec is a design target."
          % (KNEE, PAYLOAD_KG, AMBIENT_C))
    caption(fig, "Simulated. Time-lapse x%.0f: 600 s of simulated time in 10 s. First-order thermal model, see docs/ACTUATOR_MODEL.md."
            % (T_END / (N_FR / FPS)))
    axi = fig.add_axes([0.0, 0.08, 0.40, 0.80])
    axi.axis("off")
    img = axi.imshow(np.zeros((rw, rw, 3), dtype=np.uint8))
    clock = fig.text(0.02, 0.12, "", fontsize=8)
    ax1 = fig.add_axes([0.50, 0.57, 0.47, 0.28])
    ax2 = fig.add_axes([0.50, 0.17, 0.47, 0.28])
    for ax in (ax1, ax2):
        ax.set_xlim(0, T_END)
        ax.tick_params(labelsize=6.5, length=2)
    ax1.set_ylim(0, 90)
    ax1.set_ylabel("torque, N*m", fontsize=7)
    ax1.set_xticklabels([])
    ax2.set_ylim(50, 115)
    ax2.set_ylabel("winding, C", fontsize=7)
    ax2.set_xlabel("time, s (simulated)", fontsize=7)
    ax2.axhline(105, color=GREY, ls="--", lw=0.8)
    ax2.text(2, 105.8, "derate starts 105 C", fontsize=6, color=GREY)
    la, = ax1.plot([], [], color=INK, label="torque limit")
    lb, = ax1.plot([], [], color=ACCENT, label="load torque")
    ax1.legend(loc="upper right", fontsize=6.5, frameon=False, ncol=1)
    lc, = ax2.plot([], [], color=INK)
    note = fig.text(0.50, 0.49, "", fontsize=7.5, color=ACCENT)
    t_der = next((t[i] for i in range(len(t)) if D["derate"][i] < 1.0), None)

    def make_frame(i):
        k = min(len(t) - 1, int(round(i / (N_FR - 1) * (len(t) - 1))))
        data.qpos[:] = D["qpos"][k]
        mujoco.mj_forward(model, data)
        f = min(1.0, max(0.0, (tw[k] - 55.0) / 55.0))
        model.geom_rgba[gid] = (1 - f) * base_rgba + f * np.array([0.70, 0.15, 0.10, 1.0])
        model.geom_rgba[gid2] = model.geom_rgba[gid]
        renderer.update_scene(data, cam)
        img.set_data(renderer.render())
        sl = slice(0, k + 1)
        la.set_data(t[sl], lim[sl]); lb.set_data(t[sl], tau[sl]); lc.set_data(t[sl], tw[sl])
        clock.set_text("t = %d:%02d  winding %.0f C" % (t[k] // 60, t[k] % 60, tw[k]))
        note.set_text("above 105 C: peak limit removed, limit derates toward the load" if (t_der is not None and t[k] >= t_der) else "")
        return fig

    render_clip("actuator_thermal_hold", make_frame, N_FR, fps=FPS)
