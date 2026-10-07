/* SPDX-License-Identifier: Apache-2.0 */
/* Torque estimator and momentum-observer collision detector. Line-by-line port of
 * python/am_sdk/estimation (core.py, collision.py, calibration.py); the cross-check in
 * python/tests/test_cross_check_est.py holds the two to the same numbers.
 * SIMULATED, PRE-HARDWARE: every torque here is an estimate. No allocation, no I/O. */
#include "am_est.h"
#include "am_internal.h"

#include <math.h>
#include <string.h>

#define EST_TWO_PI 6.283185307179586
#define NO_TEMP_UNC_C 40.0
#define DITHER_MAX_FRAC_RATED 0.1
#define DITHER_MAX_OF_BREAKAWAY 1.5

static int est_finite(double x)
{
    return x == x && x - x == 0.0;   /* not NaN, not +-inf (C99 isfinite without the macro) */
}

static double est_minf(double a, double b) { return a < b ? a : b; }
static double est_maxf(double a, double b) { return a > b ? a : b; }

/* ---------------------------------------------------------------- calibration */
am_status am_est_calibration_default(const char *unit, am_est_calibration *cal)
{
    const am_unit_data *ud;
    double ts, eta_f;
    if (!unit || !cal) return AM_ERR_PARAM;
    ud = am__unit_find(unit, strlen(unit));
    if (!ud) return AM_ERR_PARAM;
    memset(cal, 0, sizeof *cal);
    ts = ud->backdrive_nm;
    eta_f = ud->gear_eta;
    cal->schema_version = AM_EST_CAL_SCHEMA_VERSION;
    cal->basis = AM_BASIS_DESIGN_TARGET;
    cal->gear_ratio = ud->ratio;
    cal->kt_Nm_per_A = ud->kt;
    cal->kt_ref_temp_C = 60.0;
    cal->kt_temp_coeff_per_K = -0.00094;
    cal->reflected_inertia_kgm2 = ud->armature_kgm2;
    cal->rated_torque_Nm = ud->rated_nm;
    cal->coulomb_Nm = 0.7 * ts;
    cal->breakaway_Nm = ts;
    cal->stribeck_vel_rad_s = 0.05;
    cal->viscous_Nm_s_per_rad = 0.06 * ts;
    cal->friction_ref_temp_C = 40.0;
    cal->coulomb_temp_coeff_per_K = 0.004;
    cal->viscous_temp_exp_per_K = 0.0225;
    cal->eta_forward = eta_f;
    cal->eta_reverse = est_maxf(0.5, 1.0 - 2.0 * (1.0 - eta_f));
    cal->unc_kt_rel = 0.02;
    cal->unc_temp_C = 6.0;
    cal->unc_friction_rel = 0.3;
    cal->unc_eta_abs = 0.02;
    cal->unc_inertia_rel = 0.1;
    return AM_OK;
}

static int est_eta_ok(double v) { return est_finite(v) && v >= 0.3 && v <= 1.0; }

am_status am_est_calibration_check(const am_est_calibration *c)
{
    uint32_t i, j;
    double nonneg[11];
    if (!c || c->schema_version != AM_EST_CAL_SCHEMA_VERSION) return AM_ERR_PARAM;
    nonneg[0] = c->kt_sat_frac;
    nonneg[1] = c->kt_sat_current_A;
    nonneg[2] = c->coulomb_Nm;
    nonneg[3] = c->breakaway_Nm;
    nonneg[4] = c->viscous_Nm_s_per_rad;
    nonneg[5] = c->bearing_Nm;
    nonneg[6] = c->unc_kt_rel;
    nonneg[7] = c->unc_temp_C;
    nonneg[8] = c->unc_friction_rel;
    nonneg[9] = c->unc_eta_abs;
    nonneg[10] = c->unc_inertia_rel;
    if (!(est_finite(c->gear_ratio) && c->gear_ratio > 0 && est_finite(c->kt_Nm_per_A) && c->kt_Nm_per_A > 0 &&
          est_finite(c->reflected_inertia_kgm2) && c->reflected_inertia_kgm2 > 0 &&
          est_finite(c->rated_torque_Nm) && c->rated_torque_Nm > 0 &&
          est_finite(c->stribeck_vel_rad_s) && c->stribeck_vel_rad_s > 0))
        return AM_ERR_PARAM;
    for (i = 0; i < sizeof nonneg / sizeof nonneg[0]; i++)
        if (!est_finite(nonneg[i]) || nonneg[i] < 0) return AM_ERR_PARAM;
    if (!est_finite(c->kt_ref_temp_C) || !est_finite(c->kt_temp_coeff_per_K) || !est_finite(c->friction_ref_temp_C) ||
        !est_finite(c->coulomb_temp_coeff_per_K) || !est_finite(c->viscous_temp_exp_per_K))
        return AM_ERR_PARAM;
    if (!est_eta_ok(c->eta_forward) || !est_eta_ok(c->eta_reverse)) return AM_ERR_PARAM;
    if (c->kt_sat_frac >= 0.5) return AM_ERR_PARAM;
    if (c->coulomb_Nm > c->breakaway_Nm + 1e-12) return AM_ERR_PARAM;
    if (c->friction_map_n > AM_EST_FRICTION_MAP_MAX) return AM_ERR_PARAM;
    for (i = 0; i < c->friction_map_n; i++) {
        double w = c->friction_map_w_rad_s[i], f = c->friction_map_Nm[i];
        if (!est_finite(w) || !est_finite(f) || w < 0 || f < 0) return AM_ERR_PARAM;
        if (i > 0 && !(w > c->friction_map_w_rad_s[i - 1])) return AM_ERR_PARAM;
    }
    if (c->eff_n_temps || c->eff_n_loads) {
        if (c->eff_n_temps < 1 || c->eff_n_temps > AM_EST_EFF_TEMPS_MAX || c->eff_n_loads < 1 ||
            c->eff_n_loads > AM_EST_EFF_LOADS_MAX)
            return AM_ERR_PARAM;
        for (i = 0; i < c->eff_n_temps; i++) {
            if (!est_finite(c->eff_temps_C[i]) || (i > 0 && !(c->eff_temps_C[i] > c->eff_temps_C[i - 1])))
                return AM_ERR_PARAM;
            for (j = 0; j < c->eff_n_loads; j++)
                if (!est_eta_ok(c->eff_forward[i][j]) || !est_eta_ok(c->eff_reverse[i][j])) return AM_ERR_PARAM;
        }
        for (j = 0; j < c->eff_n_loads; j++)
            if (!est_finite(c->eff_loads_Nm[j]) || (j > 0 && !(c->eff_loads_Nm[j] > c->eff_loads_Nm[j - 1])))
                return AM_ERR_PARAM;
    }
    return AM_OK;
}

/* ---------------------------------------------------------------- model */
void am_est_config_default(am_est_config *cfg)
{
    if (!cfg) return;
    cfg->svf_hz = 40.0;
    cfg->svf_zeta = 0.8;
    cfg->output_lp_hz = 100.0;
    cfg->dither_lp_hz = 5.0;
    cfg->stick_vel_rad_s = 0.01;
    cfg->ambiguous_vel_factor = 2.0;
    cfg->reversal_hold_s = 0.02;
    cfg->w_eps_rad_s = 0.003;
    cfg->power_eps_W = 0.02;
    cfg->bearing_w_eps_rad_s = 0.01;
    cfg->confidence_band_frac_rated = 0.10;
}

double am_est_kt_at(const am_est_calibration *cal, double temp_C, double iq_A)
{
    double kt = cal->kt_Nm_per_A * (1.0 + cal->kt_temp_coeff_per_K * (temp_C - cal->kt_ref_temp_C));
    if (cal->kt_sat_current_A > 0.0) {
        double x = iq_A / cal->kt_sat_current_A;
        kt *= 1.0 - cal->kt_sat_frac * x * x;
    }
    return kt;
}

static double est_interp(double x, const double *xs, const double *ys, uint32_t n)
{
    uint32_t k;
    if (x <= xs[0]) return ys[0];
    for (k = 0; k + 1 < n; k++)
        if (x <= xs[k + 1]) return ys[k] + (ys[k + 1] - ys[k]) * (x - xs[k]) / (xs[k + 1] - xs[k]);
    return ys[n - 1];
}

static double est_breakaway_at(const am_est_calibration *cal, double temp_C)
{
    double ct = 1.0 + cal->coulomb_temp_coeff_per_K * (cal->friction_ref_temp_C - temp_C);
    if (cal->friction_map_n) return cal->friction_map_Nm[0] * ct;
    return cal->breakaway_Nm * ct;
}

double am_est_friction_at(const am_est_calibration *cal, double w, double temp_C, double w_eps)
{
    double dT = cal->friction_ref_temp_C - temp_C;
    double ct = 1.0 + cal->coulomb_temp_coeff_per_K * dT;
    double sg = tanh(w / w_eps);
    double tc, ts, b, x;
    if (cal->friction_map_n)
        return est_interp(fabs(w), cal->friction_map_w_rad_s, cal->friction_map_Nm, cal->friction_map_n) * ct * sg;
    tc = cal->coulomb_Nm * ct;
    ts = cal->breakaway_Nm * ct;
    b = cal->viscous_Nm_s_per_rad * exp(cal->viscous_temp_exp_per_K * dT);
    x = w / cal->stribeck_vel_rad_s;
    return (tc + (ts - tc) * exp(-x * x)) * sg + b * w;
}

static void est_axis(double v, const double *xs, uint32_t n, uint32_t *i0, uint32_t *i1, double *f)
{
    uint32_t k;
    *i0 = *i1 = 0;
    *f = 0.0;
    if (n == 1 || v <= xs[0]) return;
    if (v >= xs[n - 1]) {
        *i0 = *i1 = n - 1;
        return;
    }
    for (k = 0; k + 1 < n; k++) {
        if (v <= xs[k + 1]) {
            *i0 = k;
            *i1 = k + 1;
            *f = (v - xs[k]) / (xs[k + 1] - xs[k]);
            return;
        }
    }
    *i0 = *i1 = n - 1;
}

void am_est_efficiency_at(const am_est_calibration *cal, double temp_C, double load_Nm, double *eta_forward,
                          double *eta_reverse)
{
    uint32_t i0, i1, j0, j1;
    double ft, fl, a, b;
    if (cal->eff_n_temps == 0 || cal->eff_n_loads == 0) {
        *eta_forward = cal->eta_forward;
        *eta_reverse = cal->eta_reverse;
        return;
    }
    est_axis(temp_C, cal->eff_temps_C, cal->eff_n_temps, &i0, &i1, &ft);
    est_axis(fabs(load_Nm), cal->eff_loads_Nm, cal->eff_n_loads, &j0, &j1, &fl);
    a = cal->eff_forward[i0][j0] + (cal->eff_forward[i0][j1] - cal->eff_forward[i0][j0]) * fl;
    b = cal->eff_forward[i1][j0] + (cal->eff_forward[i1][j1] - cal->eff_forward[i1][j0]) * fl;
    *eta_forward = a + (b - a) * ft;
    a = cal->eff_reverse[i0][j0] + (cal->eff_reverse[i0][j1] - cal->eff_reverse[i0][j0]) * fl;
    b = cal->eff_reverse[i1][j0] + (cal->eff_reverse[i1][j1] - cal->eff_reverse[i1][j0]) * fl;
    *eta_reverse = a + (b - a) * ft;
}

static double est_gear(double x, double w, double ef, double er, double p_eps)
{
    double s = 0.5 * (1.0 + tanh(x * w / p_eps));
    return s * x * ef + (1.0 - s) * x / er;
}

double am_est_stick_band(const am_est_calibration *cal, double tau_m, double temp_C)
{
    double ef, er, ts;
    am_est_efficiency_at(cal, temp_C, fabs(tau_m), &ef, &er);
    ts = est_breakaway_at(cal, temp_C);
    return 0.5 * fabs(tau_m) * (1.0 / er - ef) + 0.5 * ts * (1.0 / er + ef);
}

static double est_confidence(const am_est_calibration *cal, const am_est_config *cfg, double bound)
{
    double band = cfg->confidence_band_frac_rated * cal->rated_torque_Nm;
    return band > 0 ? est_maxf(0.0, est_minf(1.0, 1.0 - bound / band)) : 0.0;
}

static double est_bound(const am_est_calibration *cal, double tau_m, double f, double tau_out, double j_term,
                        double temp_C, double unc_T, uint32_t q)
{
    double a = cal->unc_kt_rel * tau_m;
    double b = cal->kt_temp_coeff_per_K * unc_T * tau_m;
    double c = cal->unc_friction_rel * f;
    double d = cal->unc_eta_abs * tau_out;
    double e = cal->unc_inertia_rel * j_term;
    double mov = sqrt(a * a + b * b + c * c + d * d + e * e);
    if (q & AM_EST_Q_STICK) return mov + am_est_stick_band(cal, tau_m, temp_C);
    if (q & AM_EST_Q_DITHER) return mov + cal->unc_friction_rel * est_breakaway_at(cal, temp_C);
    if (q & AM_EST_Q_AMBIGUOUS) return mov + 0.5 * am_est_stick_band(cal, tau_m, temp_C);
    return mov;
}

am_status am_est_current_model(const am_est_calibration *cal, const am_est_config *cfg_in, double iq_A,
                               double qd, double qdd, double temp_C, bool dither, am_est_torque *out)
{
    am_est_config dc;
    const am_est_config *cfg = cfg_in;
    uint32_t q = AM_EST_Q_OK;
    double unc_T, kt, tau_m, f, drive, ef, er, brg, j_term, aw;
    if (!cal || !out) return AM_ERR_PARAM;
    if (!cfg) {
        am_est_config_default(&dc);
        cfg = &dc;
    }
    unc_T = cal->unc_temp_C;
    if (!est_finite(temp_C)) {
        temp_C = cal->kt_ref_temp_C;
        unc_T = NO_TEMP_UNC_C;
        q |= AM_EST_Q_NO_TEMPERATURE;
    }
    kt = am_est_kt_at(cal, temp_C, iq_A);
    tau_m = cal->gear_ratio * kt * iq_A;
    f = am_est_friction_at(cal, qd, temp_C, cfg->w_eps_rad_s);
    drive = tau_m - f;
    am_est_efficiency_at(cal, temp_C, fabs(drive), &ef, &er);
    brg = cal->bearing_Nm * tanh(qd / cfg->bearing_w_eps_rad_s);
    j_term = cal->reflected_inertia_kgm2 * qdd;
    memset(out, 0, sizeof *out);
    out->tau_drive_Nm = est_gear(drive, qd, ef, er, cfg->power_eps_W) - brg;
    out->tau_out_Nm = est_gear(drive - j_term, qd, ef, er, cfg->power_eps_W) - brg;
    out->tau_motor_Nm = tau_m;
    out->friction_Nm = f;
    out->qd_rad_s = qd;
    out->qdd_rad_s2 = qdd;
    if (cal->kt_sat_current_A > 0.0 && fabs(iq_A) > cal->kt_sat_current_A) q |= AM_EST_Q_SATURATED;
    aw = fabs(qd);
    if (aw < cfg->stick_vel_rad_s)
        q |= dither ? AM_EST_Q_DITHER : AM_EST_Q_STICK;
    else if (aw < cfg->ambiguous_vel_factor * cal->stribeck_vel_rad_s && cal->friction_map_n == 0)
        q |= AM_EST_Q_AMBIGUOUS;
    out->bound_Nm = est_bound(cal, tau_m, f, out->tau_out_Nm, j_term, temp_C, unc_T, q);
    out->confidence = est_confidence(cal, cfg, out->bound_Nm);
    out->quality = q;
    return AM_OK;
}

/* ---------------------------------------------------------------- stateful estimator */
am_status am_est_init(am_est *e, const am_est_calibration *cal, const am_est_config *cfg)
{
    if (!e || !cal) return AM_ERR_PARAM;
    if (am_est_calibration_check(cal) != AM_OK) return AM_ERR_PARAM;
    memset(e, 0, sizeof *e);
    e->cal = *cal;
    if (cfg)
        e->cfg = *cfg;
    else
        am_est_config_default(&e->cfg);
    am_est_reset(e);
    return AM_OK;
}

void am_est_reset(am_est *e)
{
    if (!e) return;
    e->initialised = false;
    e->p = e->v = e->a = e->tau_f = e->tau_d = 0.0;
    e->since_stick = HUGE_VAL;
}

am_status am_est_update(am_est *e, const am_est_input *in, am_est_torque *out)
{
    const am_est_config *cfg;
    double wn, fc;
    bool first = false;
    if (!e || !in || !out) return AM_ERR_PARAM;
    cfg = &e->cfg;
    if (!(est_finite(in->dt_s) && est_finite(in->q_rad) && est_finite(in->iq_A) && in->dt_s > 0.0)) {
        memset(out, 0, sizeof *out);
        out->tau_out_Nm = e->tau_f;
        out->tau_drive_Nm = e->tau_d;
        out->qd_rad_s = e->v;
        out->qdd_rad_s2 = e->a;
        out->bound_Nm = HUGE_VAL;
        out->confidence = 0.0;
        out->quality = AM_EST_Q_INVALID;
        return AM_OK;
    }
    if (!e->initialised) {
        e->p = in->q_rad;
        e->v = e->a = 0.0;
        e->initialised = true;
        first = true;
    }
    wn = EST_TWO_PI * cfg->svf_hz;
    e->a = wn * wn * (in->q_rad - e->p) - 2.0 * cfg->svf_zeta * wn * e->v;
    e->v += e->a * in->dt_s;
    e->p += e->v * in->dt_s;
    am_est_current_model(&e->cal, cfg, in->iq_A, e->v, e->a, in->temp_C, in->dither_active, out);
    if (out->quality & (AM_EST_Q_STICK | AM_EST_Q_DITHER)) {
        e->since_stick = 0.0;
    } else {
        e->since_stick += in->dt_s;
        if (e->since_stick < cfg->reversal_hold_s && !(out->quality & AM_EST_Q_AMBIGUOUS)) {
            double t_used = est_finite(in->temp_C) ? in->temp_C : e->cal.kt_ref_temp_C;
            out->quality |= AM_EST_Q_AMBIGUOUS;
            out->bound_Nm += 0.5 * am_est_stick_band(&e->cal, out->tau_motor_Nm, t_used);
            out->confidence = est_confidence(&e->cal, cfg, out->bound_Nm);
        }
    }
    fc = in->dither_active ? cfg->dither_lp_hz : cfg->output_lp_hz;
    if (first || fc <= 0.0) {
        e->tau_f = out->tau_out_Nm;
        e->tau_d = out->tau_drive_Nm;
    } else {
        double k = 1.0 - exp(-EST_TWO_PI * fc * in->dt_s);
        e->tau_f += k * (out->tau_out_Nm - e->tau_f);
        e->tau_d += k * (out->tau_drive_Nm - e->tau_d);
    }
    out->tau_out_Nm = e->tau_f;
    out->tau_drive_Nm = e->tau_d;
    if (in->saturated) out->quality |= AM_EST_Q_SATURATED;
    if (!in->valid) {
        out->quality |= AM_EST_Q_INVALID;
        out->confidence = 0.0;
    }
    return AM_OK;
}

am_status am_est_input_from_telemetry(const am_telemetry *t, double dt_s, am_est_input *in)
{
    if (!t || !in) return AM_ERR_PARAM;
    memset(in, 0, sizeof *in);
    in->dt_s = dt_s;
    in->q_rad = t->q_rad;
    in->iq_A = t->iq_A;
    in->temp_C = AM_NAN;
    if (t->extended_valid) {
        if (t->t_magnet_C != 0.0f)
            in->temp_C = t->t_magnet_C;
        else if (t->t_winding_C != 0.0f)
            in->temp_C = t->t_winding_C;
    }
    in->valid = true;
    return AM_OK;
}

double am_est_dither_amplitude_max(const am_est_calibration *cal)
{
    if (!cal) return 0.0;
    return est_minf(DITHER_MAX_OF_BREAKAWAY * cal->breakaway_Nm, DITHER_MAX_FRAC_RATED * cal->rated_torque_Nm);
}

double am_est_dither_torque(const am_est_calibration *cal, double amplitude_Nm, double freq_Hz, double t_s)
{
    double a;
    if (!cal) return 0.0;
    a = est_maxf(0.0, est_minf(amplitude_Nm, am_est_dither_amplitude_max(cal)));
    return a * sin(EST_TWO_PI * freq_Hz * t_s);
}

/* ---------------------------------------------------------------- collision observer */
void am_collision_config_default(am_collision_config *cfg)
{
    if (!cfg) return;
    cfg->inertia_kgm2 = 0.0;
    cfg->gain_hz = 30.0;
    cfg->threshold_Nm = HUGE_VAL;
    cfg->stick_scale = 2.0;
    cfg->release_ratio = 0.5;
    cfg->min_hold_s = 0.05;
}

am_status am_collision_init(am_collision *c, const am_collision_config *cfg)
{
    if (!c || !cfg) return AM_ERR_PARAM;
    if (!(est_finite(cfg->inertia_kgm2) && cfg->inertia_kgm2 > 0.0 && est_finite(cfg->gain_hz) && cfg->gain_hz > 0.0 &&
          cfg->threshold_Nm >= 0.0 && est_finite(cfg->stick_scale) && cfg->stick_scale > 0.0 &&
          est_finite(cfg->release_ratio) && cfg->release_ratio >= 0.0 && est_finite(cfg->min_hold_s)))
        return AM_ERR_PARAM;
    memset(c, 0, sizeof *c);
    c->cfg = *cfg;
    am_collision_reset(c);
    return AM_OK;
}

void am_collision_reset(am_collision *c)
{
    if (!c) return;
    c->initialised = false;
    c->p0 = c->integ = c->r = c->hold = c->t = 0.0;
    c->active = false;
}

am_status am_collision_update(am_collision *c, double dt, double qd, double tau_drive, double tau_known,
                              uint32_t quality, am_collision_output *out)
{
    const am_collision_config *k;
    double th;
    bool onset = false;
    if (!c || !out) return AM_ERR_PARAM;
    if (!(est_finite(dt) && est_finite(qd) && est_finite(tau_drive) && est_finite(tau_known))) return AM_ERR_PARAM;
    k = &c->cfg;
    if (!c->initialised) {
        c->p0 = k->inertia_kgm2 * qd;
        c->initialised = true;
    }
    c->t += dt;
    c->integ += (tau_drive + tau_known + c->r) * dt;
    c->r = EST_TWO_PI * k->gain_hz * (k->inertia_kgm2 * qd - c->p0 - c->integ);
    th = k->threshold_Nm * ((quality & (AM_EST_Q_STICK | AM_EST_Q_AMBIGUOUS)) ? k->stick_scale : 1.0);
    if (!c->active) {
        if (fabs(c->r) > th) {
            c->active = true;
            onset = true;
            c->hold = 0.0;
            c->events++;
        }
    } else {
        if (fabs(c->r) < k->release_ratio * th) {
            c->hold += dt;
            if (c->hold >= k->min_hold_s) c->active = false;
        } else {
            c->hold = 0.0;
        }
    }
    out->residual_Nm = c->r;
    out->threshold_Nm = th;
    out->active = c->active;
    out->onset = onset;
    out->t_s = c->t;
    return AM_OK;
}

double am_collision_calibrate_threshold(const double *r, size_t n, double margin)
{
    size_t i;
    double m = 0.0;
    if (!r) return 0.0;
    for (i = 0; i < n; i++)
        if (est_finite(r[i])) m = est_maxf(m, fabs(r[i]));
    return margin * m;
}
