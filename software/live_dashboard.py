"""
iCAN Smart Insole — v8.7 (Two-Column UI & Live CSV Logging)
========================
"""

import sys, asyncio, time, math, threading
import pygame
import numpy as np
from collections import deque

from bleak import BleakScanner, BleakClient

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
ALLOWED_NAMES      = ["XIAO_Smart_Shoe", "nRF52_DLE_Data"]
UART_TX_CHAR_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
IMG_PATH           = "../assets/Insole_scale_v5.png"
TARGET_MAC_ADDRESS = "CA:11:AF:57:6F:3E" #adjust according to your MCU's MAC

TARGET_HZ = 100

ZUPT_ACCEL_SIGMA  = 0.50       
ZUPT_GYRO_SIGMA   = 0.15       
ZUPT_WINDOW       = 5          
ZUPT_CHI2_THRESH  = 12.59      
ZUPT_IMU_VETO_MULT = 5.0       

ZUPT_PRESSURE_DELTA_THRESH = 15  
ZUPT_MIN_ACTIVE_SENSORS    = 2   
ZUPT_MIN_TOTAL_LOAD        = 10  

# --- Fall Detection --- #Feel Free to modify, still needs fine tuning
FALL_IMPACT_G           = 4.8    
FALL_FREE_FALL_G        = 2.5    
FALL_FREE_FALL_TTL      = 1.0    
FALL_IMPACT_WINDOW      = 0.15   
FALL_POST_WINDOW        = 2.2    
FALL_MPR_SPIKE_SENSORS  = 5      
FALL_MPR_SPIKE_LOAD     = 1200   
FALL_MPR_SENSOR_THRESH  = 60     
FALL_MPR_POST_LOAD      = 200    
FALL_MPR_SLAM_SILENCE_CONFIRM = 0.20  

FALL_MPR_GAIT_LOAD_DEFAULT = 200   
FALL_MPR_GAIT_LOAD      = 200    
FALL_MPR_GAIT_TIME      = 2.5    
FALL_MPR_SILENCE_LOAD   = 150    
FALL_MPR_ABRUPT_MS      = 200    
FALL_MPR_SILENCE_HOLD   = 0.15   
FALL_MPR_SILENCE_MAX    = 4.0    
FALL_MPR_PROLONG_S      = 2.5    
FALL_MPR_STILL_LA       = 0.5    
FALL_MPR_STILL_GYRO     = 0.15   
FALL_MPR_BARO_DROP      = 0.15   
FALL_MPR_TILT_DEG       = 40.0   

FALL_LOADED_TILT_DEG    = 55.0   
FALL_LOADED_TILT_HOLD   = 0.4    
FALL_LOADED_TILT_GYRO   = 3.5    
FALL_LOADED_MIN_LOAD    = 80     
FALL_LOADED_TILT_EXTREME_DEG  = 75.0   
FALL_LOADED_TILT_EXTREME_HOLD = 0.25   
FALL_LOADED_TILT_EXTREME_WEIGHT = 1.8  
FALL_LOADED_TILT_WALK_GYRO = 1.5  

FALL_BARO_DROP_THRESH   = 0.20   
FALL_BARO_WINDOW        = 1.0    
FALL_VOTE_THRESHOLD     = 1.8    
FALL_ALERT_HOLD         = 6.0    

UNWORN_LOAD_THRESH      = 50     

VEL_DECAY_SWING   = 0.985
MAX_PATH_POINTS   = 3000

VEL_MA_WINDOW     = 10           
YAW_MA_WINDOW     = 17           
POS_MA_WINDOW     = 6            
PATH_MIN_STEP_M   = 0.005

ZUPT_STANCE_CONFIRM_WINDOW = 3   
ZUPT_SWING_CONFIRM_WINDOW  = 1   
ZUPT_LONG_SIT_FREEZE_S     = 8.0 

SCREEN_W, SCREEN_H = 1400, 900

# ─────────────────────────────────────────────────────────────────────────────
# SHARED STATE
# ─────────────────────────────────────────────────────────────────────────────
class SharedState:
    def __init__(self):
        self.lock          = threading.Lock()
        self.q             = np.array([1.0, 0.0, 0.0, 0.0])
        self.la            = np.zeros(3)
        self.gyro          = np.zeros(3)
        self.mpr           = np.zeros(10, dtype=np.float32)
        self.on_ground     = False
        self.altitude      = 0.0
        self.calib         = [0, 0, 0, 0]
        self.battery_pct   = -1        
        self.connected     = False
        self.packet_count  = 0
        self.last_packet_t = 0.0
        self._ble_buf      = ""
        
        # Stability Tracking
        self.seq_last      = -1
        self.seq_drops     = 0
        self.seq_total     = 0
        self.stability_pct = 100.0

        # CSV Logging 
        self.is_csv_logging = False
        self.csv_file       = None

state = SharedState()

# ─────────────────────────────────────────────────────────────────────────────
# JITTER BUFFER 
# ─────────────────────────────────────────────────────────────────────────────
class JitterBuffer:
    def __init__(self, target_hz=TARGET_HZ, max_lag_s=0.2):
        self._buf     = deque(maxlen=64)
        self._dt      = 1.0 / target_hz
        self._next_t  = None
        self._max_lag = max_lag_s
        self._lock    = threading.Lock()

    def push(self, snapshot: dict):
        with self._lock:
            self._buf.append(snapshot)
            if self._next_t is None:
                self._next_t = snapshot['t']

    def get_samples(self):
        out = []
        with self._lock:
            if len(self._buf) < 2 or self._next_t is None:
                return out
            now = time.perf_counter()
            if now - self._next_t > self._max_lag:
                self._next_t = now - self._max_lag
            while self._next_t <= now:
                s = self._interpolate(self._next_t)
                if s is None:
                    break
                out.append(s)
                self._next_t += self._dt
        return out

    def _interpolate(self, t):
        buf = self._buf
        if len(buf) < 2:
            return None
        before = after = None
        for i in range(len(buf) - 1):
            if buf[i]['t'] <= t <= buf[i+1]['t']:
                before, after = buf[i], buf[i+1]
                break
        if before is None:
            if t < buf[0]['t']:
                return None
            before = after = buf[-1]
        if before is after:
            return dict(before)

        span  = after['t'] - before['t']
        alpha = (t - before['t']) / span if span > 1e-9 else 0.0

        q0, q1 = before['q'], after['q']
        dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
        if dot < 0:
            q1 = -q1; dot = -dot
        if dot > 0.9995:
            qi = q0 + alpha * (q1 - q0)
            qi /= np.linalg.norm(qi)
        else:
            th0 = math.acos(dot)
            th  = th0 * alpha
            qi  = (math.sin(th0 - th) * q0 + math.sin(th) * q1) / math.sin(th0)

        return {
            't':         t,
            'q':         qi,
            'la':        before['la']   + alpha * (after['la']   - before['la']),
            'gyro':      before['gyro'] + alpha * (after['gyro'] - before['gyro']),
            'mpr':       before['mpr']  + alpha * (after['mpr']  - before['mpr']),
            'on_ground': after['on_ground'],
            'altitude':  before['altitude'] + alpha * (after['altitude'] - before['altitude']),
        }

jitter_buf = JitterBuffer()

# ─────────────────────────────────────────────────────────────────────────────
# BLE PARSING
# ─────────────────────────────────────────────────────────────────────────────
def _parse_f_packet(vals):
    if len(vals) < 22:
        return
    try:
        q  = np.array([float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])])
        nq = np.linalg.norm(q)
        if nq < 0.5:
            return
        q /= nq

        la   = np.array([float(vals[4]),  float(vals[5]),  float(vals[6])])
        gyro = np.array([float(vals[7]),  float(vals[8]),  float(vals[9])])
        mpr  = np.array([float(vals[10+i]) for i in range(10)], dtype=np.float32)
        gnd  = bool(int(vals[20]))
        alt  = float(vals[21])

        bat = -1.0
        if len(vals) >= 23:
            try: bat = float(vals[22])
            except ValueError: pass
            
        seq = -1
        if len(vals) >= 24:
            try: seq = int(float(vals[23]))
            except ValueError: pass

        snap = {
            't':         time.perf_counter(),
            'q':         q,
            'la':        la,
            'gyro':      gyro,
            'mpr':       mpr,
            'on_ground': gnd,
            'altitude':  alt,
        }
        jitter_buf.push(snap)

    

        with state.lock:
            state.q             = q
            state.la            = la
            state.gyro          = gyro
            state.mpr           = mpr
            state.on_ground     = gnd
            state.altitude      = alt
            state.packet_count += 1
            state.last_packet_t = snap['t']
            if bat >= 0:
                state.battery_pct = bat

            # Evaluate Sequence & Stability
            if seq >= 0:
                if state.seq_last >= 0:
                    diff = seq - state.seq_last
                    if diff > 1:
                        state.seq_drops += (diff - 1)
                    elif diff < 0:
                        state.seq_drops = 0
                        state.seq_total = 0
                state.seq_last = seq
                state.seq_total += 1
                
                total_expected = state.seq_total + state.seq_drops
                if total_expected > 0:
                    state.stability_pct = (state.seq_total / total_expected) * 100.0

           # Live CSV Logging
            if state.is_csv_logging and state.csv_file:
                try:
                    real_time = time.time()
                    row = (f"{real_time:.3f},{snap['t']:.3f},{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f},"
                           f"{la[0]:.4f},{la[1]:.4f},{la[2]:.4f},"
                           f"{gyro[0]:.4f},{gyro[1]:.4f},{gyro[2]:.4f},")
                    row += ",".join(f"{v:.0f}" for v in mpr)
                    # Change {bat} to {bat:.2f}
                    row += f",{int(gnd)},{alt:.3f},{bat:.2f},{seq},{state.stability_pct:.1f}\n"
                    state.csv_file.write(row)
                    
                    state.csv_file.flush()  # <--- ADD THIS LINE HERE
                    
                except Exception:
                    pass

    except Exception as e:
        pass

def notification_handler(sender, data):
    try:
        text = data.decode('utf-8', errors='ignore')
        with state.lock:
            state._ble_buf += text
            buf = state._ble_buf

        if '\n' not in buf:
            return

        lines = buf.split('\n')
        with state.lock:
            state._ble_buf = lines[-1]

        for line in lines[:-1]:
            line = line.strip()
            if not line:
                continue

            if line.startswith("F:"):
                _parse_f_packet(line[2:].split(','))
            elif line.startswith("C:"):
                vals = line[2:].split(',')
                if len(vals) == 4:
                    with state.lock:
                        state.calib = [int(v) for v in vals]
            elif line.startswith("B:"):
                try:
                    pct = int(float(line[2:].strip()))
                    with state.lock:
                        state.battery_pct = pct
                except Exception as e:
                    pass
    except Exception as e:
        print(f"[BLE CRITICAL ERROR] {e}")

async def ble_loop():
    while True:
        print(f"[BLE] Connecting to {TARGET_MAC_ADDRESS}...")
        try:
            device = await BleakScanner.find_device_by_address(TARGET_MAC_ADDRESS, timeout=10.0)
            if not device:
                print("[BLE] Device not found. Retrying...")
                await asyncio.sleep(3)
                continue
            async with BleakClient(device, timeout=15.0) as client:
                state.connected = True
                print("[BLE] CONNECTED.")
                await client.start_notify(UART_TX_CHAR_UUID, notification_handler)
                while client.is_connected:
                    await asyncio.sleep(0.2)
        except Exception as e:
            print(f"[BLE] Error: {e}")
        finally:
            state.connected = False
            print("[BLE] Disconnected. Reconnecting...")
            await asyncio.sleep(2)

def start_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_loop())

# ─────────────────────────────────────────────────────────────────────────────
# QUATERNION UTILITIES
# ─────────────────────────────────────────────────────────────────────────────
def quat_rotate(q, v):
    w, x, y, z = q
    t = 2.0 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat_to_euler(q):
    w, x, y, z = q
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw

def get_tilt_from_upright(q, ref_q_inv=None, tilt_offset=0.0):
    local_up      = np.array([0.0, 0.0, 1.0])
    world_up_shoe = quat_rotate(q, local_up)
    world_up      = np.array([0.0, 0.0, 1.0])
    dot           = float(np.clip(np.dot(world_up_shoe, world_up), -1.0, 1.0))
    raw_tilt      = math.degrees(math.acos(dot))
    return max(0.0, raw_tilt - tilt_offset)

# ─────────────────────────────────────────────────────────────────────────────
# ADAPTIVE ZUPT + ZARU  
# ─────────────────────────────────────────────────────────────────────────────
class AdaptiveZARU:
    HEEL_IDX = [0, 4]
    TOE_IDX  = [1, 5]
    ARCH_IDX = [2, 3, 8, 9]

    STATE_UNWORN  = "UNWORN"
    STATE_SWING   = "SWING"
    STATE_STANCE  = "STANCE"

    def __init__(self):
        self.chi2_thresh = ZUPT_CHI2_THRESH
        self._history    = deque(maxlen=max(ZUPT_STANCE_CONFIRM_WINDOW, ZUPT_SWING_CONFIRM_WINDOW))

        self._sigma_a = ZUPT_ACCEL_SIGMA
        self._sigma_g = ZUPT_GYRO_SIGMA

        self._cold_buf_a   = deque(maxlen=300)
        self._cold_buf_g   = deque(maxlen=300)
        self._cold_done    = False

        self._stance_accel_buf = deque(maxlen=200)
        self._stance_gyro_buf  = deque(maxlen=200)

        self._swing_baseline = np.zeros(10, dtype=np.float32)
        self._baseline_init  = False
        self._baseline_frozen = False   

        self._static_baseline     = np.zeros(10, dtype=np.float32)
        self._static_baseline_set = False  

        self._swing_confirm_count  = 0
        self._stance_confirm_count = 0

        self.motion_state       = self.STATE_SWING  
        self.is_stationary      = False
        self.is_unworn          = False
        self.test_stat          = 0.0
        self.stance_time        = 0.0
        self._stance_start      = None

        self.pressure_contact   = False
        self.imu_quiet          = False
        self.imu_veto           = False
        self.contact_delta      = np.zeros(10, dtype=np.float32)
        self.n_active_sensors   = 0
        self.total_contact_load = 0.0

    def _eval_pressure(self, mpr: np.ndarray):
        if not self._baseline_init:
            self._swing_baseline = mpr.copy().astype(np.float32)
            self._baseline_init  = True

        baseline = (self._static_baseline if self._static_baseline_set else self._swing_baseline)
        delta  = np.maximum(mpr - baseline, 0.0)
        active = int(np.sum(delta > ZUPT_PRESSURE_DELTA_THRESH))
        total  = float(np.sum(delta))

        confirmed = (active >= ZUPT_MIN_ACTIVE_SENSORS and total  >= ZUPT_MIN_TOTAL_LOAD)
        return confirmed, delta, active, total

    def update(self, la_world: np.ndarray, gyro_body: np.ndarray, mpr: np.ndarray, dt: float) -> bool:
        if not self._cold_done:
            for c in la_world: self._cold_buf_a.append(abs(float(c)))
            for c in gyro_body: self._cold_buf_g.append(abs(float(c)))
            if len(self._cold_buf_a) >= 200:
                sa = float(np.std(self._cold_buf_a))
                sg = float(np.std(self._cold_buf_g))
                if sa > 0.005: self._sigma_a = max(ZUPT_ACCEL_SIGMA, sa * 3.0)
                if sg > 0.001: self._sigma_g = max(ZUPT_GYRO_SIGMA,  sg * 3.0)
                self._cold_done = True

        raw_total = float(np.sum(mpr))
        if raw_total < UNWORN_LOAD_THRESH:
            self.is_unworn          = True
            self.is_stationary      = False
            self.motion_state       = self.STATE_UNWORN
            self.pressure_contact   = False
            self.imu_quiet          = False
            self.imu_veto           = False
            self.contact_delta      = np.zeros(10, dtype=np.float32)
            self.n_active_sensors   = 0
            self.total_contact_load = 0.0
            self.test_stat          = 999.0
            self._stance_start      = None
            self.stance_time        = 0.0
            self._swing_confirm_count  = 0
            self._stance_confirm_count = 0
            if self._baseline_init and not self._baseline_frozen:
                ema = self._swing_baseline * 0.99 + mpr * 0.01
                self._swing_baseline = np.minimum(ema, mpr).astype(np.float32)
            return False
        
        self.is_unworn = False

        p_contact, delta, n_active, total_load = self._eval_pressure(mpr)
        self.pressure_contact   = p_contact
        self.contact_delta      = delta
        self.n_active_sensors   = n_active
        self.total_contact_load = total_load

        if not p_contact and not self._baseline_frozen:
            ema = self._swing_baseline * 0.992 + mpr * 0.008
            self._swing_baseline = np.minimum(ema, mpr).astype(np.float32)

        if not p_contact:
            self._swing_confirm_count  += 1
            self._stance_confirm_count  = 0
            if self._swing_confirm_count >= ZUPT_SWING_CONFIRM_WINDOW:
                self.imu_quiet     = False
                self.imu_veto      = False
                self.is_stationary = False
                self.motion_state  = self.STATE_SWING
                self._stance_start = None
                self.stance_time   = 0.0
                self.test_stat     = 999.0
                self._baseline_frozen = False  
            return False

        self._swing_confirm_count = 0

        T = (float(np.sum((la_world  / self._sigma_a) ** 2)) + float(np.sum((gyro_body / self._sigma_g) ** 2)))
        self.test_stat = T
        self.imu_quiet = T < self.chi2_thresh

        veto_thresh    = self.chi2_thresh * ZUPT_IMU_VETO_MULT
        self.imu_veto  = T > veto_thresh

        not_vetoed = not self.imu_veto
        self._stance_confirm_count = (self._stance_confirm_count + 1) if not_vetoed else 0
        self.is_stationary = self._stance_confirm_count >= ZUPT_STANCE_CONFIRM_WINDOW
        self.motion_state  = self.STATE_STANCE if self.is_stationary else self.STATE_SWING

        if self.is_stationary:
            for c in la_world: self._stance_accel_buf.append(abs(float(c)))
            for c in gyro_body: self._stance_gyro_buf.append(abs(float(c)))
            n = len(self._stance_accel_buf)
            if n >= 100 and n % 100 == 0:
                sa = float(np.std(self._stance_accel_buf))
                sg = float(np.std(self._stance_gyro_buf))
                if sa > 0.005: self._sigma_a = max(ZUPT_ACCEL_SIGMA, sa * 3.0)
                if sg > 0.001: self._sigma_g = max(ZUPT_GYRO_SIGMA,  sg * 3.0)

            if self._stance_start is None: self._stance_start = time.perf_counter()
            self.stance_time = time.perf_counter() - self._stance_start
            self._baseline_frozen = (self.stance_time > ZUPT_LONG_SIT_FREEZE_S)
        else:
            self._stance_start = None
            self.stance_time   = 0.0

        return self.is_stationary

# ─────────────────────────────────────────────────────────────────────────────
# FALL DETECTOR
# ─────────────────────────────────────────────────────────────────────────────
class FallDetector:
    REPLAY_DURATION_S = 10.0  

    def __init__(self):
        win = int(FALL_IMPACT_WINDOW * TARGET_HZ)
        self._accel_buf = deque(maxlen=max(win, 10))

        self._free_fall_seen = False
        self._free_fall_t    = None
        self._impact_t       = None
        self._imu_event_t    = None

        self._mpr_spike_seen = False
        self._mpr_spike_t    = None

        self._mpr_gait_state      = "IDLE"        
        self._mpr_gait_timer      = 0.0            
        self._mpr_gait_steps      = 0              
        self._mpr_prev_stance     = False          
        self._mpr_load_history    = deque(maxlen=int(FALL_MPR_ABRUPT_MS / 1000 * TARGET_HZ))
        self._mpr_silence_t       = None           
        self._mpr_silence_abrupt  = False          
        self.mpr_flag_tilt        = False
        self.mpr_flag_imu_event   = False
        self.mpr_flag_baro        = False
        self.mpr_flag_prolonged   = False
        self.mpr_flag_still       = False
        self.mpr_red_flags        = 0              
        self.ev_disappear         = False          

        self._alt_buf        = deque(maxlen=int(FALL_BARO_WINDOW * TARGET_HZ))

        self._tilt_high_since      = None   
        self._tilt_quiet_elapsed   = 0.0    
        self._tilt_last_tick       = None   
        self._tilt_gyro_high_since = None   
        self.ev_tilt               = False
        self.ev_tilt_extreme       = False
        self.tilt_deg_live         = 0.0

        self.alert           = False
        self._alert_start_t  = None
        self.confidence      = 0.0

        self.ev_imu          = False
        self.ev_mpr          = False
        self.ev_baro         = False
        self.last_impact_g   = 0.0
        self.last_baro_drop_m = 0.0

        self.ref_q_inv       = np.array([1.0, 0.0, 0.0, 0.0])
        self._last_q         = np.array([1.0, 0.0, 0.0, 0.0])
        self._step_period    = None
        self.tilt_offset     = 0.0  

        replay_len = int(self.REPLAY_DURATION_S * TARGET_HZ)
        self._replay_buf     = deque(maxlen=replay_len)
        self.replay_snapshot = None   
        self.replay_available = False

    def _update_imu_stream(self, la_world, q, now):
        a_mag = float(np.linalg.norm(la_world))          
        a_g   = a_mag / 9.81                              
        self._accel_buf.append(a_g)
        self.last_impact_g = a_g

        if a_mag < FALL_FREE_FALL_G:
            self._free_fall_seen = True
            self._free_fall_t    = now

        if self._free_fall_seen and self._free_fall_t is not None:
            if now - self._free_fall_t > FALL_FREE_FALL_TTL:
                self._free_fall_seen = False
                self._free_fall_t    = None

        if a_g > FALL_IMPACT_G and self._free_fall_seen:
            self._impact_t       = now
            self._imu_event_t    = now   
            self._free_fall_seen = False
            self._free_fall_t    = None

        if self._impact_t is None:
            self.ev_imu = False
            return

        if now - self._impact_t > FALL_POST_WINDOW:
            self._impact_t = None
            self.ev_imu    = False
            return

        roll, pitch, _ = quat_to_euler(q)
        self.ev_imu = (abs(roll) < 20 or abs(pitch) < 40)

    def _update_mpr_stream(self, mpr, gyro_body, q, now, zupt_stationary):
        total    = float(np.sum(mpr))
        n_active = int(np.sum(mpr > FALL_MPR_SENSOR_THRESH))

        slam_ev = False

        if n_active >= FALL_MPR_SPIKE_SENSORS and total >= FALL_MPR_SPIKE_LOAD:
            self._mpr_spike_seen = True
            self._mpr_spike_t    = now

        if self._mpr_spike_seen:
            if now - self._mpr_spike_t > FALL_POST_WINDOW:
                self._mpr_spike_seen = False
            else:
                elapsed_since_spike = now - self._mpr_spike_t
                if elapsed_since_spike > FALL_MPR_SLAM_SILENCE_CONFIRM:
                    slam_ev = (total < FALL_MPR_POST_LOAD)

        self._mpr_load_history.append(total)

        if zupt_stationary and not self._mpr_prev_stance:
            self._mpr_gait_steps += 1
        self._mpr_prev_stance = zupt_stationary

        if self._mpr_gait_state == "IDLE":
            if total >= FALL_MPR_GAIT_LOAD:
                self._mpr_gait_timer += 1.0 / TARGET_HZ
            established = (self._mpr_gait_timer >= FALL_MPR_GAIT_TIME or self._mpr_gait_steps >= 2)
            if established:
                self._mpr_gait_state = "GAIT"

        elif self._mpr_gait_state == "GAIT":
            if total >= FALL_MPR_GAIT_LOAD:
                self._mpr_gait_timer += 1.0 / TARGET_HZ

            if total < FALL_MPR_SILENCE_LOAD:
                hist = list(self._mpr_load_history)
                was_loaded = any(v >= FALL_MPR_GAIT_LOAD for v in hist)
                self._mpr_silence_abrupt = was_loaded
                self._mpr_silence_t      = now
                self._mpr_gait_state     = "SUSPICIOUS"
                self.mpr_flag_tilt       = False
                self.mpr_flag_imu_event  = False
                self.mpr_flag_baro       = False
                self.mpr_flag_prolonged  = False
                self.mpr_flag_still      = False
                self.mpr_red_flags       = 0
                self.ev_disappear        = False

        elif self._mpr_gait_state == "SUSPICIOUS":
            silence_dur = now - self._mpr_silence_t

            if total >= FALL_MPR_GAIT_LOAD:
                self._mpr_gait_state = "GAIT"
                self.ev_disappear    = False
                return slam_ev

            if silence_dur > FALL_MPR_SILENCE_MAX:
                self._mpr_gait_state  = "IDLE"
                self._mpr_gait_timer  = 0.0
                self._mpr_gait_steps  = 0
                self.ev_disappear     = False
                return slam_ev

            if self._step_period is not None:
                adaptive_hold = min(self._step_period * 1.5, 1.5)  
            else:
                adaptive_hold = FALL_MPR_SILENCE_HOLD
            if silence_dur < adaptive_hold:
                return slam_ev

            tilt_deg = get_tilt_from_upright(q, self.ref_q_inv, self.tilt_offset)
            self.mpr_flag_tilt = (tilt_deg > FALL_MPR_TILT_DEG)

            imu_recent = (self._imu_event_t is not None and abs(now - self._imu_event_t) < FALL_POST_WINDOW + 1.0)
            self.mpr_flag_imu_event = imu_recent

            if len(self._alt_buf) >= 2:
                t0_alt, a0_alt = self._alt_buf[0]
                t1_alt, a1_alt = self._alt_buf[-1]
                drop = a0_alt - a1_alt
                self.mpr_flag_baro = (drop > FALL_MPR_BARO_DROP)

            self.mpr_flag_prolonged = (silence_dur > FALL_MPR_PROLONG_S)

            if self._mpr_silence_abrupt and hasattr(self, '_last_la_mag'):
                gyro_mag = float(np.linalg.norm(gyro_body))
                self.mpr_flag_still = (self._last_la_mag < FALL_MPR_STILL_LA and gyro_mag < FALL_MPR_STILL_GYRO)

            self.mpr_red_flags = sum([
                self.mpr_flag_tilt, self.mpr_flag_imu_event, self.mpr_flag_baro, 
                self.mpr_flag_prolonged, self.mpr_flag_still
            ])
            self.ev_disappear = (self._mpr_silence_abrupt and self.mpr_red_flags >= 1)

        return slam_ev

    def _update_baro_stream(self, altitude, now):
        self._alt_buf.append((now, altitude))

        if len(self._alt_buf) < 5:
            self.ev_baro = False
            return

        imu_recent = (self._imu_event_t is not None and now - self._imu_event_t < FALL_POST_WINDOW + 1.0)
        if not imu_recent:
            self.ev_baro          = False
            self.last_baro_drop_m = 0.0
            return

        t0, a0 = self._alt_buf[0]
        t1, a1 = self._alt_buf[-1]
        drop    = a0 - a1   
        self.last_baro_drop_m = drop

        self.ev_baro = (drop > FALL_BARO_DROP_THRESH)

    def _update_tilt_stream(self, q, gyro_body, mpr, now, dr_step_count=0):
        tilt_deg   = get_tilt_from_upright(q, self.ref_q_inv, self.tilt_offset)
        self.tilt_deg_live = tilt_deg
        total_load = float(np.sum(mpr))
        gyro_mag   = float(np.linalg.norm(gyro_body))

        gait_established = (self._mpr_gait_state in ("GAIT", "SUSPICIOUS") or
                            self._mpr_gait_timer >= FALL_MPR_GAIT_TIME or
                            self._mpr_gait_steps >= 2 or
                            dr_step_count >= 2)

        tilt_ok      = tilt_deg  > FALL_LOADED_TILT_DEG
        load_ok      = total_load > FALL_LOADED_MIN_LOAD
        extreme      = tilt_deg  > FALL_LOADED_TILT_EXTREME_DEG
        gyro_moving  = gyro_mag  > FALL_LOADED_TILT_WALK_GYRO   

        if gyro_moving:
            if self._tilt_gyro_high_since is None:
                self._tilt_gyro_high_since = now
        else:
            self._tilt_gyro_high_since = None
        sustained_walk = (self._tilt_gyro_high_since is not None and now - self._tilt_gyro_high_since > 0.4)

        hard_reset = (not load_ok or not tilt_ok or sustained_walk)   

        if hard_reset:
            self._tilt_high_since      = None
            self._tilt_quiet_elapsed   = 0.0
            self._tilt_gyro_high_since = None
            self._tilt_last_tick       = None
            self.ev_tilt               = False
            self.ev_tilt_extreme       = False
            return

        if not gait_established:
            self.ev_tilt         = False
            self.ev_tilt_extreme = False
            return

        dt = (now - self._tilt_last_tick) if self._tilt_last_tick is not None else 0.0
        self._tilt_last_tick = now

        if self._tilt_high_since is None:
            self._tilt_high_since    = now
            self._tilt_quiet_elapsed = 0.0

        if not gyro_moving:
            self._tilt_quiet_elapsed += dt

        quiet_elapsed  = self._tilt_quiet_elapsed
        still_rotating = (gyro_mag > FALL_LOADED_TILT_GYRO)

        if extreme and quiet_elapsed >= FALL_LOADED_TILT_EXTREME_HOLD:
            self.ev_tilt         = not still_rotating
            self.ev_tilt_extreme = not still_rotating
        elif quiet_elapsed >= FALL_LOADED_TILT_HOLD:
            self.ev_tilt         = not still_rotating
            self.ev_tilt_extreme = False
        else:
            self.ev_tilt         = False
            self.ev_tilt_extreme = False

    def update(self, la_world, gyro_body, q, mpr, altitude, on_ground, zupt_stationary=False, snap=None, step_period=None, dr_step_count=0):
        now         = time.perf_counter()
        total_press = float(np.sum(mpr))

        self._step_period = step_period
        self._last_la_mag = float(np.linalg.norm(la_world))
        self._last_q      = q.copy()
        roll, pitch, _    = quat_to_euler(q)
        self._last_roll   = roll
        self._last_pitch  = pitch

        if snap is not None:
            self._replay_buf.append(snap)

        self._update_imu_stream(la_world, q, now)
        self._update_baro_stream(altitude, now)
        slam_ev   = self._update_mpr_stream(mpr, gyro_body, q, now, zupt_stationary)
        self._update_tilt_stream(q, gyro_body, mpr, now, dr_step_count=dr_step_count)
        self.ev_mpr = slam_ev or self.ev_disappear

        mpr_high_conf = ((slam_ev and self.mpr_red_flags >= 1) or (self.ev_disappear and self.mpr_red_flags >= 2))
        mpr_low_conf  = self.ev_mpr and not mpr_high_conf

        tilt_weight = (FALL_LOADED_TILT_EXTREME_WEIGHT if self.ev_tilt_extreme else (1.2 if self.ev_tilt else 0.0))

        weighted = (float(self.ev_imu) * 1.0 + float(mpr_high_conf) * 1.0 + float(mpr_low_conf) * 1.0 + tilt_weight + float(self.ev_baro) * 0.5)
        self.confidence = min(weighted / 2.0, 1.0)

        if weighted >= FALL_VOTE_THRESHOLD and not self.alert:
            self.alert          = True
            self._alert_start_t = now
            self.replay_snapshot  = list(self._replay_buf)
            self.replay_available = True

        if self.alert:
            held      = (now - self._alert_start_t) > FALL_ALERT_HOLD
            recovered = total_press > 80 and on_ground
            if held and recovered:
                self.alert               = False
                self.confidence          = 0.0
                self.ev_imu              = False
                self.ev_mpr              = False
                self.ev_baro             = False
                self.ev_tilt             = False
                self.ev_tilt_extreme     = False
                self._tilt_high_since    = None
                self._tilt_quiet_elapsed = 0.0
                self._tilt_last_tick     = None
                self._tilt_gyro_high_since = None

        return self.alert

    def inject_test_fall(self):
        now = time.perf_counter()
        self.ev_imu           = True
        self.ev_mpr           = True
        self.ev_disappear     = True
        self.ev_baro          = True
        self.ev_tilt          = True
        self.ev_tilt_extreme  = True
        self.confidence       = 1.0
        self.alert            = True
        self._alert_start_t   = now
        self._imu_event_t     = now
        self.last_impact_g    = 8.2
        self.last_baro_drop_m = 0.35
        self.replay_snapshot  = list(self._replay_buf)
        self.replay_available = True

# ─────────────────────────────────────────────────────────────────────────────
# MOVING AVERAGE FILTERS
# ─────────────────────────────────────────────────────────────────────────────
class VectorMA:
    def __init__(self, size: int, dim: int = 3):
        self._buf  = deque(maxlen=size)
        self._dim  = dim
        self._zero = np.zeros(dim)
    def update(self, v: np.ndarray) -> np.ndarray:
        self._buf.append(v.copy())
        if not self._buf: return self._zero.copy()
        return np.mean(self._buf, axis=0)
    def reset(self):
        self._buf.clear()

class CircularMA:
    def __init__(self, size: int):
        self._buf = deque(maxlen=size)
    def update(self, deg: float) -> float:
        self._buf.append(math.radians(deg))
        sin_sum = sum(math.sin(a) for a in self._buf)
        cos_sum = sum(math.cos(a) for a in self._buf)
        return math.degrees(math.atan2(sin_sum, cos_sum))
    def reset(self):
        self._buf.clear()

# ─────────────────────────────────────────────────────────────────────────────
# DEAD-RECKONING ENGINE
# ─────────────────────────────────────────────────────────────────────────────
class DeadReckoning:
    def __init__(self):
        self.pos        = np.zeros(3)
        self.vel        = np.zeros(3)
        self.total_dist = 0.0
        self.path       = deque(maxlen=MAX_PATH_POINTS)
        self.path.append((0.0, 0.0))
        self._last_path_pos = np.zeros(2)

        self.step_count = 0
        self._in_stance = False
        self._step_times    = deque(maxlen=20)   
        self.cadence_spm    = 0.0                 

        self._vel_ma  = VectorMA(VEL_MA_WINDOW, dim=3)
        self._yaw_ma  = CircularMA(YAW_MA_WINDOW)
        self._pos_ma  = VectorMA(POS_MA_WINDOW, dim=2)   
        self.yaw_smooth = 0.0

        self.zaru = AdaptiveZARU()
        self.fall = FallDetector()

    def get_step_period(self):
        if len(self._step_times) < 3: return None
        times = list(self._step_times)
        intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
        return float(np.median(intervals[-5:]))

    def update(self, snap: dict, dt: float):
        q, la, gyro = snap['q'], snap['la'], snap['gyro']
        mpr, on_ground, altitude = snap['mpr'], snap['on_ground'], snap['altitude']

        la_world   = quat_rotate(q, la)
        gyro_world = quat_rotate(q, gyro)

        stationary = self.zaru.update(la_world, gyro_world, mpr, dt)

        _, _, raw_yaw   = quat_to_euler(q)
        self.yaw_smooth = self._yaw_ma.update(raw_yaw)

        now = time.perf_counter()
        if stationary and not self._in_stance:
            self.step_count += 1
            self._step_times.append(now)
            if len(self._step_times) >= 2:
                window_s = self._step_times[-1] - self._step_times[0]
                if window_s > 0: self.cadence_spm = (len(self._step_times) - 1) / window_s * 60.0
        self._in_stance = stationary

        self.fall.update(la_world, gyro, q, mpr, altitude, on_ground,
                         zupt_stationary=stationary, snap=snap,
                         step_period=self.get_step_period(), dr_step_count=self.step_count)

        if stationary or self.zaru.is_unworn:
            self.vel[:] = 0.0
            self._vel_ma.reset()
            self.pos[2] = altitude
        else:
            la_f = la_world.copy()
            la_f[np.abs(la_f) < 0.05] = 0.0
            self.vel += la_f * dt
            self.vel *= VEL_DECAY_SWING
            vel_smooth = self._vel_ma.update(self.vel)

            dp = vel_smooth * dt
            self.pos[0] += dp[0]
            self.pos[1] += dp[1]
            self.pos[2]  = altitude

            dist_2d = math.sqrt(dp[0]**2 + dp[1]**2)
            self.total_dist += dist_2d

            pos_smooth = self._pos_ma.update(self.pos[:2])
            moved = math.sqrt((pos_smooth[0]-self._last_path_pos[0])**2 + (pos_smooth[1]-self._last_path_pos[1])**2)
            if moved > PATH_MIN_STEP_M:
                self.path.append((float(pos_smooth[0]), float(pos_smooth[1])))
                self._last_path_pos[:] = pos_smooth

    def reset(self):
        self.pos[:]     = 0.0
        self.vel[:]     = 0.0
        self.total_dist = 0.0
        self.step_count = 0
        self._in_stance = False
        self._last_path_pos[:] = 0.0
        self._step_times.clear()
        self.cadence_spm = 0.0
        self.path.clear()
        self.path.append((0.0, 0.0))
        self._vel_ma.reset()
        self._yaw_ma.reset()
        self._pos_ma.reset()
        self.yaw_smooth = 0.0
        self.zaru = AdaptiveZARU()
        self.fall = FallDetector()

# ─────────────────────────────────────────────────────────────────────────────
# PYGAME DASHBOARD
# ─────────────────────────────────────────────────────────────────────────────
class Dashboard:
    C = {
        "bg":          (8,   10,  14),
        "panel":       (12,  15,  20),
        "border":      (30,  40,  55),
        "border_hi":   (0,   160, 220),
        "text_dim":    (80,  95,  110),
        "text":        (170, 185, 200),
        "text_bright": (230, 240, 250),
        "accent":      (0,   200, 255),
        "accent2":     (0,   255, 160),
        "warn":        (255, 200, 0),
        "danger":      (255, 50,  50),
        "danger_dim":  (120, 20,  20),
        "stance":      (255, 80,  80),
        "swing":       (0,   255, 140),
        "chart_grid":  (20,  28,  38),
        "ev_on":       (255, 80,  80),
        "ev_off":      (35,  45,  60),
    }

    SENSOR_POSITIONS = [
        (172, 628), (161, 161), (164, 447), (142, 281), (252, 628),
        (224, 125), (212, 273), (279, 161), (239, 446), (279, 279),
    ]

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)
        pygame.display.set_caption("iCAN Smart Insole v8.7 (CSV Logging)")
        import ctypes, platform
        if platform.system() == "Windows":
            hwnd = pygame.display.get_wm_info().get("window")
            if hwnd:
                SW_MAXIMIZE = 3
                ctypes.windll.user32.ShowWindow(hwnd, SW_MAXIMIZE)
        self.clock  = pygame.time.Clock()

        self.fnt_mono_xl  = pygame.font.SysFont("Courier New", 40, bold=True)
        self.fnt_mono_lg  = pygame.font.SysFont("Courier New", 32, bold=True)
        self.fnt_mono_md  = pygame.font.SysFont("Courier New", 16, bold=True)
        self.fnt_mono_sm  = pygame.font.SysFont("Courier New", 12)
        self.fnt_mono_xs  = pygame.font.SysFont("Courier New", 10)

        self.dr            = DeadReckoning()
        self.is_calibrated = False
        self.show_debug    = False
        self._ref_q_inv    = np.array([1.0, 0.0, 0.0, 0.0]) 

        self._last_pkt_count = 0
        self._rate_t         = time.perf_counter()
        self.ble_rate_hz     = 0.0
        self._calib_flash_t  = None   
        self._swing_flash_t  = None   

        self._replay_mode       = False
        self._replay_data       = []
        self._replay_idx        = 0
        self._replay_playing    = False
        self._replay_t          = 0.0
        self._replay_btn_rect   = None 
        self._replay_speed      = 1.0   
        self._replay_last_tick  = 0.0   

        try:
            self.insole_img            = pygame.image.load(IMG_PATH).convert_alpha()
            self.img_w, self.img_h     = self.insole_img.get_size()
        except Exception:
            self.insole_img    = None
            self.img_w, self.img_h = 400, 800

        self.blob_r    = 52
        self.base_blob = self._make_blob(self.blob_r)

        self.chi2_history  = deque(maxlen=300)
        self.vel_history   = deque(maxlen=300)
        self._scanlines    = self._make_scanlines()

        self._fall_overlay_alpha = 0
        self._fall_pulse_t       = 0.0

    @staticmethod
    def _make_blob(r):
        surf = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
        sig  = r / 2.5
        for ri in range(r, 0, -1):
            a = int(255 * math.exp(-(ri**2) / (2*sig**2)))
            pygame.draw.circle(surf, (255, 255, 255, a), (r, r), ri)
        return surf

    def _make_scanlines(self):
        surf = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
        for y in range(0, SCREEN_H, 3):
            pygame.draw.line(surf, (0, 0, 0, 30), (0, y), (SCREEN_W, y))
        return surf

    def _txt(self, text, color, x, y, font=None, anchor="topleft"):
        f = font or self.fnt_mono_sm
        s = f.render(str(text), True, color)
        r = s.get_rect(**{anchor: (x, y)})
        self.screen.blit(s, r)

    def _panel(self, rx, ry, rw, rh, title="", accent=False):
        pygame.draw.rect(self.screen, self.C["panel"],  (rx, ry, rw, rh))
        bc = self.C["border_hi"] if accent else self.C["border"]
        pygame.draw.rect(self.screen, bc, (rx, ry, rw, rh), 1)
        if title:
            pygame.draw.rect(self.screen, (18, 24, 35), (rx+1, ry+1, rw-2, 28))
            pygame.draw.line(self.screen, bc, (rx, ry+28), (rx+rw, ry+28), 1)
            self._txt(title, self.C["accent"], rx+10, ry+7, self.fnt_mono_xs)

    @staticmethod
    def _pressure_color(v):
        v = max(0, min(100, v))
        if v < 20:  return (0,   30,  180)
        if v < 40:  return (0,   120, 255)
        if v < 60:  return (0,   220, 180)
        if v < 80:  return (80,  255, 80)
        if v < 92:  return (255, 220, 0)
        return             (255, 40,  40)

    def _sparkline(self, buf, x, y, w, h, color, max_val=1.0, threshold=None, thr_color=None, fill=False):
        pygame.draw.rect(self.screen, self.C["chart_grid"], (x, y, w, h))
        pygame.draw.rect(self.screen, self.C["border"], (x, y, w, h), 1)
        data = list(buf)
        if len(data) < 2: return
        pts = []
        for i, v in enumerate(data):
            px = x + int(i * w / len(data))
            py = y + h - int(min(v / max_val, 1.0) * h)
            py = max(y, min(y+h, py))
            pts.append((px, py))
        if fill and len(pts) > 1:
            poly = [(pts[0][0], y+h)] + pts + [(pts[-1][0], y+h)]
            fs   = pygame.Surface((w, h), pygame.SRCALPHA)
            adj  = [(p[0]-x, p[1]-y) for p in poly]
            pygame.draw.polygon(fs, (*color, 40), adj)
            self.screen.blit(fs, (x, y))
        if len(pts) > 1:
            pygame.draw.lines(self.screen, color, False, pts, 1)
        if threshold is not None:
            ty = y + h - int(min(threshold / max_val, 1.0) * h)
            ty = max(y, min(y+h, ty))
            pygame.draw.line(self.screen, thr_color or self.C["warn"], (x, ty), (x+w, ty), 1)

    def _draw_calibration(self):
        cx, cy = SCREEN_W//2, SCREEN_H//2
        self._txt("CALIBRATING BNO055", self.C["warn"], cx, cy-120, self.fnt_mono_lg, anchor="center")
        with state.lock: c = list(state.calib)
        labels = ["SYS", "GYRO", "ACCEL", "MAG"]
        hints  = ["Awaiting sensors...", "Rest shoe flat on table", "Tilt 45° · 90° · flip upside-down", "Swing in figure-8 pattern"]
        for i, (lbl, hint, val) in enumerate(zip(labels, hints, c)):
            row_y = cy - 40 + i * 52
            done  = val == 3
            bar_w = int((val / 3) * 180)
            col   = self.C["accent2"] if done else self.C["warn"]
            self._txt(lbl, col, cx-220, row_y, self.fnt_mono_md)
            pygame.draw.rect(self.screen, self.C["border"], (cx-120, row_y+2, 180, 18))
            pygame.draw.rect(self.screen, col, (cx-120, row_y+2, bar_w, 18))
            self._txt(f"{val}/3", self.C["text_bright"], cx+75, row_y, self.fnt_mono_md)
            if not done: self._txt(hint, self.C["text_dim"], cx-220, row_y+22, self.fnt_mono_xs)
        self._txt("Press 'C' to force skip calibration (then 'S' with foot airborne for pressure baseline)", self.C["danger"], cx, cy+220, self.fnt_mono_md, anchor="center")

    def _draw_map(self):
        PX, PY, PW, PH = 0, 0, 520, SCREEN_H
        self._panel(PX, PY, PW, PH, "FOOT TRAJECTORY  (ZUPT+ZARU dead-reckoning)")
        path  = list(self.dr.path)
        all_x = [p[0] for p in path] + [float(self.dr.pos[0])]
        all_y = [p[1] for p in path] + [float(self.dr.pos[1])]
        rx  = max(max(all_x) - min(all_x), 0.05)
        ry  = max(max(all_y) - min(all_y), 0.05)
        cx  = (min(all_x) + max(all_x)) / 2
        cy  = (min(all_y) + max(all_y)) / 2
        scl = min((PW-80)/rx, (PH-80)/ry, 600.0)

        def ws(x, y): return (int(PX + PW//2 + (x-cx)*scl), int(PY + PH//2 - (y-cy)*scl))

        for gv in np.arange(math.floor(min(all_x)*2)/2, math.ceil(max(all_x)*2)/2 + 0.5, 0.5):
            gx = ws(gv, 0)[0]
            if PX+4 < gx < PX+PW-4: pygame.draw.line(self.screen, self.C["chart_grid"], (gx, PY+30), (gx, PY+PH-4), 1)
        for gv in np.arange(math.floor(min(all_y)*2)/2, math.ceil(max(all_y)*2)/2 + 0.5, 0.5):
            gy = ws(0, gv)[1]
            if PY+30 < gy < PY+PH-4: pygame.draw.line(self.screen, self.C["chart_grid"], (PX+4, gy), (PX+PW-4, gy), 1)

        ox, oy = ws(0, 0)
        pygame.draw.line(self.screen, (25,35,50), (ox, PY+30), (ox, PY+PH-4))
        pygame.draw.line(self.screen, (25,35,50), (PX+4, oy), (PX+PW-4, oy))

        if len(path) > 1:
            n = len(path)
            for i in range(n-1):
                frac = i / n
                col  = (0, int(150 + frac*105), int(255 - frac*55))
                pygame.draw.line(self.screen, col, ws(*path[i]), ws(*path[i+1]), 2)

        fx, fy  = ws(float(self.dr.pos[0]), float(self.dr.pos[1]))
        is_stat = self.dr.zaru.is_stationary
        dot_col = self.C["stance"] if is_stat else self.C["swing"]
        pygame.draw.circle(self.screen, dot_col, (fx, fy), 10)
        pygame.draw.circle(self.screen, self.C["text_bright"], (fx, fy), 10, 1)

        yr  = math.radians(self.dr.yaw_smooth)
        ax2 = int(fx - 28 * math.sin(yr))
        ay2 = int(fy - 28 * math.cos(yr))
        pygame.draw.line(self.screen, self.C["warn"], (fx, fy), (ax2, ay2), 2)
        pygame.draw.circle(self.screen, self.C["warn"], (ax2, ay2), 3)

    def _draw_heatmap(self):
        PX, PY, PW, PH = 520, 0, 260, SCREEN_H
        self._panel(PX, PY, PW, PH, "PRESSURE HEATMAP  (% of total)")

        scl = min(220/self.img_w, 820/self.img_h)
        sw, sh = int(self.img_w*scl), int(self.img_h*scl)
        ox = PX + (PW - sw)//2
        oy = PY + 34 + (PH - 34 - sh)//2

        if self.insole_img:
            si = pygame.transform.smoothscale(self.insole_img, (sw, sh))
            self.screen.blit(si, (ox, oy))
        else:
            pygame.draw.ellipse(self.screen, (30,35,45), (ox,oy,sw,sh), 1)

        with state.lock: mpr = state.mpr.copy()
        delta = self.dr.zaru.contact_delta
        total_delta = float(np.sum(delta))
        pct = np.zeros(10, dtype=np.float32)
        if total_delta > 1.0: pct = (delta / total_delta * 100.0).astype(np.float32)

        hs = pygame.Surface((PW, PH), pygame.SRCALPHA)
        for i, val in enumerate(delta):
            if val > 3:
                ox2, oy2 = self.SENSOR_POSITIONS[i]
                px = int(ox2*scl) + ox - PX
                py = int(oy2*scl) + oy
                blob = self.base_blob.copy()
                col  = self._pressure_color(float(pct[i]))
                blob.fill((*col, int((val/100)*220)), special_flags=pygame.BLEND_RGBA_MULT)
                hs.blit(blob, (px-self.blob_r, py-self.blob_r), special_flags=pygame.BLEND_RGBA_ADD)
        self.screen.blit(hs, (PX, 0))

        for i, val in enumerate(delta):
            ox2, oy2 = self.SENSOR_POSITIONS[i]
            px, py   = int(ox2*scl) + ox, int(oy2*scl) + oy
            col = self._pressure_color(float(pct[i])) if val > 3 else self.C["border"]
            pygame.draw.circle(self.screen, col, (px, py), 5)
            if val > 3: self._txt(f"{pct[i]:.0f}%", self.C["text_bright"], px, py-14, self.fnt_mono_xs, anchor="center")

    def _draw_telemetry(self):
        """
        Two-Column Layout specifically designed to fit within 900px height.
        """
        PX, PY, PW = 780, 0, SCREEN_W - 780
        dr = self.dr

        with state.lock:
            q, on_gnd, mpr, calib = (state.q.copy(), state.on_ground, state.mpr.copy(), list(state.calib))
            alt     = state.altitude
            bat_pct = state.battery_pct
            pkt     = state.packet_count
            stab    = state.stability_pct
            drops   = state.seq_drops
            is_log  = state.is_csv_logging

        q_corrected = quat_mul(self._ref_q_inv, q)
        total_press = float(np.sum(mpr))
        is_stat     = dr.zaru.is_stationary
        is_unworn   = dr.zaru.is_unworn

        col1_x, col1_w = PX, 310
        col2_x, col2_w = PX + 320, PW - 320

        # ── COLUMN 1 (Left) ──

        # 1. Motion state
        py = PY
        accent_col = is_stat and not is_unworn
        self._panel(col1_x, py, col1_w, 130, "MOTION STATE", accent=accent_col)
        
        if is_unworn:
            state_label, state_col = "UNWORN", self.C["text_dim"]
        elif is_stat:
            state_label, state_col = "STANCE", self.C["stance"]
        else:
            state_label, state_col = "SWING", self.C["swing"]
        self._txt(state_label, state_col, col1_x + col1_w//2, py+45, self.fnt_mono_lg, anchor="center")

        if bat_pct >= 0:
            # Map the 3.0V - 4.2V range back to 0 - 100% for the UI bar rendering
            calc_pct = int(max(0.0, min(100.0, (bat_pct - 3.0) / (4.2 - 3.0) * 100.0)))
            
            bat_col = (self.C["accent2"] if calc_pct > 40 else self.C["warn"] if calc_pct > 15 else self.C["danger"])
            bx, by = col1_x + col1_w - 70, py + 8
            pygame.draw.rect(self.screen, self.C["border"], (bx, by, 48, 22), 1)
            pygame.draw.rect(self.screen, self.C["border"], (bx+48, by+7, 4, 8))
            fill_w = int(44 * calc_pct / 100)
            if fill_w > 0: pygame.draw.rect(self.screen, bat_col, (bx+2, by+2, fill_w, 18))
            
            # Display the actual voltage underneath
            self._txt(f"{bat_pct:.2f}V", bat_col, bx+24, by+25, self.fnt_mono_xs, anchor="center")
        else:
            self._txt("BAT: N/A", self.C["text_dim"], col1_x + col1_w - 70, py + 8, self.fnt_mono_xs)

        pg_col = self.C["accent2"] if dr.zaru.pressure_contact else self.C["danger"]
        n_act  = dr.zaru.n_active_sensors
        load   = dr.zaru.total_contact_load
        self._txt(f"PRESS: {'CONTACT' if dr.zaru.pressure_contact else 'AIRBORNE'}  {n_act}s Δ={load:.0f}", pg_col, col1_x+16, py+70, self.fnt_mono_xs)

        chi2      = dr.zaru.test_stat
        veto_thr  = ZUPT_CHI2_THRESH * ZUPT_IMU_VETO_MULT
        bar_max   = veto_thr * 1.5
        bar_w     = int(min(chi2 / bar_max, 1.0) * (col1_w-32))
        pygame.draw.rect(self.screen, self.C["border"], (col1_x+16, py+85, col1_w-32, 12))
        bar_col = (self.C["danger"] if dr.zaru.imu_veto else self.C["warn"] if chi2 > ZUPT_CHI2_THRESH else self.C["accent2"])
        pygame.draw.rect(self.screen, bar_col, (col1_x+16, py+85, bar_w, 12))
        vthr_x = col1_x+16 + int((col1_w-32) * (veto_thr / bar_max))
        pygame.draw.line(self.screen, self.C["danger"], (vthr_x, py+82), (vthr_x, py+100), 2)

        frozen_lbl = " [FROZEN]" if dr.zaru._baseline_frozen else ""
        self._txt(f"χ²={chi2:.1f}  σa={dr.zaru._sigma_a:.3f}{frozen_lbl}", self.C["text_dim"], col1_x+16, py+105, self.fnt_mono_xs)

        # 2. Orientation
        py = PY + 140
        tilt_deg = get_tilt_from_upright(q, self.dr.fall.ref_q_inv, self.dr.fall.tilt_offset)
        self._panel(col1_x, py, col1_w, 150, "ORIENTATION")
        qw, qx, qy, qz = float(q_corrected[0]), float(q_corrected[1]), float(q_corrected[2]), float(q_corrected[3])
        for i, (lbl, val) in enumerate([("W", qw), ("X", qx), ("Y", qy), ("Z", qz)]):
            y   = py + 35 + i*20
            identity_val = 1.0 if lbl == "W" else 0.0
            deviation    = abs(val - identity_val)
            col = (self.C["danger"] if deviation > 0.7 else self.C["warn"] if deviation > 0.3 else self.C["accent"])
            self._txt(lbl, self.C["text_dim"], col1_x+16, y)
            self._txt(f"{val:+.4f}", col, col1_x+col1_w-16, y, self.fnt_mono_md, anchor="topright")
            
        tilt_col = (self.C["danger"] if tilt_deg > 60 else self.C["warn"] if tilt_deg > 30 else self.C["accent2"])
        tilt_q   = self.dr.fall._tilt_quiet_elapsed   
        tilt_active = self.dr.fall._tilt_high_since is not None
        tilt_suffix = (f" ★{tilt_q:.2f}s" if self.dr.fall.ev_tilt_extreme else f" ▲{tilt_q:.2f}s" if tilt_active else "")
        self._txt("TILT", self.C["text_dim"], col1_x+16, py + 120)
        self._txt(f"{tilt_deg:.1f}°{tilt_suffix}", tilt_col, col1_x+col1_w-16, py + 120, self.fnt_mono_md, anchor="topright")

        # 3. BLE & Logging
        py = PY + 300
        self._panel(col1_x, py, col1_w, 140, "BLE & LOGGING")
        conn_col = self.C["accent2"] if state.connected else self.C["danger"]
        stab_col = self.C["accent2"] if stab >= 98.0 else self.C["warn"] if stab >= 85.0 else self.C["danger"]
        
        self._txt("● " + ("CONNECTED" if state.connected else "DISCONNECTED"), conn_col, col1_x+16, py+35, self.fnt_mono_md)
        self._txt(f"BLE RATE:  {self.ble_rate_hz:.1f} Hz  (PKT {pkt})", self.C["text_dim"], col1_x+16, py+55, self.fnt_mono_xs)
        self._txt(f"STABILITY: {stab:.1f}% ({drops} dropped)", stab_col, col1_x+16, py+75, self.fnt_mono_xs)
        self._txt(f"CALIB: SYS {calib[0]}  GYR {calib[1]}  ACC {calib[2]}  MAG {calib[3]}", self.C["text_dim"], col1_x+16, py+95, self.fnt_mono_xs)
        
        log_col = self.C["danger"] if is_log else self.C["text_dim"]
        if is_log:
            if int(time.time() * 2) % 2 == 0: log_col = (255, 100, 100)
            log_lbl = "● RECORDING CSV (Press 'L' to Stop)"
        else:
            log_lbl = "○ NOT RECORDING (Press 'L' to Start)"
        self._txt(log_lbl, log_col, col1_x+16, py+115, self.fnt_mono_xs)


        # ── COLUMN 2 (Right) ──

        # 1. Metrics
        py = PY
        self._panel(col2_x, py, col2_w, 210, "METRICS")
        rows = [
            ("DIST",    f"{dr.total_dist:.3f} m",        self.C["accent"]),
            ("STEPS",   str(dr.step_count),              self.C["accent2"]),
            ("CADENCE", f"{dr.cadence_spm:.0f} spm",     (180, 255, 200)),
            ("ELEV",    f"{alt:.3f} m",                  (180, 180, 255)),
            ("LOAD",    f"{int(total_press)}",           self.C["warn"]),
            ("STANCE",  f"{dr.zaru.stance_time:.2f} s",  self.C["text"]),
        ]
        for i, (lbl, val, col) in enumerate(rows):
            y = py + 35 + i*28
            self._txt(lbl, self.C["text_dim"], col2_x+16, y)
            self._txt(val, col, col2_x+col2_w-16, y, self.fnt_mono_md, anchor="topright")

        # 2. Chi² Sparkline
        py = PY + 220
        self._panel(col2_x, py, col2_w, 220, "CHI² (ZUPT+ZARU) — 6-DOF")
        self._sparkline(self.chi2_history, col2_x+8, py+35, col2_w-16, 75, self.C["accent"], max_val=ZUPT_CHI2_THRESH * 2, threshold=ZUPT_CHI2_THRESH, thr_color=self.C["warn"], fill=True)
        self._sparkline(self.vel_history, col2_x+8, py+130, col2_w-16, 75, self.C["accent2"], max_val=2.0, fill=True)


        # ── FALL PANEL (Full Width at Bottom) ──
        # Gives ~450px of room to ensure it never gets cut off
        self._draw_fall_panel(PX, PY + 450, PW, 200)

    def _draw_fall_panel(self, x, y, w, h):
        fall = self.dr.fall
        self._panel(x, y, w, h, "FALL DETECTION", accent=fall.alert)

        if not fall.alert:
            self._txt("NOMINAL", self.C["accent2"], x+w//2, y+80, self.fnt_mono_lg, anchor="center")
        else:
            if int(time.time() * 4) % 2 == 0: pygame.draw.rect(self.screen, (80, 8, 8), (x+1, y+29, w-2, h-30))
            self._txt("⚠  FALL", self.C["text_bright"], x+w//2, y+80, self.fnt_mono_lg, anchor="center")

        labels  = ["IMU", "MPR", "TLT", "TLT★", "BAR"]
        flags   = [fall.ev_imu, fall.ev_mpr, fall.ev_tilt and not fall.ev_tilt_extreme, fall.ev_tilt_extreme, fall.ev_baro]
        
        # Center the indicator dots beautifully inside the panel
        dots_w = len(labels) * 50
        start_x = x + (w - dots_w) // 2 + 25
        dot_y = y + 150
        
        for j, (lbl, flag) in enumerate(zip(labels, flags)):
            col = (self.C["warn"] if lbl == "TLT★" and flag else self.C["ev_on"] if flag else self.C["ev_off"])
            pygame.draw.circle(self.screen, col, (start_x + j*50, dot_y), 8)
            self._txt(lbl, col, start_x + j*50, dot_y+16, self.fnt_mono_sm, anchor="center")

    def _draw_fall_overlay(self):
        fall  = self.dr.fall
        now   = time.perf_counter()

        if fall.alert:
            self._fall_overlay_alpha = min(self._fall_overlay_alpha + 18, 210)
        else:
            self._fall_overlay_alpha = max(self._fall_overlay_alpha - 12, 0)

        if self._fall_overlay_alpha < 5: return

        alpha = self._fall_overlay_alpha

        overlay = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
        pulse = 0.5 + 0.5 * math.sin(now * 5.0)
        base_a = int(alpha * 0.55)
        pulse_a = int(alpha * 0.20 * pulse)
        overlay.fill((120, 0, 0, base_a + pulse_a))
        self.screen.blit(overlay, (0, 0))

        chevron_col = (255, 40, 40, min(alpha, 200))
        chev_size   = 60 + int(20 * pulse)
        corners = [(0, 0), (SCREEN_W, 0), (0, SCREEN_H), (SCREEN_W, SCREEN_H)]
        dirs    = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        chev_surf = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
        for (cx, cy), (dx, dy) in zip(corners, dirs):
            pts = [(cx, cy), (cx + dx * chev_size, cy), (cx, cy + dy * chev_size)]
            pygame.draw.polygon(chev_surf, chevron_col, pts)
            pygame.draw.polygon(chev_surf, (255, 100, 100, min(alpha, 180)), pts, 2)
        self.screen.blit(chev_surf, (0, 0))

        ring_col = (255, int(30 + 40*pulse), int(30 + 40*pulse))
        ring_w   = 3 + int(3 * pulse)
        pygame.draw.rect(self.screen, ring_col, (0, 0, SCREEN_W, SCREEN_H), ring_w)

        card_w, card_h = 540, 350
        card_x = SCREEN_W//2 - card_w//2
        card_y = SCREEN_H//2 - card_h//2

        card_surf = pygame.Surface((card_w, card_h), pygame.SRCALPHA)
        card_surf.fill((20, 0, 0, min(alpha + 40, 240)))
        self.screen.blit(card_surf, (card_x, card_y))

        border_col = (255, int(50 + 80*pulse), 50)
        pygame.draw.rect(self.screen, border_col, (card_x, card_y, card_w, card_h), 2)

        ic_cx, ic_cy = SCREEN_W//2, card_y + 55
        tri_size = 28 + int(6 * pulse)
        pygame.draw.polygon(self.screen, border_col, [(ic_cx, ic_cy - tri_size), (ic_cx - tri_size, ic_cy + tri_size), (ic_cx + tri_size, ic_cy + tri_size)], 3)
        self._txt("!", (255, 255, 255), ic_cx, ic_cy - 8, self.fnt_mono_md, anchor="center")

        self._txt("FALL DETECTED", (255, 60, 60), SCREEN_W//2, card_y + 95, self.fnt_mono_xl, anchor="center")

        conf_w = card_w - 80
        conf_x = card_x + 40
        conf_y = card_y + 145
        pygame.draw.rect(self.screen, (40, 10, 10), (conf_x, conf_y, conf_w, 14))
        filled = int(fall.confidence * conf_w)
        bar_col = (255, int(60 + 100*(1-fall.confidence)), 30)
        pygame.draw.rect(self.screen, bar_col, (conf_x, conf_y, filled, 14))
        self._txt(f"CONFIDENCE  {int(fall.confidence*100)}%", self.C["text_dim"], conf_x, conf_y + 18, self.fnt_mono_xs)

        ev_y = card_y + 185
        self._txt("EVIDENCE:", self.C["text_dim"], card_x + 40, ev_y, self.fnt_mono_xs)

        sensors = [
            ("IMU",  fall.ev_imu,  f"impact {fall.last_impact_g:.1f}g  posture tilt"),
            ("MPR",  fall.ev_mpr,  "full-foot spike → load lost"),
            ("TILT", fall.ev_tilt, f"loaded tilt {fall.tilt_deg_live:.0f}° > {FALL_LOADED_TILT_DEG:.0f}° sustained"),
            ("BARO", fall.ev_baro, f"altitude drop {fall.last_baro_drop_m*100:.0f} cm"),
        ]
        for j, (lbl, active, desc) in enumerate(sensors):
            ey = ev_y + 24 + j * 30
            dot_col = (255, 60, 60) if active else (50, 20, 20)
            pygame.draw.circle(self.screen, dot_col, (card_x + 55, ey + 6), 7)
            pygame.draw.circle(self.screen, (255, 255, 255) if active else (80, 40, 40), (card_x + 55, ey + 6), 7, 1)
            lbl_col  = (255, 100, 100) if active else (60, 30, 30)
            desc_col = self.C["text"] if active else self.C["text_dim"]
            self._txt(lbl, lbl_col, card_x + 70, ey, self.fnt_mono_md)
            self._txt(desc, desc_col, card_x + 115, ey + 2, self.fnt_mono_xs)

        held_s = (time.perf_counter() - fall._alert_start_t) if fall._alert_start_t else 0
        remain = max(0, FALL_ALERT_HOLD - held_s)
        self._txt(f"Auto-clear in {remain:.0f}s  (or stand up to recover)", (120, 50, 50), SCREEN_W//2, card_y + card_h - 38, self.fnt_mono_xs, anchor="center")

        if fall.replay_available:
            btn_w, btn_h = 200, 34
            btn_x = SCREEN_W//2 - btn_w//2
            btn_y = card_y + card_h - 20
            self._replay_btn_rect = pygame.Rect(btn_x, btn_y, btn_w, btn_h)
            pulse_col = (int(200 + 55*pulse), int(80 + 40*pulse), 20)
            pygame.draw.rect(self.screen, (40, 15, 5), (btn_x, btn_y, btn_w, btn_h))
            pygame.draw.rect(self.screen, pulse_col, (btn_x, btn_y, btn_w, btn_h), 2)
            self._txt("▶  FALL REPLAY", pulse_col, btn_x + btn_w//2, btn_y + 8, self.fnt_mono_md, anchor="center")
        else:
            self._replay_btn_rect = None

    def _draw_debug(self):
        with state.lock:
            la, q, gyro = state.la.copy(), state.q.copy(), state.gyro.copy()
        la_w  = quat_rotate(q, la)
        zaru  = self.dr.zaru
        fall  = self.dr.fall
        a_mag = float(np.linalg.norm(la_w))

        lines = [
            "── ZUPT DEBUG ─────────────────────────────────",
            f"la_world   {la_w[0]:+.3f} {la_w[1]:+.3f} {la_w[2]:+.3f}  |la|={a_mag:.3f} m/s²",
            f"gyro_body  {gyro[0]:+.3f} {gyro[1]:+.3f} {gyro[2]:+.3f} rad/s",
            f"sigma_a={zaru._sigma_a:.3f}  sigma_g={zaru._sigma_g:.3f}  cold={'done' if zaru._cold_done else f'{len(zaru._cold_buf_a)}/200'}",
            f"chi²={zaru.test_stat:.1f}  veto_thresh={ZUPT_CHI2_THRESH*ZUPT_IMU_VETO_MULT:.1f}  veto={'YES←swing' if zaru.imu_veto else 'no'}",
            f"press_gate={'CONTACT' if zaru.pressure_contact else 'AIRBORNE'}  n={zaru.n_active_sensors}  Δload={zaru.total_contact_load:.1f}",
            f"baseline[{'STATIC-CAL' if zaru._static_baseline_set else 'EMA-dynamic'}]: {' '.join(f'{v:.0f}' for v in (zaru._static_baseline if zaru._static_baseline_set else zaru._swing_baseline))}",
            f"mpr_raw:        {' '.join(f'{v:.0f}' for v in state.mpr)}",
            f"contact_delta:  {' '.join(f'{v:.0f}' for v in zaru.contact_delta)}",
            "── FALL DEBUG ──────────────────────────────────",
            f"|la|={a_mag:.2f} m/s²  ff_thresh={FALL_FREE_FALL_G}  free_fall={'ACTIVE' if fall._free_fall_seen else 'clear'}",
            f"impact_g={fall.last_impact_g:.2f}  impact_thresh={FALL_IMPACT_G}g  impact_t={'PENDING' if fall._impact_t else 'clear'}",
            f"baro_drop={fall.last_baro_drop_m*100:.1f}cm  thresh={FALL_BARO_DROP_THRESH*100:.0f}cm  IMU-gated",
            f"vel_raw  {self.dr.vel[0]:+.4f} {self.dr.vel[1]:+.4f} {self.dr.vel[2]:+.4f}",
            f"vel_MA{VEL_MA_WINDOW}  (window={VEL_MA_WINDOW} samples={len(self.dr._vel_ma._buf)})",
            f"yaw_raw={quat_to_euler(state.q)[2]:.1f}°  yaw_smooth={self.dr.yaw_smooth:.1f}°  window={YAW_MA_WINDOW}",
            f"ev_imu×1.0={fall.ev_imu}  ev_mpr_hi×1.0={fall.ev_mpr and fall.mpr_red_flags>=1}  ev_mpr_lo×1.0={fall.ev_mpr and fall.mpr_red_flags<1}  ev_tilt×1.2={fall.ev_tilt}  ev_baro×0.5={fall.ev_baro}  thresh={FALL_VOTE_THRESHOLD}",
            "── MPR SUB-DETECTOR B ──────────────────────────────",
            f"gait_state={fall._mpr_gait_state}  gait_timer={fall._mpr_gait_timer:.1f}s  steps={fall._mpr_gait_steps}",
            f"step_period={'N/A' if fall._step_period is None else f'{fall._step_period:.3f}s'}  adaptive_hold={'N/A' if fall._step_period is None else f'{min(fall._step_period*1.5,1.5):.3f}s'}  (fallback={FALL_MPR_SILENCE_HOLD}s)",
            f"silence_abrupt={fall._mpr_silence_abrupt}  ev_disappear={fall.ev_disappear}  red_flags={fall.mpr_red_flags}",
            f"tilt_deg={get_tilt_from_upright(fall._last_q, fall.ref_q_inv, fall.tilt_offset):.1f}°  tilt_flag={fall.mpr_flag_tilt}  (thresh={FALL_MPR_TILT_DEG}°  gimal-free)",
            f"flags: tilt={fall.mpr_flag_tilt}  imu={fall.mpr_flag_imu_event}  baro={fall.mpr_flag_baro}  prolonged={fall.mpr_flag_prolonged}  still={fall.mpr_flag_still}",
            "── TILT SUB-DETECTOR C (loaded-tilt) ───────────────",
            f"tilt_live={fall.tilt_deg_live:.1f}°  thresh={FALL_LOADED_TILT_DEG}°  min_load={FALL_LOADED_MIN_LOAD}  hold={FALL_LOADED_TILT_HOLD}s",
            f"tilt_timer={'%.2fs' % (time.perf_counter()-fall._tilt_high_since) if fall._tilt_high_since else 'not_started'}  gyro_mag={float(np.linalg.norm(state.gyro)):.2f}rad/s  gyro_suppress_thresh={FALL_LOADED_TILT_GYRO}  ev_tilt={fall.ev_tilt}",
        ]
        for i, ln in enumerate(lines):
            col = (0, 200, 255) if ln.startswith("──") else self.C["warn"]
            self._txt(ln, col, 4, 40 + i * 14, self.fnt_mono_xs)

    def _draw_replay(self):
        data = self._replay_data
        if not data: return
        total    = len(data)
        idx      = self._replay_idx
        now_tick = time.perf_counter()

        if self._replay_playing:
            if self._replay_last_tick > 0:
                wall_elapsed = now_tick - self._replay_last_tick
                data_elapsed = wall_elapsed * self._replay_speed
                t0         = data[0]['t']
                cur_data_t = data[idx]['t'] - t0 + data_elapsed
                new_idx = idx
                for i in range(idx, total):
                    if (data[i]['t'] - t0) >= cur_data_t:
                        new_idx = i
                        break
                else:
                    new_idx = total - 1
                self._replay_idx = new_idx
                idx = new_idx
                if idx >= total - 1:
                    self._replay_playing = False
            self._replay_last_tick = now_tick

        self.screen.fill(self.C["bg"])
        self._panel(0, 0, SCREEN_W, SCREEN_H, "FALL REPLAY  (press ESC or click ✕ to exit)")

        x_btn_rect = pygame.Rect(SCREEN_W - 42, 4, 36, 28)
        pygame.draw.rect(self.screen, (60, 15, 15), x_btn_rect)
        pygame.draw.rect(self.screen, self.C["danger"], x_btn_rect, 1)
        self._txt("✕", self.C["danger"], x_btn_rect.centerx, x_btn_rect.y + 4, self.fnt_mono_md, anchor="center")
        self._replay_x_btn_rect = x_btn_rect

        self._txt("FALL REPLAY", (255, 80, 80), SCREEN_W//2, 20, self.fnt_mono_lg, anchor="center")

        bar_x, bar_y, bar_w, bar_h = 60, 60, SCREEN_W-120, 16
        pygame.draw.rect(self.screen, (30, 20, 20), (bar_x, bar_y, bar_w, bar_h))
        pygame.draw.rect(self.screen, (80, 30, 30), (bar_x, bar_y, bar_w, bar_h), 1)
        filled = int(bar_w * idx / max(total-1, 1))
        pygame.draw.rect(self.screen, (255, 60, 60), (bar_x, bar_y, filled, bar_h))
        hx = bar_x + filled
        pygame.draw.circle(self.screen, (255, 140, 60), (hx, bar_y + bar_h//2), 9)

        snap = data[idx]
        t0   = data[0]['t']
        elapsed  = snap['t'] - t0
        duration = data[-1]['t'] - t0
        self._txt(f"{elapsed:.2f}s / {duration:.2f}s   frame {idx+1}/{total}   speed {self._replay_speed:.1f}×", self.C["text_dim"], SCREEN_W//2, bar_y+22, self.fnt_mono_xs, anchor="center")

        mpr  = snap['mpr']
        scl  = min(200/self.img_w, 700/self.img_h) if self.insole_img else 1.0
        sw   = int(self.img_w*scl) if self.insole_img else 200
        sh   = int(self.img_h*scl) if self.insole_img else 700
        hm_x = 60
        hm_y = 100
        if self.insole_img:
            si = pygame.transform.smoothscale(self.insole_img, (sw, sh))
            self.screen.blit(si, (hm_x, hm_y))
        total_mpr = float(np.sum(mpr))
        pct = np.zeros(10, dtype=np.float32)
        if total_mpr > 1.0:
            pct = (mpr / total_mpr * 100.0).astype(np.float32)

        hs = pygame.Surface((sw+20, sh+20), pygame.SRCALPHA)
        for i, val in enumerate(mpr):
            if val > 3:
                ox2, oy2 = self.SENSOR_POSITIONS[i]
                px, py = int(ox2*scl), int(oy2*scl)
                blob = self.base_blob.copy()
                col  = self._pressure_color(float(pct[i]))
                blob.fill((*col, int((val/255)*220)), special_flags=pygame.BLEND_RGBA_MULT)
                hs.blit(blob, (px-self.blob_r, py-self.blob_r), special_flags=pygame.BLEND_RGBA_ADD)
        self.screen.blit(hs, (hm_x, hm_y))
        for i, val in enumerate(mpr):
            ox2, oy2 = self.SENSOR_POSITIONS[i]
            px, py = int(ox2*scl) + hm_x, int(oy2*scl) + hm_y
            col = self._pressure_color(float(pct[i])) if val > 3 else self.C["border"]
            pygame.draw.circle(self.screen, col, (px, py), 5)
            if val > 3:
                self._txt(f"{pct[i]:.0f}%", self.C["text_bright"], px, py-14, self.fnt_mono_xs, anchor="center")

        q    = snap['q']
        la   = snap['la']
        gyro = snap['gyro']
        q_rel = quat_mul(self.dr.fall.ref_q_inv, q)
        tilt_deg = get_tilt_from_upright(q, self.dr.fall.ref_q_inv, self.dr.fall.tilt_offset)
        a_mag = float(np.linalg.norm(quat_rotate(q, la)))

        info_x = hm_x + sw + 40
        info_y = 100
        lines = [
            ("W",       f"{q_rel[0]:+.4f}", self.C["accent"]),
            ("X",       f"{q_rel[1]:+.4f}", self.C["accent"]),
            ("Y",       f"{q_rel[2]:+.4f}", self.C["accent"]),
            ("Z",       f"{q_rel[3]:+.4f}", self.C["accent"]),
            ("TILT",    f"{tilt_deg:.1f}°", self.C["danger"] if tilt_deg > 60 else self.C["warn"]   if tilt_deg > 30 else self.C["accent2"]),
            ("",        "",                 self.C["text_dim"]),
            ("|ACCEL|", f"{a_mag:.2f} m/s²", self.C["warn"]),
            ("LOAD",    f"{int(total_mpr)}", self.C["warn"]),
            ("ALT",     f"{snap['altitude']:.3f} m", (180,180,255)),
        ]
        for i, (lbl, val, col) in enumerate(lines):
            y = info_y + i * 36
            self._txt(lbl, self.C["text_dim"], info_x, y, self.fnt_mono_sm)
            self._txt(val, col, info_x + 140, y, self.fnt_mono_md)

        ctrl_y = SCREEN_H - 50
        pp_rect = pygame.Rect(SCREEN_W//2 - 50, ctrl_y, 100, 34)
        pygame.draw.rect(self.screen, (20, 40, 20), pp_rect)
        pygame.draw.rect(self.screen, self.C["accent2"], pp_rect, 1)
        lbl = "⏸ PAUSE" if self._replay_playing else "▶ PLAY"
        self._txt(lbl, self.C["accent2"], pp_rect.centerx, pp_rect.y+8, self.fnt_mono_md, anchor="center")

        speed_rects = []
        for si, spd in enumerate([0.5, 1.0, 2.0]):
            sr = pygame.Rect(SCREEN_W//2 + 70 + si*60, ctrl_y, 54, 34)
            is_active = abs(self._replay_speed - spd) < 0.1
            bg_col  = (20, 50, 50) if is_active else (15, 25, 30)
            brd_col = self.C["accent"] if is_active else self.C["border"]
            pygame.draw.rect(self.screen, bg_col, sr)
            pygame.draw.rect(self.screen, brd_col, sr, 1)
            self._txt(f"{spd:.0f}×" if spd >= 1 else "½×", brd_col, sr.centerx, sr.y+8, self.fnt_mono_md, anchor="center")
            speed_rects.append((sr, spd))
        self._replay_speed_rects = speed_rects

        exit_rect = pygame.Rect(SCREEN_W - 120, ctrl_y, 110, 34)
        pygame.draw.rect(self.screen, (40, 10, 10), exit_rect)
        pygame.draw.rect(self.screen, self.C["danger"], exit_rect, 1)
        self._txt("✕ EXIT", self.C["danger"], exit_rect.centerx, exit_rect.y+8, self.fnt_mono_md, anchor="center")

        self._replay_play_rect  = pp_rect
        self._replay_exit_rect  = exit_rect
        self._replay_scrub_rect = pygame.Rect(bar_x, bar_y-4, bar_w, bar_h+8)

    def _update_dynamic_fall_loads(self, swing_baseline: np.ndarray):
        global FALL_MPR_GAIT_LOAD, FALL_MPR_SILENCE_LOAD
        swing_total = float(np.sum(swing_baseline))
        contact_headroom = ZUPT_PRESSURE_DELTA_THRESH * ZUPT_MIN_ACTIVE_SENSORS * 1.5
        new_gait_load    = max(50.0, swing_total + contact_headroom)
        silence_margin   = 20.0
        new_silence_load = max(30.0, swing_total + silence_margin)
        FALL_MPR_GAIT_LOAD    = new_gait_load
        FALL_MPR_SILENCE_LOAD = new_silence_load
        print(f"[CALIB-S] Dynamic fall loads updated: gait_load={new_gait_load:.1f}, silence_load={new_silence_load:.1f}")

    def run(self):
        ble_t = threading.Thread(target=start_ble_thread, daemon=True)
        ble_t.start()

        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    # Clean up CSV logging if user closes the window
                    with state.lock:
                        if state.csv_file:
                            state.csv_file.close()
                    pygame.quit(); sys.exit()

                elif ev.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode((ev.w, ev.h), pygame.RESIZABLE)

                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE and self._replay_mode:
                        self._replay_mode    = False
                        self._replay_playing = False
                    
                    elif ev.key == pygame.K_r and not self._replay_mode:
                        self.dr.reset()
                        self._fall_overlay_alpha = 0
                        print("[RESET] Path and counters cleared.")
                    
                    elif ev.key == pygame.K_l and not self._replay_mode:
                        # Toggle Live CSV Logging
                        with state.lock:
                            state.is_csv_logging = not state.is_csv_logging
                            if state.is_csv_logging:
                                filename = time.strftime("live_log_%Y%m%d_%H%M%S.csv")
                                try:
                                    state.csv_file = open(filename, "w", encoding='utf-8')
                                    state.csv_file.write("t,qw,qx,qy,qz,lax,lay,laz,gyrx,gyry,gyrz,p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,gnd,alt,bat,seq,stability\n")
                                    print(f"[LOG] Started logging to {filename}")
                                except Exception as e:
                                    print(f"[LOG ERR] Could not open file: {e}")
                                    state.is_csv_logging = False
                            else:
                                if state.csv_file:
                                    state.csv_file.close()
                                    state.csv_file = None
                                print("[LOG] Stopped logging.")

                    elif ev.key == pygame.K_t:
                        self.show_debug = not self.show_debug
                    
                    elif ev.key == pygame.K_c:
                        self.is_calibrated = True
                        with state.lock: ref_q = state.q.copy()
                        self._ref_q_inv          = quat_conjugate(ref_q)
                        self.dr.fall.ref_q_inv   = self._ref_q_inv
                        self.dr.fall.tilt_offset = get_tilt_from_upright(ref_q)
                        self._calib_flash_t      = time.perf_counter()
                        print(f"[CALIB-C] Tilt pose captured. Offset: {self.dr.fall.tilt_offset:.1f}°")
                    
                    elif ev.key == pygame.K_s:
                        with state.lock: ref_mpr = state.mpr.copy()
                        self.dr.zaru._static_baseline     = ref_mpr.astype(np.float32)
                        self.dr.zaru._static_baseline_set = True
                        self.dr.zaru._swing_baseline      = ref_mpr.astype(np.float32)
                        self.dr.zaru._baseline_init       = True
                        self._swing_flash_t               = time.perf_counter()
                        print(f"[CALIB-S] Swing pressure baseline captured: {ref_mpr}")
                        self._update_dynamic_fall_loads(ref_mpr)
                    
                    elif ev.key == pygame.K_f:
                        self.dr.fall.inject_test_fall()
                        print("[DEBUG] Test fall injected.")
                    
                    elif ev.key == pygame.K_SPACE and self._replay_mode:
                        self._replay_playing = not self._replay_playing

                elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                    mx, my = ev.pos
                    if self._replay_mode:
                        if hasattr(self, '_replay_x_btn_rect') and self._replay_x_btn_rect.collidepoint(mx, my):
                            self._replay_mode    = False
                            self._replay_playing = False
                        elif hasattr(self, '_replay_exit_rect') and self._replay_exit_rect.collidepoint(mx, my):
                            self._replay_mode    = False
                            self._replay_playing = False
                        elif hasattr(self, '_replay_play_rect') and self._replay_play_rect.collidepoint(mx, my):
                            self._replay_playing = not self._replay_playing
                            self._replay_last_tick = 0.0
                        elif hasattr(self, '_replay_speed_rects'):
                            for sr, spd in self._replay_speed_rects:
                                if sr.collidepoint(mx, my):
                                    self._replay_speed = spd
                                    break
                            else:
                                if hasattr(self, '_replay_scrub_rect') and self._replay_scrub_rect.collidepoint(mx, my):
                                    bar_x = self._replay_scrub_rect.x
                                    bar_w = self._replay_scrub_rect.width
                                    frac  = max(0.0, min(1.0, (mx - bar_x) / bar_w))
                                    self._replay_idx     = int(frac * max(len(self._replay_data)-1, 0))
                                    self._replay_playing = False
                        elif hasattr(self, '_replay_scrub_rect') and self._replay_scrub_rect.collidepoint(mx, my):
                            bar_x = self._replay_scrub_rect.x
                            bar_w = self._replay_scrub_rect.width
                            frac  = max(0.0, min(1.0, (mx - bar_x) / bar_w))
                            self._replay_idx     = int(frac * max(len(self._replay_data)-1, 0))
                            self._replay_playing = False
                    else:
                        if (self._replay_btn_rect and self._replay_btn_rect.collidepoint(mx, my) and self.dr.fall.replay_available):
                            self._replay_data      = self.dr.fall.replay_snapshot or []
                            self._replay_idx       = 0
                            self._replay_playing   = False
                            self._replay_mode      = True
                            self._replay_last_tick = 0.0

            now = time.perf_counter()
            if now - self._rate_t >= 1.0:
                with state.lock: pkt = state.packet_count
                self.ble_rate_hz         = (pkt - self._last_pkt_count) / (now - self._rate_t)
                self._last_pkt_count, self._rate_t = pkt, now

            if self.is_calibrated and state.connected and not self._replay_mode:
                samples = jitter_buf.get_samples()
                for snap in samples[:10]:
                    self.dr.update(snap, 1.0 / TARGET_HZ)
                    self.chi2_history.append(self.dr.zaru.test_stat)
                    self.vel_history.append(float(np.linalg.norm(self.dr.vel)))

            if not self.is_calibrated:
                with state.lock: c = list(state.calib)
                if c[1] == 3 and c[2] == 3 and c[3] == 3:
                    self.is_calibrated = True
                    with state.lock: ref_q = state.q.copy()
                    self._ref_q_inv          = quat_conjugate(ref_q)
                    self.dr.fall.ref_q_inv   = self._ref_q_inv
                    self.dr.fall.tilt_offset = get_tilt_from_upright(ref_q)
                    print(f"[CALIB] Reference quaternion captured: {ref_q}")
                    print(f"[CALIB] Tilt offset captured: {self.dr.fall.tilt_offset:.1f}°")
                    print("[CALIB] Press S (foot in swing/airborne) to calibrate pressure baseline.")

            self.screen.fill(self.C["bg"])

            if self._replay_mode:
                self._draw_replay()
            elif not state.connected:
                self._txt("SCANNING FOR  XIAO_Smart_Shoe ...", self.C["warn"], SCREEN_W//2, SCREEN_H//2, self.fnt_mono_lg, anchor="center")
            elif not self.is_calibrated:
                self._draw_calibration()
            else:
                self._draw_map()
                self._draw_heatmap()
                self._draw_telemetry()
                if self.show_debug: self._draw_debug()

            if not self._replay_mode:
                self._draw_fall_overlay()

            if self._calib_flash_t is not None:
                elapsed = time.perf_counter() - self._calib_flash_t
                if elapsed < 2.0:
                    alpha = int(220 * min(1.0, (2.0 - elapsed) / 0.5))
                    surf  = pygame.Surface((400, 52), pygame.SRCALPHA)
                    surf.fill((10, 60, 30, alpha))
                    pygame.draw.rect(surf, (*self.C["accent2"], alpha), (0, 0, 400, 52), 2)
                    cx = SCREEN_W // 2
                    self.screen.blit(surf, (cx - 200, 18))
                    msg_col = tuple(int(c * alpha / 220) for c in self.C["accent2"])
                    self._txt(f"TILT POSE CAPTURED  offset={self.dr.fall.tilt_offset:.1f}°", msg_col, cx, 30, self.fnt_mono_md, anchor="center")
                    self._txt("Now lift foot and press S to calibrate pressure baseline", msg_col, cx, 52, self.fnt_mono_xs, anchor="center")
                else:
                    self._calib_flash_t = None

            if self._swing_flash_t is not None:
                elapsed = time.perf_counter() - self._swing_flash_t
                if elapsed < 2.0:
                    alpha = int(220 * min(1.0, (2.0 - elapsed) / 0.5))
                    surf  = pygame.Surface((480, 52), pygame.SRCALPHA)
                    surf.fill((10, 30, 60, alpha))
                    pygame.draw.rect(surf, (*self.C["accent"], alpha), (0, 0, 480, 52), 2)
                    cx = SCREEN_W // 2
                    self.screen.blit(surf, (cx - 240, 76))
                    msg_col = tuple(int(c * alpha / 220) for c in self.C["accent"])
                    self._txt(f"SWING BASELINE CAPTURED  gait_thr={FALL_MPR_GAIT_LOAD:.0f}  sil_thr={FALL_MPR_SILENCE_LOAD:.0f}", msg_col, cx, 88, self.fnt_mono_md, anchor="center")
                    self._txt("Lift foot fully off ground when pressing S", msg_col, cx, 110, self.fnt_mono_xs, anchor="center")
                else:
                    self._swing_flash_t = None

            self.screen.blit(self._scanlines, (0, 0))
            hint = "ESC=ExitReplay SPACE=Play/Pause" if self._replay_mode else "R=Reset T=Debug C=Tilt-cal S=Swing-cal F=TestFall L=Toggle CSV Log"
            self._txt(hint, self.C["text_dim"], SCREEN_W-10, SCREEN_H-14, self.fnt_mono_xs, anchor="topright")

            pygame.display.flip()
            self.clock.tick(100)

if __name__ == "__main__":
    Dashboard().run()
