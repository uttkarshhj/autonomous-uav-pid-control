import math
import datetime
import os


EARTH_R = 6_371_000.0

TAKEOFF = (28.748611, 77.117222) 
CP1     = (28.744444, 77.138056)
TARGET  = (28.723611, 77.113333)

#haversine formula for latitude to radians than finding out shortest distance between them

def haversine(lat1, lon1, lat2, lon2):
    rad1, rad2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(rad1)*math.cos(rad2)*math.sin(d_lon/2)**2
    return 2 * EARTH_R * math.asin(math.sqrt(max(0, min(1, a))))


def bearing(lat1, lon1, lat2, lon2):
    rad1, rad2 = math.radians(lat1), math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)
    x = math.sin(d_lon)*math.cos(rad2)
    y = math.cos(rad1)*math.sin(rad2) - math.sin(rad1)*math.cos(rad2)*math.cos(d_lon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def move_pos(lat, lon, dist_m, brng):
    dr   = dist_m / EARTH_R
    rad1 = math.radians(lat)
    lon1 = math.radians(lon)
    th   = math.radians(brng)
    rad2 = math.asin(math.sin(rad1)*math.cos(dr) +
                      math.cos(rad1)*math.sin(dr)*math.cos(th))
    lon2 = lon1 + math.atan2(math.sin(th)*math.sin(dr)*math.cos(rad1),
                               math.cos(dr) - math.sin(rad1)*math.sin(rad2))
    return math.degrees(rad2), math.degrees(lon2)



out_min=-6.0
out_max=6.0 #for clamping out output
i_min=-3.0 #integral windup prevention
i_max=3.0
rate_limit=6.0 #for clamping of rate change in velo
DT         = 1.0  
gap_time    = 5.0    # simulation time stamp(between each print)
reach_radius    = 10.0 
max_vel    = 80.0 
cruise_velo = 70.0
safe_velo   = 9.0    # for <=10ms-1
cross_safe_vel    = 9.0 
APP_R_TGT = 1500.0  #target approach radius (starts smoooth velocity breakin agter this)
APP_R_CP  = 700.0 # checkpoint approach radius



# PID STARTS HERE
class PID:

    def __init__(self, kp, ki, kd,out_min,out_max,i_min,i_max,rate_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min, self.out_max = out_min, out_max
        self.i_min, self.i_max = i_min, i_max
        self.rate_limit = rate_limit
        self._I = 0.0
        self._ep = None

    def reset(self):
        self._I = 0.0
        self._ep = None

    def update(self, sp, pv, dt):
        e = sp - pv
        P = self.kp * e
        self._I = max(self.i_min, min(self.i_max, self._I + e * dt)) # anti wind up
        I = self.ki * self._I
        D = 0.0 if (self._ep is None or dt <= 0) else self.kd * (e - self._ep) / dt
        self._ep = e
        output_ = P + I + D
        # Clamp output
        output_ = max(self.out_min, min(self.out_max, output_))
        # clamping rate of change of velocity too (precautions)
        return max(-self.rate_limit, min(self.rate_limit, output_))





WAYPOINTS = [
    {"name": "Checkpoint 1",    "coords": CP1,    "is_target": False},
    {"name": "Target Location", "coords": TARGET, "is_target": True},
]




PID_CFG = dict(kp=0.9, ki=0.01, kd=0.2,
               out_min=-6.0, out_max=6.0,
               i_min=-3.0,   i_max=3.0,
               rate_limit=6.0)



#implementation od deaccelration using cosine S curve
def velocity_profile(dist, final_vel, app_r):
    if dist <= reach_radius:
        return final_vel
    if dist >= app_r:
        return cruise_velo
    
    #Cosine S-Curve 
    t = 1.0 - (dist - reach_radius) / (app_r - reach_radius)
    blend = 0.5 * (1.0 - math.cos(math.pi * t)) 
    return cruise_velo + blend * (final_vel - cruise_velo)




class UAV_sim:

    def __init__(self, log_path="simulation_log.txt"):
        self.lat, self.lon = TAKEOFF
        self.vel    = 0.0
        self.t      = 0.0
        self._path  = log_path
        self._lines = []
        self._next  = 0.0
        self.pid    = PID(**PID_CFG)

    def _log(self, m, pr=True):
        self._lines.append(m)
        if pr:
            print(m)

    def _save(self):
        with open(self._path, "w", encoding="utf-8") as f:
            f.write("\n".join(self._lines))
        print(f"\n[INFO] Log -> {os.path.abspath(self._path)}")

    def _fly(self, wp):
        ...
        wp_lat, wp_lon = wp["coords"]
        name   = wp["name"]
        is_tgt = wp["is_target"]
        app_r  = APP_R_TGT if is_tgt else APP_R_CP
        final  = safe_velo  if is_tgt else cross_safe_vel

        self._log(f"\n{'='*72}")
        self._log(f"  Navigating to : {name}")
        self._log(f"  Destination   : {wp_lat:.6f} lat, {wp_lon:.6f} lon")
        self._log(f"  Approach zone : {app_r:.0f} m  |  Reach radius : {reach_radius:.0f} m")
        self._log(f"{'='*72}")

        self.pid.reset()

        while True:
            dist = haversine(self.lat, self.lon, wp_lat, wp_lon)

            # --- arrival check ---
            if dist <= reach_radius:
                self._log(
                    f"\n  *** {name} reached (within {reach_radius:.0f} m). "
                    f"Velocity at reach : {self.vel:.2f} m/s ***"
                )
                break

            # desired dmooth velocity
            v_sp = velocity_profile(dist, final, app_r)

            # velocity change
            dv = self.pid.update(sp=v_sp, pv=self.vel, dt=DT)

            # velocity
            self.vel = max(0.0, min(max_vel, self.vel + dv))

        
            brng = bearing(self.lat, self.lon, wp_lat, wp_lon)
            step = min(self.vel * DT, dist)
            self.lat, self.lon = move_pos(self.lat, self.lon, step, brng)
            self.t += DT

            # log on simulation
            if self.t >= self._next:
                d_now = haversine(self.lat, self.lon, wp_lat, wp_lon)
                self._log(
                    f"  t = {int(self.t)}s | [{self.lat:.4f}, {self.lon:.4f}] | "
                    f"Target: {name} | "
                    f"Distance remaining: {d_now:.1f} m | "
                    f"Velocity: {self.vel:.1f} m/s"
                )
                self._next = self.t + gap_time

    def run(self):
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._log("  ----------PID Velocity Log----------")
        self._log("-" * 72)
        H = "=" * 72
        self._log(H)
        self._log("  UAV PID Navigation Simulation")
        self._log(f"  Started : {ts}")
        self._log(H)
        self._log(f"  Takeoff    : {TAKEOFF[0]}, {TAKEOFF[1]}")

   
        d0 = haversine(self.lat, self.lon, *WAYPOINTS[0]["coords"])
        self._log(
            f"  t = 0s | [{self.lat:.4f}, {self.lon:.4f}] | "
            f"Target: {WAYPOINTS[0]['name']} | "
            f"Distance remaining: {d0:.1f} m | "
            f"Velocity: {self.vel:.1f} m/s"
        )
        self._next = gap_time

        for wp in WAYPOINTS:
            self._fly(wp)

        total = haversine(*TAKEOFF, *CP1) + haversine(*CP1, *TARGET)
        ok = self.vel < 10.0
        self._log("\n" + H)
        self._log(f"  Mission complete. Total flight time : {self.t:.1f} s")
        self._log(f"  Total route distance : {total:.1f} m  ({total/1000:.3f} km)")
        self._log(f"  Final velocity at target : {self.vel:.2f} m/s  "
                  + ("[SAFE < 10 m/s]" if ok else "[WARNING: exceeded 10 m/s]"))
        self._log(H)
        self._save()



if __name__ == "__main__":
    base = os.path.dirname(os.path.abspath(__file__))
    UAV_sim(log_path=os.path.join(base, "simulation_log.txt")).run()
