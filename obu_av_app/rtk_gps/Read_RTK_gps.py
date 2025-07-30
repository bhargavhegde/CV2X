'''
[CRH TODO] make a sbp.json file translator
'''

import json
import base64

import pandas as pd
import matplotlib.pyplot as plt
from sbp.msg import SBP

# Only include messages that are actually available
from sbp.navigation import (
    SBP_MSG_POS_LLH, MsgPosLLH,
    SBP_MSG_VEL_NED, MsgVelNED,
    SBP_MSG_DOPS, MsgDops,
    SBP_MSG_BASELINE_NED, MsgBaselineNED,
    SBP_MSG_BASELINE_ECEF, MsgBaselineECEF
)
from sbp.logging import SBP_MSG_LOG, MsgLog
from sbp.tracking import SBP_MSG_TRACKING_STATE, MsgTrackingState
import os

DISPATCH_TABLE = {
    SBP_MSG_POS_LLH: MsgPosLLH,
    SBP_MSG_VEL_NED: MsgVelNED,
    SBP_MSG_DOPS: MsgDops,
    SBP_MSG_BASELINE_NED: MsgBaselineNED,
    SBP_MSG_BASELINE_ECEF: MsgBaselineECEF,
    SBP_MSG_LOG: MsgLog,
    SBP_MSG_TRACKING_STATE: MsgTrackingState,
}

input_file = 'static_point_sample'

input_file = input_file + '.sbp.json'
output_file = input_file.replace('.sbp.json', '.decoded.json')
output_csv = input_file.replace('.sbp.json', '.decoded.csv')
decoded_msgs = []

# If CSV already exists, load DataFrame directly and skip decoding
if os.path.exists(output_csv):
    print(f"CSV file already exists. Loading: {output_csv}")
    df = pd.read_csv(output_csv)
    decoded_msgs = None  # Indicate that decoding was skipped
else:
    with open(input_file, 'r') as f:
        for line in f:
            entry = json.loads(line)
            payload_b64 = entry.get("payload")
            if not payload_b64:
                continue

            msg_type = entry.get("msg_type")
            sender = entry.get("sender")
            time = entry.get("time", {})

            try:
                # Correctly decode base64 payload
                payload_bytes = base64.b64decode(payload_b64)
                sbp_msg = SBP(msg_type=msg_type, sender=sender, payload=payload_bytes)

                msg_class = DISPATCH_TABLE.get(msg_type)
                if msg_class:
                    try:
                        msg = msg_class(sbp_msg)  # ✅ decode by constructing from SBP object
                        decoded_msgs.append({
                            "msg_type": msg_type,
                            "sender": sender,
                            "time": time,
                            "decoded_msg": msg.to_json_dict()
                        })
                    except Exception as e:
                        print(f"Error decoding msg_type {msg_type} with {msg_class.__name__}: {e}")
                else:
                    print(f"Skipping unknown msg_type: {msg_type}")
            except Exception as e:
                print(f"Failed to decode msg_type {msg_type}: {e}")

    # Save decoded messages
    with open(output_file, "w") as f:
        json.dump(decoded_msgs, f, indent=2)

    # --- Extract from MsgPosLLH (msg_type = 522) ---
    records = []
    for entry in decoded_msgs:
        if entry.get("msg_type") == 522:  # MsgPosLLH
            msg = entry.get("decoded_msg", {})
            lat = msg.get("lat")
            lon = msg.get("lon")
            if abs(lat) < 1e-6 or abs(lon) < 1e-6:
                continue
            msg = entry.get("decoded_msg", {})
            tow = msg.get("tow")  # GPS Time of Week in milliseconds
            # Preserve full precision by parsing lat/lon as float from their string representation
            lat_raw = msg.get("lat")
            lon_raw = msg.get("lon")
            # Convert to float using repr to preserve all digits
            lat = float(repr(lat_raw)) if lat_raw is not None else None
            lon = float(repr(lon_raw)) if lon_raw is not None else None
            height = msg.get("height")
            if tow is not None and lat is not None and lon is not None:
                records.append({
                    "tow": tow,
                    "lat": lat,
                    "lon": lon,
                    "height": height
                })
        
    # --- Create DataFrame ---
    df = pd.DataFrame(records)
    df = df.sort_values(by="tow").reset_index(drop=True)
    df["time_sec"] = df["tow"] / 1000.0
    df["time_delta"] = df["time_sec"] - df["time_sec"].iloc[0]  # Relative to start

    # --- Save to CSV ---
    df.to_csv(output_csv, index=False)
    print(f"Saved: {output_csv}")


# --- Display DataFrame statistics ---
statistics = df.describe()
print(statistics)

lat_mean = df["lat"].mean()
lon_mean = df["lon"].mean()
lat_std = df["lat"].std()
lon_std = df["lon"].std()
print(f"Average Latitude: {lat_mean:.10f} (std: {lat_std:.10f})")
print(f"Average Longitude: {lon_mean:.10f} (std: {lon_std:.10f})")


# --- Save figures ---
trajectory_fig_path = input_file.replace('.sbp.json', '_gps_trajectory.png')
profile_fig_path = input_file.replace('.sbp.json', '_gps_profile.png')

# --- Plot Latitude and Longitude vs Time ---
plt.figure(figsize=(10, 4))
fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axs[0].plot(df["time_delta"], df["lat"], label="Latitude", color='tab:blue')
axs[0].set_ylabel("Latitude (deg)")
axs[0].set_title("Latitude vs. Time (TOW)")
axs[0].grid(True)
axs[0].legend()

axs[1].plot(df["time_delta"], df["lon"], label="Longitude", color='tab:orange')
axs[1].set_ylabel("Longitude (deg)")
axs[1].set_xlabel("Time since start (s)")
axs[1].set_title("Longitude vs. Time (TOW)")
axs[1].grid(True)
axs[1].legend()
plt.tight_layout()
# plt.show()
plt.savefig(profile_fig_path)
print(f"Saved: {profile_fig_path}")

# --- Plot 2D Trajectory ---
plt.figure(figsize=(6, 6))
plt.plot(df["lon"], df["lat"], marker='o', markersize=2)
plt.title("2D Trajectory (Longitude vs Latitude)")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.grid(True)
plt.tight_layout()
# plt.show()
plt.savefig(trajectory_fig_path)
print(f"Saved: {trajectory_fig_path}")
