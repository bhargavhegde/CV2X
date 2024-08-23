# native python built in
import json
# need to pip install
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ID_1 = "2a3c642d"
ID_2 = "2a3c645e"
ID_3 = "2a3c6435"

timestamp_data = input('Enter the timestamp of the data: ')
user_input_sync_path = "data/" + timestamp_data + "sync.txt"
user_input_ctrl_path = "data/" + timestamp_data + "ctrl.txt"
user_input_nav_path = "data/" + timestamp_data + "nav.txt"
user_input_fac_path = "data/" + timestamp_data + "fac.txt"

# sync path
sync_path = user_input_sync_path
# ctrl path
ctrl_path = user_input_ctrl_path
# nav path
nav_path = user_input_nav_path
# fac path
fac_path = user_input_fac_path


def read_data(path):
    data = []
    with open(path, 'r') as file:
        for line in file:
            data.append(json.loads(line))
    return data


def read_nav_data(pd_data):
    native_NAV_time = pd_data.iloc[:, 0].to_numpy()
    native_NAV_lat = pd_data.iloc[:, 1].to_numpy() # latest NAV time
    native_NAV_long = pd_data.iloc[:, 2].to_numpy() # speed
    return native_NAV_time, native_NAV_lat, native_NAV_long


def read_fac_data(pd_data, ID1, ID2):
    ID = pd_data.iloc[:, 0].to_numpy(dtype=str)
    time = pd_data.iloc[:, -1].to_numpy()
    speed = pd_data.iloc[:, -5].to_numpy()
    speed = speed * 10 * 2
    lat = pd_data.iloc[:, 4].to_numpy()
    long = pd_data.iloc[:, 5].to_numpy()

    ID1_index = np.where(ID == ID1)
    ID2_index = np.where(ID == ID2)

    ID1_lat = lat[ID1_index]
    ID2_lat = lat[ID2_index]
    ID1_long = long[ID1_index]
    ID2_long = long[ID2_index]
    ID1_speed = speed[ID1_index]
    ID2_speed = speed[ID2_index]
    ID1_time = time[ID1_index]
    ID2_time = time[ID2_index]
    return ID1_time, ID2_time, ID1_lat, ID2_lat, ID1_long, ID2_long, ID1_speed, ID2_speed


def read_sync_data(pd_data):
    latest_NAV_time = pd_data.iloc[:, 0].to_numpy() # latest NAV time
    latest_pc_time = pd_data.iloc[:, 1].to_numpy() # latest PC time
    synced_NAV_profile = pd_data.iloc[:, 2] # synced NAV profile
    FAC1_profile = pd_data.iloc[:, 3] # FAC1 profile
    FAC2_profile = pd_data.iloc[:, 4] # FAC2 profile

    ego_time = synced_NAV_profile.apply(lambda x: x[0]).to_numpy()
    ego_speed = synced_NAV_profile.apply(lambda x: x[-3]).to_numpy()
    FAC1_time = FAC1_profile.apply(lambda x: x[-1]).to_numpy()
    FAC1_speed = FAC1_profile.apply(lambda x: x[-5]).to_numpy() * 20
    FAC2_time = FAC2_profile.apply(lambda x: x[-1]).to_numpy()
    FAC2_speed = FAC2_profile.apply(lambda x: x[-5]).to_numpy() * 20
    return latest_NAV_time, latest_pc_time, synced_NAV_profile, FAC1_profile, FAC2_profile, ego_time, FAC1_time, FAC2_time, ego_speed, FAC1_speed, FAC2_speed


def read_ctrl_data(pd_data, ID1, ID2):
    latest_NAVcb_main = pd_data.iloc[:, 0].to_numpy() # latest NAV time in main
    latest_pc_main = pd_data.iloc[:, 1].to_numpy() # latest PC time in main
    latest_pccb_main = pd_data.iloc[:, -1].to_numpy() # latest PC time in
    ctrl_data = pd_data.iloc[:, 2] # ctrl data
    ego_speed = ctrl_data.apply(lambda x: x[0]).to_numpy()
    ID1_index = ctrl_data.apply(lambda x: x.index(ID1))[0]
    ID2_index = ctrl_data.apply(lambda x: x.index(ID2))[0]
    # print(ID1_index)
    ID1_speed = ctrl_data.apply(lambda x: x[ID1_index + 1]).to_numpy() * 20
    ID1_hdwy = ctrl_data.apply(lambda x: x[ID1_index + 2]).to_numpy()
    ID2_speed = ctrl_data.apply(lambda x: x[ID2_index + 1]).to_numpy() * 20
    ID2_hdwy = ctrl_data.apply(lambda x: x[ID2_index + 2]).to_numpy()
    return latest_NAVcb_main, latest_pc_main, latest_pccb_main, ego_speed, ID1_speed, ID1_hdwy, ID2_speed, ID2_hdwy


sync_data = read_data(sync_path)
ctrl_data = read_data(ctrl_path)
nav_data = read_data(nav_path)
fac_data = read_data(fac_path)

sync_df = pd.DataFrame(sync_data)
ctrl_df = pd.DataFrame(ctrl_data)
nav_df = pd.DataFrame(nav_data)
fac_df = pd.DataFrame(fac_data)

latest_NAV_time, latest_pc_time, synced_NAV_profile, FAC1_profile, FAC2_profile, ego_time, FAC1_time, FAC2_time, ego_speed, FAC1_speed, FAC2_speed = read_sync_data(sync_df)
latest_NAVcb_main, latest_pc_main, latest_pccb_main, ctrl_ego_speed, ctrl_ID1_speed, ctrl_ID1_hdwy, ctrl_ID2_speed, ctrl_ID2_hdwy = read_ctrl_data(ctrl_df, ID_2, ID_3)
ID1_time, ID2_time, ID1_lat, ID2_lat, ID1_long, ID2_long, ID1_speed, ID2_speed = read_fac_data(fac_df, ID_2, ID_3)
nav_time, nav_lat, nav_long = read_nav_data(nav_df)

# for sync check
nav_nativecb_time = latest_NAV_time - latest_NAV_time[0]
nav_nativecb_diff = latest_NAV_time - latest_NAV_time
pc_nativecb_diff = latest_pc_time - latest_NAV_time
ego_nativecb_diff = ego_time - latest_NAV_time
FAC1_nativecb_diff = FAC1_time - latest_NAV_time
FAC2_nativecb_diff = FAC2_time - latest_NAV_time

# for msg loss check
nav_nativemain_time = latest_NAVcb_main - latest_NAVcb_main[0]  # latest NAV time in main
latest_pcmain_time = latest_pccb_main - latest_NAVcb_main  # latest PC time in main


# for sync msg check
fig1, axs1 = plt.subplots(2, 1, sharex=True)

# sync panel
axs1[0].plot(nav_nativecb_time / 1000, nav_nativecb_diff, label='native time')
axs1[0].plot(nav_nativecb_time / 1000, pc_nativecb_diff, label='pc time')
axs1[0].plot(nav_nativecb_time / 1000, ego_nativecb_diff, label='ego time')
axs1[0].plot(nav_nativecb_time / 1000, FAC1_nativecb_diff, label='fac1 time')
axs1[0].plot(nav_nativecb_time / 1000, FAC2_nativecb_diff, label='fac2 time')
axs1[0].legend()
axs1[0].set_title('Sync check')
axs1[0].set_xlabel('time [s]')
axs1[0].set_ylabel('time difference [ms]')

# msg loss panel
axs1[1].plot(nav_nativecb_time / 1000, pc_nativecb_diff, 'o-', label='native msg', color='blue', markersize=10)
axs1[1].plot(nav_nativemain_time / 1000, latest_pcmain_time, 'o-', label='main msg', color='red')
axs1[1].legend()
axs1[1].set_title('Sync check')
axs1[1].set_xlabel('time [s]')
axs1[1].set_ylabel('time difference [ms]')

axs1[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axs1[1].grid(True, which='both', linestyle='--', linewidth=0.5)


# for gps route check
fig2, axs2 = plt.subplots(1, 1, sharex=True)

# sync panel
axs2.plot(nav_lat, nav_long, label='ego trajectory')
axs2.plot(ID1_lat, ID1_long, label=ID_2 + 'trajectory')
axs2.plot(ID2_lat, ID2_long, label=ID_3 + 'trajectory')
axs2.legend()
axs2.grid(True, which='both', linestyle='--', linewidth=0.5)
axs2.set_title('GPS route')
axs2.set_xlabel('latitude')
axs2.set_ylabel('longitude')


# for speed penal
fig3, axs3 = plt.subplots(1, 1, sharex=True)

# sync panel
axs3.plot((ego_time - ego_time[0]) / 1000, ego_speed, label='ego speed')
axs3.plot((FAC1_time - FAC1_time[0]) / 1000, FAC1_speed, label=ID_2 + 'speed')
axs3.plot((FAC2_time - FAC2_time[0]) / 1000, FAC2_speed, label=ID_3 + 'speed')
axs3.legend()
axs3.grid(True, which='both', linestyle='--', linewidth=0.5)
axs3.set_title('Speed info of each vehicle')
axs3.set_xlabel('Time [s]')
axs3.set_ylabel('Speed [m/s]')


# for speed penal
fig4, axs4 = plt.subplots(2, 1, sharex=True)

# speed panel
axs4[0].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ego_speed, label='ego speed')
axs4[0].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ID1_speed, label=ID_2 + 'speed')
axs4[0].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ID2_speed, label=ID_3 + 'speed')
axs4[0].legend()
axs4[0].grid(True, which='both', linestyle='--', linewidth=0.5)
axs4[0].set_title('Speed info of each vehicle')
axs4[0].set_xlabel('Time [s]')
axs4[0].set_ylabel('Speed [m/s]')

# headway panel
axs4[1].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ID1_hdwy, label=ID_2 + 'headway')
axs4[1].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ID2_hdwy, label=ID_3 + 'headway')
axs4[1].legend()
axs4[1].grid(True, which='both', linestyle='--', linewidth=0.5)
axs4[1].set_title('Headway info of each vehicle')
axs4[1].set_xlabel('Time [s]')
axs4[1].set_ylabel('Headway [m]')

plt.tight_layout()
plt.show()
