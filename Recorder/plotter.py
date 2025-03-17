# native python built in
import json
# need to pip install
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import argparse

# FAC Indexes
FAC_time_idx = -1
FAC_speed_idx = -5

with open('./device_dictionary.json','r') as file:
    ID_dict = json.load(file)

def dict_alter(id_dict, path):
    alt_dict = {}
    lines_verified = 10
    with open(path, 'r') as file:
        for id in id_dict:
            search_flag = False
            i=0
            for line in file:
                i+=1
                if id_dict[id][1] in line:
                    search_flag = True
                    break
                if i > lines_verified:
                    break
            if (search_flag):
                alt_dict[id] = id_dict[id]
    return alt_dict

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

def read_fac_data(pd_data, dict_id):
    ID = pd_data.iloc[:, 0].to_numpy(dtype=str)
    time = pd_data.iloc[:, FAC_time_idx].to_numpy()
    speed = pd_data.iloc[:, FAC_speed_idx].to_numpy()
    speed = speed * 10 * 2
    lat = pd_data.iloc[:, 4].to_numpy()
    long = pd_data.iloc[:, 5].to_numpy()

    dict_fac = {}

    for key in dict_id:
        ID_index = np.where(ID == dict_id[key][1])
        ID_lat = lat[ID_index]
        ID_long = long[ID_index]
        ID_speed = speed[ID_index]
        ID_time = time[ID_index]
        dict_fac[key] = [dict_id[key][2], ID_lat, ID_long, ID_speed,ID_time]
    return dict_fac

def read_sync_data(pd_data, dict_id):
    latest_NAV_time = pd_data.iloc[:, 0].to_numpy() # latest NAV time
    latest_pc_time = pd_data.iloc[:, 1].to_numpy() # latest PC time
    synced_NAV_profile = pd_data.iloc[:, 2] # synced NAV profile
    ego_time = synced_NAV_profile.apply(lambda x: x[0]).to_numpy()
    ego_speed = synced_NAV_profile.apply(lambda x: x[-3]).to_numpy()

    dict_sync_nav = {'latest_NAV_time': latest_NAV_time, 'latest_pc_time': latest_pc_time, 'synced_NAV_profile':synced_NAV_profile, 'ego_time':ego_time, 'ego_speed':ego_speed}

    i = 0
    dict_sync_fac = {}
    for key in dict_id:
        FAC_profile = pd_data.iloc[:, 3+i] # FAC1 profile
        FAC_time = FAC_profile.apply(lambda x: x[FAC_time_idx]).to_numpy()
        FAC_speed = FAC_profile.apply(lambda x: x[FAC_speed_idx]).to_numpy() * 20
        i = i+1
        dict_sync_fac[key] = [dict_id[key][2], FAC_profile, FAC_time, FAC_speed]

    return dict_sync_nav, dict_sync_fac

def read_ctrl_data(pd_data, dict_id):
    latest_NAVcb_main = pd_data.iloc[:, 0].to_numpy() # latest NAV time in main
    latest_pc_main = pd_data.iloc[:, 1].to_numpy() # latest PC time in main
    latest_pccb_main = pd_data.iloc[:, -1].to_numpy() # latest PC time in
    ctrl_data = pd_data.iloc[:, 2] # ctrl data
    ego_speed = ctrl_data.apply(lambda x: x[0]).to_numpy()

    time_ctrl = {'latest_NAVcb_main':latest_NAVcb_main, 'latest_pc_main':latest_pc_main, 'latest_pccb_main':latest_pccb_main, 'ctrl_ego_speed':ego_speed}
    
    dict_ctrl={}

    for key in dict_id:
        ID_index = ctrl_data.apply(lambda x: x.index(dict_id[key][1]))[0]
        ID_speed = ctrl_data.apply(lambda x: x[ID_index + 1]).to_numpy() * 20
        ID_hdwy = ctrl_data.apply(lambda x: x[ID_index + 2]).to_numpy()
        dict_ctrl[key] = [dict_id[key][2], ID_speed, ID_hdwy]
    return dict_ctrl, time_ctrl

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--ts', type=str, help='timestamp of the relevant data', required=True)
    args=parser.parse_args()
    time_stamp = args.ts

    timestamp_data = time_stamp
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

    sync_data = read_data(sync_path)
    ctrl_data = read_data(ctrl_path)
    nav_data = read_data(nav_path)
    fac_data = read_data(fac_path)

    sync_df = pd.DataFrame(sync_data)
    ctrl_df = pd.DataFrame(ctrl_data)
    nav_df = pd.DataFrame(nav_data)
    fac_df = pd.DataFrame(fac_data)

    # Altered Dictionary

    new_dict = dict_alter(ID_dict, sync_path)

    dict_sync_nav, dict_sync_fac = read_sync_data(sync_df, new_dict)
    latest_NAV_time = dict_sync_nav['latest_NAV_time'] 
    latest_pc_time = dict_sync_nav['latest_pc_time']
    synced_NAV_profile = dict_sync_nav['synced_NAV_profile']
    ego_time = dict_sync_nav['ego_time']
    ego_speed = dict_sync_nav['ego_speed']


    dict_ctrl, time_ctrl = read_ctrl_data(ctrl_df, new_dict)

    latest_NAVcb_main = time_ctrl['latest_NAVcb_main']
    latest_pc_main = time_ctrl['latest_pc_main']
    latest_pccb_main = time_ctrl['latest_pccb_main']
    ctrl_ego_speed = time_ctrl['ctrl_ego_speed']

    dict_fac = read_fac_data(fac_df, new_dict)

    nav_time, nav_lat, nav_long = read_nav_data(nav_df)

    # for sync check
    nav_nativecb_time = latest_NAV_time - latest_NAV_time[0]
    nav_nativecb_diff = latest_NAV_time - latest_NAV_time
    pc_nativecb_diff = latest_pc_time - latest_NAV_time
    ego_nativecb_diff = ego_time - latest_NAV_time

    dict_fac_nativecb_diff = {}

    for key in new_dict:
        dict_fac_nativecb_diff[key] = dict_sync_fac[key][2] - latest_NAV_time

    # for msg loss check
    nav_nativemain_time = latest_NAVcb_main - latest_NAVcb_main[0]  # latest NAV time in main
    latest_pcmain_time = latest_pccb_main - latest_NAVcb_main  # latest PC time in main


    # for sync msg check
    fig1, axs1 = plt.subplots(2, 1, sharex=True)

    # sync panel
    axs1[0].plot(nav_nativecb_time / 1000, nav_nativecb_diff, label='native time')
    axs1[0].plot(nav_nativecb_time / 1000, pc_nativecb_diff, label='pc time')
    axs1[0].plot(nav_nativecb_time / 1000, ego_nativecb_diff, label='ego time')

    fac_no = 1

    for key in dict_fac_nativecb_diff:
        label_def = 'fac-'+ str(fac_no) +' time'
        fac_no = fac_no + 1
        axs1[0].plot(nav_nativecb_time / 1000, dict_fac_nativecb_diff[key], label=label_def, linestyle=':')
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
    #dict_fac[key] = [ID_index, ID_lat, ID_long, ID_speed,ID_time]
    for key in dict_fac:
        axs2.plot(dict_fac[key][1], dict_fac[key][2], label=str(dict_fac[key][0]) + ' trajectory', linestyle=':')
    axs2.legend()
    axs2.grid(True, which='both', linestyle='--', linewidth=0.5)
    axs2.set_title('GPS route')
    axs2.set_xlabel('latitude')
    axs2.set_ylabel('longitude')


    # for speed penal
    fig3, axs3 = plt.subplots(1, 1, sharex=True)

    # sync panel
    #dict_sync_fac[key] = [dict_id[key][1], FAC_profile, FAC_time, FAC_speed]
    axs3.plot((ego_time - ego_time[0]) / 1000, ego_speed, label='ego speed')
    for key in dict_sync_fac:
        axs3.plot((dict_sync_fac[key][2] - dict_sync_fac[key][2][0]) / 1000, dict_sync_fac[key][3], label=str(dict_sync_fac[key][0]) + ' speed',linestyle=':')
    axs3.legend()
    axs3.grid(True, which='both', linestyle='--', linewidth=0.5)
    axs3.set_title('Speed info of each vehicle')
    axs3.set_xlabel('Time [s]')
    axs3.set_ylabel('Speed [m/s]')


    # for speed penal
    fig4, axs4 = plt.subplots(2, 1, sharex=True)

    # speed panel
    axs4[0].plot((latest_pc_main - latest_pc_main[0]) / 1000, ctrl_ego_speed, label='ego speed')
    for key in dict_ctrl:
        axs4[0].plot((latest_pc_main - latest_pc_main[0]) / 1000, dict_ctrl[key][1], label=str(dict_ctrl[key][0]) + ' speed', linestyle=':')
    axs4[0].legend()
    axs4[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    axs4[0].set_title('Speed info of each vehicle')
    axs4[0].set_xlabel('Time [s]')
    axs4[0].set_ylabel('Speed [m/s]')

    # headway panel
    for key in dict_ctrl:
        axs4[1].plot((latest_pc_main - latest_pc_main[0]) / 1000, dict_ctrl[key][2], label=dict_ctrl[key][0] + ' headway')
    axs4[1].legend()
    axs4[1].grid(True, which='both', linestyle='--', linewidth=0.5)
    axs4[1].set_title('Headway info of each vehicle')
    axs4[1].set_xlabel('Time [s]')
    axs4[1].set_ylabel('Headway [m]')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
    
