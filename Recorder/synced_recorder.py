# This is a debug version of the recorder, to fix the time sync issue of latest NAV not updating on time
# current version is able to sync data form NAV and FAC, however, still requires to do real time h computation
# built-in pacakage
import os
import time
import json
from collections import deque
# using pip to install
import numpy as np # this will be used for later haversine headway computation

# from Commsignia API
from pycmssdk import(
     Asn1Type,
     FacMsgType,
     FacNotifData,
     asn1_decode,
     create_cms_api)
from pycmssdk import(# noqa: F401
     NavDriveDirection,
     NavFix,
     NavNotifData,
     NavSetManual,
     NavSource,
     UtcTimestampMs,
     create_cms_api)
from pycmssdk import WILDCARD, FacNotifData, create_cms_api

# Defining all variables needed in global space
# for message print
NAV_msg = {}
FAC_msg = {}
# for message sync
NAV_MSG_BUFFER = {}
FAC_MSG_BUFFER = {}
sync_buffer = {}
sync_info = {}
sync_in_main = []

Want_to_check_msg = False
# for data saving path directories
if not os.path.exists("data"):
    os.mkdir("data")
trail_time = time.strftime("%Y%m%d%H%M%S")
nav_directory = os.path.join("data", f"{trail_time}nav.txt")
fac_directory = os.path.join("data", f"{trail_time}fac.txt")
sync_directory = os.path.join("data", f"{trail_time}sync.txt")
ctrl_directory = os.path.join("data", f"{trail_time}ctrl.txt")


def nav_callback(data: NavNotifData):
    # global sync_buffer
    NAV_msg = {"timestamp": data.timestamp,
               "latitude": data.latitude / 10e6,
               "longitude": data.longitude / 10e6,
               "altitude": data.altitude / 10e3,
               "heading": data.heading,
               "speed": data.speed / 1000,
               "satellites": data.number_of_used_satellites,
               "pc_nav_log_time": round(time.time() * 1000)}
    
    with open(nav_directory, 'a') as file:
        # nav_values = [str(item) for item in NAV_msg.values()]
        nav_values = list(NAV_msg.values())
        json.dump(nav_values, file)
        file.write("\n")

    if 'NAV_msg' not in NAV_MSG_BUFFER:
        NAV_MSG_BUFFER['NAV_msg'] = deque([list(NAV_msg.values())], maxlen=10)
    else:
        NAV_MSG_BUFFER['NAV_msg'].appendleft(list(NAV_msg.values()))

    if not FAC_MSG_BUFFER:
        print("other vehicles message DNE, NAV message only")
    else:
        sync_buffer = synchronization(NAV_MSG_BUFFER, FAC_MSG_BUFFER)

        if sync_buffer:
            sync_save = list(element[0] for element in sync_buffer.values())
            # print('from callback :', sync_save)
            with open(sync_directory, 'a') as file:
                # sync_values = [str(item) for item in sync_save]
                json.dump(sync_save, file)
                file.write("\n")
                # file.write(",".join(sync_values) + "\n")
          

def synchronization(nav_msg_buffer, fac_msg_buffer):
        
    NAV_profiles = list(nav_msg_buffer['NAV_msg'])
    NAV_time = [item[0] for item in NAV_profiles]
    FAC_profiles = list(fac_msg_buffer.values())
    FAC_time = [item[0][-1] for item in FAC_profiles]
    # print('FAC_time', FAC_time)
    latest_FAC_time = min(FAC_time) # reference earliest among all latest updates to sync FAC message for integration
    if len(NAV_time) > 1:
        NAVindex, _ = finding_diffmin(NAV_time, latest_FAC_time)
        # print('nav_ind', NAVindex)
        # sync_buffer['latest_NAV_time'] = NAV_time[0]
        if 'latest_NAV_time' not in sync_buffer:
            sync_buffer['latest_NAV_time'] = deque([NAV_time[0]], maxlen=20)
        else:
            sync_buffer['latest_NAV_time'].appendleft(NAV_time[0])

        pc_time = round(time.time() * 1000)
        if 'pc_time' not in sync_buffer:
            sync_buffer['pc_time'] = deque([pc_time], maxlen=20)
        else:
            sync_buffer['pc_time'].appendleft(pc_time)

        # sync_buffer['synced_NAV_profile'] = NAV_profiles[NAVindex]
        if 'synced_NAV_profile' not in sync_buffer:
            sync_buffer['synced_NAV_profile'] = deque([NAV_profiles[NAVindex]], maxlen=20)
        else:
            sync_buffer['synced_NAV_profile'].appendleft(NAV_profiles[NAVindex])
        
        for keys in fac_msg_buffer:
            FAC_history = list(fac_msg_buffer[keys])
            FAC_timehistory = [item[-1] for item in FAC_history]
            # find the msg with time cloest to the latest NAV message
            Findex, _ = finding_diffmin(FAC_timehistory, NAV_time[NAVindex])
            # print('FAC history', FAC_history)
            # # print(FAC_timehistory)
            # # print(NAV_time[NAVindex])
            # print('sync nav time', NAV_time[NAVindex])
            # print('sync fac index', Findex)
            if keys not in sync_buffer:
                sync_buffer[keys] = deque([FAC_history[Findex]], maxlen=20)
            else:
                sync_buffer[keys].appendleft(FAC_history[Findex])
        return sync_buffer
    else:
        return {}


def controller_input(synced_data):
    control_input = {}
    # ego vehicle info
    ego_vehicle = synced_data[2]
    # print(synced_data)
    # print('ego_vehicle', ego_vehicle)
    ego_vehicle_speed = ego_vehicle[-3]
    ego_vehicle_lat = ego_vehicle[1]
    ego_vehicle_long = ego_vehicle[2]
    # control_input["pctime from NAV"] = [synced_data[-1]]
    control_input["ego_vehicle_speed"] = [ego_vehicle_speed]
    # front vehicle info
    number_of_devices = len(synced_data) - 4
    # print('number_of_devices', number_of_devices)
    for i in range(number_of_devices):
        i = i + 3
        front_vehicle_info = synced_data[i]
        # print('here',front_vehicle_info)
        front_vehicle_ID = front_vehicle_info[0]
        front_vehicle_speed = front_vehicle_info[-5]
        front_vehicle_lat = front_vehicle_info[4]
        front_vehicle_long = front_vehicle_info[5]
        front_vehicle_hdwy = headway_real(ego_vehicle_lat, ego_vehicle_long, front_vehicle_lat, front_vehicle_long)
        control_input[front_vehicle_ID] = [front_vehicle_ID, front_vehicle_speed, front_vehicle_hdwy]
    # print(control_input)
    return control_input


def headway_real(lat_ego, lon_ego, hearing_lat, hearing_lon):
    r = 6371000
    # e for point 1 , and f for point 2
    lat_e = lat_ego
    lon_e = lon_ego
    lat_f = hearing_lat
    lon_f = hearing_lon

    lat_er = np.radians(lat_e)
    lat_fr = np.radians(lat_f)

    lat_diff = lat_f - lat_e
    lon_diff = lon_f - lon_e

    lat_diffr = np.radians(lat_diff)
    lon_diffr = np.radians(lon_diff)

    intermediate = np.sqrt((1 - np.cos(lat_diffr) + np.cos(lat_er) * np.cos(lat_fr) * (1 - np.cos(lon_diffr))) / 2)
    haver_d = 2 * r * np.arctan(intermediate)

    return haver_d


def fac_callback(key: int, data: FacNotifData, buffer: bytes) -> None:

    # global FAC_MSG_BUFFER
    # global FAC_msg
    decoded_message = asn1_decode(buffer, Asn1Type.US_MESSAGE_FRAME)
    hearing_ID = decoded_message["value"][1]["coreData"]["id"].hex()
    timestamp = round(time.time() * 1000)
    time_generated_theory = integrate_time(timestamp, decoded_message["value"][1]["coreData"]["secMark"])
    time_generated_bounded = generate_fac_time(timestamp, decoded_message["value"][1]["coreData"]["secMark"])
    # print("timestamp",timestamp)
    # print("time_generated_theory",time_generated_theory)
    # print("time_generated_bounded",time_generated_bounded)
    
    # use entry to update fac receive in format of dictionary
    # update buffer dictionary
    FAC_msg = {"OBU_ID": hearing_ID,
               "timestamp": timestamp,
               "msgCnt": decoded_message["value"][1]["coreData"]["msgCnt"],
               "secMark": decoded_message["value"][1]["coreData"]["secMark"],
               "latitude": decoded_message["value"][1]["coreData"]["lat"] / 10e6,
               "longitude": decoded_message["value"][1]["coreData"]["long"] / 10e6,
               "elevation": decoded_message["value"][1]["coreData"]["elev"] / 10e3,
               "speed": decoded_message["value"][1]["coreData"]["speed"] / 1000,
               "heading": decoded_message["value"][1]["coreData"]["heading"],
               "angle": decoded_message["value"][1]["coreData"]["angle"],
               "time_generated_theory": time_generated_theory,
               "time_generated_bounded": time_generated_bounded}
    
    with open(fac_directory, 'a') as ffile:
        # fac_values = [str(item) for item in FAC_msg.values()]
        fac_values = list(FAC_msg.values())
        json.dump(fac_values, ffile)
        ffile.write("\n")

    # creating FAC_MSG_BUFFER
    if hearing_ID not in FAC_MSG_BUFFER:
        FAC_MSG_BUFFER[hearing_ID] = deque([list(FAC_msg.values())], maxlen=10)
    else:
        FAC_MSG_BUFFER[hearing_ID].appendleft(list(FAC_msg.values()))


def buffer_limit(buffer):
    if len(buffer) > 10:
        buffer.pop()
        buffer = buffer[:10]
    return buffer


def finding_diffmin(array1, value):
    latest_msg = value
    array1_diff = [abs(item - latest_msg) for item in array1]
    min_value = min(array1_diff)
    return array1_diff.index(min_value), min_value


def integrate_time(timestamp, hearing_secmark):
    floored_min = timestamp // 60000
    reminder_ms = timestamp % 60000
    if reminder_ms >= hearing_secmark:
        time_integrated = hearing_secmark + floored_min * 60000
    else:
        time_integrated = hearing_secmark + (floored_min - 1) * 60000
    return time_integrated


def generate_fac_time(timestamp, secmark):
    time_generated = integrate_time(timestamp, secmark)
    if abs(time_generated - timestamp) > 50000:
        time_generated = time_generated + 60000
        if time_generated > timestamp:
            time_generated = timestamp
    return time_generated


def main() -> None:
    with create_cms_api(host='192.168.0.54') as api:
        api.nav_subscribe(nav_callback)
        api.fac_subscribe(FacMsgType.FAC_MSG_US_BSM, fac_callback)
        time_main_start = -10000 # starting with large, then overwrite
        intended_frequency = 50 # ms
        print_counter = 0
        sync_in_main = []
        while True:
            time_current = round(time.time() * 1000)
            time_interval_since_lastrun = time_current - time_main_start
            if time_interval_since_lastrun > intended_frequency:
                # reset the time_main_start
                time_main_start = time_current
                print_counter += 1
                if len(sync_buffer) != 0:
                    sync_in_main = list(element[0] for element in sync_buffer.values())
                    sync_in_main.extend([time_current])
            
                # print(sync_in_main)
                if len(sync_in_main) > 1:
                    #    print(sync_info)
                    #    print(sync_info[0][0], round(time.time() * 1000))
                    control_input = controller_input(sync_in_main)
                    # print('ctrl_input', control_input)
                    speed_hdwy = list(control_input.values())
                    ctrl_intermediate = []
                    for sub in speed_hdwy:
                        for element in sub:
                            ctrl_intermediate.append(element)
                    control_input_save = [sync_in_main[0], time_current, ctrl_intermediate, sync_in_main[1]]

                    if Want_to_check_msg is True:
                        print('other vehicle info received, syncing')
                        if print_counter > 20:
                            print('Current time is:', time_current)

                    with open(ctrl_directory, 'a') as file:
                        json.dump(control_input_save, file)
                        file.write("\n")
                else:
                    print("Other vehicle exist, wait for synced info.")
                    pass
                

if __name__ == "__main__":
    main()
