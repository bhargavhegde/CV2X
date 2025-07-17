# native import
import os
import json
import argparse
from collections import deque
# self-written module, already in same directory
from utility import synchronization, finding_diffmin

with open('./device_dictionary.json','r') as file:
    ID_dictionary = json.load(file)

def read_data(path,no_of_lines, flag):
    data = []
    with open(path, 'r') as file:
        i = 0
        for line in file:
            data.append(json.loads(line))
            i = i+1
            if i > no_of_lines and flag:
                break
    return data

def read_data_fac(path,id,no_of_lines, flag):
    data = []
    with open(path, 'r') as file:
        i = 0
        for line in file:
            if id in line:
                data.append(json.loads(line))
                i = i+1
            if i > no_of_lines and flag:
                break
    return data

def dict_alter(path, id_dictionary):

    alt_dict = {}
    with open(path, 'r') as file:
        for id in id_dictionary:
            search_flag = False
            i=0
            for line in file:
                i+=1
                if id_dictionary[id][1] in line:
                    search_flag = True
                    break
                if i > 10:
                    break
            if (search_flag):
                alt_dict[id] = id_dictionary[id]
    return alt_dict

def offline_synchronization(ts_value):

    timestamp_data = str(ts_value)
    
    user_input_nav_path = "data/" + timestamp_data + "nav.txt"
    user_input_fac_path = "data/" + timestamp_data + "fac.txt"
    id_dict = dict_alter(user_input_fac_path, ID_dictionary)

    nav_data = read_data(user_input_nav_path,100, False)
    filtered_nav = [row for row in nav_data if float(row[0])<=1729789680500]

    nav_msg = {'NAV_msg': deque(nav_data)}
    dict_fac_og={}
    for idx in id_dict:
        dict_fac_og[id_dict[idx][1]] = read_data_fac(user_input_fac_path, id_dict[idx][1], 50,False)

    NAV_profiles = list(nav_msg['NAV_msg'])
    NAV_time = [item[0] for item in NAV_profiles]
    save_filename = timestamp_data + '_offline_sync.txt'
    i = 1
    dict_fac_check={}
    sync_directory = os.path.join("data", save_filename)
    for time in NAV_time:
        filtered_nav = [row for row in nav_data if float(row[0])<=float(time)]
        nav_check = filtered_nav[::-1]
        nav_checked = {'NAV_msg': deque(nav_check)}
        for idx in id_dict:
            fac_data_hold = [row for row in dict_fac_og[id_dict[idx][1]] if float(row[-1])<=float(time)]
            dict_fac_check[id_dict[idx][1]] = deque(fac_data_hold[::-1])
        i = i+1
        percentage_complete = round((i-1)*100/len(NAV_time),2)
        if percentage_complete.is_integer():
            print('Percentage Completed: ',str(percentage_complete), '%')
        sync_trial = synchronization(nav_checked, dict_fac_check)
        if sync_trial:
            sync_save = list(element[0] for element in sync_trial.values())
            with open(sync_directory, 'a') as file:
                json.dump(sync_save, file)
                file.write("\n")
    print('Offline synchronization is now completed. The result is saved in ',save_filename)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--ts', type=str, help='timestamp of the relevant data', required=True)
    args=parser.parse_args()
    offline_synchronization(args.ts)
    
