# native built-in
from collections import deque


def synchronization(NAV_MSG_BUFFER, FAC_MSG_BUFFER):
    NAV_profiles = list(NAV_MSG_BUFFER['NAV_msg'])
    NAV_time = [item[0] for item in NAV_profiles]
    FAC_profiles = list(FAC_MSG_BUFFER.values())
    FAC_time = [item[0][-1] for item in FAC_profiles]
    # print('FAC_time', FAC_time)
    latest_FAC_time = min(FAC_time) # reference earliest among all latest updates to sync FAC message for integration
    sync_buffer = {}
    if len(NAV_time) > 1:
        NAVindex, _ = finding_diffmin(NAV_time, latest_FAC_time)
        print('nav_ind', NAVindex)
        # sync_buffer['latest_NAV_time'] = NAV_time[0]
        if 'latest_NAV_time' not in sync_buffer:
            sync_buffer['latest_NAV_time'] = deque([NAV_time[0]], maxlen=20)
        else:
            sync_buffer['latest_NAV_time'].appendleft(NAV_time[0])

        # pc_time = round(time.time() * 1000)
        # if 'pc_time' not in sync_buffer:
        #     sync_buffer['pc_time'] = deque([pc_time], maxlen=20)
        # else:
        #     sync_buffer['pc_time'].appendleft(pc_time)

        # sync_buffer['synced_NAV_profile'] = NAV_profiles[NAVindex]
        if 'synced_NAV_profile' not in sync_buffer:
            sync_buffer['synced_NAV_profile'] = deque([NAV_profiles[NAVindex]], maxlen=20)
        else:
            sync_buffer['synced_NAV_profile'].appendleft(NAV_profiles[NAVindex])
        
        for keys in FAC_MSG_BUFFER:
            FAC_history = list(FAC_MSG_BUFFER[keys])
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
        

def finding_diffmin(array1, value):
    latest_msg = value
    array1_diff = [abs(item - latest_msg) for item in array1]
    min_value = min(array1_diff)
    return array1_diff.index(min_value), min_value
