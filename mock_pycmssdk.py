import time
from typing import Any, Callable, List

# Constants
WILDCARD = 0xFFFF

class MacAddr:
    def __init__(self, *args):
        self.addr = args

    def __repr__(self):
        return ":".join(f"{b:02x}" for b in self.addr)

class SignMethod:
    SIGN_METH_SIGN_CERT = 1

class RadioTxParams:
    def __init__(self, interface_id, dest_address, datarate=None, tx_power=None, user_prio=None, expiry_time=None, sps_index=None):
        self.interface_id = interface_id
        self.dest_address = dest_address
        self.datarate = datarate
        self.tx_power = tx_power
        self.user_prio = user_prio
        self.expiry_time = expiry_time
        self.sps_index = sps_index

class WsmpTxHdrInfo:
    def __init__(self, psid):
        self.psid = psid

class SecDot2TxSignInfo:
    def __init__(self, sign_method, psid):
        self.sign_method = sign_method
        self.psid = psid
        self.ssp = type('SSP', (object,), {'length': 0})()

class SecDot2TxInfo:
    def __init__(self, sign_info):
        self.sign_info = sign_info

class WsmpSendData:
    def __init__(self, radio, wsmp_hdr, security):
        self.radio = radio
        self.wsmp_hdr = wsmp_hdr
        self.security = security

class WsmpTxNotifData:
    def __init__(self, radio, wsmp_hdr, security):
        self.radio = radio
        self.wsmp_hdr = wsmp_hdr
        self.security = security

class MockStationIdData:
    def __init__(self):
        # Mock ID bytes (e.g., 'TEST_ID')
        self.id = [0x54, 0x45, 0x53, 0x54, 0x5F, 0x49, 0x44]

class MockStationId:
    def __init__(self):
        self.data = MockStationIdData()

class MockCmsApi:
    def __init__(self, host):
        self.host = host
        print(f"[MockSDK] Connecting to V2X Stack at {host}...")
        self.subscriptions = {}

    def __enter__(self):
        print("[MockSDK] API Connection Established.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("[MockSDK] API Connection Closed.")

    def wsmp_tx_subscribe(self, psid, callback: Callable):
        print(f"[MockSDK] Subscribed to WSMP TX notifications for PSID: {hex(psid)}")
        self.subscriptions[psid] = callback

    def sti_get_station_id(self):
        print("[MockSDK] Getting Station ID...")
        return MockStationId()

    def wsmp_send(self, send_data: WsmpSendData, buffer: bytes):
        print(f"[MockSDK] Broadcasting WSMP Message:")
        print(f"  > Dest: {send_data.radio.dest_address}")
        print(f"  > PSID: {hex(send_data.wsmp_hdr.psid)}")
        print(f"  > Payload: {buffer.decode(errors='replace')}")
        
        # Simulate local loopback notification if referenced in examples
        # (Though wsmp_send_customize doesn't explicitly rely on loopback confirmation logic for flow control)
        # We can trigger the callback to be helpful
        if send_data.wsmp_hdr.psid in self.subscriptions:
            mock_notif_data = WsmpTxNotifData(
                radio=send_data.radio,
                wsmp_hdr=send_data.wsmp_hdr,
                security=send_data.security
            )
            # Simulate slight network delay
            # time.sleep(0.01)
            # self.subscriptions[send_data.wsmp_hdr.psid](send_data.wsmp_hdr.psid, mock_notif_data, buffer)


def create_cms_api(host):
    return MockCmsApi(host)
