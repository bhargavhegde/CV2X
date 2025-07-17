# native import
import unittest
from collections import deque
# self-written module, already in same directory
from utility import synchronization, finding_diffmin

class TestSynchronization(unittest.TestCase):

    def test_synchronization(self):
        # test data
        nav_sample_data = [
            [1723746760500, 43.0040743, -78.7902201, 14.482, 5300, 3.89, 15, 1723746760401],
            [1723746760399, 43.0040708, -78.7902206, 14.481, 4500, 3.86, 15, 1723746760302],
            [1723746760300, 43.0040674, -78.790221, 14.482, 4400, 3.93, 15, 1723746760200],
            [1723746760199, 43.0040638, -78.7902215, 14.482, 3200, 3.92, 15, 1723746760102],
        ]
        NAV_MSG_BUFFER = {'NAV_msg': deque(nav_sample_data, maxlen=20)}

        fac_sample_data1 = [[1723746760542, 28, 40500, 43.0041528, -78.7901213, 0.1395, 0.171, 6024, 127, 1723746760500, 1723746760500], [1723746760356, 26, 40400, 43.0041522, -78.7901255, 0.1395, 0.169, 5768, 127, 1723746700400, 1723746760356]]
        fac_sample_data2 = [[1723746760517, 16, 40500, 43.0041575, -78.7899337, 0.1419, 0.121, 7480, 127, 1723746760500, 1723746760500], [1723746760445, 15, 40400, 43.0041576, -78.7899365, 0.1419, 0.127, 7496, 127, 1723746760400, 1723746760400]]
        FAC_MSG_BUFFER = {
            "2a3c6435": deque(fac_sample_data1, maxlen=20),
            "2a3c645e": deque(fac_sample_data2, maxlen=20)
        }

        # Expected output
        NAV_profiles = list(NAV_MSG_BUFFER['NAV_msg'])

        expected_sync_buffer = {
            'latest_NAV_time': deque([1723746760500], maxlen=20),
            'synced_NAV_profile': deque([NAV_profiles[0]], maxlen=20),
            '2a3c6435': deque([fac_sample_data1[0]], maxlen=20),
            '2a3c645e': deque([fac_sample_data2[0]], maxlen=20),
            
        }

        # Call the function
        sync_buffer = synchronization(NAV_MSG_BUFFER, FAC_MSG_BUFFER)

        # Assertions
        self.assertEqual(sync_buffer['latest_NAV_time'], expected_sync_buffer['latest_NAV_time'])
        self.assertEqual(sync_buffer['synced_NAV_profile'], expected_sync_buffer['synced_NAV_profile'])
        self.assertEqual(sync_buffer['2a3c6435'], expected_sync_buffer['2a3c6435'])
        self.assertEqual(sync_buffer['2a3c645e'], expected_sync_buffer['2a3c645e'])

    def test_finding_diffmin(self):
        array1 = [100, 200, 300]
        value = 200

        # Expected output
        expected_index = 1
        expected_min_value = 0

        # Call the function
        index, min_value = finding_diffmin(array1, value)

        # Assertions
        self.assertEqual(index, expected_index)
        self.assertEqual(min_value, expected_min_value)


if __name__ == '__main__':
    unittest.main()
