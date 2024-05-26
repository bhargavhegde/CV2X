/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/sti.h>

/** @file
@brief Set and get Station Info parameters.
@ingroup ex
*/


static void print_station_id(const cms_sti_station_id_t* station_id)
{
    printf("Station id: ");
    printf(" 0x%02x 0x%02x 0x%02x 0x%02x  ",
           station_id->id[0],
           station_id->id[1],
           station_id->id[2],
           station_id->id[3]
          );
    printf(" 0x%02x 0x%02x 0x%02x 0x%02x   ",
           station_id->id[4],
           station_id->id[5],
           station_id->id[6],
           station_id->id[7]
          );
    printf(" 0x%02x 0x%02x 0x%02x 0x%02x  ",
           station_id->id[8],
           station_id->id[9],
           station_id->id[10],
           station_id->id[11]
          );
    printf(" 0x%02x 0x%02x 0x%02x 0x%02x\n",
           station_id->id[12],
           station_id->id[13],
           station_id->id[14],
           station_id->id[15]
          );
}


int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Set station id */
    const cms_sti_station_id_t id_to_set = {
        .id = {
            0x01, 0x02, 0x03, 0x04,
            0x31, 0x32, 0x33, 0x34,
            0x41, 0x52, 0x63, 0x74,
            0x00, 0x00, 0x00, 0x00
        }
    };

    error = error || cms_sti_set_station_id(&session, &id_to_set, NULL);
    if(error) {
        printf("Unable to set station ID.\n");
    }

    /* Get station id*/
    cms_sti_station_id_t station_id = {0};
    error = error || cms_sti_get_station_id(&session, NULL, &station_id);
    if(error) {
        printf("Unable to get station ID.\n");
    } else {
        print_station_id(&station_id);
    }

    /* Create and fill an STI set structure for bulk change */
    cms_sti_set_items_t to_set = {0};
    to_set.items[0].type = CMS_STI_LONG_ACC;
    to_set.items[0].value = -13LL;
    to_set.items[1].type = CMS_STI_LIGHTBAR;
    to_set.items[1].value = CMS_STI_LIGHTBAR_SCHOOL_BUS_LIGHTS;
    to_set.items[2].type = CMS_STI_VEHICLE_LENGTH;
    to_set.items[2].value = 120LL;
    to_set.length = 3U;

    /* Set station parameters */
    error = error || cms_sti_set(&session, &to_set, NULL);
    if(error) {
        printf("Unable to set STI parameters\n");
    }

    /* Create and fill an STI get structures for bulk get */
    cms_sti_type_list_t get_types = {0};
    get_types.length = 3UL;
    get_types.types[0] = CMS_STI_LONG_ACC;
    get_types.types[1] = CMS_STI_LIGHTBAR;
    get_types.types[2] = CMS_STI_VEHICLE_LENGTH;

    /* Get station parameters */
    cms_sti_item_list_t sti_values = {0};
    error = error || cms_sti_get(&session, &get_types, &sti_values);
    if(error) {
        printf("Unable to get STI parameters\n");
    } else {

        printf("Number of output STI parameters: %lu\n",
               (unsigned long)sti_values.length);
        printf("\tLongitudinal acceleration:\n\t\tType: %2d\tValue: %lld\n",
               sti_values.items[0].type,
               (long long)sti_values.items[0].value);
        printf("\tLightbar:\n\t\tType: %d\tValue: %lld\n",
               sti_values.items[1].type,
               (long long)sti_values.items[1].value);
        printf("\tVehicle length:\n\t\tType: %d\tValue: %lld\n",
               sti_values.items[2].type,
               (long long)sti_values.items[2].value);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}

