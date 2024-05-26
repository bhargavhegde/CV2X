/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stdio.h>

#include <cms_v2x/api.h>
#include <cms_v2x/cbp.h>

/** @file
@brief Get Context Based Prioritisation status
@ingroup ex
*/

static bool l_print_status_json(cms_cbp_status_t* status);

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create connected session to host */
    cms_session_t session = cms_get_session();
    bool error = cms_api_connect(&session, host, 2U, NULL, NULL);

    /* Get CBP status and print it in JSON format */
    cms_cbp_status_t status = {0};
    error = error || cms_cbp_get_status(&session, NULL, &status);
    error = error || l_print_status_json(&status);

    cms_api_disconnect(&session);

    return error ? 1 : 0;
}

static bool l_print_status_json(cms_cbp_status_t* status)
{
    bool error = (NULL == status);

    if(!error) {
        printf("{");
        printf("\"lower_pps_threshold\": %d, ", status->lower_pps_threshold);
        printf("\"upper_pps_threshold\": %d, ", status->upper_pps_threshold);
        printf("\"measured_pps\": %d, ", status->measured_pps);
        printf("\"filtering_stage\": %d, ", status->filtering_stage);

        printf("\"levels\": [");
        printf("{\"cnt\": %d, \"pps\": %d}", status->levels[0].cnt, status->levels[0].pps);
        for(size_t i = 1; i < CMS_CBP_PRIO_NUM_OF_LEVELS; i++) {
            printf(", {\"cnt\": %d, \"pps\": %d}", status->levels[i].cnt, status->levels[i].pps);
        }
        printf("], ");

        printf("\"addr_cnt\": %d", status->addr_cnt);
        printf("}\n");
    }

    return error;
}
