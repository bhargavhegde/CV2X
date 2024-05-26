/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <limits.h>
#include <string.h>
#include <stdio.h>

#include <cms_v2x/api.h>
#include <cms_v2x/vam.h>

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Path prediction */
    cms_vam_path_prediction_t path_prediction = {0};

    /* Error if a coordinate is missing (lat-lon pairs) */
    if(argc % 2 != 0) {
        printf("Odd number of coordinates entered.\n");
        error = true;
    }

    if(!error) {

        /* Get arguments and convert them to coordinates */
        int i = 0;
        for(int j = 2; j < argc; j += 2) {
            path_prediction.points[i].latitude = strtol(argv[j], NULL, 10);
            path_prediction.points[i].longitude = strtol(argv[j + 1], NULL, 10);
            ++i;
        }
        path_prediction.length = i;

        /* Add path prediction points to message */
        error = cms_vam_add_path_prediction(&session, &path_prediction, NULL);
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
