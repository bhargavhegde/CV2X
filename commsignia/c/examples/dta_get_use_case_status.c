/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <cms_v2x/api.h>
#include <cms_v2x/dta.h>

/** @file
@brief Get triggering status of C2C-DTA use-cases.
@ingroup ex
*/

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    cms_dta_use_case_status_t status = {0};

    error = error || cms_dta_get_use_case_status(&session, NULL, &status);
    if(error) {
        fprintf(stderr, "Unable to get DTA triggering status\n");
    } else {
        fprintf(stderr, "DTA triggering status:\n");
        fprintf(stderr, "  EEBL: %s\n", (status.electronic_emergency_brake_light) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  ABI: %s\n", (status.automatic_brake_intervention) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  RORSI: %s\n", (status.reversible_occupant_restraint_system_intervention) ? "Triggering" :
                "Not triggering");
        fprintf(stderr, "  Fog: %s\n", (status.fog) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  Precipitation: %s\n", (status.precipitation) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  Traction Loss: %s\n", (status.traction_loss) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  Postcrash: %s\n", (status.postcrash) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  Broken down vehicle: %s\n", (status.broken_down_vehicle) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  Stopped vehicle: %s\n", (status.stopped_vehicle) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  DEOQ: %s\n", (status.dangerous_end_of_queue) ? "Triggering" : "Not triggering");
        fprintf(stderr, "  TJA: %s\n", (status.traffic_jam_ahead) ? "Triggering" : "Not triggering");
    }

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
