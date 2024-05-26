/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2022-2023
*/

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <cms_v2x/api.h>
#include <cms_v2x/gf.h>

/** @file
@brief Set Geofencing module control state and subscribe for notifications.
@ingroup ex
*/

/* Flag that is set when the notification callback gets called. */
static bool l_cb_called = false;

/* mutex to protect l_cb_called */
static pthread_mutex_t l_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Notification callback to print received message details */
static void gf_notif_cb(const cms_gf_notif_data_t* gf_notif, void* ctx)
{
    (void)ctx;

    if(NULL == gf_notif) {
        fprintf(stderr, "%s NULL argument\n", __func__);
    } else if(pthread_mutex_lock(&l_mutex)) {
        fprintf(stderr, "%s Failed to lock mutex\n", __func__);
    } else {
        l_cb_called = true;
        pthread_mutex_unlock(&l_mutex);
        printf("Notification callback, the state of the module %s has changed to %s\n",
               cms_gf_module_to_string(gf_notif->module),
               (gf_notif->module_action == CMS_GF_MODULE_ACTION_ENABLE) ? "enabled" : "disabled");
    }

}

static bool was_cb_called()
{
    bool result = false;
    if(!pthread_mutex_lock(&l_mutex)) {
        result = l_cb_called;
    } else {
        fprintf(stderr, "%s Failed to lock mutex\n", __func__);
    }
    pthread_mutex_unlock(&l_mutex);
    return result;
}

bool set_module_control(cms_session_t* session, cms_gf_module_t module, cms_gf_module_state_t module_state)
{
    cms_gf_module_control_t module_control_to_set = {
        .module = module,
        .module_state = module_state
    };
    printf("Setting module control of %s to %s...\n",
           cms_gf_module_to_string(module),
           cms_gf_module_state_to_string(module_state));
    bool error = cms_gf_set_module_control(session, &module_control_to_set, NULL);
    error = error || (0 != sleep(1));
    return error;
}

bool get_and_print_module_control(cms_session_t* session, cms_gf_module_t module, cms_gf_module_state_t* module_state)
{
    cms_gf_module_ref_t module_ref = {
        .module = module
    };
    cms_gf_module_control_t module_control = {0};
    bool error = cms_gf_get_module_control(session, &module_ref, &module_control);
    if(!error) {
        if(NULL != module_state) {
            *module_state = module_control.module_state;
        }
        printf("Module control of %s is %s\n",
               cms_gf_module_to_string(module_control.module),
               cms_gf_module_state_to_string(module_control.module_state));
    }
    return error;
}

bool set_gf_state(cms_session_t* session, bool enable)
{
    cms_gf_state_t state_to_set = {.enable = enable};
    printf("%s geofencing module...\n", enable ? "Activating" : "Deactivating");
    bool error = cms_gf_enable(session, &state_to_set, NULL);
    error = error || (0 != sleep(1));
    return error;
}

bool get_and_print_gf_state(cms_session_t* session, cms_gf_state_t* state)
{
    cms_gf_state_t current_state;
    bool error = cms_gf_get_enabled(session, NULL, &current_state);
    if(!error) {
        if(NULL != state) {
            *state = current_state;
        }
        printf("Geofencing module is %s\n", current_state.enable ? "enabled" : "disabled");
    }
    return error;
}

int main(int argc, char* argv[])
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";

    /* Create a session */
    cms_session_t session = cms_get_session();

    /* Connect to the host */
    bool error = cms_api_connect_easy(&session, host);

    /* Subscribe to geofencing notifications. */
    cms_subs_id_t subs_id = CMS_SUBS_ID_INVALID;
    error = error || cms_gf_subscribe(&session,
                                      gf_notif_cb,
                                      NULL,
                                      &subs_id);

    /* Query the state of the geofencing module */
    cms_gf_state_t gf_original_state = {0};
    error = error || get_and_print_gf_state(&session, &gf_original_state);

    /* Activate the geofencing module if not already active*/
    if((!error) && (!gf_original_state.enable)) {
        error = set_gf_state(&session, true);

        error = error || get_and_print_gf_state(&session, NULL);
    }

    cms_gf_module_state_t original_gnp_state = 0;
    error = error || get_and_print_module_control(&session, CMS_GF_MODULE_GNP, &original_gnp_state);

    if(CMS_GF_MODULE_STATE_GEOFENCED != original_gnp_state) {
        error = error || pthread_mutex_lock(&l_mutex);
        l_cb_called = false;
        error = error || set_module_control(&session, CMS_GF_MODULE_GNP, CMS_GF_MODULE_STATE_GEOFENCED);
        error = error || pthread_mutex_unlock(&l_mutex);
    }

    error = error || get_and_print_module_control(&session, CMS_GF_MODULE_GNP, NULL);

    printf("Waiting for up to 30 seconds if notification callback gets called\n");
    for(int i = 0; i < 30; ++i) {
        if(error || was_cb_called()) {
            break;
        }
        error = error || (0 != sleep(1));
    }

    error = error || pthread_mutex_lock(&l_mutex);
    l_cb_called = false;
    error = error || set_module_control(&session, CMS_GF_MODULE_GNP, CMS_GF_MODULE_STATE_PRESERVE);
    error = error || pthread_mutex_unlock(&l_mutex);

    error = error || get_and_print_module_control(&session, CMS_GF_MODULE_GNP, NULL);

    printf("Waiting for up to 30 seconds if notification callback gets called\n");
    for(int i = 0; i < 30; ++i) {
        if(error || was_cb_called()) {
            break;
        }
        error = error || (0 != sleep(1));
    }

    if(CMS_GF_MODULE_STATE_PRESERVE != original_gnp_state) {
        error = error || set_module_control(&session, CMS_GF_MODULE_GNP, original_gnp_state);

        error = error || get_and_print_module_control(&session, CMS_GF_MODULE_GNP, NULL);
    }

    /* Deactivate geofencing if it was not active at start*/
    if((!error) && (!gf_original_state.enable)) {
        error = set_gf_state(&session, false);

        error = error || get_and_print_gf_state(&session, NULL);
    }

    /* Unsubscribe */
    error = error || cms_gf_unsubscribe(&session, subs_id);

    /* Close connection and cleanup */
    cms_api_disconnect(&session);
    cms_api_clean();

    return (int)error;
}
