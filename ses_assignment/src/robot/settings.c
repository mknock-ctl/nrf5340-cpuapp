#include "robot/settings.h"
#include "ses_assignment.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(robot_settings, LOG_LEVEL_INF);

static const setting_entry_t *g_entries = NULL;
static size_t g_num_entries = 0;
static struct settings_handler g_handler;

static int delete_setting_key(const char *name)
{
    char key_buf[128];
    int ret;
    
    if (g_handler.name && g_handler.name[0] != '\0') {
        ret = snprintf(key_buf, sizeof(key_buf), "%s/%s", g_handler.name, name);
        if (ret < 0 || ret >= sizeof(key_buf)) {
            return -ENOMEM;
        }
    } else {
        ret = snprintf(key_buf, sizeof(key_buf), "%s", name);
        if (ret < 0 || ret >= sizeof(key_buf)) {
            return -ENOMEM;
        }
    }
    
    TRY_ERR(int, settings_delete(key_buf));
    return 0;
}

static int generic_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                void *cb_arg)
{
    const char *next;
    int rc;
    
    for (size_t i = 0; i < g_num_entries; i++) {
        if (settings_name_steq(name, g_entries[i].name, &next) && !next) {
            if (len != g_entries[i].size) {
                delete_setting_key(name);
                return -EINVAL;
            }
            
            rc = read_cb(cb_arg, g_entries[i].ptr, g_entries[i].size);
            if (rc >= 0) {
                // Only log as float if size matches float size
                if (g_entries[i].size == sizeof(float)) {
                    LOG_DBG("Restored %s with %.3f", g_entries[i].name,
                            (double)(*(float *)g_entries[i].ptr));
                } else {
                    LOG_DBG("Restored %s (%zu bytes)", g_entries[i].name, g_entries[i].size);
                }
            } else {
                delete_setting_key(name);
            }
            return (rc < 0) ? rc : 0;
        }
    }
    return -ENOENT;
}

int settings_init_and_register(const char *namespace, const setting_entry_t *entries,
                               size_t num_entries)
{
    g_entries = entries;
    g_num_entries = num_entries;
    g_handler.name = namespace;
    g_handler.h_set = generic_settings_set;
    
    TRY_ERR(int, settings_subsys_init());
    TRY_ERR(int, settings_register(&g_handler));
    TRY_ERR(int, settings_load());
    
    LOG_DBG("Settings initialized for namespace '%s'", namespace);
    return 0;
}

int settings_save_float(const char *key, float value)
{
    TRY_ERR(int, settings_save_one(key, &value, sizeof(value)));
    LOG_DBG("Saved '%s' = %.3f", key, (double)value);
    return 0;
}