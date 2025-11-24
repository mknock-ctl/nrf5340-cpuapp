#ifndef ROBOT_SETTINGS_H
#define ROBOT_SETTINGS_H

#include <zephyr/settings/settings.h>
typedef struct {
    const char *name; // Setting name
    void *ptr;        // Pointer to variable to store value
    size_t size;      // Size of the variable
} setting_entry_t;

int settings_init_and_register(const char *namespace, const setting_entry_t *entries,
                               size_t num_entries);
int settings_save_float(const char *key, float value);

#endif
