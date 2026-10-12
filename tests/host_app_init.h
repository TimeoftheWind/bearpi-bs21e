/* Host-only shim: retain the entry type check without an ELF-only section. */
#define APP_INIT_H
#define app_run(func) static void (*const host_app_entry)(void) = (func)
