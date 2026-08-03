#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void usb_cdc_activate(void* cdc_acm_instance);
void usb_cdc_deactivate(void* cdc_acm_instance);
void usb_cdc_parameter_change(void* cdc_acm_instance);

#ifdef __cplusplus
}
#endif
