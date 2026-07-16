#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void usb_cdc_activate(void* cdc_acm_instance);
void usb_cdc_deactivate(void* cdc_acm_instance);


struct UX_SLAVE_CLASS_CDC_ACM_STRUCT;
typedef struct UX_SLAVE_CLASS_CDC_ACM_STRUCT UX_SLAVE_CLASS_CDC_ACM;

UX_SLAVE_CLASS_CDC_ACM* usb_cdc_handle(void);

#ifdef __cplusplus
}
#endif
