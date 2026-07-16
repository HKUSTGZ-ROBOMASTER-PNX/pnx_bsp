#include "bridge_usb.h"

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

static UX_SLAVE_CLASS_CDC_ACM* cdc_acm_handle = UX_NULL;

void usb_cdc_activate(void* cdc_acm_instance)
{
    cdc_acm_handle = (UX_SLAVE_CLASS_CDC_ACM*)cdc_acm_instance;
}

void usb_cdc_deactivate(void* cdc_acm_instance)
{
    (void)cdc_acm_instance;
    cdc_acm_handle = UX_NULL;
}

UX_SLAVE_CLASS_CDC_ACM* usb_cdc_handle(void)
{
    return cdc_acm_handle;
}
