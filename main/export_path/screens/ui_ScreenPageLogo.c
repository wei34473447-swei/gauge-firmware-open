// Project-owned startup branding, using original iN/X vector assets.
#include "../ui.h"
#include "../ui_phase1.h"
void ui_ScreenPageLogo_screen_init(void)
{
    ui_ScreenPageLogo=lv_obj_create(NULL);
    ui_phase1_brand_create(ui_ScreenPageLogo);
    imageLogo=NULL;
    lv_obj_add_event_cb(ui_ScreenPageLogo,ui_event_logo_background,LV_EVENT_ALL,NULL);
}
