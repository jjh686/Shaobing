#include "rm_chassis_comm/basic_framework_port.h"
#include <math.h>
#include <string.h>
int rm_basic_map(const rm_output_t *input,const rm_basic_calibration_t *cal,rm_basic_values_t *out)
{
  float values[3],scales[3],limits[3],mapped[3];unsigned i;
  if(!out)return 0;
  memset(out,0,sizeof(*out));
  if(!input||!cal||!input->enabled||!cal->verified)return 0;
  values[0]=input->vx;values[1]=input->vy;values[2]=input->wz;
  scales[0]=cal->scale_vx;scales[1]=cal->scale_vy;scales[2]=cal->scale_wz;
  limits[0]=cal->max_abs_vx;limits[1]=cal->max_abs_vy;limits[2]=cal->max_abs_wz;
  for(i=0;i<3;++i){
    if(!isfinite(values[i])||!isfinite(scales[i])||scales[i]==0||
      !isfinite(limits[i])||limits[i]<=0)return 0;
    mapped[i]=values[i]*scales[i];
    if(!isfinite(mapped[i])||fabsf(mapped[i])>limits[i])return 0;
  }
  out->navigation_enabled=true;out->vx=mapped[0];out->vy=mapped[1];out->wz=mapped[2];
  /* Already body coordinates: never rotate by the gimbal angle a second time. */
  out->offset_angle=0;return 1;
}
