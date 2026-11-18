#ifdef NDEBUG
#undef NDEBUG
#endif
#include "rm_chassis_comm/basic_framework_port.h"
#include <assert.h>
#include <math.h>
int main(void){
  rm_output_t in={true,.5f,.25f,1.f};
  rm_basic_calibration_t cal={0};rm_basic_values_t out;
  assert(!rm_basic_map(&in,&cal,&out)&&!out.navigation_enabled&&out.vx==0);
  /* These numbers only test arithmetic, not the real vehicle's calibration. */
  cal=(rm_basic_calibration_t){true,1000,-1000,10,2000,2000,100};
  assert(rm_basic_map(&in,&cal,&out));
  assert(out.vx==500&&out.vy==-250&&out.wz==10&&out.offset_angle==0);
  in.enabled=false;assert(!rm_basic_map(&in,&cal,&out)&&out.vy==0);
  in.enabled=true;in.vx=NAN;assert(!rm_basic_map(&in,&cal,&out));
  in.vx=3;assert(!rm_basic_map(&in,&cal,&out));
  in.vx=.5;cal.scale_vy=0;assert(!rm_basic_map(&in,&cal,&out));
  cal.scale_vy=-1000;cal.max_abs_wz=INFINITY;assert(!rm_basic_map(&in,&cal,&out));
  return 0;
}
