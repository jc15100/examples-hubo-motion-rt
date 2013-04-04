#include "Hubo_Control.h"

ach_channel_t chan_hubo_ref;

int main( int argc, char **argv ) {
  Hubo_Control hubo;
  /*int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
  assert(ACH_OK == r);
  
  struct hubo_ref H_ref;
  memset(&H_ref, 0, sizeof(H_ref));
  
  size_t fs;
  r = ach_get(&chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_COPY);
  
  H_ref.ref[RF1] = -0.4;
  H_ref.ref[RF2] = -0.4;
  H_ref.ref[RF3] = -0.4;
  H_ref.ref[RF4] = -0.4;
  H_ref.ref[RF5] = -0.4;
  ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));*/
  
  hubo.passJointAngle(RF1, -0.5, true);
  

}
