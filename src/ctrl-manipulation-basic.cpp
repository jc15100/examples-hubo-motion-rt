#include "Hubo_Control.h"

#define OPEN_HAND true
#define CLOSE_HAND false

using namespace std;

/// Pre-calculated finger joint limits to avoid self-destructing collision!
/*------------------------
-0.398068
-0.660379
-0.864015
-0.398068
-0.660379
--------------------------*/
/// Pre-calculated transformation matrix for target location; table with foam
/*
0.924661  -0.368975  0.0941248   0.421227
  0.371466   0.928395 -0.0098302  -0.169458
-0.0837579  0.0440537   0.995512  -0.142446
0          0          0          1*/

ach_channel_t chan_hubo_ref;

void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd &values);
Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals, Eigen::VectorXd desired);
void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action);

int main( int argc, char **argv ) {
  Hubo_Control hubo;

  //int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
  //assert(ACH_OK == r);

  //struct hubo_ref H_ref
  //memset(&H_ref, 0, sizeof(H_ref));

  //size_t fs;
  ///r = ach_get(&chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_COPY);
  //assert(ACH_OK == r);

  Eigen::VectorXd dofs(5); dofs << RF1, RF2, RF3, RF4, RF5;
  Eigen::VectorXd vals(5);
  Eigen::VectorXd torques(5);
  Eigen::VectorXd desiredGrasp(5); desiredGrasp << -0.39, -0.66, -0.86, -0.39, -0.66;
  double sum;

  double ptime, dt;
  int i;
  double k = 3;
  Eigen::Isometry3d trans;
  trans(0,0) = 0.924661; trans(0,1) = -0.368975;  trans(0,2) = 0.0941248; trans(0,3) = 0.421227;
  trans(1,0) = 0.371466; trans(1,1) =  0.928395;  trans(1,2) = -0.0098302; trans(1,3) = -0.169458;
  trans(2,0) = -0.0837579; trans(2,1) = 0.0440537; trans(2,2) =  0.995512; trans(2,3) = -0.142446;
  trans(3,0) = 0; trans(3,1) = 0; trans(3,2) = 0; trans(3,3) = 1;
  
  Eigen::Isometry3d cTrans;

  Vector6d armAngles;
  Vector6d current;
  
  Eigen::VectorXd desiredLoc(3); desiredLoc << trans(0,3), trans(1,3), trans(2,3);

  Eigen::VectorXd loc(3);
  bool grasping = false;
  
  //open hand initially
  openCloseHand(torques, dofs, hubo, OPEN_HAND);
  hubo.sendControls();

  while(!daemon_sig_quit){
    hubo.update();
    dt = hubo.getTime() - ptime;

    hubo.getRightArmAngles(current);
    hubo.huboArmIK(armAngles, trans, current, RIGHT);
    
    hubo.huboArmFK(cTrans, current, RIGHT);
    loc = cTrans.translation();
    
    //only do processing if new info has arrived
    if(dt > 0){
      //check for dangerous limits
      getFingersEncValues(hubo, dofs, vals);
      //cout << vals.transpose() << endl;

      // go to desired point
      hubo.setArmAngles(RIGHT, armAngles, true);

      // compute grasp torques
      torques = proportionalGraspController(k, vals, desiredGrasp);
      cout << torques.transpose() << endl;

      //cout << (desiredLoc - loc).norm() << endl;
      if((desiredLoc - loc).norm() <= 0.01 || grasping){
	grasping = true;
	openCloseHand(torques, dofs, hubo, CLOSE_HAND);
      }

      //cout << torques.transpose() << endl;
      hubo.sendControls();
    }
  }
}

void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd& values){  
  for(int i = 0; i < dof.size(); i++)
     values(i) = hubo.getJointAngleState(dof(i));
}

Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals, Eigen::VectorXd desired){
  return -gain*(desired - vals);

}

void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action){
  for(int i=0; i < dofs.size(); i++){
    // set torques to -0.6 if action = OPEN_HAND = true
    torques(i) = (action) ? -0.6: torques(i);
    // apply limits before sending values
    torques(i) = (torques(i) > 1) ? 1 : (torques(i) < -1) ? -1 : torques(i);
    hubo.passJointAngle(dofs(i), torques(i));
  }
}
