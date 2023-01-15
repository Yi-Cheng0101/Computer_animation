#include "kinematics.h"

#include <algorithm>

#include "utils.h"
#include <math.h>
#include <vector>
#include <queue>
using namespace std;
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton

  // Write your code here
  queue<Bone*> q;
  vector<bool> discorvered(64, false);
  Bone *parent = nullptr;
  Bone *child  = nullptr;
  

  // push the root and label the root as a discorvered point
  q.push(bone);
  discorvered[bone->idx] = true;
  
  // initialize the root 
  bone->startPosition = posture.translations[bone->idx];
  bone->endPosition = bone->startPosition + bone->direction * bone->length;
  // ------------- Rasf -----------------------  ----------- Ramc -----------
  bone->rotation = posture.rotations[bone->idx];
  

  // loop till queue is empty
  while (!q.empty()) {
    
    parent = q.front();
    q.pop();

    if (parent->child != nullptr) {
      child = parent->child;
      q.push(child);
      discorvered[child->idx]=true;
      

      // update the value of the child
      child->startPosition = parent->endPosition;
      // bone->rotation = Rasf * Ramc
      // ---------------------------------------- Rasf -----------------------  ----------- Ramc ------------
      child->rotation = child->parent->rotation * child->rotationParentCurrent * posture.rotations[child->idx];
      child->endPosition = child->startPosition + child->rotation * child->direction * child->length;
      
      while (child->sibling != nullptr) {
        if (discorvered[child->sibling->idx] == false) {
          q.push(child->sibling);
          discorvered[child->sibling->idx] = true;
        }
        
        // update the child
        child = child->sibling;

        // update the value of the sibling 
        child->startPosition = parent->endPosition;
        // bone->rotation = Rasf * Ramc
        // --------------------------------------- Rasf -----------------------  ----------- Ramc ------------
        child->rotation = child->parent->rotation * child->rotationParentCurrent * posture.rotations[child->idx];
        child->endPosition = child->startPosition + child->rotation * child->direction * child->length;
      }
    }

  }

}

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());
  for (int i = 0; i < totalFrames; ++i) {
    // Maybe set some per=Frame variables here
    for (int j = 0; j < totalBones; ++j) {
      // TODO (Time warping)
      // original: |--------------|---------------|
      // new     : |------------------|-----------|
      // OR
      // original: |--------------|---------------|
      // new     : |----------|-------------------|
      // You should set these variables:
      //     newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      //     newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      // The sample above just set to initial state
      // Hint:
      //   1. Your should scale the frames before and after key frames.
      //   2. You can use linear interpolation with translations.
      //   3. You should use spherical linear interpolation for rotations.

      // Write your code here
      if (i <= newKeyframe) {
        float pre_interval = (float)oldKeyframe/(float)newKeyframe;
        float pos = pre_interval * (float)i;
        float a = floor(pos);
        if (a>oldKeyframe) a = oldKeyframe; 
        float b = a + 1;
        if (b>oldKeyframe) b = oldKeyframe;
        float ratio = pos - (int)pos;
        // translations
        //newMotion.posture(i).translations[j] = motion.posture(a).translations[j] * (1-ratio) + motion.posture(b).translations[j] * (ratio);    
        newMotion.posture(i).translations[j] = motion.posture(a).translations[j] + (motion.posture(b).translations[j]-motion.posture(a).translations[j]) * ratio;
         // rotations
        Eigen::Quaternionf q = motion.posture(a).rotations[j].slerp(ratio, motion.posture(b).rotations[j]);
        newMotion.posture(i).rotations[j] = q;
      }
      
      
    
      if (i > newKeyframe) {
        //float pos_interval = (float)(totalFrames-oldKeyframe)/(float)(totalFrames-newKeyframe);
        //float pos = pos_interval * (float)i;
          float pos_interval = (float)(totalFrames-oldKeyframe - 1)/(float)(totalFrames-newKeyframe - 1);
        float pos = oldKeyframe + pos_interval * (float)(i - newKeyframe);
        float a = floor(pos);
        float b = a + 1;
        if (b > (totalFrames-1) ) {
            b = totalFrames-1;
        }
        float ratio = pos - (int)pos;
        // translations
        //newMotion.posture(i).translations[j] = motion.posture(a).translations[j] * (1-ratio) + motion.posture(b).translations[j] * (ratio);
        newMotion.posture(i).translations[j] = motion.posture(a).translations[j] + (motion.posture(b).translations[j]-motion.posture(a).translations[j]) * ratio;
        // rotations
        //Eigen::Quaternionf q = motion.posture(a).rotations[j].slerp((1-ratio), motion.posture(b).rotations[j]);
        Eigen::Quaternionf q = motion.posture(a).rotations[j].slerp(ratio, motion.posture(b).rotations[j]);
        newMotion.posture(i).rotations[j] = q;
      }
    
     
    }
  }
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.
  
  // Write your code here
  
  return newMotion;
}
