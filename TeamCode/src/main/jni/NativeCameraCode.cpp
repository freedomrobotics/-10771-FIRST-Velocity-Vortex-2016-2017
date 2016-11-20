//
// Created by joelv on 11/19/2016.
//

#include <jni.h>
#include <android/log.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <Vuforia/CameraDevice.h>

#include "NativeCameraCode.h"


extern "C"
{

JNIEXPORT void JNICALL Java_org_fhs_robotics_ftcteam10771_lepamplemousse_actions_maneuvers_CameraClass_setup(JNIEnv *, jobject) {
    __android_log_print(ANDROID_LOG_VERBOSE, "test", "The value of 1 + 1 is %d", 1+1);
}

}