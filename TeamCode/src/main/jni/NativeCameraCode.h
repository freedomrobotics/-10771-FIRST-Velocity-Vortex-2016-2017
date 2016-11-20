//
// Created by joelv on 11/16/2016.
//

#include <jni.h>
#include <android/log.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#ifndef VORTEXVELOCITY_NATIVECAMERACODE_H
#define VORTEXVELOCITY_NATIVECAMERACODE_H

#include <Vuforia/CameraDevice.h>
extern "C"
{

class NativeCameraCode : public Vuforia::CameraDevice{
};


}

#endif //VORTEXVELOCITY_NATIVECAMERACODE_H
