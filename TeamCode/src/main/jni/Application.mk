#===============================================================================
#Copyright (c) 2012-2014 Qualcomm Connected Experiences, Inc. All Rights Reserved.
#
#Vuforia is a trademark of PTC Inc., registered in the United States and other 
#countries.
#==============================================================================


# Build both ARMv5TE and ARMv7-A machine code.

APP_ABI := armeabi-v7a

# Set target Android API level to the application's minimum SDK version.

APP_PLATFORM := android-19

# This optional variable can be defined to either 'release' or
# 'debug'. This is used to alter the optimization level when
# building your application's modules.
#
# A 'release' mode is the default, and will generate highly
# optimized binaries. The 'debug' mode will generate un-optimized
# binaries which are much easier to debug.
#
# Note that it is possible to debug both 'release' and 'debug'
# binaries, but the 'release' builds tend to provide less information
# during debugging sessions: some variables are optimized out and
# can't be inspected, code re-ordering can make stepping through
# the code difficult, stack traces may not be reliable, etc... 

# APP_OPTIM := release OR debug
