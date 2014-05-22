#!/bin/sh
# IMPORTANT: Specify the location where .o files of kernel modules reside. In KERNEL_OBJ_PATH.
# Specify where to look for out/target/... resides. In APQ_KERNEL.
# Specify options such as 
#   "Device Tree Support", 
#   "To include HID code or not" and 
#   "to include integration code for APQ8074 or not"
#APQ_KERNEL=../
KERNEL_OBJ_PATH=${APQ_KERNEL}/out/target/product/msm8974/obj/KERNEL_OBJ
KERNEL_TOOL_RELATIVE_PATH=./../../../../../../prebuilts/arm-eabi-4.7/bin/arm-eabi-
BOARD_SUPPORT_OPTIONS="MHL_PRODUCT_NUM=8620 LINUX_KERNEL_VER=304 DT_SUPPORT=1 INCLUDE_HID=0 INCLUDE_SII6031=1  MHL_BUILD_NUM=1.02.03"

# Look at the settings just in case something stands out
echo ${BOARD_SUPPORT_OPTIONS} ${KERNEL_OBJ_PATH} ${KERNEL_TOOL_RELATIVE_PATH} ${APQ_KERNEL}

# Now build.
make ARCH=arm KERNELPATH=${KERNEL_OBJ_PATH} CROSS_COMPILE=${KERNEL_TOOL_RELATIVE_PATH} ${BOARD_SUPPORT_OPTIONS} clean debug

