#!/bin/sh
export PLATFORM="TW"
export MREV="JB4.2"
export CURDATE=`date "+%m.%d.%Y"`
export MUXEDNAMELONG="Alucard-SGS4-$MREV-$PLATFORM-$CARRIER-$CURDATE"
export MUXEDNAMESHRT="Alucard-SGS4-$MREV-$PLATFORM-$CARRIER*"
export KTVER="--$MUXEDNAMELONG--"
export KERNELDIR=`readlink -f .`
export PARENT_DIR=`readlink -f ..`
export INITRAMFS_DEST=$KERNELDIR/kernel/usr/initramfs
export INITRAMFS_SOURCE=`readlink -f ..`/Ramdisks/$PLATFORM"_"$CARRIER
export CONFIG_$PLATFORM_BUILD=y
export PACKAGEDIR=$PARENT_DIR/Packages/$PLATFORM
#Enable FIPS mode
export USE_SEC_FIPS_MODE=true
export ARCH=arm
# export CROSS_COMPILE=$PARENT_DIR/linaro4.5/bin/arm-eabi-
# export CROSS_COMPILE=/home/ktoonsez/kernel/siyah/arm-2011.03/bin/arm-none-eabi-
# export CROSS_COMPILE=/home/ktoonsez/android/system/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
# export CROSS_COMPILE=/home/ktoonsez/aokp4.2/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
export CROSS_COMPILE=$PARENT_DIR/arm-eabi-4.6/bin/arm-eabi-

time_start=$(date +%s.%N)

echo "Remove old Package Files"
rm -rf $PACKAGEDIR/*

echo "Setup Package Directory"
mkdir -p $PACKAGEDIR/system/app
mkdir -p $PACKAGEDIR/system/lib/modules
mkdir -p $PACKAGEDIR/system/etc/init.d

echo "Create initramfs dir"
mkdir -p $INITRAMFS_DEST

echo "Remove old initramfs dir"
rm -rf $INITRAMFS_DEST/*

echo "Copy new initramfs dir"
cp -R $INITRAMFS_SOURCE/* $INITRAMFS_DEST

echo "chmod initramfs dir"
chmod -R g-w $INITRAMFS_DEST/*
rm $(find $INITRAMFS_DEST -name EMPTY_DIRECTORY -print)
rm -rf $(find $INITRAMFS_DEST -name .git -print)

echo "Remove old zImage"
rm $PACKAGEDIR/zImage
rm arch/arm/boot/zImage

echo "Make the kernel"
make VARIANT_DEFCONFIG=jf_$CARRIER"_defconfig" jf_defconfig SELINUX_DEFCONFIG=jfselinux_defconfig

echo "Modding .config file - "$KTVER
sed -i 's,CONFIG_LOCALVERSION="-Alucard-SGS4",CONFIG_LOCALVERSION="'$KTVER'",' .config

HOST_CHECK=`uname -n`
NAMBEROFCPUS=$(expr `grep processor /proc/cpuinfo | wc -l` + 1);
echo $HOST_CHECK
make -j${NAMBEROFCPUS}

echo "Copy modules to Package"
cp -av $(find . -name *.ko -print |grep -v initramfs) $PACKAGEDIR/system/lib/modules/

if [ -e $KERNELDIR/arch/arm/boot/zImage ]; then
	echo "Copy zImage to Package"
	cp arch/arm/boot/zImage $PACKAGEDIR/zImage

	echo "Make boot.img"
	./mkbootfs $INITRAMFS_DEST | gzip > $PACKAGEDIR/ramdisk.gz
	./mkbootimg --cmdline 'console = null androidboot.hardware=qcom user_debug=31 zcache' --kernel $PACKAGEDIR/zImage --ramdisk $PACKAGEDIR/ramdisk.gz --base 0x80200000 --pagesize 2048 --ramdisk_offset 0x02000000 --output $PACKAGEDIR/boot.img 
	cd $PACKAGEDIR
	cp -R ../META-INF .
	rm ramdisk.gz
	rm zImage
	rm ../$MUXEDNAMESHRT.zip
	zip -r ../$MUXEDNAMELONG.zip .

	time_end=$(date +%s.%N)
	echo -e "${BLDYLW}Total time elapsed: ${TCTCLR}${TXTGRN}$(echo "($time_end - $time_start) / 60"|bc ) ${TXTYLW}minutes${TXTGRN} ($(echo "$time_end - $time_start"|bc ) ${TXTYLW}seconds) ${TXTCLR}"

	FILENAME=../$MUXEDNAMELONG.zip
	FILESIZE=$(stat -c%s "$FILENAME")
	echo "Size of $FILENAME = $FILESIZE bytes."
	
	cd $KERNELDIR
else
	echo "KERNEL DID NOT BUILD! no zImage exist"
fi;
