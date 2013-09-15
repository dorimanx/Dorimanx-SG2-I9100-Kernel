#!/bin/sh
export KERNELDIR=`readlink -f .`
export PARENT_DIR=`readlink -f ..`
export INITRAMFS_DEST=$KERNELDIR/kernel/usr/initramfs
export INITRAMFS_SOURCE=`readlink -f ..`/Ramdisk/
export PACKAGEDIR=$KERNELDIR/READY-JB
#Enable FIPS mode
export USE_SEC_FIPS_MODE=true
export ARCH=arm
export CROSS_COMPILE=$PARENT_DIR/arm-eabi-4.7/bin/arm-eabi-
export KERNEL_CONFIG=alucard_defconfig;

time_start=$(date +%s.%N)

echo "Setup Package Directory"
mkdir -p $PACKAGEDIR/system/lib/modules

if [ -d $INITRAMFS_DEST ]; then
	echo "removing old temp initramfs_source";
	rm -rf $INITRAMFS_DEST;
fi;

echo "Create initramfs dir"
mkdir -p $INITRAMFS_DEST

# copy new config
cp $KERNELDIR/.config $KERNELDIR/arch/arm/configs/$KERNEL_CONFIG;

# remove all old modules before compile
for i in `find $KERNELDIR/ -name "*.ko"`; do
	rm -f $i;
done;
for i in `find $PACKAGEDIR/system/lib/modules/ -name "*.ko"`; do
	rm -f $i;
done;

echo "Copy new initramfs dir"
cp -R $INITRAMFS_SOURCE/* $INITRAMFS_DEST

echo "chmod initramfs dir"
chmod -R g-w $INITRAMFS_DEST/*

# clear git repository from tmp-initramfs
if [ -d $INITRAMFS_DEST/.git ]; then
	rm -rf $INITRAMFS_DEST/.git;
fi;

# clear mercurial repository from tmp-initramfs
if [ -d $INITRAMFS_DEST/.hg ]; then
	rm -rf $INITRAMFS_DEST/.hg;
fi;

# remove empty directory placeholders from tmp-initramfs
for i in `find $INITRAMFS_DEST -name EMPTY_DIRECTORY`; do
	rm -f $i;
done;

# copy config
if [ ! -f $KERNELDIR/.config ]; then
	cp $KERNELDIR/arch/arm/configs/$KERNEL_CONFIG $KERNELDIR/.config;
fi;

# read config
. $KERNELDIR/.config;

# get version from config
GETVER=`grep 'Alucard-*-V' .config |sed 's/Alucard-//g' | sed 's/.*".//g' | sed 's/-T.*//g'`;

echo "Remove old zImage"
# remove previous zImage files
if [ -e $PACKAGEDIR/boot.img ]; then
	rm $PACKAGEDIR/boot.img;
fi;

if [ -e $KERNELDIR/arch/arm/boot/zImage ]; then
	rm $KERNELDIR/arch/arm/boot/zImage;
fi;

HOST_CHECK=`uname -n`
NAMBEROFCPUS=$(expr `grep processor /proc/cpuinfo | wc -l` + 1);
echo $HOST_CHECK

echo "Making kernel";
make -j${NAMBEROFCPUS};

echo "Copy modules to Package"
for i in `find $KERNELDIR -name '*.ko'`; do
	cp -av $i $PACKAGEDIR/system/lib/modules/;
done;

for i in `find $PACKAGEDIR/system/lib/modules/ -name '*.ko'`; do
	${CROSS_COMPILE}strip --strip-unneeded $i;
done;

chmod 644 $PACKAGEDIR/system/lib/modules/*;

if [ -e $KERNELDIR/arch/arm/boot/zImage ]; then
	echo "Copy zImage to Package"
	cp arch/arm/boot/zImage $PACKAGEDIR/zImage

	echo "Make boot.img"
	./mkbootfs $INITRAMFS_DEST | gzip > $PACKAGEDIR/ramdisk.gz
	./mkbootimg --cmdline 'console = null androidboot.hardware=qcom user_debug=31 zcache' --kernel $PACKAGEDIR/zImage --ramdisk $PACKAGEDIR/ramdisk.gz --base 0x80200000 --pagesize 2048 --ramdisk_offset 0x02000000 --output $PACKAGEDIR/boot.img 
	cd $PACKAGEDIR

	if [ -e ramdisk.gz ]; then
		rm ramdisk.gz;
	fi;

	if [ -e zImage ]; then
		rm zImage;
	fi;

	echo "Remove old Package Zip Files"
	for i in `find $PACKAGEDIR/ -name '*.zip'`; do
	 rm $i;
	done;

	FILENAME=Kernel-Alucard-${GETVER}-`date +"[%H-%M]-[%d-%m]-TW-EUR-JB4.2-SGIV-PWR-CORE"`.zip
	zip -r $FILENAME .;

	time_end=$(date +%s.%N)
	echo -e "${BLDYLW}Total time elapsed: ${TCTCLR}${TXTGRN}$(echo "($time_end - $time_start) / 60"|bc ) ${TXTYLW}minutes${TXTGRN} ($(echo "$time_end - $time_start"|bc ) ${TXTYLW}seconds) ${TXTCLR}"

	FILESIZE=$(stat -c%s "$FILENAME")
	echo "Size of $FILENAME = $FILESIZE bytes."
	
	cd $KERNELDIR
else
	echo "KERNEL DID NOT BUILD! no zImage exist"
fi;
