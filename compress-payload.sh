#!/bin/bash

EXTRA_COMPRESS()
{
	cd PAYLOAD/res/misc/payload;

	if [ `ls -A1 | grep -v tar.xz | wc -l` -gt "0" ]; then

		for i in `ls -A1 | grep -v tar.xz`; do
			tar --xz -c $i > ${i}.tar.xz;
		done;

		mv STweaks.apk.tar.xz STweaks.tar.xz
		mv SuperSU.apk.tar.xz SuperSU.tar.xz

		for i in `ls -A1 | grep -v tar.xz`; do
			rm $i;
		done;
	fi;

	cd ../../../../;
}
#EXTRA_COMPRESS #not needed for now

# create a *.tar.xz archive
tar -C PAYLOAD/ -cvpf - . | xz -9 -c - > payload_new.tar.xz;

# check 
stat payload_new.tar.xz || exit 1;

# overwrite
mv payload_new.tar.xz payload.tar.xz;

echo "all done"
