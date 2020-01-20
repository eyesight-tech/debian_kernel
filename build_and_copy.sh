#!/bin/bash
echo "building kernal"
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs modules

echo "Copying zImage"
cp ./arch/arm/boot/zImage /media/orensanderovich/4C89-2DED/

echo "Copying dtb file"
cp ./arch/arm/boot/dts/rk3288-miniarm.dtb /media/orensanderovich/4C89-2DED/

echo "copying dts file"
cp ./arch/arm/boot/dts/rk3288-miniarm.dts /media/orensanderovich/4C89-2DED/

echo "Syncing"
sync

for line in $(mount | grep orensanderovich | awk -F' ' '{print $3}'); 
do 
    echo "unmounting $line" ; 
    umount $line ; 
done
echo "Done"
