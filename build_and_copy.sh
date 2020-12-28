#!/bin/bash
echo "building kernal"
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs modules

if [ -d "/media/orensanderovich/4C89-2DED/" ] ;
then 
    echo "Copying zImage to sdcard"
    cp ./arch/arm/boot/zImage /media/orensanderovich/4C89-2DED/   

    echo "Copying dtb file to sdcard"
    cp ./arch/arm/boot/dts/rk3288-miniarm.dtb /media/orensanderovich/4C89-2DED/
    cp ./arch/arm/boot/dts/rk3288-miniarm-ar0135.dtb /media/orensanderovich/4C89-2DED/

    echo "Syncing"
    sync -f /media/orensanderovich/4C89-2DED/

    for line in $(mount | grep orensanderovich | awk -F' ' '{print $3}'); 
    do 
        echo "unmounting $line" ; 
        sudo umount $line ; 
    done
fi    

echo "copying to dmskit"
cp ./arch/arm/boot/zImage ../dms-kit/fs/boot/zImage
cp ./arch/arm/boot/dts/rk3288-miniarm.dtb ../dms-kit/fs/opt/DMSKit/dtb/AR0144/rk3288-miniarm.dtb    
cp ./arch/arm/boot/dts/rk3288-miniarm-ar0135.dtb ../dms-kit/fs/opt/DMSKit/dtb/AR0135/rk3288-miniarm-ar0135.dtb
    
echo "Done"
