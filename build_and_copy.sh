#!/bin/bash
echo "building kernal"

if [ "$1" == "clean" ] ;
then
    docker run -v $PWD:/xenial -w /xenial --privileged  -it 831510679322.dkr.ecr.eu-central-1.amazonaws.com/ubuntu_xenial_16.04:basic make clean
fi

docker run -v $PWD:/xenial -w /xenial --privileged  -it 831510679322.dkr.ecr.eu-central-1.amazonaws.com/ubuntu_xenial_16.04:basic make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs modules


sdcard_path=`lsblk | grep -i mmcblk0p1 | awk -F" " '{print $7}'`
if [ -n "$sdcard_path" ] ;
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
