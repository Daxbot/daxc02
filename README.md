Dax Circuit 02
============

Status on Jetson TX1:
* Fully working under l4t-24.2 (see releases)
* Compiles/probes under l4t-28.1, but no image

Status on Jetson TX2:
* Compiles/probes under l4t-28.1, but no image

Dev Environment Setup
---------------------
This section will describe the process of setting up the development environment on a fresh Ubuntu installation using [Jetpack](https://developer.nvidia.com/embedded/jetpack).

First download Jetpack and make the file executable

    chmod +x JetPack-L4T-3.1-linux-x64.run

Next transfer the installer to its own folder.

    mkdir ~/l4t_r28.1
    mv JetPack-L4T-3.1-linux-x64.run ~/l4t_r28.1/

Export ```$DEVDIR``` as the new JetPack location.

    export DEVDIR=~/l4t_r28.1

Now run the installer and follow the instructions.  Once you get to the Package list change the action next to "Install on Target" to "no action."  This will install the dev environment but skip flashing the TX1 with the image.  We will be modifying the kernel to include this driver before flashing.

### Download the Source

Download the kernel source code by running the source_sync script.  This will take a few minutes.  Specify which version using the -k tag.

    cd $DEVDIR/64_TX1/Linux_for_Tegra_64_tx1/
    ./source_sync.sh -k tegra-l4t-r28.1

### Toolchain
In addition to the code you'll need the toolchain for cross-compiling the kernel for ARM.  Download the following two binaries.

* [64bit ARM](https://releases.linaro.org/components/toolchain/binaries/5.4-2017.01/aarch64-linux-gnu/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz)
* [32bit ARM](https://releases.linaro.org/components/toolchain/binaries/5.4-2017.01/arm-linux-gnueabihf/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz)

Install using the following commands:

    sudo mkdir /opt/linaro
    sudo chmod -R 775 /opt/linaro
    sudo chown -R $USER /opt/linaro
    tar -xf gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz -C /opt/linaro/
    tar -xf gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz -C /opt/linaro/

### Jetson TX1: Adding the Driver
Export ```$SOURCEDIR``` as the sources directory created by ```source_sync.sh```.

    export SOURCEDIR=$DEVDIR/64_TX1/Linux_for_Tegra_64_tx1/sources

Clone the driver repository to the sources directory.

    git clone https://github.com/DaxBot/daxc02.git $SOURCEDIR/daxc02

Create symbolic links to the new files and insert them into the kernel.

    # Link driver files
    ln -s $SOURCEDIR/daxc02/tegra210-daxc02.dtsi $SOURCEDIR/hardware/nvidia/platform/t210/jetson/kernel-dts/
    ln -s $SOURCEDIR/daxc02/daxc02_mode_tbls.h $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/
    ln -s $SOURCEDIR/daxc02/daxc02.h $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/
    ln -s $SOURCEDIR/daxc02/daxc02.c $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/

    # Update device tree to include new entires
    sed -e '/tegra210-jetson-cv-camera-plugin-manager.dtsi/ s;^;//;' -i $SOURCEDIR/hardware/nvidia/platform/t210/jetson/kernel-dts/tegra210-jetson-cv-base-p2597-2180-a00.dts
    sed -e '/tegra210-jetson-cv-camera-modules.dtsi/ s;^;//;' -i $SOURCEDIR/hardware/nvidia/platform/t210/jetson/kernel-dts/tegra210-jetson-cv-base-p2597-2180-a00.dts
    echo "#include \"tegra210-daxc02.dtsi\"" >> $SOURCEDIR/hardware/nvidia/platform/t210/jetson/kernel-dts/tegra210-jetson-cv-base-p2597-2180-a00.dts

### Jetson TX2: Adding the Driver
Export ```$SOURCEDIR``` as the sources directory created by ```source_sync.sh```.

    export SOURCEDIR=$DEVDIR/64_TX2/Linux_for_Tegra_tx2/sources

Clone the driver repository to the sources directory.

    git clone https://github.com/DaxBot/daxc02.git $SOURCEDIR/daxc02

Create symbolic links to the new files and insert them into the kernel.

    # Link driver files
    ln -s $SOURCEDIR/daxc02/tegra186-daxc02.dtsi $SOURCEDIR/hardware/nvidia/platform/t18x/quill/kernel-dts/
    ln -s $SOURCEDIR/daxc02/daxc02_mode_tbls.h $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/
    ln -s $SOURCEDIR/daxc02/daxc02.h $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/
    ln -s $SOURCEDIR/daxc02/daxc02.c $SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/

    # Update device tree to include new entires
    sed -e '/tegra186-quill-camera-plugin-manager.dtsi/ s;^;//;' -i $SOURCEDIR/hardware/nvidia/platform/t18x/quill/kernel-dts/tegra186-quill-p3310-1000-a00-00-base.dts
    sed -e '/tegra186-quill-camera-modules.dtsi/ s;^;//;' -i $SOURCEDIR/hardware/nvidia/platform/t18x/quill/kernel-dts/tegra186-quill-p3310-1000-a00-00-base.dts
    echo "#include \"tegra186-daxc02.dtsi\"" >> $SOURCEDIR/hardware/nvidia/platform/t18x/quill/kernel-dts/tegra186-quill-p3310-1000-a00-00-base.dts

### Update Kconfig
Insert the following at the top of ```$SOURCEDIR/kernel/kernel-4.4/drivers/media/i2c/Kconfig``` after "```if VIDEO_V4L2```"
```
config VIDEO_I2C_DAXC02
    tristate "DAXC02 camera sensor support"
    depends on I2C && VIDEO_V4L2 && VIDEO_V4L2_SUBDEV_API
    ---help---
        This is a Video4Linux2 sensor-level driver for DAX-C02
```

Insert the following line at the top of ```$SOURCEDIR/drivers/media/i2c/Makefile```
```
obj-$(CONFIG_VIDEO_I2C_DAXC02) += daxc02.o
```

### Compile the Kernel
Export the following environment variables:

    export CROSS_COMPILE=/opt/linaro/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
    export CROSS32CC=/opt/linaro/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
    export TEGRA_KERNEL_OUT=$SOURCEDIR/compiled

Create the default configuration:

    cd $SOURCEDIR/kernel/kernel-4.4
    mkdir $TEGRA_KERNEL_OUT
    make ARCH=arm64 O=$TEGRA_KERNEL_OUT <defconfig>

where `<defconfig>` is either:
* Jetson TX1: tegra21_defconfig
* Jetson TX2: tegra18_defconfig

Enable DAXC02 using menuconfig

    make ARCH=arm64 O=$TEGRA_KERNEL_OUT menuconfig

```
Device Drivers --->
    <*> Multimedia support --->
        <*> DAXC02 camera sensor support
```

Now you can compile the kernel image and device tree blob

    make ARCH=arm64 O=$TEGRA_KERNEL_OUT -j4 zImage dtbs

### Transfer to the TX

    cp $SOURCEDIR/compiled/arch/arm64/boot/Image $SOURCEDIR/../kernel/
    cp $SOURCEDIR/compiled/arch/arm64/boot/zImage $SOURCEDIR/../kernel/
    cp $SOURCEDIR/compiled/arch/arm64/boot/dts/*.dtb $SOURCEDIR/../kernel/dtb/

    cd $SOURCEDIR/../
    sudo ./flash.sh <platform> mmcblk0p1

where `<platform>` is either:
* Jetson TX1: jetson-tx1
* Jetson TX2: jetson-tx2