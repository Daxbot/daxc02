Dax Circuit 02
============

Dev Environment Setup
---------------------
JetPack is currently only supported for Linux.  This section will describe the process of setting up the dev environment on a fresh Ubuntu VM.

If you're working with a VirtualBox VM I recommend installing "Guest Additions" to enable features such as shared clipboard and file drag-and-drop.  See the article [here](https://www.virtualbox.org/manual/ch04.html) for installation instructions. 

Once the VM is set up download [Jetpack](https://developer.nvidia.com/embedded/jetpack), Nvidia's SDK package manager 

Navigate to the downloads directory, open a termminal (right click, 'Open in Terminal'), and make the download executable ```chmod +x JetPack-L4T-3.0-linux-x64.run```.  You'll also want to transfer the installer to its own folder.  ```mkdir ~/l4t; mv JetPack-L4T-3.0-linux-x64.run ~/l4t/; cd ~/l4t/```

Next run the installer and follow the instructions ```./JetPack-L4T-3.0-linux-x64.run```.  Once you get to the Package list change the action next to "Install on Target" to "no action."  This will install the dev environment but skip flashing the TX1 with the default image.  We will be modifying the kernel to include the DAX drivers before flashing.

To make some things easier define the following environment variable.  You may want to add this definition to the bottom of your ~/.bashrc file so it is defined whenever a new terminal is opened.  ```export DEVDIR=~/l4t/```

The following steps are taken from Ridgerun's Guide: https://developer.ridgerun.com/wiki/index.php?title=Compiling_Tegra_X1_source_code

### Download the Source

Download the kernel source code by running the source_sync script.  This will take a few minutes.  Specify which version using the -k and -u tags.
```bash
cd $DEVDIR/64_TX1/Linux_for_Tegra_64_tx1/
./source_sync.sh -k tegra-l4t-r24.2 -u tegra-l4t-r24.2
```

### Toolchain
In addition to the code you'll need the toolchain for cross-compiling the kernel for ARM.  Download the following two binaries.

* [64bit ARM](https://releases.linaro.org/components/toolchain/binaries/5.4-2017.01/aarch64-linux-gnu/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz)
* [32bit ARM](https://releases.linaro.org/components/toolchain/binaries/5.4-2017.01/arm-linux-gnueabihf/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz)

Install using the following commands:
```bash
sudo mkdir /opt/linaro
sudo chmod -R 775 /opt/linaro
sudo chown -R $USER /opt/linaro
mv gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz /opt/linaro/
mv gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz /opt/linaro/
cd /opt/linaro/
tar -xf gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu.tar.xz
tar -xf gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf.tar.xz
```

### Add Dax Drivers
Create symbolic links to the new files and insert them into the kernel.
```bash
# Export environment variables
export GIT=/git/
export SOURCEDIR=$DEVDIR/64_TX1/Linux_for_Tegra_64_tx1/sources/kernel_source/

# Link new device tree entries
cd $SOURCEDIR/arch/arm64/boot/dts/tegra210-platforms/
ln -s $GIT/daxc02/tegra210-daxc02.dtsi

# Update device tree to include new entires
cd $SOURCEDIR/arch/arm64/boot/dts/
sed -e '/tegra210-jetson-cv-camera-modules.dtsi/ s;^;//;' -i tegra210-jetson-cv-base-p2597-2180-a00.dts
echo "#include \"tegra210-platforms/tegra210-daxc02.dtsi\"" >> tegra210-jetson-cv-base-p2597-2180-a00.dts
cd $SOURCEDIR/arch/arm64/boot/dts/tegra210-plugin-manager/
sed -e '/tegra210-jetson-cv-camera-plugin-manager.dtsi/ s;^;//;' -i tegra210-jetson-cv-plugin-manager.dtsi

# Streamline the dtb compilation process
cd $SOURCEDIR/arch/arm64/boot/dts/
sed -e '/CONFIG_ARCH_TEGRA_21x_SOC/ s;^;#;' -i Makefile
sed -e '/^#.* tegra210-jetson-tx1-p2597-2180-a01-devkit.dtb/ s;^#*;;' -i Makefile

# Link DAX-C02 driver
cd $SOURCEDIR/drivers/media/i2c/
ln -s $GIT/daxc02/daxc02.c
```

Insert the following at the top of ```$SOURCEDIR/drivers/media/i2c/Kconfig```
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
Set the configuration
```bash
cd $SOURCEDIR
make ARCH=arm64 tegra21_defconfig
sudo apt install libncurses-dev
make menuconfig
```

Enable DAXC02
```bash
Device Drivers --->
    <*> Multimedia support --->
        <*> DAXC02 camera sensor support
```

Apply this patch to avoid [error: r7 cannot be used in asm here](https://tls.mbed.org/kb/development/arm-thumb-error-r7-cannot-be-used-in-asm-here)
```bash
cd $SOURCEDIR/arch/arm64/kernel/vdso32/
sed -i 's/.*ccflags-y := -shared -fPIC -fno-common -fno-builtin -march=armv7-a.*/ccflags-y := -shared -fPIC -fomit-frame-pointer -fno-common -fno-builtin -march=armv7-a/' Makefile
```

Apply this patch to avoid [error: logical not is only applied to the left hand side of comparison](https://devtalk.nvidia.com/default/topic/894945/jetson-tx1/jetson-tx1/11)
```bash
cd $SOURCEDIR/drivers/platform/tegra/
sed -i 's/.*c->state = (!is_lp_cluster() == (c->u.cpu.mode == MODE_G)) ? ON : OFF;.*/c->state = ((!is_lp_cluster()) == (c->u.cpu.mode == MODE_G)) ? ON : OFF;/' tegra21_clocks.c
```

Compile the kernel image, device tree blob, and modules
```bash
cd $SOURCEDIR
export ARCH=arm64
export CROSS_COMPILE=/opt/linaro/gcc-linaro-5.4.1-2017.01-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
export CROSS32CC=/opt/linaro/gcc-linaro-5.4.1-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc

make zImage
make dtbs
```

### Transfer to the TX
Now that the Image, zImage, and dtb has been compiled installation is as easy as copying these three files to the /boot/ directory of the TX.
```
scp $SOURCEDIR/arch/arm64/boot/Image [name]@[ip-address]:/boot/
scp $SOURCEDIR/arch/arm64/boot/zImage [name]@[ip-address]:/boot/
scp $SOURCEDIR/arch/arm64/boot/dts/tegra210-jetson-tx1-p2597-2180-a01-devkit.dtb [name]@[ip-address]:/boot/
```
