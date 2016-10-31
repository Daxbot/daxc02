# obj-m is a list of what kernel modules to build.  The .o and other
# objects will be automatically built from the corresponding .c file -
# no need to list the source files explicitly.

TX1_SSH = ubuntu@10.100.0.74

obj-m += daxc02.o
obj-m += tps22994.o

# Define cross compiler
export DEVDIR :=/home/ww/l4t
export CROSS_COMPILE := /opt/linaro/gcc-linaro-5.3-2016.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
export CROSS32CC := /opt/linaro/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
export KERNEL_MODULES_OUT := $(DEVDIR)/images/modules
export ARCH := arm64

# KDIR is the location of the kernel source.  The current standard is
# to link to the associated source tree from the directory containing
# the compiled modules.
KDIR  := $(DEVDIR)/64_TX1/Linux_for_Tegra_64_tx1/sources/kernel_source

# PWD is the current working directory and the location of our module
# source files.
PWD   := $(shell pwd)

# default is the default make target.  The rule here says to run make
# with a working directory of the directory containing the kernel
# source and compile only the modules in the PWD (local) directory.
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

copy: default
	scp *.ko $(TX1_SSH):/lib/modules/3.10.96-tegra+/kernel/drivers/media/i2c/

clean:
	$(RM) *.o *.mod.c modules.order Module.symvers
