##
## Makefile for Miosix embedded OS
##

## Path to kernel/config directories (edited by init_project_out_of_git_repo.pl)
KPATH := miosix-kernel/miosix
CONFPATH := .
MAKEFILE_VERSION := 1.16
include $(KPATH)/Makefile.kcommon

##
## List here your source files (both .s, .c and .cpp)
##
SRC :=                                             \
main.cpp \
application.cpp renderer.cpp colormap.cpp \
version.cpp                            \
drivers/display_er_oledm015.cpp drivers/misc.cpp   \
drivers/options_save.cpp         \
drivers/mlx90640.cpp drivers/MLX90640_API.cpp      \
drivers/rp2040_spi.cpp \
drivers/arm_pl022_spi.cpp \
# drivers/usb_tinyusb.cpp

IMG :=  \
images/batt0icon.png \
images/batt25icon.png \
images/batt50icon.png \
images/batt75icon.png \
images/batt100icon.png \
images/miosixlogoicon.png \
images/emissivityicon.png \
images/smallcelsiusicon.png \
images/largecelsiusicon.png \
images/pauseicon.png \
images/usbicon.png

SRC2 := $(IMG:.png=.cpp)
# Images should be compiled first to prevent missing includes
SRC := $(SRC2) $(SRC)
%.cpp : %.png
	./mxgui/_tools/code_generators/build/pngconverter --in $< --depth 16

# This prevents make from deleting the intermediate .cpp files
.PRECIOUS: $(SRC2)

# Consider the version number file to be always outdated
.PHONY: version.cpp
# Dynamically get the version from git
VERSION := $(shell if ! git describe --always 2> /dev/null; then echo 0000000; fi)

##
## List here additional include directories (in the form -Iinclude_dir)
##
INCLUDE_DIRS := -I. -I./mxgui

##
## List here additional static libraries with relative path
##
LIBS := mxgui/libmxgui.a

##
## List here subdirectories which contains makefiles
##
SUBDIRS += mxgui

##
## Attach a romfs filesystem image after the kernel
##
ROMFS_DIR :=

all: $(if $(ROMFS_DIR), image, main)

main: $(OBJ) all-recursive
	$(ECHO) "[LD  ] main.elf"
	$(Q)$(CXX) $(LFLAGS) -o main.elf $(OBJ) $(LINK_LIBS)
	$(ECHO) "[CP  ] main.hex"
	$(Q)$(CP) -O ihex   main.elf main.hex
	$(ECHO) "[CP  ] main.bin"
	$(Q)$(CP) -O binary main.elf main.bin
	$(Q)$(SZ) main.elf

clean: clean-recursive
	$(Q)rm -f $(OBJ) $(OBJ:.o=.d) main.elf main.hex main.bin main.map

-include $(OBJ:.o=.d)
