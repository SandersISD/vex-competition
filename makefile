# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files (native VEXcode layout)
SRC_C  = $(wildcard src/*.cpp)
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard motion-lib/src/*.cpp)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
# NOTE: must glob *.hpp too - the stock makefile only watched *.h, so
# header-only edits (MotionLib/*.hpp, include/*.h) never triggered rebuilds and
# stale binaries got flashed.
SRC_H  = $(wildcard include/*.h)
SRC_H += $(wildcard include/*.hpp)
SRC_H += $(wildcard motion-lib/include/MotionLib/*.h)
SRC_H += $(wildcard motion-lib/include/MotionLib/*.hpp)

# additional dependancies
SRC_A  = makefile

# project header file locations (library first, then the app's own)
INC_F  = motion-lib/include include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
