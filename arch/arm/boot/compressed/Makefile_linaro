#
# linux/arch/arm/boot/compressed/Makefile
#
# create a compressed vmlinuz image from the original vmlinux
#

# Check GCC version
GCCVERSION  := $(shell $(CC) --version | grep ^gcc | sed 's/^.* //g' | cut -c1-3)

OBJS		=
plus_sec := $(call as-instr,.arch_extension sec,+sec)

# Ensure that mmcif loader code appears early in the image
# to minimise that number of bocks that have to be read in
# order to load it.
ifeq ($(CONFIG_ZBOOT_ROM_MMCIF),y)
ifeq ($(CONFIG_ARCH_SH7372),y)
OBJS		+= mmcif-sh7372.o
endif
endif

AFLAGS_head.o := -DTEXT_OFFSET=$(TEXT_OFFSET)
AFLAGS_head.o += -Wa,-march=armv7-a$(plus_sec)
AFLAGS_head.o += -Wa,-mcpu=cortex-a9
HEAD = head.o

AFLAGS_misc.o +=-Wa,-march=armv7-a$(plus_sec)
AFLAGS_misc.o += -Wa,-mcpu=cortex-a9
MISC = misc.o

AFLAGS_decompress.o += -Wa,-march=armv7-a$(plus_sec)
AFLAGS_decompress.o += -Wa,-mcpu=cortex-a9
DECOMPRESS = decompress.o
# change@wtl.rsengott
# FIPS_KERNEL_RAM_BASE is start of kernel text in RAM
ifeq ($(CONFIG_CRYPTO_FIPS),y)
AFLAGS_head.o += -DFIPS_KERNEL_RAM_BASE=0x40008000
endif
HEAD	= head.o
OBJS	+= misc.o decompress.o
FONTC	= $(srctree)/drivers/video/console/font_acorn_8x8.c

# string library code (-Os is enforced to keep it much smaller)
OBJS    += string.o
CFLAGS_string.o  := -Os

#
# Architecture dependencies
#
ifeq ($(CONFIG_ARCH_ACORN),y)
OBJS		+= ll_char_wr.o font.o
endif

ifeq ($(CONFIG_ARCH_SHARK),y)
OBJS		+= head-shark.o ofw-shark.o
endif

ifeq ($(CONFIG_ARCH_P720T),y)
# Borrow this code from SA1100
OBJS		+= head-sa1100.o
endif

ifeq ($(CONFIG_ARCH_SA1100),y)
OBJS		+= head-sa1100.o
endif

ifeq ($(CONFIG_ARCH_VT8500),y)
OBJS		+= head-vt8500.o
endif

ifeq ($(CONFIG_CPU_XSCALE),y)
OBJS		+= head-xscale.o
endif

ifeq ($(CONFIG_PXA_SHARPSL_DETECT_MACH_ID),y)
OBJS		+= head-sharpsl.o
endif

ifeq ($(CONFIG_CPU_ENDIAN_BE32),y)
ifeq ($(CONFIG_CPU_CP15),y)
OBJS		+= big-endian.o
else
# The endian should be set by h/w design.
endif
endif

ifeq ($(CONFIG_ARCH_SHMOBILE),y)
OBJS		+= head-shmobile.o
endif

#
# We now have a PIC decompressor implementation.  Decompressors running
# from RAM should not define ZTEXTADDR.  Decompressors running directly
# from ROM or Flash must define ZTEXTADDR (preferably via the config)
# FIXME: Previous assignment to ztextaddr-y is lost here. See SHARK
ifeq ($(CONFIG_ZBOOT_ROM),y)
ZTEXTADDR	:= $(CONFIG_ZBOOT_ROM_TEXT)
ZBSSADDR	:= $(CONFIG_ZBOOT_ROM_BSS)
else
ZTEXTADDR	:= 0
ZBSSADDR	:= ALIGN(8)
endif

SEDFLAGS	= s/TEXT_START/$(ZTEXTADDR)/;s/BSS_START/$(ZBSSADDR)/

suffix_$(CONFIG_KERNEL_GZIP) := gzip
suffix_$(CONFIG_KERNEL_LZO)  := lzo
suffix_$(CONFIG_KERNEL_LZMA) := lzma
suffix_$(CONFIG_KERNEL_XZ)   := xzkern
suffix_$(CONFIG_KERNEL_LZ4)  := lz4

targets := vmlinux vmlinux.lds \
		 piggy.$(suffix_y) piggy.$(suffix_y).o \
		 lib1funcs.o lib1funcs.S font.o font.c head.o misc.o decompress.o $(OBJS)

# Make sure files are removed during clean
extra-y       += piggy.gzip piggy.lzo piggy.lzma piggy.lz4 piggy.xzkern lib1funcs.S ashldi3.S

ifeq ($(CONFIG_FUNCTION_TRACER),y)
ORIG_CFLAGS := $(KBUILD_CFLAGS)
KBUILD_CFLAGS = $(subst -pg, , $(ORIG_CFLAGS))
endif

ccflags-y := -fpic -fno-builtin -I$(obj)
asflags-y := -Wa,-march=armv7-a

# Supply kernel BSS size to the decompressor via a linker symbol.
KBSS_SZ = $(shell size $(obj)/../../../../vmlinux | awk 'END{print $$3}')
LDFLAGS_vmlinux = --defsym _kernel_bss_size=$(KBSS_SZ)
# Supply ZRELADDR to the decompressor via a linker symbol.
ifneq ($(CONFIG_AUTO_ZRELADDR),y)
LDFLAGS_vmlinux += --defsym zreladdr=$(ZRELADDR)
endif
ifeq ($(CONFIG_CPU_ENDIAN_BE8),y)
LDFLAGS_vmlinux += --be8
endif
# ?
LDFLAGS_vmlinux += -p
# Report unresolved symbol references
LDFLAGS_vmlinux += --no-undefined
# Delete all temporary local symbols
LDFLAGS_vmlinux += -X
# Next argument is a linker script
LDFLAGS_vmlinux += -T

# For __aeabi_uidivmod
AFLAGS_lib1funcs.o +=-Wa,-march=armv7-a$(plus_sec)
AFLAGS_lib1funcs.o += -Wa,-mcpu=cortex-a9
lib1funcs = $(obj)/lib1funcs.o

$(obj)/lib1funcs.S: $(srctree)/arch/$(SRCARCH)/lib/lib1funcs.S
	$(call cmd,shipped)

# For __aeabi_llsl
AFLAGS_ashldi3.o +=-Wa,-march=armv7-a$(plus_sec)
ashldi3 = $(obj)/ashldi3.o

$(obj)/ashldi3.S: $(srctree)/arch/$(SRCARCH)/lib/ashldi3.S FORCE
	$(call cmd,shipped)

# We need to prevent any GOTOFF relocs being used with references
# to symbols in the .bss section since we cannot relocate them
# independently from the rest at run time.  This can be achieved by
# ensuring that no private .bss symbols exist, as global symbols
# always have a GOT entry which is what we need.
# The .data section is already discarded by the linker script so no need
# to bother about it here.
check_for_bad_syms = \
bad_syms=$$($(CROSS_COMPILE)nm $@ | sed -n 's/^.\{8\} [bc] \(.*\)/\1/p') && \
[ -z "$$bad_syms" ] || \
  ( echo "following symbols must have non local/private scope:" >&2; \
    echo "$$bad_syms" >&2; rm -f $@; false )

$(obj)/vmlinux: $(obj)/vmlinux.lds $(obj)/$(HEAD) $(obj)/$(MISC) $(obj)/$(DECOMPRESS) $(obj)/piggy.$(suffix_y).o \
        $(addprefix $(obj)/, $(OBJS)) $(lib1funcs) $(ashldi3) FORCE
	$(call if_changed,ld)
	@$(check_for_bad_syms)

$(obj)/piggy.$(suffix_y): $(obj)/../Image FORCE
	$(call if_changed,$(suffix_y))

AFLAGS_piggy.$(suffix_y).o += -Wa,-march=armv7-a$(plus_sec)
AFLAGS_piggy.$(suffix_y).o += -Wa,-mcpu=cortex-a9
$(obj)/piggy.$(suffix_y).o:  $(obj)/piggy.$(suffix_y) FORCE

CFLAGS_font.o := -Dstatic=

$(obj)/font.c: $(FONTC)
	$(call cmd,shipped)

$(obj)/vmlinux.lds: $(obj)/vmlinux.lds.in arch/arm/boot/Makefile $(KCONFIG_CONFIG)
	@sed "$(SEDFLAGS)" < $< > $@

