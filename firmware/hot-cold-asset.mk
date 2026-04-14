# if "template" is in the make command, do not include static.lib files
STATIC_FILES=$(wildcard static/*)
STATIC_SOURCE_FILES=$(filter %.c %.cc %.cpp %.cxx %.s %.S,$(STATIC_FILES))
STATIC_ASSET_FILES=$(filter-out $(STATIC_SOURCE_FILES),$(STATIC_FILES))

ifneq (,$(findstring template,$(MAKECMDGOALS)))
ASSET_FILES=$(STATIC_ASSET_FILES)
else
ASSET_FILES=$(STATIC_ASSET_FILES) $(wildcard static.lib/*)
endif

TEMPLATE_FILES+=$(wildcard static/*) $(wildcard firmware/hot-cold-asset.mk)

ASSET_OBJ=$(addprefix $(BINDIR)/, $(addsuffix .o, $(ASSET_FILES)) )

GETALLOBJ=$(sort $(call ASMOBJ,$1) $(call COBJ,$1) $(call CXXOBJ,$1)) $(ASSET_OBJ)

# Keep large LVGL image descriptor/source in cold package so hot uploads stay fast.
ifeq ($(USE_PACKAGE),1)
COLD_ONLY_SRC_FILES=$(SRCDIR)/bm2_small_asset.c
COLD_ONLY_OBJ=$(addprefix $(BINDIR)/,$(patsubst $(SRCDIR)/%,%.o,$(COLD_ONLY_SRC_FILES)))
EXCLUDE_SRCDIRS+=$(COLD_ONLY_SRC_FILES)
COLD_LIBRARIES+=$(COLD_ONLY_OBJ)
endif

.SECONDEXPANSION:
$(ASSET_OBJ): $$(patsubst bin/%,%,$$(basename $$@))
	$(VV)mkdir -p $(BINDIR)/static
	$(VV)mkdir -p $(BINDIR)/static.lib
	@echo "ASSET $@"
	$(VV)$(OBJCOPY) -I binary -O elf32-littlearm -B arm $^ $@
