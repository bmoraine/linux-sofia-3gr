ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es1-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es1-svb-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2-svb-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G_tablet.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es1-svb.dtb

targets += dtbs
targets += $(dtb-y)
endif

# *.dtb used to be generated in the directory above. Clean out the
# old build results so people don't accidentally use them.
dtbs: $(addprefix $(obj)/, $(dtb-y))
	$(Q)rm -f $(obj)/../*.dtb

clean-files := *.dtb
