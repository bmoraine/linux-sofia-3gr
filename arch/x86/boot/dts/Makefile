ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2-svb-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2.1-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2.1-svb-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd0-p0.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd0-p0-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es1-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es2-svb.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es2-butr.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es2-svb-psvon.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es2-svb-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd7s.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd7s-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd5s.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-mrd5s-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-es1-svb-rk.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-es1-svb-rk-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-es1-svb-garnet.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-es1-sr-garnet.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-es1-butr.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2.1-mrd7s.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3G-es2.1-mrd7s-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-mrd6-p0.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-mrd6-p0-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-mrd6-p1.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_LTE-mrd6-p1-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-mrd6-p2.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-mrd6-p2-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-mrd6-720.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-mrd6-720-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-crb-v1.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-crb-v1-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-s303.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-s303-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-s303-evt.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-s303-evt-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-ecs-th8.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-ecs-th8-recovery.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-ecs-th10.dtb
dtb-$(CONFIG_X86_INTEL_XGOLD) += SF_3GR-ecs-th10-recovery.dtb
targets += dtbs
targets += $(dtb-y)
endif

# *.dtb used to be generated in the directory above. Clean out the
# old build results so people don't accidentally use them.
dtbs: $(addprefix $(obj)/, $(dtb-y))
	$(Q)rm -f $(obj)/../*.dtb

clean-files := *.dtb
