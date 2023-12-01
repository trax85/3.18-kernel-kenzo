#
# Makefile for kernel SPMI framework.
#
<<<<<<< HEAD
obj-$(CONFIG_SPMI)	+= spmi.o

obj-$(CONFIG_SPMI_MSM_PMIC_ARB)	+= spmi-pmic-arb.o
=======
obj-$(CONFIG_SPMI)			+= spmi.o spmi-resources.o
obj-$(CONFIG_SPMI_MSM_PMIC_ARB)		+= spmi-pmic-arb.o
obj-$(CONFIG_MSM_QPNP_INT)		+= qpnp-int.o

ifdef CONFIG_DEBUG_FS
obj-$(CONFIG_SPMI)			+= spmi-dbgfs.o
endif
>>>>>>> p9x
