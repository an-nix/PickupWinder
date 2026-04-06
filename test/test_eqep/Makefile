
# Makefile pour construire/installer les overlays eQEP (kernel 6.12+)
# Génère les 4 DTBOs: bone_eqep0-00A0.dtbo, bone_eqep1-00A0.dtbo,
# bone_eqep2-00A0.dtbo, bone_eqep2b-00A0.dtbo

DTSDIR := .
OUTDIR := $(DTSDIR)

DTBS = \
	$(OUTDIR)/BONE_EQEP0-00A0.dtbo \
	$(OUTDIR)/BONE_EQEP1-00A0.dtbo \
	$(OUTDIR)/BONE_EQEP2-00A0.dtbo \
	$(OUTDIR)/BONE_EQEP2B-00A0.dtbo

DTSS = \
    $(DTSDIR)/bone_eqep0.dts \
    $(DTSDIR)/bone_eqep1.dts \
    $(DTSDIR)/bone_eqep2.dts \
    $(DTSDIR)/bone_eqep2b.dts

.PHONY: all dtbo install-overlay clean run dtbo0 dtbo1 dtbo2 dtbo2b

all: dtbo

dtbo: $(DTBS)
	@echo "DTBOs générés:"
	@printf "  %s\n" $(DTBS)

$(OUTDIR)/BONE_EQEP0-00A0.dtbo: $(DTSDIR)/bone_eqep0.dts
	@echo "Compiling $< -> $@"
	dtc -I dts -O dtb -o $@ -b 0 -@ $<

$(OUTDIR)/BONE_EQEP1-00A0.dtbo: $(DTSDIR)/bone_eqep1.dts
	@echo "Compiling $< -> $@"
	dtc -I dts -O dtb -o $@ -b 0 -@ $<

$(OUTDIR)/BONE_EQEP2-00A0.dtbo: $(DTSDIR)/bone_eqep2.dts
	@echo "Compiling $< -> $@"
	dtc -I dts -O dtb -o $@ -b 0 -@ $<

$(OUTDIR)/BONE_EQEP2B-00A0.dtbo: $(DTSDIR)/bone_eqep2b.dts
	@echo "Compiling $< -> $@"
	dtc -I dts -O dtb -o $@ -b 0 -@ $<

install-overlay: dtbo
	@echo "Copie des DTBOs dans /lib/firmware (privilèges root requis)"
	sudo cp $(DTBS) /lib/firmware/

	@echo "Pour activer à chaud (legacy bone_capemgr), exécutez sur la BBB:"
	@echo "  echo BONE_EQEP0-00A0.dtbo | sudo tee /sys/devices/platform/bone_capemgr/slots"
	@echo "ou ajoutez les entrées suivantes dans /boot/uEnv.txt (uboot_overlay_addrN):"
	@echo "  uboot_overlay_addr4=/lib/firmware/BONE_EQEP0-00A0.dtbo"

run:
	python3 read_eqep.py

clean:
	rm -f $(DTBS)
