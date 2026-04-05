# Makefile minimal pour construire/installer l'overlay d'exemple
DTBO=BB-EQEP-EXAMPLE-00A0.dtbo
DTS=eqep_example.dts

.PHONY: all dtbo install-overlay clean run

all: dtbo

dtbo: $(DTS)
	dtc -O dtb -o $(DTBO) -b 0 -@ $(DTS)
	@echo "DTBO généré: $(DTBO)"

install-overlay: dtbo
	@echo "Copie dans /lib/firmware (privilèges root requis)"
	sudo cp $(DTBO) /lib/firmware/
	@echo "Tentative d'activation (méthode ancienne: bone_capemgr)."
	@echo "Si votre image utilise configfs, adaptez la procédure."
	sudo sh -c 'echo BB-EQEP-EXAMPLE > /sys/devices/platform/bone_capemgr/slots || true'

run:
	python3 read_eqep.py

clean:
	rm -f $(DTBO)
