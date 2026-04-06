DTSDIR := src/dts
OUTDIR := build/dtbo
SUFFIX := -00A0.dtbo

.PHONY: all clean install

all: $(OUTDIR)
	@echo "Compiling DTBOs from $(DTSDIR) -> $(OUTDIR)"
	@for f in $(DTSDIR)/*.dts; do \
		[ -f "$$f" ] || continue; \
		base=$$(basename "$$f" .dts); \
		up=$$(echo "$$base" | tr '[:lower:]' '[:upper:]'); \
		out=$(OUTDIR)/$$up$(SUFFIX); \
		echo "  $$f -> $$out"; \
		dtc -O dtb -b 0 -@ "$$f" -o "$$out" || { echo "dtc failed for $$f"; exit 1; }; \
	done

$(OUTDIR):
	@mkdir -p $(OUTDIR)

clean:
	@rm -rf $(OUTDIR)/*
	@echo "cleaned $(OUTDIR)"

install: all
	@echo "Install not implemented; copy $(OUTDIR)/*.dtbo to /lib/firmware on BBB"
