This folder contains a scalable SVG pinout for the PickupWinder project.

Files:
- `p8-p9-pinout.svg` : vector diagram of P8 / P9 headers with used pins highlighted.

Convert to PNG or PDF locally (examples):

# Using rsvg-convert (librsvg)
# Install: Debian/Ubuntu: sudo apt-get install librsvg2-bin
rsvg-convert -w 1600 docs/pinouts/p8-p9-pinout.svg -o docs/pinouts/p8-p9-pinout.png

# Using Inkscape (recommended for higher fidelity)
# Install: sudo apt-get install inkscape
inkscape docs/pinouts/p8-p9-pinout.svg --export-type=png --export-filename=docs/pinouts/p8-p9-pinout.png --export-width=1600

# To export PDF with Inkscape
inkscape docs/pinouts/p8-p9-pinout.svg --export-type=pdf --export-filename=docs/pinouts/p8-p9-pinout.pdf

# Quick view (Linux)
xdg-open docs/pinouts/p8-p9-pinout.svg

Notes:
- The diagram is simplified and shows only the project-used pins with approximate physical locations on the headers; verify exact mechanical placement against the BeagleBone Black reference when designing the PCB.
- If you want a different layout (horizontal headers, pin numbering, or exact pad coordinates), tell me which format (CSV/JSON/Gerber primer) and I will produce it.