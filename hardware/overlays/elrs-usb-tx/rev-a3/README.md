# ELRS USB TX · AERIAL KIT REV A3

This is the hardware and Ubuntu handoff for the fabricated ELRS USB TX project in Autodesk Fusion. The recovered design is marked **AERIAL KIT / REV A3 / 2.4GHz TRANSCEIVER**. Compare that silkscreen and the actual population to this design before powering or programming a unit. The user reports fabrication is complete. Assembly, board measurements, flash contents and RF operation have yet to be verified.

The corresponding Aerial Kit firmware, diagnostic and build instructions are
under [`firmware/elrs-usb-tx-rev-a3/`](../../../../firmware/elrs-usb-tx-rev-a3/).
The hardware sources are author-supplied; no hardware licence has been
asserted yet.

## Start here

- [Ubuntu bring-up and flashing](UBUNTU-BRINGUP.md) has the USB ROM boot procedure, read-only identification, flash backup and UART recovery.
- [Hardware reference](HARDWARE.md) has the GPIO assignments, headers, test pads, power and RF design.
- [Fusion 3D preview](previews/3d-board-big.png) is a CAD view of the assembly.
- [Component list and coordinates](data/components.csv), [ESP32 pin map](data/esp32-pin-map.csv), [schematic nets](data/schematic-nets.csv), [PCB nets](data/pcb-nets.csv), [EDA rules](data/design-rules.csv), [silkscreen](data/layout-silkscreen.csv) and [schematic sheet notes](data/schematic-notes.md) are the machine-readable or text references.

## CAD and provenance

- `eagle/ELRS_USB_TX.sch` and `eagle/ELRS_USB_TX.brd` are the paired EAGLE XML documents embedded in Fusion's cached schematic and board containers. Both are byte-for-byte identical to the corresponding files in Fusion's temporary Electronics output. The project library is embedded. Use the XML directly for analysis on Ubuntu or import the pair into a compatible PCB editor. Verify any imported geometry against Fusion, particularly Fusion-specific pours and slots.
- `fusion-cache/` holds the original cached electronics project (`.fprj`), schematic (`.fsch`), PCB (`.fbrd`) and 3D PCB (`.f3d`). These individually passed archive integrity checks. Cloud reference resolution and re-import of the cache copies have yet to be tested.
- The schematic contains five sheets. All 87 PCB elements have matching schematic references and values. All 60 nets agree at the pad level after resolving schematic pins through the embedded device mappings. [Comparison result](data/connectivity-comparison.json).
- Availability of the exact order-submitted Gerbers, drill files, fabrication stackup, supplier BOM, placement files, serial/lab logs, board photos and verified firmware image remains open. The component CSV gives CAD values, footprints and positions. Confirm supplier part numbers and assembly exceptions from the order.
- `SHA256SUMS` records checksums for the handoff files. From its directory on Ubuntu, run `sha256sum -c SHA256SUMS`.

On Ubuntu, copy or extract the folder to a working directory. In WSL, the Windows copy is normally at `/mnt/c/PCB_Fusion/ELRS_USB_TX`. On native Ubuntu, use the mounted Windows volume's `PCB_Fusion/ELRS_USB_TX` or transfer the zip. Give the next agent this README and the bring-up guide.
