# The ESP32-C3 half of the family's build configuration. See sdkconfig.s3 for
# why the chip is a separate file applied after the platform defaults.

CONFIG_IDF_TARGET="esp32c3"

# The DevKitM-1 carries 4 MB of flash, and the application is smaller here than
# the other chips' - RISC-V, and a board with two motors and no GPS.
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
