Use ADB (Android Debug Bridge) to do command-line reflash
- Power on device.
- Connect device to host via USB.
- Open command prompt on host and go to directory where adb,
  synaptics_fw_updater, and FW image (e.g. PR1234567.img) reside.
- Run "adb devices" to ensure connection with device.
- Run "adb root" to have root privileges.
- Run "adb push synaptics_fw_updater /data" to copy synaptics_fw_updater to
  /data directory on device.
- Run "adb push PR1234567.img /data" to copy PR1234567.img to /data directory
  on device.
- Run "adb shell chmod 777 /data/synaptics_fw_updater" to make
  synaptics_fw_updater executable.
- Run "adb shell /data/synaptics_fw_updater -b /data/PR1234567.img" to
  start reflash process.

Parameters
[-b {image_file}] - Name of image file
[-ld] - Do lockdown
[-r] - Read config area
[-ui] - UI config area
[-pm] - Permanent config area
[-bl] - BL config area
[-dp] - Display config area
[-f] - Force reflash
[-v] - Verbose output

Procedures for checking whether to proceed with reflash
- If [-f] flag is set, proceed with reflash.
- If device is in flash prog (bootloader) mode, proceed with reflash.
- If PR number of new FW image is greater than PR number of FW on device,
  proceed with reflash.
- If PR number of new FW image is equal to PR number of FW on device, check
  config ID information. If config ID of new FW image is greater than config
  ID of FW on device, proceed with updating config data only.
- Otherwise, no reflash is performed.

Usage examples
- Perform reflash using PR1234567.img
   synaptics_fw_updater -b PR1234567.img
- Perform reflash using PR1234567.img and do lockdown
   synaptics_fw_updater -b PR1234567.img -ld
- Perform reflash using PR1234567.img regardless of PR number of FW on device
   synaptics_fw_updater -b PR1234567.img -f
- Write config data from PR1234567.img (parsing config data from firmware
  image file)
   synaptics_fw_updater -b PR1234567.img -ui
- Write permanent config area from pmconfig.img (binary file containing
  permanent config data)
   synaptics_fw_updater -b pmconfig.img -pm
- Read UI config area
   synaptics_fw_updater -r -ui
- Read permanent config area
   synaptics_fw_updater -r -pm

Pleae make sure CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE is enabled in
the defconfig file when compiling the driver.