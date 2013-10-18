Use ADB (Android Debug Bridge) to do report reading
- Power on device.
- Connect device to host via USB.
- Open command prompt on host and go to directory where adb and
  synaptics_read_report reside.
- Run "adb devices" to ensure connection with device.
- Run "adb root" to have root privileges.
- Run "adb push synaptics_read_report /data" to copy synaptics_read_report to
  /data directory on device.
- Run "adb shell chmod 777 /data/synaptics_read_report" to make
  synaptics_read_report executable.
- Follow instructions below to run synaptics_read_report.

Parameters
[-n] - number of report readings to take
[-c] - display output in 2D format

Usage examples
- Read report type 3 once
   adb shell /data/synaptics_read_report 3
- Read report type 20 once and display in 2D format
   adb shell /data/synaptics_read_report 20 -c
- Read report type 2 15 times and save output in 2D format to report_output.txt
   adb shell "/data/synaptics_read_report 3 -n 15 -c > /data/report_output.txt"
- Retrieve report_output.txt
   adb pull /data/report_output.txt

Note
- Preparation procedures (turning off CBC, turning off signal clarity, force
  update, force cal, etc...) are enabled by default except for report types 2,
  3, 22, 23, 38, and 40.
