# flexDrive-mcu

Application for control of flexDrive model. This application is intended to be used with the [flexDrive hardware]([https://lampa-fs.gitbook.io/knowledge-base/laboratorijska-oprema/makete/flexdrive)

Operation modes:
## Open loop control
Open loop control is used to control the motor without feedback. The motor is controlled by setting the desired speed and direction. The motor will run at the desired speed until the speed is changed or the motor is stopped. Both encoders are read and transmitted to the host.

## Closed loop control