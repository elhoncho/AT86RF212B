#ifndef MYINC_MAINCONTROLLER_H_
#define MYINC_MAINCONTROLLER_H_

#define MODE_RAW_TX 0
#define MODE_RAW_RX 1
#define MODE_TERMINAL 2

void MainControllerOpen();
void MainControllerLoop();
void MainControllerSetMode(uint8_t newMode);
uint8_t MainControllerGetMode();

#endif /* MYINC_MAINCONTROLLER_H_ */
