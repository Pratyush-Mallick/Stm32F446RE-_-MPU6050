# Stm32F446RE-_-MPU6050
Project to interface MPU6050 with STM32F446RE using bare metal drivers

The Project is designed with bare-metal drivers for Nucleo-f446re board with underlying MPU6050 library 
from Harinadha Reddy Chintalapalli's repository http://www.pudn.com/Download/item/id/2693210.html

The project is currently being debugged in semihosting mode using Open-OCD . Just comment out 
extern void initialise_monitor_handles(void) to use it with ST-LINK.
