# Projekt dyplomowy inżynierski [ENG version below]
## Wykorzystany sprzęt:
1. Raspberry Pi
2. STM32L476 + nakładka BLE X-NUCLEO-IDB05A1 (min. 2 pary)
3. Czujniki DHT22

## Uruchamianie projektu:
1) Podłączyć Raspberry Pi i płytkę centralną(wg Generic Access Profile - GAP) i płytkę/płytki peryferyjne(GAP) do zasilania
2) Połączyć się z RPi na PC przez SSH(np. PuTTy)/VNC(np. VNC Viewer) - domyślne IP RPi to 192.168.0.102, sprawdzić na PC czy serwer działa (dla IP 192.168.0.102 na PC strona http://192.168.0.102/test/test.php)
3) Podłączyć płytkę centralną(GAP) do RPi - wspólna masa (np. RPi: pin 6 <-> STM: pin 9 na CN9) i 2 kable na UART (RPi: pin 8 <-> STM: pin PC10, RPi: pin 10 <-> STM: pin PC11)
4) Podłączyć czujniki DHT22 do płytki peryferyjnej(GAP): do dyspozycji są piny PA4, PA9, PA10, ew. PA11 i PA12 (w tej kolejnosci)
5) Przez SSH/VNC uruchomić na RPi skrypty z katalogu `/home/pi/html/test`: 
`cd /home/pi/html/test`
`python file-to-uart.py`
(w nowym oknie terminala - w PuTTy na pasku PPM->Duplicate Session) `python uart-to-file.py`
6) Zresetować płytki centralną i peryferyjną, poczekać ok. 10 sek., płytki powinny same połączyć się przez BLE i następnie można dodawać nowe czujniki i nowe płytki w serwisie przegladarkowym na PC: http://192.168.0.102/test/test.php

# Engineer Thesis Project
## Hardware used:
1. Raspberry Pi
2. STM32L476 + BLE X-NUCLEO-IDB05A1 shield (min. 2 pairs)
3. DHT22 sensors

## Launching the project:
1) Power up Raspberry Pi and STM32L476 board(s)
2) Connect with Raspberry Pi from PC via SSH (with PuTTy) or VNC (with VNC Viewer)
3) Connect the 'central' (in Generic Access Profile - GAP terminology) with the RPi - common ground (e.g. RPi: pin 6 <-> STM: pin 9 on CN9) and 2 cables for UART (RPi: pin 8 <-> STM: pin PC10, RPi: pin 10 <-> STM: pin PC11)
4) Connect DHT22 sensors to the peripheral (GAP) board on pins PA4, PA9, PA10, and PA11 and PA12 (in that order)
5) Run scripts on RPi using:
`cd /home/pi/html/test`
`python file-to-uart.py`
(new terminal window) `python uart-to-file.py`
6) Reset the central and peripheral board, wait about 10 seconds, visit http://192.168.0.102/test/test.php in the PC browser to configure and observe the functioning of the system
