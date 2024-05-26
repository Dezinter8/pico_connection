# Setup

Instalacja projektu

```
cd ~/piperws/src

git clone https://github.com/Dezinter8/pico_connection.git

cd ../

colcon build --symlink-install
```

Nadanie uprawnień do portu szeregowego

```
sudo chmod 777 /dev/ttyACM0

sudo chmod 777 /dev/serial/by-id/usb-Raspberry_Pi_Pico_E660D4A0A772982F-if00
```

# Run

Uruchomienie projektu

```
cd ~/piperws

source install/setup.bash

ros2 run pico_connection serial_publisher
```

# udev

Stwórz nowy plik reguł w /etc/udev/rules.d/ żeby nadawać uprawnienia do portu automatycznie:

```
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Dodaj następującą zawartość, zastępując E660D4A0A772982F rzeczywistym ID serialu Twojego urządzenia:

```
SUBSYSTEM=="tty", ATTRS{serial}=="E660D4A0A772982F", MODE="0666"
```

Po utworzeniu pliku reguł, musisz załadować nową konfigurację udev i zastosować zmiany:

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```
