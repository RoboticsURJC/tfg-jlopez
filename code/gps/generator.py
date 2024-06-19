import serial
import time

uart = serial.Serial("/dev/ttyS0", 9600)
print("Cargando")
uart.flush()
while True:
    try: 
        data = uart.readline().decode('utf-8')
        # $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,-164.0,M,,,,*47 
        # GGA         -> Global Positioning System Fix Data
        # 123519.00   -> Hora 12:35:19 UTC
        # 4807.038,N  -> Latitud 48 grados 07.038' N
        # 01131.000,E -> Longitud 11 grados 31.000' E
        # 08          -> Cantidad de satélites recibidos 
        # 545.4,M     -> Altura en metros sobre el nivel del mar
        # *47         -> Checksum o valor de control, siempre comienza con *
        if(data[0:6] == "$GPGGA"):
            #print(data)
            latitud = data[17:27]
            signo_la = data[28]
            longitud = data[30:41]
            signo_lo = data[42]

            print("latitud: {} {}".format(latitud, signo_la))
            print("longitud: {} {}".format(longitud, signo_lo))
            uart.flush()
    except: 
        print(".")
        uart = serial.Serial("/dev/ttyS0", 9600)
            