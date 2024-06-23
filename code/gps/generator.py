import time
import string
import pynmea2
import serial

# $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,-164.0,M,,,,*47 
    # GGA         -> Global Positioning System Fix Data
    # 123519.00   -> Hora 12:35:19 UTC
    # 4807.038,N  -> Latitud 48 grados 07.038' N
    # 01131.000,E -> Longitud 11 grados 31.000' E
    # 08          -> Cantidad de satélites recibidos 
    # 545.4,M     -> Altura en metros sobre el nivel del mar
    # *47         -> Checksum o valor de control, siempre comienza con *

while True:
    port = "/dev/ttyS0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata = ser.readline()
    #print(newdata)
    
    if newdata:
        try:
            message = pynmea2.parse(newdata.decode('ascii', errors='replace'))
            # imprime el mensaje del tipo $GPGGA 
            if(isinstance(message, pynmea2.types.talker.GGA)):
                print(message.latitude)
                print(message.longitude)
        except pynmea2.ParseError as e:
            pass
            #print(f"Error parsing NMEA data")