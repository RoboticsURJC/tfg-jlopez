import pyrebase
import serial
import pynmea2

firebaseConfig={
    "apiKey": "AIzaSyBgrLRMuPB8bkaMUNg4XlQKTbY34JgWwic",
    "authDomain": "gps-tracker-c5d0e.firebaseapp.com",
    "databaseURL": "https://gps-tracker-c5d0e-default-rtdb.europe-west1.firebasedatabase.app",
    "projectId": "gps-tracker-c5d0e",
    "storageBucket": "gps-tracker-c5d0e.appspot.com",
    "messagingSenderId": "443475027920",
    "appId": "1:443475027920:web:dc2a4bf0259b861f3d7bd5",
    "measurementId": "G-6JPHY9G9S1"
    }

firebase=pyrebase.initialize_app(firebaseConfig)
db=firebase.database()

while True:
    port="/dev/ttyS0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    n_data = newdata.decode('latin-1')
    if n_data[0:6] == '$GPRMC':
        try:
            newmsg=pynmea2.parse(n_data)
            lat=newmsg.latitude
            lng=newmsg.longitude
            gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
            print(gps)
            data = {"LAT": lat, "LNG": lng}
            db.update(data)
            print("Data sent")
        except pynmea2.ParseError:
            pass
