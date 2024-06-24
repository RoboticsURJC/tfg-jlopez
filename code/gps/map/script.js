// Configuración de Firebase
const firebaseConfig = {
  apiKey: "AIzaSyBgrLRMuPB8bkaMUNg4XlQKTbY34JgWwic",
  authDomain: "gps-tracker-c5d0e.firebaseapp.com",
  databaseURL: "https://gps-tracker-c5d0e-default-rtdb.europe-west1.firebasedatabase.app",
  projectId: "gps-tracker-c5d0e",
  storageBucket: "gps-tracker-c5d0e.appspot.com",
  messagingSenderId: "443475027920",
  appId: "1:443475027920:web:dc2a4bf0259b861f3d7bd5",
  measurementId: "G-6JPHY9G9S1"
};

// Inicializar Firebase
firebase.initializeApp(firebaseConfig);
const database = firebase.database();

const tilesProvider = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
let myMap = L.map('myMap').setView([51.505, -0.09], 13);

L.tileLayer(tilesProvider, {
  maxZoom: 18,
}).addTo(myMap);

let marker;

// Escuchar los cambios en la base de datos de Firebase
database.ref('location').on('value', (snapshot) => {
  const data = snapshot.val();
  const lat = data.LAT;
  const lng = data.LNG;

  if (marker) {
    myMap.removeLayer(marker);
  }

  marker = L.marker([lat, lng]).addTo(myMap);
  myMap.setView([lat, lng], 13);
});
