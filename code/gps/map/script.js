// Import necessary functions from Firebase modules
import { initializeApp } from 'firebase/app';
import { getDatabase, ref, onValue } from 'firebase/database';
import L from 'leaflet';

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
//firebase.initializeApp(firebaseConfig);

const app = initializeApp(firebaseConfig);

// Get a reference to the Firebase Realtime Database service
const database = getDatabase(app);

const tilesProvider = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
let myMap = L.map('myMap').setView([51.505, -0.09], 13);

L.tileLayer(tilesProvider, {
  maxZoom: 18,
}).addTo(myMap);

let marker;

// Listen for changes in Firebase Realtime Database
onValue(ref(database, 'location'), (snapshot) => {
  const data = snapshot.val();
  const lat = data.LAT;
  const lng = data.LNG;

  if (marker) {
    myMap.removeLayer(marker);
  }

  marker = L.marker([lat, lng]).addTo(myMap);
  myMap.setView([lat, lng], 13);

  console.log(`Marker updated: Latitude=${lat}, Longitude=${lng}`);
});
