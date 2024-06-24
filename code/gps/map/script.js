var config = {
    apiKey: "AIzaSyBgrLRMuPB8bkaMUNg4XlQKTbY34JgWwic",
    authDomain: "gps-tracker-c5d0e.firebaseapp.com",
    databaseURL: "https://gps-tracker-c5d0e-default-rtdb.europe-west1.firebasedatabase.app",
    projectId: "gps-tracker-c5d0e",
    storageBucket: "gps-tracker-c5d0e.appspot.com",
    messagingSenderId: "443475027920",
    appId: "1:443475027920:web:dc2a4bf0259b861f3d7bd5",
    measurementId: "G-6JPHY9G9S1"
};
var db = firebase.initializeApp(config).database();
  
var { LMap, LTileLayer, LMarker } = Vue2Leaflet;
var userRefs = db.ref('users')
new Vue({
    el: '#app',
    components: { LMap, LTileLayer, LMarker },
    data() {
      return {
        
        myUuid : localStorage.getItem('myUuid'),
        zoom:13,
        center: L.latLng(47.413220, -1.219482),
        url:'http://{s}.tile.osm.org/{z}/{x}/{y}.png',
        attribution:'&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
        marker: L.latLng(47.413220, -1.219482),
        watchPositionId : null
      }
    },
    mounted(){
      var vm = this
      if (!vm.myUuid) {
        vm.myUuid = vm.guid();
        localStorage.setItem('myUuid', vm.myUuid);
      }else{
        
        vm.watchPositionId = navigator.geolocation.watchPosition(vm.successCoords, vm.errorCoords);
        
      }
      
      
      
    },
    firebase: {
      users: userRefs
    },
    methods:{
      successCoords(position) {
        var vm = this
        if (!position.coords) return
        
        userRefs.child(vm.myUuid).set({
          coords: {
            latitude: position.coords.latitude,
            longitude: position.coords.longitude,
          },
          timestamp: Math.floor(Date.now() / 1000)
        })
        vm.center = L.latLng([position.coords.latitude, position.coords.longitude])
        vm.marker = L.latLng([position.coords.latitude, position.coords.longitude])
      },
      formatLocation(lat, lng){
        return L.latLng(lat,lng)
      },
      errorCoords() {
        console.log('Unable to get current position')
      },
      guid() {
        function s4() {
          return Math.floor((1 + Math.random()) * 0x10000).toString(16).substring(1);
        }
        return s4() + s4() + '-' + s4() + '-' + s4() + '-' + s4() + '-' + s4() + s4() + s4();
      }
    }
  });