<template>
  <v-col v-bind:class="cols">
  <div id="map" height="90%"></div></v-col>
</template>
<script>
import mapboxgl from "mapbox-gl";
import 'mapbox-gl/dist/mapbox-gl.css';
import { useWaypointStore } from './stores/waypoints';


export default {
  setup() {
    const waypoints = useWaypointStore();
    return waypoints;
  },
  props: {
    cols: {
      default: "",
      type: String,
    },
    center_lat: {
      default: 47.366738,
      type: Number,
    },
    center_long: {
      default: 8.542107,
      type: Number,
    },
    zoom: {
      default: 16,
      type: Number,
    },
    SW_bound_lat: {
      default: 47.163841,
      type: Number,
    },
    SW_bound_long: {
      default: 8.435882,
      type: Number,
    },
    NE_bound_lat: {
      default: 47.542943,
      type: Number,
    },
    NE_bound_long: {
      default: 9.005846,
      type: Number,
    },
  },

  head() {
    return {
      script: [
        { src: "https://api.mapbox.com/mapbox-gl-js/v2.11.0/mapbox-gl.js" },
      ],
      link: [
        {
          href: "https://api.mapbox.com/mapbox-gl-js/v2.11.0/mapbox-gl.css",
          rel: "stylesheet",
        },
      ],
    };
  },

  methods: {
    makeMap: function () {
      // Creates and adds a mapbox to the element with id 'map'
      mapboxgl.accessToken =
        "pk.eyJ1IjoiaGxpbjkxIiwiYSI6ImNrbDQ2MjY4NzE0ZXEycHFpaXBya2tvN3gifQ.Tqa8iLUqXeKZQ8SmhLoRtg";
      var map = null;
      // User supplied a valid bound
      var bounds = [
        [this.SW_bound_long, this.SW_bound_lat],
        [this.NE_bound_long, this.NE_bound_lat],
      ];
      map = new mapboxgl.Map({
        container: "map",
        center: [this.center_long, this.center_lat],
        zoom: this.zoom,
        scrollZoom: true,
        style: "mapbox://styles/mapbox/satellite-v9",
        maxBounds: bounds,
      });

      map.doubleClickZoom.disable();

      map.on('click', (e) => {
        this.$emit('clicked', e.lngLat);
      });

      return map;
    },

    updateMarkers() {
      if(this.markers.length>0)
        this.markers.forEach(waypoint => {waypoint.remove(); });
      this.markers = [];

      this.waypoints.forEach((waypoint, i) => {
        const el = document.createElement('div')
        el.className = 'marker';
        el.innerHTML = i
        var way = new mapboxgl.Marker(el).setLngLat([waypoint.lng, waypoint.lat]).addTo(this.map);
        this.markers.push(way);
      });

    }

  },

  data() {
      return {
        map: null, // Reference to the mapbox object
        markers: [],
      };
  },
  mounted() {
    this.map = this.makeMap();
    if(this.waypoints)
      this.updateMarkers();
  },
};
</script>

<style>
.marker {
  background-color: white;
  background-size: cover;
  width: 1.5em;
  height: 1.5em;
  border-radius: 50%;
  position: absolute;
  cursor: pointer;
  display: flex;
  justify-content: center;
  align-items: center;
  font-size: 3em;
}

#map {
  position: absolute;
  top: 0;
  bottom: 0;
  left: 0;
  right: 0;
  width: 100%;
}
</style>
