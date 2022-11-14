<template>
  <nav id="nav">
    <v-app-bar fixed app class="primary">
      <v-img
        class="rounded-circle"
        contain
        height="100%"
        aspect-ratio=1
        src="../assets/logo.png"
      ></v-img>
      <v-toolbar-title>
        <span><h1 class="accent--text font-weight-light">Mission Planner</h1></span>
      </v-toolbar-title>
      <v-spacer></v-spacer>
      <router-link :to="{ name: 'Main', params: { statusCode, waypoints} }"
        ><h1 class="font-weight-light">Main</h1></router-link
      >
      <router-link :to="{ name: 'Path', params: { statusCode, waypoints } }"
        ><h1 class="font-weight-light">Path</h1></router-link
      >
      <router-link :to="{ name: 'Data', params: { statusCode } }"
        ><h1 class="font-weight-light">Data</h1></router-link
      >
      <router-link to="/geofence"
        ><h1 class="font-weight-light">Geofence</h1></router-link
      >
    </v-app-bar>
    <v-main>
      <router-view @setGeneralStatus="setGeneralStatus"></router-view>
    </v-main>
  </nav>
</template>

<script>
export default {
  data() {
    return {
      statusCode: 0,
      waypoints:  [
              {lat: 10, lng: 15, depth_max: -2, depth_min: 0}, 
              {lat: 10, lng: 15, depth_max: -100, depth_min: -90}, 
              {lat: 15, lng: 20, depth_max: -20, depth_min: -15},
              {lat: 15, lng: 20, depth_max: -90, depth_min: -80},
              {lat: 10, lng: 15, depth_max: -100, depth_min: -90}, 
              {lat: 15, lng: 20, depth_max: -20, depth_min: -15},
              {lat: 15, lng: 20, depth_max: -90, depth_min: -80}, 
              {lat: 10, lng: 15, depth_max: -2, depth_min: 0}]
    };
  },
  methods: {
    setGeneralStatus(statusCode) {
      this.statusCode = statusCode;
      console.log(statusCode)
      this.$forceUpdate();
    },
  },
};
</script>

<style scoped>
#nav a {
  padding: 40px;
  font-size: xx-large;
  text-decoration: none;
  color: #c9e7ff;
}

#nav a.router-link-exact-active {
  color: #61a3ff;
  text-decoration: underline;
}

#nav a:hover {
  text-decoration: underline;
}
</style>