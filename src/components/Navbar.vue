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
    };
  },
  methods: {
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