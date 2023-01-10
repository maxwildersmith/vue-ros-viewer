<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 pr-4">
          Waypoint Editor
        </h2>
      </v-row>
      <div v-if="waypoints.length > 0">
      <v-row>
        <v-text-field v-model.number="waypoint.lat" type="number" label="Latitude" step=".00001" @input="changed"></v-text-field>
      </v-row>
      <v-row>
        <v-text-field v-model.number="waypoint.lng" type="number" label="Longitude" step=".00001" @input="changed"></v-text-field>
      </v-row>
      <v-row>
        <v-slider v-model.number="waypoint.depth_min" min="0" max="100" label="Min Depth"><template v-slot:append>
            <v-text-field v-model.number="waypoint.depth_min" class="mt-0 pt-0" hide-details single-line type="number"
              style="width: 60px"></v-text-field>
          </template></v-slider>
      </v-row>
      <v-row>
        <v-slider v-model.number="waypoint.depth_max" min="0" max="100" label="Max Depth"><template v-slot:append>
            <v-text-field v-model.number="waypoint.depth_max" class="mt-0 pt-0" hide-details single-line type="number"
              style="width: 60px"></v-text-field>
          </template></v-slider>
      </v-row>
    </div>
    </v-container>
  </v-card>
</template>

<script>
import { useWaypointStore } from '../stores/waypoints';

export default {
  setup() {
    const waypoints = useWaypointStore();

    return waypoints;
  },
  data() {
    return {waypoint: {lat: 0, lng: 0, depth_max: 0, depth_min: 0}};
  },
  methods: {
    select(index) {
      this.waypoint = this.waypoints[index];
    },
    changed(){
      this.$emit('changed');
    }
  }
};
</script>