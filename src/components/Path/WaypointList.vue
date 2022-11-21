<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 purple--text pr-4">
          Waypoints
        </h2>
      </v-row>
      <v-row justify="center" class="pb-2">
        <v-btn color="green" class="me-2" @click="moveUp()">
          <v-icon color="white">mdi-arrow-up</v-icon>
        </v-btn>
        <v-btn color="green" class="me-2" @click="moveDown()">
          <v-icon color="white">mdi-arrow-down</v-icon>
        </v-btn>
        <v-btn color="red" class="me-2" @click="remove()">
          <v-icon color="white">mdi-close-circle-outline</v-icon>
        </v-btn>
      </v-row>
      <v-row class="pb-2" style="max-height: 15em; overflow-y: auto">
        <v-list-item-group height="100" width="100" v-model="selected" mandatory>
          <v-list-item v-for="(pt, i) in waypoints" :key="i"> {{ i }}: {{ pt.lat.toFixed(6) }}, {{ pt.lng.toFixed(6) }};
            {{ pt.depth_min }}~{{ pt.depth_max }}m
          </v-list-item>
        </v-list-item-group>
      </v-row>
    </v-container>
  </v-card>
</template>
<style>
  .v-item-group{
    width: 100%;
  }
</style>
<script>
import { useWaypointStore } from '@/components/stores/waypoints';

export default {
  setup() {
    const waypoints = useWaypointStore();

    return waypoints;
  },
  data() {
    return { selected: 0 };
  },
  watch: {
    selected(newVal, oldVal) {
      if (newVal !== oldVal) this.$emit('selected', newVal);
    },
  },
  methods: {
    moveUp() {
      if (this.selected > 0) {
        this.waypointUp(this.selected);
        this.selected--;
        this.$emit('changed');
      }
    },
    moveDown() {
      if (this.selected < this.waypoints.length-1) {
        this.waypointDown(this.selected);
        this.selected++;
        this.$emit('changed');

      }
    },
    remove() {

      this.removeWaypoint(this.selected);
      if(this.selected == this.waypoints.length)
        this.selected--;
      this.$emit('changed');
    }
  }
};
</script>