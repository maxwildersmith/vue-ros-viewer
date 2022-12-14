<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 purple--text pr-4">
          Waypoints
        </h2>
      </v-row>
      <v-row justify="center" class="pb-2">
        <v-btn color="blue" class="me-2" @click="upload()">
          <v-icon color="white">mdi-upload</v-icon>
        </v-btn>
        <v-btn color="blue" class="me-2" @click="download()">
          <v-icon color="white">mdi-download</v-icon>
        </v-btn>
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
    <v-dialog v-model="dialog" max-width="425">
      <v-card>
        <v-card-title class="headline">
          <v-icon large color="red" class="pr-3">mdi-alert</v-icon>
          <h3 class="font-weight-light text-center red--text" justify="center">
            {{action}} waypoints
          </h3>
        </v-card-title>
        <v-card-text justify="center">
          <v-file-input v-if="action=='Load'" v-model="fileObj" label="Filename"></v-file-input>
          <v-text-field v-else v-model="fname" label="Filename"></v-text-field>
        </v-card-text>

        <v-card-actions>
          <v-spacer></v-spacer>

          <v-btn color="secondary" text @click="cancel"> Cancel </v-btn>

          <v-btn color="primary" text @click="file"> Confirm </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>
  </v-card>
</template>
<style>
  .v-item-group{
    width: 100%;
  }
</style>
<script>
import { useWaypointStore } from '@/components/stores/waypoints';
import { saveAs } from 'file-saver';
import { all } from 'q';

export default {
  setup() {
    const waypoints = useWaypointStore();

    return waypoints;
  },
  data() {
    return { selected: 0, action: 'Save', fname: '', dialog: false, fileObj: ''};
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
    },
    upload(){
      this.action = 'Load'
      this.dialog = true
    },
    download(){
      this.action = 'Save'
      this.dialog = true
    },
    cancel() {
      this.dialog = false
    },
    file(){
      if(this.action == 'Save'){
        var file = new Blob([JSON.stringify(this.waypoints)],{ type: "text/plain;charset=utf-8" });
        saveAs(file, this.fname)
        this.dialog = false
      }else {
        const reader = new FileReader();
        reader.addEventListener('load',()=>{this.waypoints = JSON.parse(reader.result)});
        reader.readAsText(this.fileObj)
        this.dialog = false
      }
    }

  }
};
</script>