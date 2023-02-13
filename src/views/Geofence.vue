<template>
  <div>

    <v-container  >
      <Viewer ref="view"/>
      <v-btn width="100%" dark x-large color="red" class="mx-2" @click="dialog = true">
      Send Data<v-icon x-large>mdi-alert-octagon</v-icon>
    </v-btn>
    </v-container>
    <v-dialog v-model="dialog" max-width="425">
      <v-card>
        <v-card-title class="headline">
          <v-icon large color="red" class="pr-3">mdi-alert</v-icon>
          <h3 class="font-weight-light text-center red--text" justify="center">
            Data connection
          </h3>
        </v-card-title>
        <v-card-text justify="center">
          Comms stuffs
          <v-text-field v-model="roll" label="roll"></v-text-field>
          <v-text-field v-model="pitch" label="pitch"></v-text-field>
          <v-text-field v-model="yaw" label="yaw"></v-text-field>

        </v-card-text>

        <v-card-actions>
          <v-spacer></v-spacer>
          <v-btn color="secondary" text @click="sendD"> rotate </v-btn>

          <v-btn color="primary" text @click="dialog = false"> Send Waypoint </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>
  </div>
</template>

<script>
import Viewer from "@/components/viewer.vue";
import Map from "@/components/Map.vue";
import { useWaypointStore } from '@/components/stores/waypoints';


export default {
  name: "",

  components: {
    Viewer,
    Map,
  },
  setup(){
    const waypoints = useWaypointStore();

    return waypoints;
  },
  watch:{
    position(newPos, oldPos){
      this.$refs.view.setQuaternion(newPos[3], newPos[4], newPos[5], newPos[6]);
    }
  },

  data: () => ({
    roll: 0,
    pitch: 0,
    yaw: 0,
    dialog: false,
  }),
  methods: {
    mapMounted() {
      this.mapMountedCheck = true;
    },
    sendD(){
      this.$refs.view.setRotation(this.roll, this.pitch, this.yaw);
    }
  },
};
</script>
