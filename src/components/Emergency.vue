<template>
  <div class="emergency">
    <v-btn width="100%" dark x-large color="red" class="mx-2" @click="dialog = true">
      Send Data<v-icon x-large>mdi-alert-octagon</v-icon>
    </v-btn>
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
          <v-text-field v-model="ip" label="SSH IP"></v-text-field>
        </v-card-text>

        <v-card-actions>
          <v-spacer></v-spacer>
          <v-btn color="secondary" text @click="toggleStatus"> Toggle Status {{ 'On' ? activated : 'Off' }} </v-btn>

          <v-btn color="primary" text @click="sendData"> Send Waypoint </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>
  </div>
</template>

<script>
import { useWaypointStore } from '@/components/stores/waypoints';

import axios from 'axios';

export default {
  setup() {
    const store = useWaypointStore();
    return store;
  },
  data() {
    return {
      dialog: false,
      activated: true,
      interId: null,
    };
  },
  mounted() {
    if(this.interId==null){
      this.interId = setInterval(this.getStatus, 1000);
    }
  },
  methods: {
    toggleStatus(){
      this.activated = !this.activated;
      this.dialog = false;


    },
    sendData() {
      this.dialog = false;

      const path = "http://"+this.ip+":5000/waypoints";
      const formData = new FormData();
      formData.append('waypoints', JSON.stringify(this.waypoints));
      axios
        .post(path, formData)
        .then(() => {
          console.log("Sent waypoint data");
        })
        .catch((error) => {
          console.log(error);
        });
    },
    getStatus() {
      if(this.activated){
        const path = "http://"+this.ip+":5000/status";
        axios
          .get(path)
          .then((resp) => {
            this.status = resp.data.code;
            this.position = resp.data.pos;
          })
          .catch((error) => {
            //console.log('error connecting, retrying...');
          });
      }
    },
  },
};
</script>