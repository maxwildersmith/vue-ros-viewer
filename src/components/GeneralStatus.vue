<template>
  <v-card class="pa-2" width="100%">
    <h1 class="font-weight-light text-center">UUV Status:<br /></h1>
    <h1 class="font-weight-regular text-center display-2"> {{formattedStatus}}</h1>
  </v-card>
</template>

<script>
import { useWaypointStore } from '@/components/stores/waypoints.js';


export default {
  setup(){
    const store = useWaypointStore();

    return {store};
  },
  data: () => ({
    statuses: [
      'Offline',        // 0
      'Ready for Path', // 1
      'Standby',        // 2
      'Executing',      // 3
      'Done',           // 4
      'ERROR'           // 5
    ]
  }),
  computed: {
    formattedStatus() {
      if(this.store.status == undefined) {this.store.status = 5}
      return this.statuses[this.store.status]
    }
  }
};
</script>