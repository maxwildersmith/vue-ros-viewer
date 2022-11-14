<template>
  <v-card class="pa-2" width="100%">
    <h1 class="font-weight-light text-center">UUV Status:<br /></h1>
    <h1 class="font-weight-regular text-center display-2"> {{formattedStatus}}</h1>
  </v-card>
</template>

<script>
export default {
  props: ["statusCode"],
  data: () => ({
    statuses: [
      'Offline',        // 0
      'Ready for Path', // 1
      'Standby',        // 2
      'Executing',      // 3
      'Done',           // 4
      'ERROR'           // -1
    ]
  }),
  computed: {
    formattedStatus() {
      if(this.statusCode == undefined) {this.statusCode = -1}
      return this.statuses[this.statusCode]
    }
  }
};
</script>