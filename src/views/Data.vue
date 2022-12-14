<template>
  <div class="mea">
    <v-container fill-height fluid flex class="pa-2 mt-3 d-flex">
      <v-col>
        <Map cols="col col-6" ref="Map" />
      </v-col>

      <v-col :cols="6">
        <v-row class="pa-2">
          <GeneralStatus />
        </v-row>
        <v-row class="pa-2">
          <EKFViewer />
        </v-row>
        <v-row class="pa-2">
          <DataViewer />
        </v-row>
      </v-col>
    </v-container>
  </div>
</template>

<script>
import GeneralStatus from "@/components/GeneralStatus.vue";
import Map from "@/components/Map.vue";
import EKFViewer from "@/components/Data/EKFViewer.vue";
import DataViewer from "@/components/Data/DataViewer.vue";

export default {

  mounted() {
  },
  updated() {
  },
  beforeDestroy() {
  },
  props: ["statusCode"],
  components: {
    GeneralStatus,
    DataViewer,
    Map,
    EKFViewer
  },

  data: () => ({
  }),
  methods: {
  },
};
</script>
