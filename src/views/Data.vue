<template>
  <div class="mea">
    <v-container fill-height fluid flex class="pa-2 mt-3 d-flex">
      <v-row align="auto">
        <Map
          cols="col col-6"
          ref="Map"
        />
        <v-col :cols="6">
          <v-container fluid flex>
            <v-row class="pb-3 px-3">
              <GeneralStatus/>
            </v-row>
            <v-row class="pt-0" align="auto">
              <DataViewer/>
            </v-row>
          </v-container>
        </v-col>
      </v-row>
    </v-container>
  </div>
</template>

<script>
import GeneralStatus from "@/components/GeneralStatus.vue";
import Map from "@/components/Map.vue";
import DataViewer from "../components/DataViewer.vue";

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
  },

  data: () => ({
  }),
  methods: {
  },
};
</script>
