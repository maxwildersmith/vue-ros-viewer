<template>
  <div class="mea">
    <v-container fill-height fluid flex class="pa-2 mt-3 d-flex">
      <v-row align="auto">
        <Map
          cols="col col-6"
          @mapMounted="mapMounted"
          ref="map"
          @clicked="mapClicked"
        />
        <v-col :cols="6">
          <v-container fluid flex>
            <v-row class="pb-3 px-5">
              <GeneralStatus
              />
            </v-row>
            <v-row class="px-5">
                  <DepthPath
                    editable=true
                  />
            </v-row>
            <v-container
              class="mt-5 pt-1 scrollable"
              style="height: 515px; overflow-y: hidden"
            >
              <v-row class="d-flex" align="auto">
                <v-col cols="6" class="ml-0 pl-3">
                  <WaypointList 
                    @selected="waypointSelected"
                    @changed="changed"/>
                </v-col>
                <v-col cols="6" class="d-flex">
                  <WaypointEditor 
                    @changed="changed"
                    ref="editor"/>
                </v-col>
              </v-row>
            </v-container>
          </v-container>
        </v-col>
      </v-row>
    </v-container>
  </div>
</template>

<script>
import DepthPath from "@/components/Path/DepthPath.vue";
import WaypointEditor from "@/components/Path/WaypointEditor.vue";
import WaypointList from "@/components/Path/WaypointList.vue";
import GeneralStatus from "@/components/GeneralStatus.vue";
import { useWaypointStore } from '@/components/stores/waypoints';
import Map from "@/components/Map.vue";

export default {
  setup(){
    const waypoints = useWaypointStore();

    return waypoints;
  },
  components: {
    DepthPath,
    GeneralStatus,
    WaypointEditor,
    WaypointList,
    Map,
  },

  data: () => ({
    selected: 0
  }),
  
  methods: {
    mapClicked(latLng) {
      this.addWaypoint(latLng.lat, latLng.lng, 0,0);
      this.$refs.map.updateMarkers();
    },
    setGeneralStatus(stage, vehicle) {
      this.$emit("setGeneralStatus", stage, vehicle);
      this.updatedStage = stage;
      this.updatedVehicle = vehicle;
    },
    waypointSelected(index){
      this.$refs.editor.select(index)
    },
    changed(){
      this.$refs.map.updateMarkers();
    }
  },
};
</script>

<style>
.scrollable:hover,
.scrollable:active,
.scrollable:focus {
  overflow-y: auto !important;
}
</style>
