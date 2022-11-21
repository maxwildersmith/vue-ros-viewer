<template>
  <div class="home">
    <v-container fill-height fluid class="pa-2 mt-1">
      <v-col>
      <Map cols="col col-8" @mapMounted="mapMounted" ref="Map" />

      </v-col>


        <v-col :cols="4">
          <v-container>
            <v-row class="pa-2 mb-3">
              <GeneralStatus/>
            </v-row>

            <v-row class="pa-2">
              <DepthPath editable=false />
            </v-row>

            <v-row class="pa-2">
              <DataViewer />
            </v-row>
            <v-row class="pa-2">
              <Emergency/>
            </v-row>
          </v-container>
        </v-col>
    </v-container>
  </div>
</template>

<script>
// @ is an alias to /src
import Emergency from "@/components/Emergency.vue";
import DataViewer from "@/components/DataViewer.vue";
import DepthPath from "@/components/Path/DepthPath.vue";
import GeneralStatus from "@/components/GeneralStatus.vue";
import Map from "@/components/Map.vue";
import axios from "axios";

export default {
  components: {
    Emergency,
    DataViewer,
    DepthPath,
    GeneralStatus,
    Map,
  },
  data: () => ({

    updatedStage: null,
    updatedVehicle: null,
    mac_data: [],
    hiker_data: [],
    Search_Area: [],
    firstGetMAC: true,
    firstGetHiker: true,
    firstGetHome: true,
    firstGetEvac: true,
    MACHomePointExists: false,
    evacPointExists: false,
    current_mac_lng: -117.6316988,
    current_mac_lat: 33.9336,
    current_mac_yaw: null,
    current_yaw: 42,
  }),
  beforeDestroy() {
    clearInterval(this.interval);
  },
  methods: {
    mapMounted() {
      this.interval = setInterval(() => this.updateLoop(), 500);
    },
    updateLoop() {
      if (!this.firstGetHiker && !this.firstGetMAC) {
        // this.getMACCurrentData();
        // this.getHikerCurrentData();
      }
    },
    setGeneralStatus(statusCode) {
      this.$emit("setGeneralStatus", statusCode);
      this.updatedStage = statusCode;
    },
    getMACCurrentData() {
      //MAC information
      let path = "http://127.0.0.1:5000/MAC_XBEE";
      axios
        .get(path)
        .then((res) => {
          this.mac_data = res.data.MAC;
          this.setMACPosition();
        })
        .catch((error) => {
          console.error(error.response);
        });
    },
    setMACPosition() {
      for (let i = 0; i < this.mac_data.length; i++) {
        if (this.mac_data[i].title == "Latitude") {
          this.current_mac_lat = this.mac_data[i].value;
        } else if (this.mac_data[i].title == "Longitude") {
          this.current_mac_lng = this.mac_data[i].value;
        } else if (this.mac_data[i].title == "Yaw") {
          this.current_mac_yaw = this.mac_data[i].value;
        }
      }
      let coord = [this.current_mac_lng, this.current_mac_lat]; //array for editPointSource
      let pointExists = this.$refs.Map.editPointSource("mac", coord);
      if (pointExists) {
        console.log("edited MAC point");
      } else {
        console.log("added MAC point");
        this.$refs.Map.addCoord(
          "mac",
          "mac",
          this.current_mac_lng,
          this.current_mac_lat
        );
      }
      this.$refs.Map.setRotation("mac", this.current_mac_yaw);
      this.firstGetMAC = false;
    },
    getHikerCurrentData() {
      //Hiker information
      let path = "http://127.0.0.1:5000/Hiker";
      axios
        .get(path)
        .then((res) => {
          this.hiker_data = res.data.Hiker;
          this.setHikerPosition();
        })
        .catch((error) => {
          console.error(error.response);
        });
    },
    setHikerPosition() {
      for (let i = 0; i < this.hiker_data.length; i++) {
        if (this.hiker_data[i].title == "Hiker_lat") {
          this.hiker_lat = this.hiker_data[i].value;
        } else if (this.hiker_data[i].title == "Hiker_lng") {
          this.hiker_lng = this.hiker_data[i].value;
        }
      }
      let coord = [this.hiker_lng, this.hiker_lat]; //array for editPointSource
      let pointExists = this.$refs.Map.editPointSource("hiker", coord);
      if (pointExists) {
        console.log("edited point");
      } else {
        console.log("added point");
        this.$refs.Map.addCoord(
          "hiker",
          "hiker",
          this.hiker_lng,
          this.hiker_lat
        );
      }
      this.firstGetHiker = false;
    },
    getCurrentTravelTo() {
      const path = "http://127.0.0.1:5000/MAC_INPUT";
      axios
        .get(path)
        .then((res) => {
          if (this.firstGetHome) {
            if (
              res.data.Travel_to_lng == 0 &&
              res.data.Travel_to_lat == 0 &&
              res.data.Travel_to_alt == 0
            ) {
            } else {
              this.setHomePosition(
                res.data.Travel_to_lng,
                res.data.Travel_to_lat
              );
            }
          } else {
            this.setHomePosition(
              res.data.Travel_to_lng,
              res.data.Travel_to_lat
            );
          }
        })
        .catch((error) => {
          console.error(error);
        });
    },
    setHomePosition(lng, lat) {
      let coord = [lng, lat]; //array for editPointSource
      this.editMACHome(coord);
      if (this.MACHomePointExists) {
        console.log("edited MACHome point");
      } else {
        console.log("added MACHome point");
        this.addMACHome(lng, lat);
      }
      this.firstGetHome = false;
    },
    editMACHome(coord) {
      this.MACHomePointExists = this.$refs.Map.editPointSource(
        "mac_home",
        coord
      );
    },
    addMACHome(lng, lat) {
      this.$refs.Map.addCoord("mac_home", "home", lng, lat);
    },
    getCurrentDropLocation() {
      const path = "http://127.0.0.1:5000/MAC_INPUT";
      axios
        .get(path)
        .then((res) => {
          if (this.firstGetERUDrop) {
            if (res.data.Drop_Loc_lng == 0 && res.data.Drop_Loc_lng == 0) {
            } else {
              this.setDropLocationPosition(
                res.data.Drop_Loc_lng,
                res.data.Drop_Loc_lat
              );
            }
          } else {
            this.setDropLocationPosition(
              res.data.Drop_Loc_lng,
              res.data.Drop_Loc_lat
            );
          }
        })
        .catch((error) => {
          console.error(error);
        });
    },
    setDropLocationPosition(lng, lat) {
      let coord = [lng, lat]; //array for editPointSource
      this.editERUDrop(coord);
      if (this.pointExists) {
        console.log("edited ERUDrop point");
      } else {
        console.log("added ERUDrop point");
        this.addERUDrop(lng, lat);
      }
      this.firstGetERUDrop = false;
    },
    editERUDrop(coord) {
      this.ERUDropPointExists = this.$refs.Map.editPointSource(
        "eru_drop_loc",
        coord
      );
    },
    addERUDrop(lng, lat) {
      this.$refs.Map.addCoord("eru_drop_loc", "drop-location", lng, lat); //not sure if naming is correct
    },
    getCurrentEvac() {
      const path = "http://127.0.0.1:5000/ERU_INPUT";
      axios
        .get(path)
        .then((res) => {
          if (this.firstGetEvac) {
            if (res.data.EZ_lng == 0 && res.data.EZ_lat == 0) {
            } else {
              this.setEvacPosition(res.data.EZ_lng, res.data.EZ_lat);
            }
          } else {
            this.setEvacPosition(res.data.EZ_lng, res.data.EZ_lat);
          }
        })
        .catch((error) => {
          console.error(error);
        });
    },
    setEvacPosition(lng, lat) {
      let coord = [lng, lat]; //array for editPointSource
      this.editEvac(coord);
      if (this.evacPointExists) {
        console.log("edited point");
      } else {
        console.log("added point");
        this.addEvac(lng, lat);
      }
      this.firstGetEvac = false;
    },
    editEvac(coord) {
      this.evacPointExists = this.$refs.Map.editPointSource("evac_zone", coord);
    },
    addEvac(lng, lat) {
      this.$refs.Map.addCoord("evac_zone", "evac-point", lng, lat);
    },
    getMACSearchArea() {
      const path = "http://127.0.0.1:5000/MAC_INPUT";
      axios
        .get(path)
        .then((res) => {
          this.Search_Area = res.data.Search_Area;
          // console.log(this.Search_Area)
          this.setSearchArea();
        })
        .catch((error) => {
          console.error(error);
        });
    },
    setSearchArea() {
      console.log("length: " + this.Search_Area.Coordinates.length);
      if (this.Search_Area.Coordinates.length > 0) {
        if (this.Search_Area.Circle_inputs.rad == null) {
          //Polygon
          this.setPolygonCoordinates();
        }
      }
      if (this.Search_Area.Circle_inputs.rad != null) {
        //Circle
        this.setCircleCoordinates();
      }
    },
    setPolygonCoordinates() {
      if (this.Search_Area.Coordinates.length > 0) {
        let tempCoordinates = new Array(this.Search_Area.Coordinates.length);
        let temp = [];
        for (let i = 0; i < this.Search_Area.Coordinates.length; i++) {
          temp = new Array(2);
          temp[0] = this.Search_Area.Coordinates[i].lng;
          temp[1] = this.Search_Area.Coordinates[i].lat;
          tempCoordinates[i] = temp;
        }
        this.addPolygon(tempCoordinates);
        // console.log(this.$refs.PolygonForm.Coordinates);
      }
    },
    setCircleCoordinates() {
      // console.log(this.Search_Area.Circle_inputs.lat);
      if (this.Search_Area.Circle_inputs.rad != null) {
        this.addCircle(
          this.Search_Area.Circle_inputs.lng,
          this.Search_Area.Circle_inputs.lat,
          this.Search_Area.Circle_inputs.rad
        );
      }
    },
    addPolygon(coordinates) {
      this.$refs.Map.removeLayer("Search Area");
      // console.log(coordinates);
      this.$refs.Map.addPoly(coordinates, "Search Area", "#00ff6a", 0.3);
      console.log("added Search Area from endpoint");
    },
    addCircle(lng, lat, rad) {
      this.$refs.Map.removeLayer("Search Area");
      this.circleCoords = this.$refs.Map.addCircle(
        lng,
        lat,
        rad,
        16,
        "Search Area",
        "#00ff6a",
        0.3
      );
    },
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
