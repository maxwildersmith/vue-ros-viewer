<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 green--text pr-4">
          EKF Data
        </h2>
      </v-row>
      <v-row justify="center">
            <h2 class="font-weight-light pr-2" v-if="waypoints.length < 0">Please add waypoints to edit depth</h2>
          <LineChart :chart-options="chartOptions" :chart-data="chartData" 
            :dataset-id-key="datasetIdKey" :styles="styles" style="width: 100%" :height="height" :tension="0.5" />

      </v-row>
    </v-container>
  </v-card>
</template>

<script>
import { useWaypointStore } from '@/components/stores/waypoints';
import { Line as LineChart } from "vue-chartjs/legacy";
import {
  Chart as ChartJS,
  Title,
  Filler,
  Tooltip,
  LineElement,
  LinearScale,
  CategoryScale,
  PointElement
} from "chart.js";

ChartJS.register(
  Filler,
  Title,
  Tooltip,
  LineElement,
  LinearScale,
  CategoryScale,
  PointElement
);

export default {
  props: ['editable'],
  components: {
    LineChart
  },
  setup(){
    const waypoints = useWaypointStore();

    return waypoints;
  },
  data() {
    return {
      height: 200,
      datasetIdKey: 'label',


      chartOptions: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false }}
      },
    };
  },
  computed: {
    chartData() {
      let out = {
        labels: [...Array(this.waypoints.length).keys()],
        datasets: [
          {
            label: "Min Depth",
            borderColor: "#66CCCC",
            data: this.waypoints.map(pt => -pt.depth_min),
            tension: 0.3,
            backgroundColor: "#66aaaa",
          }, 
          {
            label: "Max Depth",
            borderColor: "#CC6666",
            data: this.waypoints.map(pt => -pt.depth_max),
            tension: 0.3,
            backgroundColor: "#33AAAA50",
            fill: '-1',

          }
        ]
      }
      return out
    }
  },
  methods: {}

};
</script>

<style>
.scrollable:hover,
.scrollable:active,
.scrollable:focus {
  overflow-y: auto !important;
}
</style>

