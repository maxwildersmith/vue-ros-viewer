<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 green--text pr-4">
          Current Path
        </h2>
      </v-row>
      <v-row justify="center">
            <h2 class="font-weight-light pr-2" v-if="waypoints.length < 0">Please add waypoints to edit depth</h2>
          <LineChart :chart-options="chartOptions" :chart-data="chartData" 
            :dataset-id-key="datasetIdKey" :styles="styles" :width="width" :height="height" :tension="0.5" />

      </v-row>
    </v-container>
  </v-card>
</template>

<script>
import { Line as LineChart } from "vue-chartjs/legacy";
import {
  Chart as ChartJS,
  Title,
  Tooltip,
  LineElement,
  LinearScale,
  CategoryScale,
  PointElement
} from "chart.js";

ChartJS.register(
  Title,
  Tooltip,
  LineElement,
  LinearScale,
  CategoryScale,
  PointElement
);

export default {
  props: ['editable', 'waypoints'],
  components: {
    LineChart
  },
  data() {
    return {
      height: 200,
      width: 550,
      datasetIdKey: 'label',


      chartOptions: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false } }
      },
    };
  },
  computed: {
    chartData() {

      let out = {
        labels: [...Array(this.waypoints.length).keys()],
        datasets: [
          {
            borderColor: "#66CCCC",
            data: this.waypoints.map(pt => (pt.depth_max + pt.depth_min) / 2),
            tension: 0.5,
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

