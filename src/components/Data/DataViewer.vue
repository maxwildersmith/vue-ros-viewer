<template>
  <v-card class="pa-2" width="100%">
    <v-container>
      <v-row justify="center" class="pb-2">
        <h2 class="font-weight-regular text-center display-2 purple--text pr-4">
          Live Data
        </h2>
      </v-row>
      <v-row justify="center" class="pb-2">
        <h3>Data filters:</h3>
      </v-row>
      <v-row justify="center" class="pb-2">
        <v-btn-toggle v-model="filters" multiple rounded group dense color="primary">
          <template v-for="tag in tags">
            <v-btn>
              <v-icon>mdi-{{ tag.icon }}</v-icon>{{ tag.txt }}
            </v-btn>
          </template>
        </v-btn-toggle>
      </v-row>
      <v-row class="pb-2">
        <v-sheet color="grey darken-2" elevation="5" width="100%" rounded>
          <v-virtual-scroll height="100" item-height="20" :items="dataFeed">
            <template v-slot:default="{ item }">
              <v-list-item v-show="item.tag in filters"> {{ item.msg }}
              </v-list-item>

            </template>

          </v-virtual-scroll>
        </v-sheet>
      </v-row>
    </v-container>
  </v-card>
</template>

<script>

export default {
  data() {
    return {
      tags: [{ txt: 'General', icon: 'alert-circle', id: 0 }, { txt: 'Power', icon: 'battery-charging-50', id: 1 }, { txt: 'Telemetry', icon: 'crosshairs-gps', id: 2 }, { txt: 'ERROR', icon: 'alert', id: 3 }],
      dataFeed: [{ tag: 0, msg: 'init' }, { tag: 1, msg: 'Battery at 50%' }, { tag: 3, msg: 'GPS FAILED' }, { tag: 2, msg: 'GPS connected' }, { tag: 2, msg: 'Saved starting location' }],
      filters: [0, 1, 2, 3, 4],
    };
  }
};
</script>