import Vue from 'vue'
import VueRouter from 'vue-router'
import Main from '../views/Main.vue'
import Data from '../views/Data.vue'
import Path from '../views/Path.vue'
import Geofence from '../views/Geofence.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '/',
    name: 'Main',
    component: Main,
    props: true,
  },
  {
    path: '/path',
    name: 'Path',
    component: Path,
    props: true,
  },
  {
    path: '/data',
    name: 'Data',
    component: Data,
    props: true,
  },
  {
    path: '/geofence',
    name: 'Geofence',
    component: Geofence
  }
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router
