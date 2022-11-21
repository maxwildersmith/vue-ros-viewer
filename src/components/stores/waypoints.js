import {defineStore} from "pinia";

export const useWaypointStore = defineStore({
    state: () => {
        return {waypoints: [],
                msgs: [],
                status: 0,
                position: [0,0,0]}
    },
    getters: {
        filteredList: (state) => {
            return (filters) => state.msgs.filter((msg) => msg.tag in filters);
        }
    },
    actions: {
        addWaypoint(lat, lng, depth_min, depth_max ) {
            console.log("added pt");
            this.waypoints.push({lat: lat, lng: lng, depth_min: depth_min, depth_max: depth_max})
        },
        removeWaypoint(index) {
            if(index < this.waypoints.length-1 || this.waypoints.length > 1 || this.waypoints.length > index){
                this.waypoints.splice(index,1);
                console.log("removed")
                return true;
            }
            return false;
        },
        waypointUp(index) {
            if(index>0){
                var way = this.waypoints[index];
                this.waypoints.splice(index, 1);
                this.waypoints.splice(index-1, 0, way);
            }
        },
        waypointDown(index) {
            if(index<this.waypoints.length){
                var way = this.waypoints[index];
                this.waypoints.splice(index, 1);
                this.waypoints.splice(index+1, 0, way);
            }
        },
    }
});