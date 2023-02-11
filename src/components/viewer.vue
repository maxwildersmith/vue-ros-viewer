<template>
  <div id="container" ref="sceneContainer"></div>
</template>

<style lang="scss">
div#container {
  width: 100%;
  height: 500px;
  border-style: solid;
}
</style>

<script>
import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader';



export default {
  name: 'Viewer',

  props: {
    model: { type: String, default: "glider.glb" },
    width: { type: Number, default: 100 },
    height: { type: Number, default: 100 },
    spin: { type: Object, default(rawProps) { return { x: 0.0002, y: 0.0003, z: 0.0000 } } },
  },
  computed: {
    cssProps() {
      return {
        '--cont-width': this.width + 'vw',
        '--cont-height': this.height + 'vh',
      }
    }
  },
  data() {
    return {
      container: null,
      scene: null,
      camera: null,
      controls: null,
      renderer: null,
      stats: null,
      sat: null,
      compass:null,
      mixer: null,
    }
  },
  methods: {
    
  setRotation(roll,pitch,yaw){
    this.sat.scene.rotation.x = roll * 3.14/180;
    this.sat.scene.rotation.y = -yaw * 3.14/180;
    this.sat.scene.rotation.z = pitch * 3.14/180;
    console.log(roll);
    console.log(pitch);
    console.log(yaw);
  },
    init() {
      // set container
      this.container = this.$refs.sceneContainer
      // add camera
      const fov = 60 // Field of view
      const aspect = this.container.clientWidth / this.container.clientHeight
      const near = 0.1 // the near clipping plane
      const far = 30 // the far clipping plane
      const camera = new THREE.PerspectiveCamera(fov, aspect, near, far)
      camera.position.set(0, 2, 3)
      this.camera = camera
      // create scene
      this.scene = new THREE.Scene()

      const grid = new THREE.GridHelper(10, 10);

      this.scene.add(grid);

      // add lights
      const ambientLight = new THREE.HemisphereLight(
        0xffffff, // bright sky color
        0x222222, // dim ground color
        1 // intensity
      )
      const mainLight = new THREE.DirectionalLight(0xffffff, 4.0)
      mainLight.position.set(10, 10, 10)
      this.scene.add(ambientLight, mainLight)
      // add controls
      this.controls = new OrbitControls(this.camera, this.container)
      // create renderer
      this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
      this.renderer.setPixelRatio(window.devicePixelRatio)
      this.renderer.outputEncoding = THREE.sRGBEncoding
      this.renderer.physicallyCorrectLights = true
      this.container.appendChild(this.renderer.domElement)
      // set aspect ratio to match the new browser window aspect ratio
      this.camera.aspect = this.container.clientWidth / this.container.clientHeight
      this.camera.updateProjectionMatrix()
      this.renderer.setSize(this.container.clientWidth, this.container.clientHeight)
      const loader = new GLTFLoader()

      const dracoLoader = new DRACOLoader();
      dracoLoader.setDecoderPath('https://www.gstatic.com/draco/v1/decoders/');
      loader.setDRACOLoader(dracoLoader);
      this.lastframe = Date.now();

      loader.load(
        '/models/compass.glb',
        gltf => {
          this.compass = gltf
          this.compass.scene.scale = {x:.5, y:.5, z:.5}
          this.scene.add(this.compass.scene)
        },
        undefined,
        undefined
      )

      loader.load(
        '/models/' + this.model,
        gltf => {
          this.sat = gltf
          this.scene.add(this.sat.scene)
        },
        undefined,
        undefined
      )
      this.renderer.setAnimationLoop(() => {
        this.render()
      })
    },
    render() {
      this.renderer.render(this.scene, this.camera)
    }
  },
  mounted() {
    this.init()
  },




};

</script>