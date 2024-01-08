import { ArcRotateCamera } from "@babylonjs/core/Cameras/arcRotateCamera";
import { Engine } from "@babylonjs/core/Engines/engine";
import { HemisphericLight } from "@babylonjs/core/Lights/hemisphericLight";
import { Color3, Color4 } from '@babylonjs/core/Maths/math.color';
import { Vector3 } from '@babylonjs/core/Maths/math.vector';
import { CreateLineSystem } from '@babylonjs/core/Meshes/Builders/linesBuilder';
import { Scene } from "@babylonjs/core/scene";
import { objPars, readFile } from './utils/parseObj';
import { createMesh, updateMesh } from "./utils/MeshBufferUtils";
import { CreateBox } from '@babylonjs/core/Meshes/Builders/boxBuilder';
import { AdvancedDynamicTexture } from '@babylonjs/gui/2D/advancedDynamicTexture';
import { StackPanel } from '@babylonjs/gui/2D/controls/stackPanel';
import { Button } from '@babylonjs/gui/2D/controls/button';
import { Control } from '@babylonjs/gui';
import { MeshBuilder } from "@babylonjs/core";
import { STLExport } from "@babylonjs/serializers";

export function renderScene() {
    const canvas = document.getElementById("app");
    const engine = new Engine(canvas);
    const scene = new Scene(engine);

    function renderLoop(){
     scene.render();
    }
    engine.runRenderLoop(renderLoop);
    
    const camera = new ArcRotateCamera("Camera", 0, 0.8, 100, Vector3.Zero(), scene);
    camera.attachControl(canvas, false);
    camera.setPosition(new Vector3(0, 5, -10))
    
    const light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);
    
    // Default intensity is 1. Let's dim the light a small amount
    light.intensity = 0.7;

    window.addEventListener("resize", () => {
        engine.resize();
    });



    return new Promise((resolve) => {
        scene.whenReadyAsync().then(() => {
            resolve(scene);
        
        })
    })
    
}

renderScene().then((scene) => {
    const url = "/gumline.obj";
  
    readFile(url, (data) => {
        let { positions, cells } = objPars(data, null);
        console.log(cells);
        console.log('cells');
        console.log(positions);
        console.log('positions');

        const polyMesh = new global.Module.PolyMesh();
        for(let i = 0; i < positions.length; i++){
            polyMesh.addVertex(positions[i][0], positions[i][1], positions[i][2]);
        }
        
        for(let i = 0; i < cells.length; i++){
            const face = cells[i];
            polyMesh.addFace(face);
        }

        function getMeshFromPolyMesh(polyMesh) {
            let positions_buffer = polyMesh.getVertices();
            positions = Array.from(positions_buffer);
            positions_buffer = null;
    
            let triangles_buffer = polyMesh.getIndices();
            let indices = Array.from(triangles_buffer);
            triangles_buffer = null;
    
            return {positions, indices};
        }

        //retrive polymesh data
        function getMeshData(){
            let positions_buffer = polyMesh.getVertices();
            positions = Array.from(positions_buffer);
            positions_buffer = null;
    
            let triangles_buffer = polyMesh.getIndices();
            let indices = Array.from(triangles_buffer);
            triangles_buffer = null;
    
            return {positions, indices};
        }

        let dt = getMeshData();

        const mesh = createMesh(scene, {positions: dt.positions.flat(), indices: dt.indices });
        mesh.normalizeToUnitCube();
        mesh.scaling.scaleInPlace(5);



        const advancedTexture = AdvancedDynamicTexture.CreateFullscreenUI("UI");
        const stackPanel = new StackPanel();
        stackPanel.background = "green";
        stackPanel.width = "220px";
        stackPanel.horizontalAlignment = Control.HORIZONTAL_ALIGNMENT_RIGHT;
        advancedTexture.addControl(stackPanel);

        let clarkMulBt = Button.CreateSimpleButton("but1", "Catmull-Clark");
        clarkMulBt.height = "40px";
        clarkMulBt.onPointerUpObservable.add(function() {
        polyMesh.catmull_smooth();
        const mesh_data = getMeshData();
        updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(clarkMulBt);

        let loopBt = Button.CreateSimpleButton("but2", "Loop");
        loopBt.height = "40px";
        loopBt.onPointerUpObservable.add(function() {
        polyMesh.loop_smooth();
        const mesh_data = getMeshData();
        updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(loopBt);

        let dooSabinBt = Button.CreateSimpleButton("but3", "Doo-Sabin");
        dooSabinBt.height = "40px";
        dooSabinBt.onPointerUpObservable.add(function() {
        polyMesh.dooSabin_smooth();
        const mesh_data = getMeshData();
        updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(dooSabinBt);

        let sqrtBt = Button.CreateSimpleButton("but4", "Sqrt");
        sqrtBt.height = "40px";
        sqrtBt.onPointerUpObservable.add(function() {
            polyMesh.sqrt_smooth();
            const mesh_data = getMeshData();
            updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(sqrtBt);

        let delaunayRemeshBt = Button.CreateSimpleButton("but4", "Delaunay Remesh");
        delaunayRemeshBt.height = "40px";
        delaunayRemeshBt.onPointerUpObservable.add(function() {
            const outMesh = polyMesh.remesh_delaunay();
            const meshData = getMeshFromPolyMesh(outMesh);
            updateMesh(mesh, {...meshData, colors: null});
            setTimeout(() => {
                STLExport.CreateSTL([mesh], true, 'foo.stl', true);
            }, 5000);
            // console.log(meshData);
            // const n = data.length;
            // const nvecs = n / 3;
            // for (let i = 0; i < nvecs; i += 3) {
            //     const sp = MeshBuilder.CreateSphere(`ms_${i}`, {
            //         diameter: 0.1,
            //     });
            //     sp.position.x = data[i];
            //     sp.position.y = data[i + 1];
            //     sp.position.z = data[i + 2];
            // }
            // const mesh_data = getMeshData();
            // updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(delaunayRemeshBt);

        let isotropicRemeshBt = Button.CreateSimpleButton("but4", "Isotropic Remesh");
        isotropicRemeshBt.height = "40px";
        isotropicRemeshBt.onPointerUpObservable.add(function() {
            polyMesh.remesh_isotropic();
            const meshData = getMeshFromPolyMesh(polyMesh);
            updateMesh(mesh, {...meshData, colors: null});
            setTimeout(() => {
                STLExport.CreateSTL([mesh], true, 'foo.stl', true);
            }, 5000);
            // console.log(meshData);
            // const n = data.length;
            // const nvecs = n / 3;
            // for (let i = 0; i < nvecs; i += 3) {
            //     const sp = MeshBuilder.CreateSphere(`ms_${i}`, {
            //         diameter: 0.1,
            //     });
            //     sp.position.x = data[i];
            //     sp.position.y = data[i + 1];
            //     sp.position.z = data[i + 2];
            // }
            // const mesh_data = getMeshData();
            // updateMesh(mesh, {...mesh_data, colors: null});
        });
        stackPanel.addControl(isotropicRemeshBt);

        let domainRemeshBt = Button.CreateSimpleButton("but4", "Domain Remesh");
        domainRemeshBt.height = "40px";
        domainRemeshBt.onPointerUpObservable.add(function() {
            const outMesh = polyMesh.polyhedral_mesh_generation();
            console.log('done');
            console.log(outMesh);
            const meshData = getMeshFromPolyMesh(outMesh);
            updateMesh(mesh, {...meshData, colors: null});
            setTimeout(() => {
                STLExport.CreateSTL([mesh], true, 'foo.stl', true);
            }, 1000);
        });
        stackPanel.addControl(domainRemeshBt);

        let refineButton = Button.CreateSimpleButton("but4", "Refine");
        refineButton.height = "40px";
        refineButton.onPointerUpObservable.add(function() {
            // polyMesh.print_max_edge_length();
            polyMesh.refine(0.5);
            const meshData = getMeshFromPolyMesh(polyMesh);
            updateMesh(mesh, {...meshData, colors: null});
            // setTimeout(() => {
            //     STLExport.CreateSTL([mesh], true, 'foo.stl', true);
            // }, 1000);
        });
        stackPanel.addControl(refineButton);


        const myLines = [];
        for(var n = 0; n<cells.length; n++){
          var wirefaces = []
          for(var j = 0; j<cells[n].length; j++){
          var ind = cells[n][j]
          const wireframePoints = new Vector3(positions[(ind * 3) % positions.length], positions[(ind * 3 + 1) % positions.length], positions[(ind * 3 + 2) % positions.length])
          wirefaces.push(wireframePoints)
    
          }
          wirefaces.push(wirefaces[0])
          myLines.push(wirefaces)
        }
    
        const wireframe = CreateLineSystem("wireframe", {lines: myLines}, scene);
        wireframe.color = Color3.Green();
        wireframe.normalizeToUnitCube();
        wireframe.scaling.scaleInPlace(5);
        wireframe.rotation.y = Math.PI;
    })
})