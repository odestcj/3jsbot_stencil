/*

     3jsbot 
     Implementation of robot kinematics, control, and decision making 
     in HTML5/JavaScript and threejs
     
     @author odestcj / https://github.com/odestcj

     Forgive my coding style.  I am still a typedef struct kind of guy.
     Need to get a handle on all of the global variables... in the future.

*/



//////////////////////////////////////////////////
/////     INITIALIZATION FUNCTION DEFINITONS
//////////////////////////////////////////////////

function init() {

    // instantiate threejs scene graph
    scene = new THREE.Scene();

    // instantiate threejs camera and set its position in the world
    camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 1, 10000 );
    camera.position.y = 1;
    camera.position.z = 4;

    var light1 = new THREE.PointLight( 0xffffff, 0.3, 1000 ); 
    light1.position.set( 50, 50, 50 ); 
    scene.add( light1 );

    var light2 = new THREE.PointLight( 0xffffff, 0.3, 1000 ); 
    light2.position.set( 50, 50, -50 ); 
    scene.add( light2 );

    var light3 = new THREE.PointLight( 0xffffff, 0.3, 1000 ); 
    light3.position.set( -50, 50, -50 ); 
    scene.add( light3 );

    var light4 = new THREE.PointLight( 0xffffff, 0.3, 1000 ); 
    light4.position.set( -50, 50, 50 ); 
    scene.add( light4 );

    //var light = new THREE.PointLight( 0xffffff, 1, 1000 ); 
    //light.position.set( 0, 10, 10 ); 
    //robot.links[robot.base].geom.add( light );

    // instantiate threejs renderer and its dimensions
    renderer = new THREE.WebGLRenderer();
    renderer.setSize( window.innerWidth, window.innerHeight );

    // attach threejs renderer to DOM
    document.body.appendChild( renderer.domElement );

    // instantiate threejs camera controls
    camera_controls = new THREE.OrbitControls( camera );
    camera_controls.addEventListener( 'change', renderer );

    // instantiate threejs keyboard controls, for continuous interactive controls
    keyboard = new THREEx.KeyboardState();

    // create events and handlers for interaction controls
    init_keyboard_events();
    update_ik = false;
    update_pd = false;

    display_map = true;
    if (display_map) {
    // !!! add check for map and flag to turn off
    // environment floor with map texture-mapped onto ground plane
    //mapTexture = new THREE.ImageUtils.loadTexture( "maps/willow.png" );
    //mapTexture = new THREE.ImageUtils.loadTexture( "maps/480px-Optical-illusion-checkerboard-bw.svg.png" );
    mapTexture = new THREE.ImageUtils.loadTexture( "maps/Cam_Prep-VFX-checkerboard-lens_distortion-1in.jpg" );
    //88 mapTexture = new THREE.ImageUtils.loadTexture( "maps/checkerboard-squares-black-white.jpg" );
    var mapMaterial = new THREE.MeshBasicMaterial( { map: mapTexture, transparent: true, opacity: 0.2 } ); 
    var mapGeometry = new THREE.PlaneGeometry(100, 100, 1, 1);
    map = new THREE.Mesh(mapGeometry, mapMaterial);
    map.doubleSided = true;
    //map.receiveShadow = true;
    map.rotateOnAxis({x:1,y:0,z:0},-Math.PI/2),
    scene.add(map);

    // create data structure for accessing texture pixels
    var myCanvas = document.createElement("canvas"); // create separate canvas
    myCanvas.width = map.material.map.image.width; 
    myCanvas.height = map.material.map.image.height;
    myCanvasContext = myCanvas.getContext("2d"); // Get canvas 2d context
    myCanvasContext.drawImage(map.material.map.image, 0, 0); // Draw the texture
    //texels = myCanvasContext.getImageData(0,0, myCanvas.width, myCanvas.height); // Read the texels/pixels back
    //console.log((texels.data.length/texels.width)/4); // should return image height
    //console.log((texels.data.length/texels.height)/4); // should return image width
    }

    // create geometry for endeffector and Cartesian target indicators
    var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
    var temp_material = new THREE.MeshBasicMaterial( {color: 0x0088ff} )
    endeffector_geom = new THREE.Mesh(temp_geom, temp_material); // comment this for coolness
    scene.add(endeffector_geom);
    endeffector_geom.visible = false;
    temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
    temp_material = new THREE.MeshBasicMaterial( {color: 0x00ff00} )
    target_geom = new THREE.Mesh(temp_geom, temp_material); // comment this for coolness
    scene.add(target_geom);
    target_geom.visible = false;

    // call user's initialization
    my_init();

    // reminder of init_robot() ...

    // CS148: uncomment after implementing joints in kinematic hierarchy 
    /*
    // initialize the active link/joint for control
    active_link = robot.base;
    active_joint = robot.links[active_link].children[0];
    //robot.links[active_link].geom.material.wireframe = false; 
    //robot.links[active_link].geom.material.opacity = 0.5; 
    robot.joints[active_joint].display_geom.material.wireframe = false; 
    robot.joints[active_joint].display_geom.material.opacity = 0.5; 
    */

    // !!! change this to scale geometries or scale view
    // scaling geometries for view
    //tempmat = new THREE.Matrix4();
    //tempmat.makeScale(scale_factor,scale_factor,scale_factor);
    //scene.applyMatrix(tempmat);

}


function init_robot_links_geoms() {
    for (x in robot.links) {

        // create threejs mesh for link
        //material = new THREE.MeshBasicMaterial( { color: 0x0000ff, wireframe: true, wireframeLinewidth: 5 } );
        //material = new THREE.MeshBasicMaterial( { color: 0x0000ff, transparent: true, opacity: 0.5 } );
        material = new THREE.MeshLambertMaterial( { color: 0x0000ff, transparent: true, opacity: 0.7 } );
        robot.links[x].geom = new THREE.Mesh( links_geom[x], material);

        // add to threejs scene graph (where kinematics are maintained independently)
        scene.add(robot.links[x].geom);
    }

    // need to know base link; add base link to threejs scene graph 
    scene.add(robot.links[robot.base].geom);

}



function init_robot_joints_geoms() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs
    // NOTE: simpleApplyMatrix can be used to set threejs transform for a rendered object

    var x,tempmat;
       
    for (x in robot.joints) {

        // create threejs geometry for joint origin 
        material = new THREE.MeshBasicMaterial( { color: 0xff0000, wireframe: true } );
        invisible_geom = new THREE.CubeGeometry( 0.01, 0.01, 0.01 );
        robot.joints[x].origin.geom = new THREE.Mesh( invisible_geom, material );

        // create threejs geometry for joint
        material = new THREE.MeshBasicMaterial( { color: 0xff0000, wireframe: true } );
        joint_geom = new THREE.CubeGeometry( 0.01, 0.01, 0.01 );
        //joint_geom = new THREE.SphereGeometry( 0.2, 5, 5 );
        //joint_geom = new THREE.CylinderGeometry( 0.2, 0.2, 0.2, 20, 3, false );  // cylinder axis aligns with along y-axis in object space
        robot.joints[x].geom = new THREE.Mesh( joint_geom, material );


        // Note: kinematics are maintained independently from threejs scene graph
        // add joint geometry to threejs scene graph, added SG node transforms cylinder geometry
        var temp_geom = new THREE.CylinderGeometry( 0.2, 0.2, 0.2, 20, 3, false );  // cylinder axis aligns with along y-axis in object space
        //var temp_material = new THREE.MeshBasicMaterial( {color: 0x444444} );
        var temp_material = new THREE.MeshLambertMaterial( {color: 0xff0000} );
        robot.joints[x].display_geom = new THREE.Mesh(temp_geom, temp_material); 
            
        // CS148: uncomment this if you have vector_cross implemented
        // (need to find better factoring)
        /*
        // if joint axis not aligned with y-axis, rotate 3js cylinder axis to align with y
        if (!((robot.joints[x].axis[0] == 0) && (robot.joints[x].axis[2] == 0))) {
            var tempaxis = vector_cross(robot.joints[x].axis,[0,-1,0]);
            var temp3axis = new THREE.Vector3(tempaxis[0],tempaxis[1],tempaxis[2]);
            // baked in dot product given cylinder axis is normal along y-axis
            var tempangle = Math.acos(robot.joints[x].axis[1]);
            robot.joints[x].display_geom.rotateOnAxis(temp3axis,tempangle);
        }
        */
        scene.add(robot.joints[x].geom);
        robot.joints[x].geom.add(robot.joints[x].display_geom);

    }
}


//////////////////////////////////////////////////
/////     ANIMATION AND INTERACTION LOOP
//////////////////////////////////////////////////

function animate() {

    // note: three.js includes requestAnimationFrame shim
    // alternative to using setInterval for updating in-browser drawing
    // this effectively request that the animate function be called again for next draw
    // http://learningwebgl.com/blog/?p=3189

    requestAnimationFrame( animate );

    // read map occupancy value for robot's current position (!! not working currently)
    // !!! build correspondence between map/texture and robot origin coordinates
    //if (display_map) { 
    //    pixelData = myCanvasContext.getImageData(Math.round(robot.origin.xyz[2]*10)+400,Math.round(robot.origin.xyz[0]*10)+400,1,1);
    //}

    // call user's animation loop
    my_animate();

    // make sure camera controls (THREE OrbitControls) are looking at the robot base
    camera_controls.target.x = robot.links[robot.base].geom.position.x;
    camera_controls.target.y = robot.links[robot.base].geom.position.y;
    camera_controls.target.z = robot.links[robot.base].geom.position.z;


    // threejs rendering update
    renderer.render( scene, camera );

}



//////////////////////////////////////////////////
/////     TESTING SUPPORT ROUTINES (not included)
//////////////////////////////////////////////////

// credit: http://stackoverflow.com/questions/15313418/javascript-assert
function assert(condition, message) {
    if (!condition) {
        throw message || "Assertion failed";
    }
}

    /* !!! test matrix multiplication
    var a = [[1,2],[3,4]];
    //var a = [[1,2],[3,4],[5,6],[7,8]];
    //var b = [[1,2],[3,4],[5,6]];
    var b = [[1,2,3],[4,5,6]];
    //link.temp = matrix_multiply(a,b);
    //link.xform.elements[3] = 5;
    //link.xform.makeRotationZ(Math.PI/2);
    //link.temp = matrix_threejs_to_2Darray(link.xform);
    //link.temp2 = matrix_2Darray_to_threejs(link.temp);
    */





