# Demonstration for testing the april tag detection

Here I used an apriltag from the family 36h11 with an id = 3


<img src="images/tag3.png" width="200"/><br/>

#

# Apriltags in Isaac Sim

then I've implemnted this april tag in a simple Isaac Sim simulation. I created a mesh cube with physical properties and then I created another mesh cube without any physical properities and I added the april tag .png file as a texture for the latter mesh cube and to make it look like a paper I had to minimize the depth of the cube to 0.0001 and lastly I put on the first cube with a fixed joint and I got this result:

<img src="images/april_tagged_cube.png" width="350"/>

#

# Launch code results:

<img src="images/camera_feedback.png" width="350"/><br/>
<img src="images/logs.png" width="350"/><br/>
<img src="images/rviz2_tf.png" width="350"/>

