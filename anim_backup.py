import bpy
from math import *
from mathutils import *

# Variable for currently active object
#myobj = bpy.context.object
# Alternatively, if you know that the object is called 'Cube'
# you can reach it by
myobj = bpy.data.objects['Cube']

# Clear all previous animation data
myobj.animation_data_clear()

# set first and last frame index
total_time = 2*pi # Animation should be 2*pi seconds long
fps = 24 # Frames per second (fps)
bpy.context.scene.frame_start = 0
bpy.context.scene.frame_end = int(total_time*fps)+1

# loop of frames and insert keyframes every 10th frame
keyframe_freq = 10
nlast = bpy.context.scene.frame_end
for n in range(nlast):
    t = total_time*n/nlast

    # Do computations here...

    # Check if n is a multiple of keyframe_freq
    if n%keyframe_freq == 0:
        # Set frame like this
        bpy.context.scene.frame_set(n)

        # Set current location like this
        myobj.location.x = ...
        myobj.location.y = ...
        myobj.location.z = ...

        # Insert new keyframe for "location" like this
        myobj.keyframe_insert(data_path="location")

        """
        """
        """

        for obj in rel_objects:
            obj.hide_viewport = True

        """
        """
        """
        """
        frame_num = 0
        frame_feq = 10
        for i in range(num_tasks):
            bpy.context.scene.frame_set(frame_num)
            rel_objects.keyframe_insert(data_path ="hide_viewport",index=-1)
            frame_num += frame_feq
        """
        """
        """
        """
        # Reach items for items in tasks assign to
        myobj = bpy.data.objects['Cube']

        # Init animation seq
        # Clear all previous animation data
        myobj.animation_data_clear()

        # Animation properties
        # set first and last frame index
        total_time = 2*pi # Animation should be 2*pi seconds long
        fps = 24 # Frames per second (fps)
        bpy.context.scene.frame_start = 0
        bpy.context.scene.frame_end = int(total_time*fps)+1

        keyframe_freq = 10
        nlast = bpy.context.scene.frame_end
        for n in range(nlast):
            t = total_time*n/nlast

            # Do computations here...

            # Check if n is a multiple of keyframe_freq
            if n%keyframe_freq == 0:
                # Set frame like this
                bpy.context.scene.frame_set(n)

                # Set current location like this
                # Initiate all hide
                bpy.context.object.hide_viewport = True

                myobj.location.x = ...
                myobj.location.y = ...
                myobj.location.z = ...

                # Insert new keyframe for "location" like this
                myobj.keyframe_insert(data_path="location")