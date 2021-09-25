Interactive Marker
------------------

This package provides interactive marker in RViz for RoboCup objects (ball, goal, obstacles).

It can be launches with:
 
``roslaunch humanoid_league_interactive_marker interactive_marker.launch``

You should see the objects in RViz. 
You can drag them around to change their position and you can open a menu for each marker using a right click.
In the menu you can select, if this marker should be published and for the obstacles their type 
(red robot, blue robot, unknown).

The objects will be published in absolute position to the map on the topics
 
``/balls_absolute``, ``/goal_absolute`` and ``/obstacles_absolute``

as well as in relative space to the ``base_footprint`` on the topics

``/balls_relative``, ``/goal_relative`` and ``/obstacles_relative`` 


You need to provide a valid ``tf`` tree, otherwise the objects can't be transformed into relative space.