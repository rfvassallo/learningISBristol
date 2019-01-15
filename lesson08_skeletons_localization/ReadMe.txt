

In this lesson we collect the 3D positions of the detected skeletons and save the (x,y) coordinates in a map file.

You are going to find two main programs:

###

* skeleton_3Dlocalization_worldFrame.py
  Use the "SkeletonsGrouper.AreaID.Localization" service to get the skeletons localitazion in the areas 0, 1 and 2 of the Intelligent Space.
  To have all the skeletons in the same world reference frame (1000), the program uses the complementary files, transformation.py and utils.py, to convert all the skeletons to the same frame. 

###

* skeleton_3Dlocalization_worldFrame02.py
  Use the "SkeletonsGrouper.3DLocalization" service to get all the skeletons directly localitazed in the world reference frame 1000

##

Additional program:

* plot_map.py
  Plot the file given as an argument to the program
 
