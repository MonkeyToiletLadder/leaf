# leaf
    Please let me know what you think.
    I am using this for an air hockey game.
    This algorithm gets the collision points for circle vs line segment collision over a time step.
    The main logic for this is in collision.hpp.
    To compile use g++ example.cpp -O2.
    
	Line Segment vs Circle 
	
	Swept Collision Detection
	
	
	Goal: 
	Calculate the points of collision 
	for a static line segment and 
	accelerating circle 
	over a time step.
	
	
	Obstacles:
	There are two cases.  
	A circle may collide
	with one of the end points or
	a point in between.
	
	These cases require different logic when
	locating intersection points on the circle and
	line segment.
	
	Distinguishing these cases is tricky.  
	How can we tell if the circle is going to
	collide with one of the end points or 
	a point in between?
	
	
	Solution:
	Part of the following code, lines 130 - 134, does this by checking 
	if the dot product of the circle velocity onto the segment is within 
	the dot products of the segment onto itself.
	
	
	TODO: Store the point of collision of the circle for all static and semi static lines.
	TODO: Test performance of this against other algorithms.  Find a legitimate way of doing this.
	TODO: Exit early if velocity vector points away such that no collision would be possible.
	TODO: Ability to check and resolve collision between dynamic segments vs circle and circle vs circle.
