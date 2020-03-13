💣 Stewart Platform Ball Ballancer 💣
   ===============================

This is a repository containing works performed over the period of five months to create working 
model of Stewart Platform ball ballancer. Problems encountered were not that trivial i thought 
they would be, but eventually i was figuring them out on my own while doing a lot of thinking and 
testing. The process of creating this project would go like this:

	1. Concept - I saw this really cool balancing platform, i wander if i can do the same on my 
	own.

	2. Two Crappy prototypes not yet implementing Stewart Platform (SP) kinematics - i can go 
	into details here on demand, but i don't think it's worth it.

	3.Stewart platform kinematics implemented into device geometry. I had to spend a lot of 
	time in the workshop trying to get perfect wooden hexagons (DON'T TELL ME I COULD 3D PRINT 
	IT)
	
	4.While having fun in the workshop and wood tools i didn't think much about kinematics
	to be implemented and how tricky it would get to create detailed vector model that won't 
	make my really strong servos ripp spherical joints apart. So yeah, you need to have 
	decently precise measurments of your geometry to be in the buisness.
	
	5.Here's where fun begins, once having kinematics in place, you can just plug joystick
	of your choice and play with ROLL & PITCH variables which are being converted by inverse 
	kinematics algorithm into Servos positions. Being a bit concerned about time management
	i used servo.h library, its not going to change the fact that you should know how PWM 
	works! This is also a place that i thought is the tip of the hill i am on - false.

	6.Having my superpowers of providing pitch and roll i started to think how to make it 
	listen to my 15" resistive touch panel which came from china after more than month of 
	waiting (3/4 of this time was laying in the polish customs office). The task seems quite 
	easy: i have two angles and a 15" plane, lets just treat it as a xy coordinate system and 
	feed them into the inverse kinematic algorithm, this idea is called a proportional 
	controll.

	7.Little did i know that 5 wire resistive touch panels require having a driver connected
	to them (yes, i didn't have it). First i had to figure out the algorithm behind reading 
	coordinates and later on discover that it's not sufficient condition to get them. Data  
	i was collecting (XY coordinates) didn't make any sense and even i was making physically 
	straight line, the output was showing beautiful and undesirable grapth
	in the shape rotated 45degree clockwise letter "S".
	
[See it here](https://github.com/Kompan15/Stewart-Platform-Ball-Ballancer/blob/master/Pictures/delay%2C%20no%20delay.png "S-Curve")
	
	After few days of thinking and not 
	having much time to work on the project i have read somewhere about parasitic capacitances
	and it was on spot, applying 3ms read delay gives them enough time to discharge to give me 
	a proper read.  

	8.I placed touch panel on my precious SP and was in the business. Every automatics 
	engineer will tell you that proportional controll is not going to take you far when 
	dealing with more complex and delicate dynamic systems, i had to go for over 100 houndred 
	years old idea called PID controll.

	9. PID (Proportional - Integral - Derrivative) can be described as a sum of three terms
	each describing a diferent signal characteristic.
		a) P is responsible for Proportional control signal change
		b) I takes care of integrating a static error existing after P and D stop working.
		c) D Derrivative (my favourite) deals with fast changes and represents signal rate 
	of change. D deals with fast changes in signal, what results in really impressive 
	system reaction to input.

	10. I took each PID term, multiply it by some coefficient (i got them by trial and 
	error) and out their sum into roll and pitch. For some systems you can use zigler-nichols 
	method for example. Or if you have system transfer function - matlab.	
	
	for sure i forgot to mention 60% of stuff so sorry for that, i am writing this over 6 
	months after having my thesis defended.
