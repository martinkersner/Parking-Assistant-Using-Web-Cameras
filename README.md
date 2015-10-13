## Parking Assistant Using Web Cameras

###Master Thesis

Martin Kersner, <m.kersner@gmail.com>

[Full text](https://dl.dropboxusercontent.com/u/13642345/Master_Thesis_Martin_Kersner.pdf)

Automobile industry experienced growth of connection with information technologies in the past few years.
Renowned companies announced development of self-driving cars that happened to be the objective for the next decade.
Self-driving cars contain various sensors enabling detection of road lines, road signs, traffic lights or obstacles which might affect safety of ride.
All mentioned features have mostly been contributed to ordinary cars and 
gradually facilitate a car driving.

Parking assistant is also among the mentioned features that are daily exploited by million of drivers.
Currently most of the new cars have built-in radar sensor that is able to measure distance between the sensor and object.
Detection of close object helps to back out a car, whereas no one is able to see right behind the car while parking.
Some of the cars are equipped with back up camera.
The necessity and usefulness of parking assistant has even been confirmed by US law that requires each car to posses a back up camera by 2018<sup>1</sup>.
On the other hand the radar sensor also has own drawback that is its single purpose.
There are no other scenarios for which it might be utilized.

While most of the cars possess radar or laser sensor for adaptive cruise control and collision avoidance systems, Subaru proposed and implemented system based on stereo cameras.
Stereo cameras preserve spatial information about scene, and allow to run image processing algorithms, e.g. distinguish between vehicles and pedestrians.
This makes stereo cameras multipurpose equipment.
Current enhanced digital sensors are able to see in the dark, therefore the prior advantage of radar and laser sensors decrease. 
Moreover, the overall price of stereo based system is generally lower<sup>2</sup>.
According to specification of the newest Subaru stereo system called 2015 Subaru EyeSight<sup>3</sup>, differential speed, to which the system is able to respond, is up to 30mph.

In this thesis we aim to create a simple parking assistant using more web cameras that enable to detect a stationary object in a colliding distance, detect an edge of a potentially dangerous object, such as pavement, and classify an object as a human being or other object.


[1] http://www.cnet.com/news/u-s-requiring-back-up-cameras-in-cars-by-2018    

[2] http://www.subaru.com/engineering/eyesight.html    

[3] http://www.planetsubaru.com/2015-subaru-eyesight-capability-revealed.htm
