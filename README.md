# soton-multirotor
Code for 2nd year multirotor UAV project at University of Southampton

## Information
_This code was last modified on 2017 May 15._

_This code was submitted as an attempt to fulfil part of a course at the University. The content is generated by the owner and the University is not responsible for the content within. The owner is not responsible for use of this code by persons other than said owner (that includes naughty cheaters)._

## Task
The task is to design a multirotor UAV that can perform the following tasks:

+ _Autonomously fly to a height of 1.0 m and drop a payload._
+ _Autonomously fly to a height of 1.0 m and a distance of 1.0 m each from two target horizontal points_ (i.e. position (1, 1, 1)) _and drop a payload_.
+ _Be able to switch seamlessly between remote-pilot manual control and autonomous control at any point, and as many times as needed, during the mission._

The design has to be sub-700g, necessitating the use of some extreme hardware and electronics choices.

The UAV learns from the pilot during any manual control, by sensing the pilot inputs and adjusting parameters. A state machine runs the high-level control using a set of global vars and interrupts. Important data is sent via Bluetooth to a laptop a la Flight Data Recorder.

The design can be seen below:

![Quad](https://github.com/MNahad/soton-multirotor/blob/master/assets/Untitled1.jpg "The design")

The UAV can be seen flying below:

![QuadFly](https://github.com/MNahad/soton-multirotor/blob/master/assets/Untitled2.jpg "The UAV in flight")

This design was featured on the University Design Show and Open Day events.

http://uosdesign.org/designshow2017/year-2/aeronautics-and-astronautics
