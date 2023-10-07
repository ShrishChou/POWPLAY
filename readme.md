# Code from the First Tech Challenge Power Play season (2021-2022)

Table of contents
Detection
1. ContourPipeline is the pipeline used for vision in conjunction with a Logitech camera mounted to the robot
2. AprilTagAutonomousDetectionExample and AprilTagDetectionPipeline are the detection code that utilizes April Tags to detect between three separate possible tags and chooses based off the one found
3. Fruits Web is an object detection code that was developed through training a model with three possible fruits: apples, bananas, and pineapples. By placing a pattern of each on a side of the cone, the code would be able to detect which pattern was showing
4. Green_Pink_YCrCb_Detection and Vision test are two different color detection programs that utilize YCrCb scales to account for light intensity

A complete analysis of the algorithms can be found in this YouTube video: [https://youtu.be/rvT4R7bjyIU?si=NMqNmYfoqtVjJsd9](url)

Autonomous:
1. Sample Mecanum Drive and Drive Constants are two base files that contain key information about the robot ranging from the width to the wheel radius and are used in the tunning process in conjunction with RoadRunner ([https://learnroadrunner.com/](url)) to enable our robot to complete complex splines
2. AutoBlue1, ParkRIGHTREAL, shortcycle, and Detectingram are three different movement files that incorporate the detection code to bring together that autonoumous movement and detection in the robot

Controlled Period:
1. newrobottele is the code used in the robot controlled section that maps each button on a Logitech controller to various functions that work as state machines. Through relying on encoder values and servo positions, we are able to have the robot "communicate" within itself and allows automation thereby reducing pressure on drivers
