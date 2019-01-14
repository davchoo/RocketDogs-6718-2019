# RocketDog's Robot and Vision Code
This repository contains the code for RocketDog's 2019 robot. It's written in Java and uses GradleRio. You will need [WPILib](https://github.com/wpilibsuite/allwpilib/releases) to compile the code. It uses Aragon Robotics' [Raspberry Pi Vision Helper](https://github.com/Aragon-Robotics-Team/raspberrypi-vision-helper#raspberry-pi-vision-helper) to deploy the vision code to the Raspberry Pi. [FRCVision](https://github.com/wpilibsuite/FRCVision-pi-gen/releases) has to be installed on the Raspberry Pi it to work. The IP address and ssh login can be configured in `vision/build.gradle`.
## Deploying
To deploy only to the robot run `./gradlew deployRobot`
To deploy only to the Raspberry Pi run `./gradlew :vision:deploy`
Running `./gradlew deploy` will deploy to both at the same time.