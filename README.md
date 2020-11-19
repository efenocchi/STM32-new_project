# Real Time Embedded Systems 
## Università degli Studi di Modena e Reggio Emilia A.A. 2019/2020
---
# Introduction

During the Real-Time Embedded System course we were asked to create a project based on STM32’s B-L475E-IOT01A board, exploiting its sensors and connection capabilities. With the idea in mind to create something useful that could potentially be expanded from this prototype state and applied in a real world environment, we decided to develop a greenhouse management assistance system. In the next sessions a summary of the external elements utilized and a more in detail explanation of the system’s features is given.

# External hardware and software

In order to increase the set of functions that the system is capable of providing and to make it more user oriented, a set of hardware and software elements has been utilized in addition to the board provided.

External hardware:
*	Elegoo 7-segments display
*	Elegoo fan

External software:
*	TCP Server mobile app by mightyIT
*	PuTTY by Simon Tatham

# System features

As previously said, the goal of the project is to help a greenhouse owner with the management of its botanic structure. To do so, the system is composed of six tasks that utilize a shared struct to update and exploit the measures from various sensors. 
First of all, the default task establishes a Wi-Fi connection between the board and the server created by the user inside the TCP Server app. The 7-segments display, that by default displays a 0, will confirm the correct success of the operation by displaying a 1. The owner can now send commands to the board to know the current state of the greenhouse environment.
The parameters used to describe the greenhouse environment are the temperature, pressure and humidity of the air. The retrieval of this measures acquired with the correspondent sensors already available in the board are managed by a second task. The same task, in case the air temperature exceeds a certain threshold, will activate the fan, that represents the greenhouse cooling systems, in order to protect the plants from excessive heat.


A third task is activated by pressing the user button on the board, this can be used by the user to calculate the dew point. The dew point is the temperature to which air must be cooled to become saturated with water vapor. When cooled further, the airborne water vapor will condense to form liquid water (dew). The button can so be used by the owner to know at which temperature he has to take the greenhouse in order to create dew in the air to naturally water his plants. 
A fourth task is in charge of measuring the distance between the board and any object in its proximity. The idea is that since the board must be inside the greenhouse, in the eventuality that a plant or something else covers the board and obstructs the sensors, a LED will light up to notify the owner.
A fifth task is utilized to provide statistics regarding the sensors measures and it calculates the average temperature, pressure and humidity.
A sixth task has been created to print on the serial all the data acquired by the sensors, the statistics and the dew point if it has been requested. PuTTY’s software is used to listen to the data transmitted on the serial of the board and to display it on its interface for an improved user experience.

Complete Report: [Link](https://github.com/efenocchi/STM32-new_project/blob/master/ReportProgettoRTESFenocchiBellei.pdf)

![Fig.1](https://github.com/efenocchi/STM32-new_project/blob/master/pictures/board.jpg)



# Credits

| Author  | GitHub |  LinkedIn |
| ------------- | ------------- | ------------- |
| Emanuele Fenocchi  |  [@efenocchi](https://github.com/efenocchi)  | [Link](https://www.linkedin.com/in/emanuele-fenocchi-a0a29a152/) | 
| Francesco Bellei  | [@francescoBellei29](https://github.com/francescobellei29)  | [Link](https://it.linkedin.com/in/belleifrancesco) |
