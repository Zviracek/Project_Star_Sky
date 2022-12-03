# Project_Star_Sky
Such a fancy name for something as basic as a simple rocket.

The Star_Sky project, as I decided how to call it, is my take on creating a simple model scale rocket.
Something capable of just flying upwards in a stable line and then landing again would be a result, that I would consider adequate. But I am aming higher.

This repository should in the finale contain all the necesarry informations and knowladge, that I gathered trougout the procces of building everything from scratch.
So if you want to follow up, do it. I am open to any improvement to any system I created. All my systems are far from perfect, but for now, they do the job.

The first part of the project is about frame and any parts, that togeather creates the frame assembly.
Second part deals with the flight computer.
And the third is about software.

All the speech about each part would be in their coresponding section.

Additional informations and files will be added as I progress my work.

# Flight computer

In this section I would like to briefly talk about the flight computer I am currently using.
Quick disclaimer: This is version 1 revision 0. DO NOT USE THIS. It have lots of bugs, that I am aware of. I have just decided to ignore them for the time.
All files are in Flight_Computer folder.

As stated before, this iteration have many small mistakes, that leads to almost non-usable PCB. Almost because I found some workarounds.
I will not go through any of them, I don't have time nor mood to do that. 
The reason I am keeping (or uploading at first place) these files here, is that I want easy acces and sort of backup for these files. I still use the board in a funky way, so I have use for them. 
Moreover, someone can take an inspiration from my desing, and precaution from my mistakes.

There will be no revision 2. I have decided to move directly to mark II instead. 

The hearth of the computer is Teensy 4.1. Data are collected from MCU-9250 IMU and BME-280 barometric sensor.
The board also has Flash for data logging, 4 pyro channels and SPI breakout.
I won't go to details, this section will be chandged when mark II is finnished, so it'll be a waste of time.

If you need any information about function, design choices, or errors in design, don't hesitate to write me a mail. I am willing to provide you help and I will apriciate any interest.
