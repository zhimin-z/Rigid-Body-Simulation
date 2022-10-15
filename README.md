# Rigid-Body-Simulation
Implemented a real-time rigid body simulation in C++ and rendered using OpenGL for [CIS 563 - Phys Based Animation](https://www.coursicle.com/penn/courses/CIS/563/) 2015 Spring Project 1. Resolves collisions by reflecting velocity using a restitution coefficient. Contacts occur when two objects are stacking and are resolved by canceling the normal direction velocity between them. Determining intersections between rigid bodies is optimized by using the Sweep and Prune algorithm.

See https://youtu.be/3SyVzJ2v0oQ
