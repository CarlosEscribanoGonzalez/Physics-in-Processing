## Overview
Collection of physics simulations for the course *Physics in Video Games*, implemented in Processing (Java mode). Each sketch focuses on a different family of techniques: numerical integration, mass-spring systems, penalty-based contact and impulse-based rigid body collisions.

## Projectile Motion and Numerical Integration
*(ProjectileMotion.pde)*

* Cannon with adjustable angle and muzzle velocity, firing at a wall with a moving gap
* Side-by-side comparison of integration methods on the same trajectory:
  * Analytical solution (ground truth)
  * Explicit Euler
  * Symplectic (semi-implicit) Euler
  * Midpoint method
  * Verlet (with a Taylor-based first step)
* Fixed time step with configurable substeps per rendered frame
* Ball-wall collision detection against the moving gap

## Symplectic Mass-Spring System
*(SymplecticMassSpringSystem.pde)*

* Node-spring model with springs defined by node indices
* Hooke spring forces and damping along the spring direction
* Symplectic Euler integration (bounded energy error, stable at high stiffness)
* Gravity and per-node forces accumulated every substep

## Soft Body with Penalty Contact
*(SymplecticMassSpringPenaltyContact.pde)*

* Spring-based deformable square with a diagonal brace
* Penalty forces for contact with the ground and walls, based on penetration depth
* Mouse interaction through a zero-rest-length spring attached to the grabbed node
* Configurable spring, contact and damping stiffness

## Rigid Body with Impulse-Based Collisions
*(RigidBoxSpringsImpulse.pde)*

* Rigid box with position, rotation, angular velocity, mass and inertia
* Box suspended by four springs, with forces and torques accumulated on the body
* Continuous collision detection with a ray-segment test between consecutive bullet positions, which avoids tunneling at high speed
* Collision impulse with coefficient of restitution, including the angular contribution of the contact point
* Controllable cannon to fire bullets at the box

## Installation guide

* Install [Processing](https://processing.org/download)
* Open the `.pde` file of the sketch you want to run (the folder name must match the file name)
* Press Run

## Controls

* **Projectile sketch:** Up/Down arrows change the cannon angle, Left/Right change the muzzle velocity, Enter fires
* **Soft body sketch:** click and drag a node with the mouse
* **Rigid box sketch:** Left/Right move the cannon, Enter fires
