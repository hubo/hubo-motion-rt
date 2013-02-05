hubo-motion-rt
==============

(See README [without a ".md"] for a list of stuff to look over)

Pre-Requisite: hubo-ach (https://github.com/hubo/hubo-ach)

This is a software control system meant to allow a stable, modular, and easy-to-use software architecture for safely performing real-time control on Hubo.

The software here was written by M.X. Grey (mxgrey@gatech.edu), building upon the hubo-ach system developed by Dan Lofaro (dan@danlofaro.com) from Drexel University, and using the Ach IPC written by Neil Dantam (ntd@gatech.edu).

Please do not hesitate to contact Grey for help installing, using, debugging, or developing code with Hubo-Motion-RT.

As development of this architecture continues at Georgia Tech, further daemons will be developed, particularly for things like balancing, walking, and general manipulation. These daemons will allow the various functions to be performed in a consistent, stable, and reliable way so that the user can focus on algorithm/planning/autonomy development without needing to be concerned with managing the robot control or dynamics. The Hubo_Tech library will offer a convenient interface for sending commands and reading states from all of these upcoming daemons. Anyone who would like to contribute to the development of these daemons is invited to contact mxgrey@gatech.edu
