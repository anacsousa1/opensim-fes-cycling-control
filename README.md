# Control strategies for functional electrical stimulation (FES) cycling in OpenSim

In previous work, we created a simulation environment for FES cycling control (de Sousa et al., 2016). In this release, we investigated the use of passive knee orthoses in cycling (de Sousa et al., 2019). The newest version compares the cycling cadence and quadriceps excitation using an FES cycling simulation platform for different spring torques and ranges (c.f., (de Sousa et al., 2019)). Based on Hunt (2005); Bo et al. (2017), we defined ranges to excite each muscle to achieve cycling (Figures 1a and 1b, c.f., (de Sousa et al., 2016)). The control structure (Figure 2) incorporates the musculoskeletal dynamics, the predefined range angles, and the chosen controller, c.f., (de Sousa et al., 2016).

For more information about how the system works, see the [Tutorial](https://github.com/anacsousa1/opensim-fes-cycling-control/blob/master/Tutorial.pdf).

The software was tested with Windows 10 Operating System, Matlab MathWorks R2017a, R2018b and R2019a and OpenSim 3.3.

# Software requirements

- [Matlab MathWorks](https://www.mathworks.com/)
- [OpenSim 3.3](https://simtk.org/frs/?group_id=91)

# Setup and run

- Run the RUNME_BB.m in Matlab (runs the FES cycling using the open-loop (bang-bang) controller):

    ```matlab
    RUNME_BB.m
    ```

- Run the RUNME_PID.m in Matlab (runs the FES cycling using the PID controller.):

    ```matlab
    RUNME_PID.m
    ```
