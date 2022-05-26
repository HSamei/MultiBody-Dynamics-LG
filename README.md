# MultiBody-Dynamics-LG
Multibody dynamics project for rigid-flexible systems with actuated joints. Flexible bodies use Cosserat Rod Theory formulated on a Lie Group framework.

Download all files and ensure Library folder is in same directory as the RunMe_FwdDyn() and RunMe_InvDyn(). These two files are what will be run to generate the simulations and you will need to uncomment the Sample you want to run, and comment the other one out. RunMe_InvDyn() should be tested first to compute the Inverse Dynamics and generate the control input files. This output file can then be used for RunMe_FwdDyn() to compute the Forward Dynamics. Both simulations will generate output plots as included in the paper and also animations of the dynamic system.

Although only these two samples are included in the paper, any arbitrary control or joint acceleration trajectory can be defined for the manipulators and be computed (if they are numerically viable under current discretization, which can also be modified). Different manipulators can also be defined with differing bodies and joints using the libraries provided, and I'll try to provide more examples of dynamic simulations as I study further into the numerical stability of the solutions.

Thank you for checking out the code, and please reach out to me at "hossainsamei@cmail.carleton.ca" should you have any questions, comments or concerns.

Samei
