# Adaptive Newton-Euler Algorithm Simulation

This simulation for the Adaptive Newton-Euler Algorithm is from our article:

Kaya, M., Akbulut, M.A., Bayraktaroglu, Z.Y. et al. A Novel Recursive Algorithm for the Implementation of Adaptive Robot Controllers. J Intell Robot Syst 110, 115 (2024). https://doi.org/10.1007/s10846-024-02135-x

Abstract

In this paper, a novel recursive and efficient algorithm for real-time implementation of the adaptive and passivity-based controllers in model-based control of robot manipulators is proposed. Many of the previous methods on these topics involve the computation of the regressor matrix explicitly or non-recursive computations, which remains as the main challenge in practical applications. The proposed method achieves a compact and fully recursive reformulation without computing the regressor matrix or its elements. This paper is based on a comprehensive literature review of the previously proposed methods, presented in a unified mathematical framework suitable for understanding the fundamentals and making comparisons. The considered methods are implemented on several processors and their performances are compared in terms of real-time computational efficiency. Computational results show that the proposed Adaptive Newton-Euler Algorithm significantly reduces the computation time of the control law per cycle time in the implementation of adaptive control laws. In addition, using the dynamic simulation of an industrial robot with 6-DoF, trajectory tracking performances of the adaptive controllers are compared with those of non-adaptive control methods where dynamic parameters are assumed to be known.

Closer view of the robot with payload
![image](https://github.com/user-attachments/assets/99d0cc06-d252-4a8b-bca6-b2678705946c)

Task space trajectory of the robot's end-effector
![image](https://github.com/user-attachments/assets/19850d3b-014e-402b-8dab-1f787d3e6508)

