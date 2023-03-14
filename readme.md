readme.md currently in progress...

## **Introduction**

This library is for the fast implementation of model predictive control (MPC) policies in Python. The code is optimized for fast implementation and testing of nonlinear objective functions and was developed as a part of an undergraduate thesis project at the Ohio State University (linked below).

The library is currently being optimized for use in **MicroPython** and thus has been modified so that it does not require a *numpy* dependency - removing the need to install third-party libraries on the microcontroller of interest. The following sections of the *readme* will discuss the MPC policy structure and show demonstrations when applicable.

**Thesis:** https://kb.osu.edu/handle/1811/101702

**Note:** The class and associated functions shown here are in no way complete. It is my hope to use the model predictive control architecture as a platform for introducing myself to varying optimization strategies. More specifically, I intend to develop the library whenever I have freetime and new ideas.


___

## **Model Predictive Control**

<!-- First, the generalized MPC will be defined so that each of the following sections can be spent investigating a method of finding the solution to the problem.

Let us first define our model in terms of a discrete linear/nonlinear dynamical system.

$$
    x_{k+1} = f(x_k, u_k)
$$

Where $x \in \mathbb{R}^n$ is the state of the system, and $u \in \mathbb{R}^m$ defines the control terms. We can use this standard notation to describe the objective function in terms of the state and cost.

$$
    g(x, u) = 
$$ -->


___

## **Gradient Descent**

___

## **Nonlinear Newton's Optimization**

___

## **Application:** Sphere World Environment

<p align='center'>
    <img src=./Examples/.figures/sphereworld.gif width=450>
</p>