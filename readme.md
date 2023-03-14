readme.md currently in progress...

## **Introduction**

This library is for the fast implementation of model predictive control (MPC) policies in Python. The code is optimized for fast implementation and testing of nonlinear objective functions and was developed as a part of an undergraduate thesis project at the Ohio State University (linked below).

The library is currently being optimized for use in **MicroPython** and thus has been modified so that it does not require a *numpy* dependency - removing the need to install third-party libraries on the microcontroller of interest. The following sections of the *readme* will discuss the MPC policy structure and show demonstrations when applicable.

**Note:** The class and associated functions shown here are in no way complete. It is my hope to use the model predictive control architecture as a platform for introducing myself to varying optimization strategies. More specifically, I intend to develop the library whenever I have freetime and new ideas.

**Thesis:** https://kb.osu.edu/handle/1811/101702


___

## **Model Predictive Control**

### 1. Definition of a Model and Prediction Horizon
First, the generalized MPC will be defined so that each of the following sections can be spent investigating a method of finding the solution to the problem.

Let us first define our model in terms of a discrete linear/nonlinear dynamical system.

$$
    x_{k+1} = f(x_k, u_k)
$$

Where $x \in \mathbb{M} \sub \mathbb{R}^n$ defines the system's state, $u \in \mathbb{U} \sub \mathbb{R}^m$ defines the control injection and the subscript $k$ defines the $k$-th step in a discrete time series. We thus say that $f : \mathbb{M} \rightarrow \mathbb{M}$, or that it defines the translation between states in $\mathbb{M}$.

We can use this model to characterize a prediction horizon as a series of simulated steps starting at some initial position, $x_0$, and of some predetermined length, $P$. Using this idea, we can write the collection of state terms which make up the prediction horizon using set theory notation...

$$
    X = \left\{ x_0, \dots, x_{P+1}\ |\ x_0 \in \mathbb{M} \wedge x_{k+1} = f(x_k,u_k)\ \forall k \in \mathbb{N}_P \right\}
$$

Where $\mathbb{N}_P$ denotes the set of whole numbers less than, but not equal to, $P$. In other words, $X$ consists of all of the states in the prediction horizon which obey the model, $f$, and start from a given initial position, $x_0$. We can similarly define the set of inputs as the list of controls which move the state forward. That is...

$$
    U = \left\{ u_0, \dots, u_{P}\ |\ u_k \in \mathbb{U}\ \forall k \in \mathbb{N}_{P} \right\}
$$

It is important to note that in some notations $u_k$ is defined as $u_k \in \mathbb{U}(x_k)$; implying that the input is a member of the set of *allowable* controls at a given state, $x_k$. In the applications shown here, the set allowable controls are equivalent at all states - even if some may be considered impossible in the real-world. Particulary, $\mathbb{U} \subseteq \mathbb{U} (x)\ \forall x$.


___

## **Gradient Descent**

**coming soon**


___

## **Nonlinear Newton's Optimization**

**coming soon**


___

## **Application:** Sphere World Environment

**coming soon**

<p align='center'>
    <img src=./Examples/.figures/sphereworld.gif width=450>
</p>